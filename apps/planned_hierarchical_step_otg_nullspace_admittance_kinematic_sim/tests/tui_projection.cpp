#include "components/tui/planned_grouped_tui.hpp"

#include "../nullspace.hpp"

#include <Eigen/Geometry>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace mcl = motion_control_lab;
namespace nsapp = motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim;

namespace
{

void require(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

const mcl::TuiPage & page(const mcl::TuiDocument & document, const std::string & title)
{
  for (const auto & candidate : document.pages) {
    if (candidate.title == title)
      return candidate;
  }
  std::cerr << "missing page: " << title << '\n';
  std::exit(EXIT_FAILURE);
}

const mcl::TuiSection & section(const mcl::TuiPage & source, const std::string & title)
{
  for (const auto & candidate : source.sections) {
    if (candidate.title == title)
      return candidate;
  }
  std::cerr << "missing section: " << source.title << '/' << title << '\n';
  std::exit(EXIT_FAILURE);
}

bool hasUiLabel(const mcl::TuiDocument & document, const std::string & label)
{
  for (const auto & page : document.pages) {
    if (page.title == label) {
      return true;
    }
    for (const auto & section : page.sections) {
      if (section.title == label) {
        return true;
      }
      for (const auto & row : section.rows) {
        if (row.label == label) {
          return true;
        }
      }
      for (const auto & table : section.tables) {
        for (const auto & column : table.columns) {
          if (column.title == label) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

void requireSingleTableSheets(const mcl::TuiDocument & document)
{
  for (const auto & candidate_page : document.pages) {
    for (const auto & candidate_section : candidate_page.sections) {
      require(candidate_section.tables.size() <= 1U,
              "a titled sheet must not contain nested sibling tables");
    }
  }
}

mcl::Pose translated(double x, double y, double z)
{
  mcl::Pose result = mcl::Pose::Identity();
  result.translation() = Eigen::Vector3d{x, y, z};
  return result;
}

} // namespace

int main()
{
  mcl::IkDebugFrame frame;
  frame.runtime_state = mcl::IkRuntimeState::Running;
  frame.status = "Hierarchical IK running";
  frame.ik_status = "accepted";
  frame.paused = true;
  frame.selected_side = mcl::ArmSide::Right;

  mcl::CartesianPlannerDebug cartesian;
  cartesian.state = "planning";
  cartesian.sample_time_s = 0.125;
  for (const auto side : {mcl::ArmSide::Left, mcl::ArmSide::Right}) {
    mcl::PlannedArmDebug arm;
    arm.side = side;
    arm.source_goal = translated(side == mcl::ArmSide::Left ? 1.0 : -1.0, 2.0, 3.0);
    arm.reference = translated(side == mcl::ArmSide::Left ? 0.9 : -0.9, 2.0, 3.0);
    arm.forward_kinematics = translated(side == mcl::ArmSide::Left ? 0.8 : -0.8, 2.0, 3.0);
    arm.reference_twist << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    arm.reference_acceleration << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6;
    cartesian.arms.push_back(std::move(arm));
  }
  frame.cartesian_planner = std::move(cartesian);

  for (const std::string label : {"Red", "Yellow"}) {
    mcl::SolverDebug solver;
    solver.label = label;
    solver.disposition = "accepted";
    solver.termination_reason = "single-iteration";
    solver.converged = true;
    solver.ik_iterations = 1;
    solver.backend = "proxqp";
    solver.qp_status = "optimal";
    solver.native_status = "solved";
    solver.ik_solve_time_ms = 0.25;
    solver.qp_solve_time_ms = 0.12;
    solver.maximum_hard_violation = 1.0e-6;
    solver.ik_solve_time_percentiles.p95 = 0.42;
    if (label == "Red") {
      solver.qp_passes = {
          {"Primary", true, true, "optimal", "PROXQP_SOLVED", 0.21, 2,
           false},
          {"Secondary", true, true, "optimal", "PROXQP_SOLVED", 0.19, 1,
           false},
          {"Tertiary", false, false, "not-run", "", 0.0, 0, false},
          {"Terminal", true, true, "optimal", "PROXQP_SOLVED", 0.18, 1,
           false}};
      solver.qp_passes.at(0).solve_time_percentiles =
          {120, 120, 4096, 0.31, 0.41, 0.51};
      solver.qp_passes.at(1).solve_time_percentiles =
          {120, 120, 4096, 0.29, 0.39, 0.49};
      solver.qp_passes.at(3).solve_time_percentiles =
          {120, 120, 4096, 0.28, 0.38, 0.48};
      auto &primary = solver.qp_passes.at(0);
      primary.succeeded = false;
      primary.status = "maximum-iterations";
      primary.native_status = "PROXQP_MAX_ITER_REACHED";
      primary.objective_value = -12.5;
      primary.primal_residual = 7.5e-4;
      primary.dual_residual = 2.5e-3;
      primary.last_iterate_available = true;
      primary.constraint_violations.push_back(
          {"task-equation", "-", "upper", "left-position", "y", "m/s",
           1.0e-3, -2.5e-4, 2.5e-4, 7.5e-4});
    }
    solver.grouped_attempt =
        mcl::GroupedAttemptDebug{"none", 2, 10, 9, "active", 8, 77, 1000};
    solver.task_scales.push_back(
      {"red-primary/task/left-tcp-position-progress", true, 0.95, 4.0, false, false,
       "Primary", "last-accepted", true});
    solver.tasks.push_back(
      {"yellow/task/posture-preference", "posture", "rad/s", "Solve", "last-accepted",
       "active", true, 0.02, 0.0, 1.5, true, true, "soft"});
    solver.requirements.push_back(
      {"red-shared/constraints/joint-position-limits", "rad/s", "position-braking", true,
       true, 0.0, 0.0, "Shared", "last-accepted", "active", "left_arm_joint6",
       2.0e-5, false, true});
    if (label == "Red") {
      solver.task_scales.push_back(
        {"red-primary/task/left-tcp-position-progress", true, 0.25, 28.125, true, false,
         "Primary", "failed-last-iterate", true});
      solver.tasks.push_back(
        {"red-primary/task/left-tcp-position", "position", "m/s", "Primary",
         "failed-last-iterate", "violated", true, 7.5e-4, 5.0e-4, 0.0, false, true,
         "scaled"});
      solver.requirements.push_back(
        {"red-shared/constraints/joint-position-limits", "rad/s", "position-braking", true,
         true, 7.5e-4, 0.0, "Primary", "failed-last-iterate", "violated",
         "left_arm_joint6", -7.5e-4, false, true});
    }
    frame.solvers.push_back(std::move(solver));
  }
  frame.workers.push_back({"Red", 1000.0, 100, 1, 0, 2, 0.1, 0.8, 0.9, 0.0, 0.2, 3, 0.6, 0.01, 0.5,
                           0.55, 0.0, 0.15, 0.35});
  frame.workers.push_back({"Yellow", 100.0, 10, 0, 0, 0, 0.2, 1.2, 1.3, 0.0, 0.7, 0, 0.5, 0.02, 0.9,
                           0.95, 0.0, 0.6, 0.3});
  frame.cpu_affinities = {{"ui", true, 1005, {5}, {5}},
                          {"red", true, 1006, {6}, {6}},
                          {"yellow", true, 1007, {7}, {7}}};
  mcl::SelfCollisionDebug collision;
  collision.minimum_distance_m = 0.1;
  collision.influence_distance_m = 0.15;
  collision.minimum_distance_before_m = 0.12;
  collision.minimum_distance_after_m = 0.13;
  collision.pairs.push_back({"left_link", "right_link", 0.12, 0.13, true});
  frame.self_collisions.push_back(std::move(collision));

  mcl::PlannedGroupedJointOtgTuiDebug app;
  app.source_mode = "mcap replay + elbow teleop";
  app.target_mode = "future-o1-pv";
  app.feedback_topology = "split IK reference / OTG execution";
  app.left_task_scale = 0.9;
  app.right_task_scale = 0.8;
  app.cartesian_limits = {0.8, 4.0, 20.0, 1.0, 2.0, 10.0};
  app.cartesian_plan = {"planning", 0.5, 0.125, 1, 0.031};
  app.joint_plan = {"planning", 0.4, 0.001, 401, 0.016};
  app.joint_step = {"planning", 0.4, 0.001, 1, 0.002};
  app.projection_event_count = 9;
  app.projection_cycle_count = 3;
  app.modified_joint_count = 1;
  app.raw_otg_max_position_delta = 0.11;
  app.raw_otg_max_velocity_delta = 0.22;
  app.maximum_absolute_velocity = 0.33;
  app.maximum_absolute_acceleration = 0.44;
  app.maximum_absolute_jerk = 7.25;
  app.clamp_target_revision = 12;
  app.maximum_clamp_limit_ratio = 1.5;
  for (std::size_t index = 0U; index < 20U; ++index) {
    frame.joint_names.push_back("joint_" + std::to_string(index));
    frame.positions.push_back(0.6 + static_cast<double>(index));
    frame.velocities.push_back(0.7 + static_cast<double>(index));
    app.joints.push_back({"joint_" + std::to_string(index),
                          index < 2U ? "H" + std::to_string(index) : "J" + std::to_string(index),
                          0.1 + index, 0.2 + index, 0.3 + index, 0.4 + index, 0.5 + index,
                          0.6 + index, 0.7 + index, 0.8 + index, 0.9 + index, -2.0, 2.0, 5.0, 10.0,
                          100.0, index == 3U ? "VA" : "-"});
  }
  app.projection_events.push_back({"joint_3", "acceleration-limit", 12.0, 10.0, 10.0});
  app.clamp_events.push_back({"left", "linear_velocity", "x", 1.2, 0.8, 0.8});

  mcl::InteractiveIkPresentation presentation;
  presentation.base_frame_id = "base_link";
  presentation.requirements_page_enabled = true;
  const mcl::PlannedGroupedTuiSnapshot servo_snapshot{
      &frame, &presentation, std::nullopt, 37, "ws://127.0.0.1:8765", "Planned Servo", "paused"};
  const auto servo_document = mcl::makePlannedGroupedTuiDocument(servo_snapshot);
  require(servo_document.pages.size() == 6U,
          "snapshot with Requirements but without Joint-OTG must produce six pages");
  require(!hasUiLabel(servo_document, "Joint Planning"),
          "snapshot without Joint-OTG capability must not produce an N/A page");

  nsapp::NullspaceTuiDebug nullspace;
  nullspace.selected_side = mcl::ArmSide::Right;
  nullspace.control_point = nsapp::ControlPoint::Link4;
  nullspace.held_link4_side = mcl::ArmSide::Right;
  nullspace.link4_target.right_enabled = true;
  nullspace.link4_target.right = Eigen::Vector3d{-0.2, -0.3, 0.4};
  nullspace.raw_right_link4 = Eigen::Vector3d{-0.2, -0.29, 0.4};
  nullspace.executed_right_link4 = Eigen::Vector3d{-0.2, -0.28, 0.4};
  nullspace.right_link4_raw_error_m = 0.01;
  nullspace.right_link4_executed_error_m = 0.02;
  nullspace.right_tcp_position_error_m = 1.0e-5;
  nullspace.right_tcp_orientation_error_rad = 2.0e-5;
  nullspace.primary_maximum_preservation_drift = 3.0e-5;
  nullspace.primary_preservation_tolerance = 5.0e-4;
  nullspace.link4_task_error_m = 0.01;
  nullspace.yellow_posture_error_rad = 0.2;
  nullspace.link4_weight = 100.0;
  nullspace.yellow_weight = 1.0;
  nullspace.highest_completed_priority = "Tertiary";
  nullspace.primary_attempted = true;
  nullspace.secondary_attempted = true;
  nullspace.secondary_succeeded = true;
  nullspace.tertiary_attempted = true;
  nullspace.tertiary_succeeded = true;

  mcl::PlannedGroupedTuiSnapshot snapshot;
  snapshot.frame = &frame;
  snapshot.presentation = &presentation;
  snapshot.joint_otg = app;
  snapshot.publish_count = 38;
  snapshot.sink_status = "ws://127.0.0.1:8765";
  snapshot.title = "Planned OTG Null-space";
  snapshot.input_status = "paused";
  snapshot.extra_pages = {nsapp::makeNullspaceTuiPage(nullspace)};
  snapshot.header_context =
      "input replay+elbow · focus link4 · held right · step 0.0050 m";
  snapshot.footer_hints =
      "c disable elbow · ←/→ arm · wasd/qe move · x clear · ? help";
  snapshot.help_lines = {
      "Replay: Space pauses/resumes; . advances one frame; Esc exits",
      "Elbow: c enables/disables one link4 target; Left/Right selects its arm",
      "Elbow: w/s +/-x; a/d +/-y; q/e +/-z in base_link",
      "Elbow: Up/Down or m changes step; r captures executed link4; x clears/exits",
      "Paused replay accepts navigation and replay controls, not elbow edits"};
  const auto document = mcl::makePlannedGroupedTuiDocument(snapshot);

  const std::vector<std::string> expected_titles{
      "Monitor", "Motion", "Solver", "Requirements", "Joints", "System", "Null-space"};
  require(document.pages.size() == expected_titles.size(), "projection must produce seven pages");
  require(document.footer_hints.find("? help") != std::string::npos,
          "footer must expose the conventional help key");
  require(document.header_left.find("source mcap replay + elbow teleop") !=
              std::string::npos &&
              document.header_left.find("input replay+elbow") != std::string::npos &&
              document.header_left.find("focus link4") != std::string::npos &&
              document.header_left.find("step 0.0050 m") != std::string::npos &&
              document.header_right.find("Red Primary MAX_ITER") != std::string::npos,
          "compact header must expose mode, focus, step, and solver exit");
  require(document.help_lines.size() == 6U &&
              document.help_lines.at(1).find("Space") != std::string::npos &&
              document.help_lines.at(2).find("enables/disables") !=
                  std::string::npos &&
              document.help_lines.at(3).find("base_link") !=
                  std::string::npos &&
              document.help_lines.at(4).find("captures executed link4") !=
                  std::string::npos &&
              document.help_lines.back().find("not elbow edits") !=
                  std::string::npos,
          "help must expose the complete app-local hybrid replay key map");
  for (std::size_t index = 0U; index < expected_titles.size(); ++index) {
    require(document.pages[index].title == expected_titles[index], "page order mismatch");
  }
  requireSingleTableSheets(document);
  const auto & overview = page(document, "Monitor");
  require(overview.rows.size() == 3U &&
              overview.rows.at(0).column_weights == std::vector<int>({3, 2}) &&
              overview.rows.at(0).height_weight == 3 &&
              overview.rows.at(1).column_weights == std::vector<int>({3, 2}) &&
              overview.rows.at(1).height_weight == 2 &&
              overview.rows.at(2).column_weights == std::vector<int>({1}),
          "Monitor must use the shared 160x72 dashboard grid and failure preview row");

  const auto &nullspace_page = page(document, "Null-space");
  require(nullspace_page.responsive_layouts.empty() &&
              nullspace_page.rows.size() == 2U &&
              nullspace_page.rows.at(0).column_weights == std::vector<int>({1, 1}) &&
              nullspace_page.rows.at(1).column_weights == std::vector<int>({1, 1}) &&
              nullspace_page.sections.size() == 4U,
          "null-space must use the shared fixed 160x72 capability-page geometry");
  require(section(nullspace_page, "Control").style == mcl::TuiSectionStyle::Panel &&
              section(nullspace_page, "Control").tables.front().rows.at(0).at(1) == "right" &&
              section(nullspace_page, "TCP hierarchy status").column == 1U &&
              section(nullspace_page, "Tertiary objectives").row == 1U,
          "null-space semantics must be projected into shared panels");

  const auto & overview_cartesian =
      section(overview, "TCP tracking · Goal → Reference → Output");
  require(overview_cartesian.row == 0U && overview_cartesian.column == 0U &&
              overview_cartesian.style == mcl::TuiSectionStyle::Panel &&
              overview_cartesian.tables.size() == 1U &&
              overview_cartesian.tables.front().style == mcl::TuiTableStyle::Compact &&
              overview_cartesian.tables.front().rows.size() == 2U,
          "Monitor must show one compact tracking row per arm");
  require(section(overview, "Workers").row == 1U &&
              section(overview, "Safety and hold").column == 1U &&
              section(overview, "Failure focus · top violations from failed candidate")
                    .tables.front().rows.front().at(3) == "left-position",
          "Monitor panels must preserve shared scan order");

  const auto & poses =
      section(page(document, "Motion"), "Cartesian states").tables.at(0);
  require(poses.columns.size() == 9U && poses.rows.size() == 6U,
          "Cartesian pose sheet shape mismatch");
  const auto & targets =
      section(page(document, "Joints"), "Joint chain · Raw IK → Target → Executed").tables.at(0);
  require(targets.columns.size() == 14U && targets.rows.size() == 20U,
          "joint chain must expose raw, target, executed, bounds, and utilization");
  require(targets.rows.at(3).at(0) == "joint_3" && targets.rows.at(3).at(13) == "VA",
          "joint projection mapping mismatch");
  require(targets.columns.at(1).alignment == mcl::TuiTableAlignment::Right,
          "joint numeric columns must be right aligned");
  const auto &passes =
      section(page(document, "Solver"), "Passes")
          .tables.at(0);
  require(passes.columns.size() == 10U && passes.rows.size() == 4U,
          "solver pass master table shape mismatch");
  require(passes.rows.at(0).at(1) == "Primary" &&
              passes.rows.at(0).at(2) == "FAIL" &&
              passes.rows.at(0).at(3) == "MAX_ITER" &&
              passes.rows.at(0).at(5) == "0.2100" &&
              passes.rows.at(0).at(6) == "0.3100" &&
              passes.rows.at(0).at(7) == "0.4100" &&
              passes.rows.at(0).at(8) == "0.5100" &&
              passes.rows.at(2).at(2) == "SKIP" &&
              passes.rows.at(3).at(1) == "Terminal",
          "pass results and timing were not preserved");
  const auto &failed_constraints =
      section(page(document, "Solver"),
              "Last-iterate violations · diagnostic only")
          .tables.at(0);
  require(failed_constraints.columns.size() == 11U &&
              failed_constraints.rows.size() == 1U &&
              failed_constraints.rows.at(0).at(3) == "left-position" &&
              failed_constraints.rows.at(0).at(4) == "y" &&
              failed_constraints.rows.at(0).at(9) == "0.000750000",
          "ranked last-iterate constraint evidence was not preserved");
  const auto & accepted_tasks =
      section(page(document, "Requirements"), "Task costs · last accepted")
          .tables.at(0);
  require(accepted_tasks.rows.front().at(0) == "last-accepted" &&
              accepted_tasks.rows.front().at(9) == "1.5000000",
          "accepted task provenance and cost were not preserved");
  const auto & rejected_constraints =
      section(page(document, "Requirements"),
              "Constraint status and slack · failed candidate")
          .tables.at(0);
  require(rejected_constraints.rows.front().at(0) ==
              "failed-last-iterate" &&
              rejected_constraints.rows.front().at(6) ==
                  "left_arm_joint6" &&
              rejected_constraints.rows.front().at(9) == "-0.000750000" &&
              rejected_constraints.rows.front().at(11) == "-",
          "failed constraint provenance, slack, and unavailable cost were not preserved");
  const auto & projection =
      section(page(document, "System"), "Projection events").tables.at(0);
  require(projection.rows.at(0).at(0) == "joint_3" && projection.rows.at(0).at(2) == "12.000000" &&
              projection.rows.at(0).at(3) == "10.000000",
          "projection event values were not mapped");
  const auto & affinity = section(page(document, "System"), "Processor affinity");
  require(affinity.tables.at(0).rows.at(0).at(3) == "5",
          "requested/effective CPU mapping mismatch");
  for (const std::string forbidden :
       {"Overview", "Cartesian Planning", "Joint Planning", "QP Solver", "Joint State",
        "Runtime", "Events"}) {
    require(!hasUiLabel(document, forbidden), "TUI contains a retired page or panel label");
  }
  return EXIT_SUCCESS;
}
