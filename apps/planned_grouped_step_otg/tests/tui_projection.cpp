#include "components/tui/planned_grouped_tui.hpp"

#include <Eigen/Geometry>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace mcl = motion_control_lab;

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
  frame.status = "Grouped IK running";
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
    solver.grouped_attempt =
        mcl::GroupedAttemptDebug{"none", 2, 10, 9, true, true, "active", 8, 77, 1000};
    solver.task_scales.push_back({"arm", true, 0.95, 4.0, false, false});
    solver.requirements.push_back({"joint_velocity", "rad/s", "model", true, true, 0.001, 2.0});
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
  app.source_mode = "mcap replay";
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
  const mcl::PlannedGroupedTuiSnapshot servo_snapshot{
      &frame, &presentation, std::nullopt, 37, "ws://127.0.0.1:8765", "Planned Servo", "paused"};
  const auto servo_document = mcl::makePlannedGroupedTuiDocument(servo_snapshot);
  require(servo_document.pages.size() == 6U,
          "snapshot without Joint-OTG capability must produce six pages");
  require(!hasUiLabel(servo_document, "Joint Planning"),
          "snapshot without Joint-OTG capability must not produce an N/A page");

  const mcl::PlannedGroupedTuiSnapshot snapshot{
      &frame, &presentation, app, 38, "ws://127.0.0.1:8765", "Planned OTG", "paused"};
  const auto document = mcl::makePlannedGroupedTuiDocument(snapshot);

  const std::vector<std::string> expected_titles{
      "Overview", "Cartesian Planning", "Joint Planning", "Solver and Quadratic Programming",
      "Joint State", "Runtime", "Events"};
  require(document.pages.size() == expected_titles.size(), "projection must produce seven pages");
  require(document.footer_hints.find("? help") != std::string::npos,
          "footer must expose the conventional help key");
  for (std::size_t index = 0U; index < expected_titles.size(); ++index) {
    require(document.pages[index].title == expected_titles[index], "page order mismatch");
  }
  requireSingleTableSheets(document);
  const auto & overview = page(document, "Overview");
  require(overview.column_weights == std::vector<int>({3, 2}),
          "Overview must preserve its primary 3:2 columns");
  require(overview.rows.size() == 2U &&
              overview.rows.at(0).column_weights == std::vector<int>({3, 2}) &&
              overview.rows.at(0).height_weight == 3 &&
              overview.rows.at(1).column_weights == std::vector<int>({2, 3}) &&
              overview.rows.at(1).height_weight == 2,
          "Overview must use the reference 3:2 / 2:3 dashboard grid");
  require(page(document, "Runtime").column_weights == std::vector<int>({3, 2}),
          "Runtime must use 3:2 columns");
  for (const std::string title :
       {"Cartesian Planning", "Joint Planning", "Solver and Quadratic Programming",
        "Joint State", "Events"}) {
    require(page(document, title).column_weights == std::vector<int>({1}),
            "full-width page must use one column");
  }

  const auto & overview_cartesian = section(overview, "Cartesian tracking");
  require(overview_cartesian.row == 0U && overview_cartesian.column == 0U &&
              overview_cartesian.style == mcl::TuiSectionStyle::Panel &&
              overview_cartesian.tables.size() == 1U &&
              overview_cartesian.tables.front().style == mcl::TuiTableStyle::Compact &&
              overview_cartesian.tables.front().rows.size() == 6U,
          "Overview Cartesian panel must use compact reference-style pose sheets");
  require(section(overview, "Cartesian errors").row == 0U &&
            section(overview, "Cartesian errors").column == 0U,
          "Overview Cartesian errors must be a sibling sheet");
  const auto & overview_joints = section(overview, "Joint positions");
  require(overview_joints.row == 1U && overview_joints.column == 0U &&
              overview_joints.tables.front().rows.size() == 20U &&
              overview_joints.tables.front().columns.size() == 3U,
          "Overview must preserve all joint positions");
  const auto & overview_runtime = section(overview, "Worker runtime");
  require(overview_runtime.row == 1U && overview_runtime.column == 1U &&
              overview_runtime.tables.front().columns.size() == 5U &&
              section(overview, "Processor affinity").column == 1U &&
              section(overview, "Collision safety").column == 1U &&
              section(overview, "Safety").row == 0U &&
              section(overview, "Safety").column == 1U &&
              section(overview, "Planning").row == 1U &&
              section(overview, "Planning").column == 1U,
          "Overview diagnostic sheets must retain their dashboard cells");

  const auto & poses =
      section(page(document, "Cartesian Planning"), "Pose: Goal to Reference to FK").tables.at(0);
  require(poses.columns.size() == 9U && poses.rows.size() == 6U,
          "Cartesian pose sheet shape mismatch");
  const auto & targets =
      section(page(document, "Joint Planning"), "IK and projected target").tables.at(0);
  require(targets.columns.size() == 7U && targets.rows.size() == 20U,
          "joint target chain must expose seven columns and 20 joints");
  require(targets.rows.at(3).at(0) == "joint_3" && targets.rows.at(3).at(6) == "VA",
          "joint projection mapping mismatch");
  require(targets.columns.at(1).alignment == mcl::TuiTableAlignment::Right,
          "joint numeric columns must be right aligned");
  const auto & execution =
      section(page(document, "Joint State"), "Executed joint state").tables.at(0);
  require(execution.rows.size() == 20U && execution.rows.back().front() == "joint_19",
          "execution sheet must preserve all full joint names");
  const auto & joint_state = page(document, "Joint State");
  require(joint_state.rows.size() == 3U &&
              joint_state.rows.at(0).column_weights == std::vector<int>({1, 1}) &&
              section(joint_state, "Group maximum motion").row == 0U &&
              section(joint_state, "Group maximum motion").column == 0U &&
              section(joint_state, "Group maximum limit utilization").row == 0U &&
              section(joint_state, "Group maximum limit utilization").column == 1U &&
              section(joint_state, "Executed joint state").row == 1U &&
              section(joint_state, "Position limits and utilization").row == 2U,
          "Joint State summaries must share the first row above the full-width joint sheets");
  require(
    section(joint_state, "Group maximum motion").tables.front().columns.at(1).title ==
        "Velocity [rad/s]" &&
      section(joint_state, "Group maximum limit utilization")
          .tables.front()
          .columns.at(1)
          .title == "Velocity [%]",
    "Joint State summary columns must avoid repeating their panel titles");
  const auto & projection =
      section(page(document, "Events"), "Joint projection events").tables.at(0);
  require(projection.rows.at(0).at(0) == "joint_3" && projection.rows.at(0).at(2) == "12.000000" &&
              projection.rows.at(0).at(3) == "10.000000",
          "projection event values were not mapped");
  const auto & affinity = section(page(document, "Runtime"), "Processor affinity");
  require(affinity.column == 1U && affinity.tables.at(0).rows.at(0).at(3) == "5",
          "requested/effective CPU mapping mismatch");
  for (const std::string forbidden :
       {"Solver/QP", "Joints", "QP", "P95", "Proj", "Calc", "Exec", "V%", "A%", "J%",
        "CPU", "TID"}) {
    require(!hasUiLabel(document, forbidden), "TUI contains a forbidden abbreviated label");
  }
  return EXIT_SUCCESS;
}
