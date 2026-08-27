#include "components/tui/standard_ik_tui.hpp"

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
    if (candidate.title == title) {
      return candidate;
    }
  }
  std::cerr << "missing page: " << title << '\n';
  std::exit(EXIT_FAILURE);
}

const mcl::TuiSection & section(const mcl::TuiPage & source, const std::string & title)
{
  for (const auto & candidate : source.sections) {
    if (candidate.title == title) {
      return candidate;
    }
  }
  std::cerr << "missing section: " << source.title << '/' << title << '\n';
  std::exit(EXIT_FAILURE);
}

bool hasUiLabel(const mcl::TuiDocument & document, const std::string & label)
{
  for (const auto & candidate_page : document.pages) {
    if (candidate_page.title == label) {
      return true;
    }
    for (const auto & candidate_section : candidate_page.sections) {
      if (candidate_section.title == label) {
        return true;
      }
      for (const auto & row : candidate_section.rows) {
        if (row.label == label) {
          return true;
        }
      }
      for (const auto & table : candidate_section.tables) {
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

mcl::IkDebugFrame makeFrame()
{
  mcl::IkDebugFrame frame;
  frame.status = "Grouped IK running";
  frame.ik_status = "accepted";
  frame.selected_side = mcl::ArmSide::Right;
  frame.targets = {{mcl::ArmSide::Left, translated(0.7, 0.2, 1.1)},
                   {mcl::ArmSide::Right, translated(0.7, -0.2, 1.1)}};
  frame.forward_kinematics = {{mcl::ArmSide::Left, translated(0.69, 0.2, 1.1)},
                              {mcl::ArmSide::Right, translated(0.69, -0.2, 1.1)}};
  frame.target_errors = {{mcl::ArmSide::Left, 0.01, 0.001},
                         {mcl::ArmSide::Right, 0.01, 0.001}};
  frame.joint_names = {"head_yaw", "torso_yaw", "left_arm_joint1", "right_arm_joint1"};
  frame.positions = {0.1, 0.2, 0.3, -0.3};
  frame.velocities = {0.01, 0.02, 0.03, -0.03};

  for (const std::string label : {"Red", "Yellow"}) {
    mcl::SolverDebug solver;
    solver.label = label;
    solver.disposition = "accepted";
    solver.termination_reason = "single-iteration";
    solver.backend = "proxqp";
    solver.qp_status = "optimal";
    solver.native_status = "backend/native-payload";
    solver.ik_solve_time_ms = 0.25;
    solver.qp_solve_time_ms = 0.12;
    solver.maximum_hard_violation = 1.0e-6;
    solver.ik_solve_time_percentiles = {20, 20, 100, 0.30, 0.42, 0.50};
    solver.run_counters = mcl::SolverRunCounters{10, 9, 1};
    solver.grouped_attempt =
      mcl::GroupedAttemptDebug{"none", 2, 10, 9, "active", 8, 77, 1000};
    solver.task_scales.push_back({"arm", true, 0.95, 4.0, false, false});
    solver.requirements.push_back(
      {"joint_velocity", "rad/s", "model", true, true, 0.001, 2.0});
    frame.solvers.push_back(std::move(solver));
  }
  frame.workers.push_back(
    {"Red", 1000.0, 100, 1, 0, 2, 0.1, 0.8, 0.9, 0.0, 0.2, 3, 0.6, 0.01,
     0.5, 0.55, 0.0, 0.15, 0.35});
  frame.cpu_affinities = {{"ui", true, 1005, {5}, {5}}};
  mcl::SelfCollisionDebug collision;
  collision.label = "Yellow self-collision";
  collision.minimum_distance_before_m = 0.12;
  collision.minimum_distance_after_m = 0.13;
  collision.margin_shortfall_m = 0.0;
  collision.pairs.push_back({"left_link", "right_link", 0.12, 0.13, true});
  frame.self_collisions.push_back(std::move(collision));
  return frame;
}

}  // namespace

int main()
{
  mcl::InteractiveIkPresentation presentation;
  presentation.base_frame_id = "base_link";
  presentation.arms = {
    {mcl::ArmSide::Left, "left_input", "left_goal", "left_ik", {2}},
    {mcl::ArmSide::Right, "right_input", "right_goal", "right_ik", {3}}};

  auto frame = makeFrame();
  const auto standard = mcl::makeStandardIkTuiDocument(
    frame, presentation, 7, "null", "Grouped IK", "running");
  require(standard.subtitle == "Sink null · Pub 7",
          "teleop subtitle must not contain replay progress");
  const std::vector<std::string> expected_standard{
    "Monitor", "Solver", "Joints", "System"};
  require(standard.pages.size() == expected_standard.size(), "standard IK must produce four pages");
  for (std::size_t index = 0U; index < expected_standard.size(); ++index) {
    require(standard.pages[index].title == expected_standard[index], "standard page order mismatch");
  }
  const auto & overview = page(standard, "Monitor");
  require(
    overview.rows.size() == 2U && overview.rows[0].column_weights == std::vector<int>({3, 2}) &&
    overview.rows[1].column_weights == std::vector<int>({3, 2}),
    "Monitor must use the shared two-by-two grid");
  requireSingleTableSheets(standard);
  require(section(overview, "TCP tracking · Goal → Reference → Output").column == 0U &&
            section(overview, "Solver").column == 1U &&
            section(overview, "Workers").row == 1U &&
            section(overview, "Safety and hold").column == 1U,
          "Monitor panels must preserve the shared scan order");
  const auto & joints = section(page(standard, "Joints"), "Executed state").tables.front();
  require(joints.rows.size() == 4U && joints.rows[2][0] == "left_arm_joint1",
          "joint state mapping mismatch");
  require(
    section(page(standard, "System"), "Processor affinity").tables.front().rows[0][3] == "5",
    "processor affinity mapping mismatch");
  require(
    section(page(standard, "System"), "Collision pairs").tables.front().rows[0][1] ==
      "left_link",
    "collision pair mapping mismatch");

  frame.replay_frame_progress = mcl::ReplayFrameProgressDebug{127U, 1732U};
  const auto replay = mcl::makeStandardIkTuiDocument(
    frame, presentation, 7, "null", "Grouped IK", "running");
  require(replay.subtitle ==
            "Sink null · Pub 7 · Replay 128/1732",
          "replay subtitle must use one-based paired-timeline progress");
  frame.replay_frame_progress = mcl::ReplayFrameProgressDebug{1731U, 1732U};
  const auto replay_end = mcl::makeStandardIkTuiDocument(
    frame, presentation, 9, "null", "Grouped IK", "end of stream");
  require(replay_end.subtitle ==
            "Sink null · Pub 9 · Replay 1732/1732",
          "final replay frame must render as N/N");
  frame.replay_frame_progress.reset();

  mcl::CartesianPlannerDebug planner;
  planner.state = "planning";
  planner.sample_time_s = 0.125;
  for (const auto side : {mcl::ArmSide::Left, mcl::ArmSide::Right}) {
    mcl::PlannedArmDebug arm;
    arm.side = side;
    arm.source_goal = translated(1.0, side == mcl::ArmSide::Left ? 0.2 : -0.2, 1.1);
    arm.reference = translated(0.9, side == mcl::ArmSide::Left ? 0.2 : -0.2, 1.1);
    arm.forward_kinematics = translated(0.8, side == mcl::ArmSide::Left ? 0.2 : -0.2, 1.1);
    arm.reference_twist << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    arm.reference_acceleration << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6;
    planner.arms.push_back(std::move(arm));
  }
  frame.cartesian_planner = std::move(planner);
  const auto planned = mcl::makeStandardIkTuiDocument(
    frame, presentation, 8, "ws://127.0.0.1:8765", "Planned IK", "paused");
  require(planned.pages.size() == 5U, "planned IK must produce five pages");
  requireSingleTableSheets(planned);
  require(planned.pages[1].title == "Motion", "Motion page order mismatch");
  require(
    section(page(planned, "Motion"), "Cartesian states")
        .tables.front().rows.size() == 6U,
    "Cartesian planning pose mapping mismatch");
  const auto & reference_motion =
    section(page(planned, "Motion"), "Reference twist and acceleration").tables.front();
  require(reference_motion.columns.at(1).title == "      Vx" &&
            reference_motion.columns.at(2).title == "      Vy" &&
            reference_motion.rows.front().at(1) == "0.1000",
          "reference motion headers must reserve stable sign and precision space");
  require(planned.subtitle.find("Pub 8") != std::string::npos,
          "publish sequence must remain visible in the compact header");
  require(planned.status == "paused", "footer status must contain only the concise input state");

  const std::vector<std::string> forbidden_labels{
    "Overview", "Cartesian Planning", "Joint Planning", "Joint State", "Runtime", "Events",
    "Solver and Quadratic Programming"};
  for (const auto & label : forbidden_labels) {
    require(!hasUiLabel(planned, label), "standard TUI contains a forbidden abbreviation");
  }
  return EXIT_SUCCESS;
}
