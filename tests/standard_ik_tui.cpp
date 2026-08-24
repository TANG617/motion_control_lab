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
  presentation.arms = {{mcl::ArmSide::Left, "left_target", "left_fk", {2}},
                       {mcl::ArmSide::Right, "right_target", "right_fk", {3}}};

  auto frame = makeFrame();
  const auto standard = mcl::makeStandardIkTuiDocument(
    frame, presentation, 7, "null", "Grouped IK", "running");
  const std::vector<std::string> expected_standard{
    "Overview", "Solver and Quadratic Programming", "Joint State", "Runtime", "Events"};
  require(standard.pages.size() == expected_standard.size(), "standard IK must produce five pages");
  for (std::size_t index = 0U; index < expected_standard.size(); ++index) {
    require(standard.pages[index].title == expected_standard[index], "standard page order mismatch");
  }
  const auto & overview = page(standard, "Overview");
  require(
    overview.rows.size() == 2U && overview.rows[0].column_weights == std::vector<int>({3, 2}) &&
    overview.rows[1].column_weights == std::vector<int>({2, 3}),
    "Overview must use the reference two-by-two grid");
  requireSingleTableSheets(standard);
  require(section(overview, "Cartesian errors").column == 0U &&
            section(overview, "Cartesian errors").row == 0U,
          "Cartesian errors must be a sibling sheet in the tracking cell");
  require(section(overview, "Worker runtime").column == 1U &&
            section(overview, "Worker runtime").row == 1U &&
            section(overview, "Processor affinity").column == 1U &&
            section(overview, "Collision safety").column == 1U,
          "Overview runtime diagnostics must be sibling sheets");
  const auto & joints = section(overview, "Joint positions").tables.front();
  require(joints.rows.size() == 4U && joints.rows[2][0] == "Left arm",
          "joint grouping mapping mismatch");
  require(
    section(page(standard, "Runtime"), "Processor affinity").tables.front().rows[0][3] == "5",
    "processor affinity mapping mismatch");
  require(
    section(page(standard, "Events"), "Self-collision pairs").tables.front().rows[0][1] ==
      "left_link",
    "collision pair mapping mismatch");

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
  require(planned.pages.size() == 6U, "planned IK must produce six pages");
  requireSingleTableSheets(planned);
  require(planned.pages[1].title == "Cartesian Planning", "Cartesian page order mismatch");
  require(
    section(page(planned, "Cartesian Planning"), "Pose: Goal to Reference to FK")
        .tables.front().rows.size() == 6U,
    "Cartesian planning pose mapping mismatch");
  require(planned.subtitle.find("publish sequence=8") != std::string::npos,
          "publish sequence label must be written in full");
  require(planned.status == "paused", "footer status must contain only the concise input state");

  const std::vector<std::string> forbidden_labels{
    "Solver/QP", "Joints", "QP", "P95", "Proj", "Calc", "Exec", "V%", "A%", "J%",
    "CPU", "TID"};
  for (const auto & label : forbidden_labels) {
    require(!hasUiLabel(planned, label), "standard TUI contains a forbidden abbreviation");
  }
  return EXIT_SUCCESS;
}
