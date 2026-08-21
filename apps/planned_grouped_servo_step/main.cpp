#include <cstdlib>
#include <iostream>
#include <string>
#include <utility>

#include <motion_control_core/motion_control_core.hpp>

#include "components/robot/r1/r1_robot_config.hpp"
#include "loop.hpp"
#include "options.hpp"
#include "planning.hpp"
#include "solver.hpp"

namespace {

namespace app = motion_control_lab::planned_grouped_servo_step;
namespace mcc = motion_control::core;

constexpr const char *kProgramId = "mcl_planned_grouped_servo_step";

int run(int argc, char **argv, std::string &normal_exit_detail) {
  auto options = app::parseOptions(argc, argv);
  const auto &robot = motion_control_lab::r1RobotConfig();

  const auto model = app::loadRobotModel(robot, options);
  const auto collision_model = app::loadCollisionModel(model, options);
  const auto active_joint_names = app::activeJointNames(robot);
  const auto active_joint_full_indices = app::activeJointFullIndices(robot);

  mcc::KinematicsSolverBuilder builder;
  const auto solver_config = app::makeSolverConfig(options);
  app::requireOk(builder.configure(model, active_joint_names, solver_config),
                 "Failed to configure grouped IK builder");
  app::SolverHandles handles;
  app::configureSolver(builder, handles, collision_model, robot, options);

  mcc::GroupedKinematicsSolver solver;
  app::requireOk(builder.finalize(solver),
                 "Failed to finalize grouped IK solver");
  mcc::CartesianPlanner cartesian_planner;

  return app::runLoop(std::move(options), robot, solver, handles,
                      cartesian_planner, active_joint_full_indices,
                      normal_exit_detail);
}

} // namespace

int main(int argc, char **argv) {
  try {
    std::string normal_exit_detail;
    const int exit_code = run(argc, argv, normal_exit_detail);
    std::cerr << kProgramId << ": exited normally";
    if (!normal_exit_detail.empty()) {
      std::cerr << ' ' << normal_exit_detail;
    }
    std::cerr << '\n';
    return exit_code;
  } catch (const std::exception &error) {
    std::cerr << kProgramId << ": " << error.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << kProgramId << ": non-standard exception\n";
    return EXIT_FAILURE;
  }
}
