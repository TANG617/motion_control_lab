#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include <motion_control_core/motion_control_core.hpp>

#include "loop.hpp"
#include "options.hpp"
#include "planning.hpp"
#include "solver.hpp"

namespace {

namespace app = motion_control_lab::hierarchical_kinematics_step;
namespace mcc = motion_control::core;

constexpr const char *kProgramId = "mcl_hierarchical_kinematics_step";

int run(int argc, char **argv, std::string &normal_exit_detail) {
  auto options = app::parseOptions(argc, argv);
  if (options.dump_resolved_options) {
    std::cout << app::resolvedOptionsJson(options);
    return EXIT_SUCCESS;
  }
  // runLoop takes ownership of Options; retain a stable value while handing
  // the same resolved RobotOptions through the composition root.
  const auto robot = options.interactive.robot;

  const auto model = app::loadRobotModel(robot, options);
  const auto collision_model = app::loadCollisionModel(model, options);
  const auto joint_limits = app::makeJointTargetLimits(
      robot, options.interactive.robot.joint_stream);
  const auto active_joint_names =
      app::activeJointNames(robot, options.interactive.robot);
  const auto active_joint_full_indices =
      app::activeJointFullIndices(robot, options.interactive.robot);

  app::SolverHandles handles;
  app::SolverRuntime runtime;
  app::configureSolver(runtime, handles, model, active_joint_names,
                       collision_model, robot, options);
  const auto capabilities = app::profileCapabilities(options.profile);
  std::unique_ptr<mcc::CartesianPlanner> cartesian_planner;
  std::unique_ptr<mcc::JointPlanner> joint_planner;
  if (capabilities.cartesian_planning) {
    cartesian_planner = std::make_unique<mcc::CartesianPlanner>();
  }
  if (capabilities.joint_otg) {
    joint_planner = std::make_unique<mcc::JointPlanner>(
        app::makeJointPlannerConfig(options.planning));
  }

  return app::runLoop(std::move(options), robot, runtime, handles,
                      cartesian_planner.get(), joint_planner.get(), joint_limits,
                      active_joint_full_indices, normal_exit_detail);
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
