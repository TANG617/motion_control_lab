#include <cstdlib>
#include <iostream>
#include <string>

#include <motion_control_core/motion_control_core.hpp>

#include "components/robot/r1/r1_robot_config.hpp"
#include "loop.hpp"
#include "options.hpp"
#include "solver.hpp"

namespace {

namespace app = motion_control_lab::single_arm_servo_step;
namespace mcc = motion_control::core;

constexpr const char *kProgramId = "mcl_single_arm_servo_step";

int run(int argc, char **argv) {
  const auto options = app::parseOptions(argc, argv);
  const auto &robot = motion_control_lab::r1RobotConfig();
  const auto controlled_side =
      motion_control_lab::parseArmSide(options.tui.side);
  const auto model = app::loadRobotModel(robot, options);

  mcc::KinematicsSolverBuilder builder;
  app::requireOk(builder.configure(model, robot.joint_names,
                                   app::makeSolverConfig(options)));
  app::SolverHandles handles;
  app::configureSolver(builder, handles, robot, controlled_side, options);

  mcc::KinematicsSolver solver;
  app::requireOk(builder.finalize(solver));
  return app::runLoop(options, robot, controlled_side, solver, handles);
}

} // namespace

int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << kProgramId << ": " << error.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << kProgramId << ": non-standard exception\n";
    return EXIT_FAILURE;
  }
}
