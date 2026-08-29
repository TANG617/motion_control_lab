#include <motion_control_core/motion_control_core.hpp>

#include "components/robot/r1/r1_robot_config.hpp"
#include "loop.hpp"
#include "options.hpp"
#include "solver.hpp"

int main(int argc, char **argv) {
  namespace app = motion_control_lab::hierarchical_inverse_dynamics_torque_sim;
  const auto options = app::parseOptions(argc, argv);
  const auto &robot = motion_control_lab::r1RobotConfig();
  app::Handles handles;
  auto runtime = app::configureSolver(robot, options, handles);
  return app::runLoop(options, robot, runtime, handles);
}
