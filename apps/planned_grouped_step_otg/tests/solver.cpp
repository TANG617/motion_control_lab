#include "../solver.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace app = motion_control_lab::planned_grouped_step_otg;
namespace mcc = motion_control::core;

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

} // namespace

int main() {
  try {
    app::Options options;
    options.interactive.red_rate_hz = 800.0;
    options.interactive.yellow_rate_hz = 40.0;
    options.interactive.solver.regularization = 2.5e-9;
    options.interactive.solver.yellow_to_red_coupling_weight = 3.0;
    options.interactive.solver.red_proxqp_absolute_tolerance = 7.0e-6;
    options.interactive.solver.red_proxqp_primal_infeasibility_tolerance =
        9.0e-13;

    const auto config = app::makeSolverConfig(options);
    require(config.profile == mcc::GroupedSolverProfile::RedYellow,
            "wrong grouped profile");
    require(config.red.mode == mcc::IkSolveMode::ServoStep,
            "wrong Red solve mode");
    require(config.yellow.mode == mcc::IkSolveMode::ServoStep,
            "wrong Yellow solve mode");
    require(config.red.servo_period == 1.0 / 800.0, "wrong Red period");
    require(config.yellow.servo_period == 1.0 / 40.0, "wrong Yellow period");
    require(config.red.maximum_iterations == 1 &&
                config.yellow.maximum_iterations == 1,
            "ServoStep must retain one iteration");
    require(config.red.qp.backend == mcc::QpBackend::ProxQp,
            "wrong Red backend");
    require(config.yellow.qp.backend == mcc::QpBackend::ProxQp,
            "wrong Yellow backend");
    require(!config.red.qp.proxqp.warm_start_enabled,
            "Red warm start must remain disabled");
    require(config.red.qp.regularization == 2.5e-9,
            "regularization was not propagated");
    require(config.red.qp.proxqp.absolute_tolerance == 7.0e-6,
            "Red absolute tolerance was not propagated");
    require(config.red.qp.proxqp.primal_infeasibility_tolerance == 9.0e-13,
            "Red infeasibility tolerance was not propagated");

    const auto &robot = motion_control_lab::r1RobotConfig();
    const auto names = app::activeJointNames(robot);
    const auto indices = app::activeJointFullIndices(robot);
    require(names.size() == 18U && indices.size() == 18U,
            "wrong active joint count");
    for (std::size_t index = 0; index < indices.size(); ++index) {
      require(names[index] ==
                  robot.joint_names[static_cast<std::size_t>(indices[index])],
              "active joint mapping mismatch");
      require(indices[index] != 4 && indices[index] != 5,
              "waist fixed joints were not excluded");
    }
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << "planned grouped Step OTG solver test failed: " << error.what()
              << '\n';
    return EXIT_FAILURE;
  }
}
