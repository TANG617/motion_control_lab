#include "../solver.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace app = motion_control_lab::planned_hierarchical_step_otg;

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

} // namespace

int main() {
  try {
    const auto &robot = motion_control_lab::r1RobotConfig();
    const app::RobotOptions options;
    const auto names = app::activeJointNames(robot, options);
    const auto indices = app::activeJointFullIndices(robot, options);
    require(names.size() == 18U && indices.size() == 18U,
            "wrong active joint count");
    for (std::size_t index = 0; index < indices.size(); ++index) {
      require(names[index] ==
                  robot.joint_names[static_cast<std::size_t>(indices[index])],
              "active joint mapping mismatch");
      require(indices[index] != 4 && indices[index] != 5,
              "waist fixed joints were not excluded");
    }

    auto custom_options = options;
    custom_options.inactive_joint_names = {"head_yaw_joint"};
    const auto custom_names = app::activeJointNames(robot, custom_options);
    const auto custom_indices =
        app::activeJointFullIndices(robot, custom_options);
    require(custom_names.size() == 19U && custom_indices.size() == 19U,
            "custom inactive joint policy was not applied");
    require(custom_indices.front() == 1U,
            "custom inactive joint mapping mismatch");
    require(std::find(custom_indices.begin(), custom_indices.end(), 4U) !=
                custom_indices.end() &&
                std::find(custom_indices.begin(), custom_indices.end(), 5U) !=
                    custom_indices.end(),
            "custom inactive joint policy retained hidden waist behavior");
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << "planned hierarchical Step OTG solver test failed: " << error.what()
              << '\n';
    return EXIT_FAILURE;
  }
}
