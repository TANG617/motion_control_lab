#include "../solver.hpp"

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
    std::cerr << "planned hierarchical Step OTG solver test failed: " << error.what()
              << '\n';
    return EXIT_FAILURE;
  }
}
