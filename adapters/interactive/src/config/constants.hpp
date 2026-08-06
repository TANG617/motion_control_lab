#pragma once

#include <array>

namespace motion_control_lab
{

inline constexpr const char * kUrdfEnvironmentVariable = "MOTION_CONTROL_URDF";
inline constexpr double kStepScale = 2.0;
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr std::array<const char *, 3> kRotationAxes{"x", "y", "z"};

}  // namespace motion_control_lab
