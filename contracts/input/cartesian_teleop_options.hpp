#pragma once

#include <string>

namespace motion_control_lab
{

struct CartesianTeleopOptions
{
  std::string side{"left"};
  double step_m{0.005};
  double min_step_m{0.001};
  double max_step_m{0.5};
  double rotation_step_deg{5.0};
};

}  // namespace motion_control_lab
