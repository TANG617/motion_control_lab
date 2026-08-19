#pragma once

#include <string>

namespace motion_control_lab {

struct TuiTeleopOptions {
  std::string side;
  double step_m{};
  double min_step_m{};
  double max_step_m{};
  double rotation_step_deg{};
};

} // namespace motion_control_lab
