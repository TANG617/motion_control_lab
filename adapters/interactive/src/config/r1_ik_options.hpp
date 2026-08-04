#pragma once

#include "config/constants.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace motion_control_lab
{

struct R1IkOptions
{
  std::string side{"left"};
  std::string host{"127.0.0.1"};
  uint16_t port{8765};
  double rate_hz{20.0};
  double duration_s{0.0};
  double step_m{0.01};
  double min_step_m{0.001};
  double max_step_m{0.1};
  double rotation_step_deg{5.0};
  std::string urdf_path;
  std::optional<std::filesystem::path> mcap_path;
};

void printR1IkUsage(const char * program);

R1IkOptions parseR1IkOptions(int argc, char ** argv);

}  // namespace motion_control_lab
