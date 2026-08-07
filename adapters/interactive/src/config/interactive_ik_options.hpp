#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "runtime/grouped_worker.hpp"

namespace motion_control_lab
{

struct TuiTeleopOptions
{
  std::string side{"left"};
  double step_m{0.01};
  double min_step_m{0.001};
  double max_step_m{0.1};
  double rotation_step_deg{5.0};
};

struct VisualizationSinkOptions
{
  std::string host{"127.0.0.1"};
  uint16_t port{8765};
  std::optional<std::filesystem::path> mcap_path;
};

struct InteractiveIkOptions
{
  std::string urdf_path;
  double rate_hz{20.0};
  double duration_s{0.0};
  TuiTeleopOptions tui;
  VisualizationSinkOptions visualization;
};

struct GroupedInteractiveIkOptions
{
  std::string urdf_path;
  double red_rate_hz{100.0};
  double yellow_rate_hz{50.0};
  double green_rate_hz{10.0};
  double ui_rate_hz{20.0};
  DeadlinePolicy deadline_policy{DeadlinePolicy::Strict};
  double duration_s{0.0};
  TuiTeleopOptions tui;
  VisualizationSinkOptions visualization;
};

void printInteractiveIkUsage(const char * program);

InteractiveIkOptions parseInteractiveIkOptions(int argc, char ** argv);

void printGroupedInteractiveIkUsage(const char * program);

GroupedInteractiveIkOptions parseGroupedInteractiveIkOptions(int argc, char ** argv);

}  // namespace motion_control_lab
