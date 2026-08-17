#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "runtime/grouped_worker.hpp"

namespace motion_control_lab
{

enum class UiMode
{
  Tui,
  None,
};

struct TuiTeleopOptions
{
  std::string side{"left"};
  double step_m{0.005};
  double min_step_m{0.001};
  double max_step_m{0.5};
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
  double rate_hz{100.0};
  double duration_s{0.0};
  UiMode ui{UiMode::Tui};
  TuiTeleopOptions tui;
  VisualizationSinkOptions visualization;
};

struct GroupedInteractiveIkOptions
{
  std::string urdf_path;
  double red_rate_hz{1000.0};
  double yellow_rate_hz{100.0};
  double ui_rate_hz{100.0};
  DeadlinePolicy deadline_policy{DeadlinePolicy::Strict};
  double duration_s{0.0};
  UiMode ui{UiMode::Tui};
  TuiTeleopOptions tui;
  VisualizationSinkOptions visualization;
};

void printInteractiveIkUsage(const char * program);

InteractiveIkOptions parseInteractiveIkOptions(int argc, char ** argv);

void printGroupedInteractiveIkUsage(const char * program);

GroupedInteractiveIkOptions parseGroupedInteractiveIkOptions(int argc, char ** argv);

const char * uiModeName(UiMode mode);

}  // namespace motion_control_lab
