#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace motion_control_lab::cartesian_planning {

struct AppOptions {
  std::filesystem::path request_path;
  std::filesystem::path output_dir;
  std::string host{"127.0.0.1"};
  std::uint16_t port{8765};
  double playback_rate{1.0};
  double loop_delay_s{1.0};
  bool once{false};
  bool live{true};
  bool force{false};
  std::optional<std::filesystem::path> mcap_path;
};

void printUsage(const char *program);
AppOptions parseOptions(int argc, char **argv);

} // namespace motion_control_lab::cartesian_planning
