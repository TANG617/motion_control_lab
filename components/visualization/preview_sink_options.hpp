#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace motion_control_lab
{

struct PreviewSinkOptions
{
  bool enabled{true};
  std::string host;
  std::uint16_t port{};
  std::optional<std::filesystem::path> mcap_path;
};

}  // namespace motion_control_lab
