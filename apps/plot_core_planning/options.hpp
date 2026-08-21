#pragma once

#include <filesystem>

namespace motion_control_lab::plot_core_planning {

struct Options {
  std::filesystem::path output_dir{"."};
};

void printUsage(const char *program);
Options parseOptions(int argc, char **argv);

} // namespace motion_control_lab::plot_core_planning
