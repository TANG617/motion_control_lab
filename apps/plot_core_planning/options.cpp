#include "options.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace motion_control_lab::plot_core_planning {

void printUsage(const char *program) {
  std::cout << "Usage:\n"
            << "  " << program << "\n"
            << "  " << program << " --output-dir <path>\n";
}

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument{argv[index]};
    auto requireValue = [&](const std::string &option) -> std::string {
      if (index + 1 >= argc) {
        throw std::runtime_error(option + " requires a value");
      }
      return argv[++index];
    };

    if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else if (argument == "--output-dir") {
      options.output_dir = std::filesystem::path{requireValue(argument)};
    } else {
      throw std::runtime_error("unknown option: " + argument);
    }
  }
  return options;
}

} // namespace motion_control_lab::plot_core_planning
