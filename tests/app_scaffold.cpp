#include "components/app_scaffold/app_scaffold.hpp"

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

namespace mcl = motion_control_lab;

namespace
{

struct Resource
{
  std::string name;
  std::vector<std::string> & events;
  bool fail_close{false};

  void start() { events.push_back("start:" + name); }
  void requestStop() { events.push_back("stop:" + name); }
  void close()
  {
    events.push_back("close:" + name);
    if (fail_close) throw std::runtime_error("first infrastructure error");
  }
  void join() { events.push_back("join:" + name); }
};

}  // namespace

int main()
{
  std::vector<std::string> events;
  Resource first{"first", events, false};
  Resource second{"second", events, true};
  auto lifecycle = mcl::makeRuntimeLifecycle(first, second);
  lifecycle.start();
  bool lifecycle_verified = false;
  try {
    lifecycle.finish();
  } catch (const std::runtime_error & error) {
    if (std::string{error.what()} != "first infrastructure error") return EXIT_FAILURE;
    const std::vector<std::string> expected{
      "start:first", "start:second", "stop:second", "stop:first",
      "close:second", "close:first", "join:second", "join:first"};
    lifecycle_verified = events == expected;
  }
  if (!lifecycle_verified) return EXIT_FAILURE;

  int input = 1;
  int scheduler = 2;
  int terminal = 3;
  int tui = 4;
  int viz = 5;
  int artifacts = 6;
  auto services = mcl::makeRuntimeServices(
      input, scheduler, terminal, tui, viz, artifacts);
  services.input = 11;
  services.artifact_writer = 16;
  return input == 11 && scheduler == 2 && terminal == 3 && tui == 4 &&
                 viz == 5 && artifacts == 16
    ? EXIT_SUCCESS
    : EXIT_FAILURE;
}
