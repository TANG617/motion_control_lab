#include "../loop.hpp"

#include <cstdlib>
#include <stdexcept>

namespace app = motion_control_lab::
    hierarchical_kinematics_step;

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

} // namespace

int main() {
  app::ReplayPipelineGate gate;
  require(gate.acquireRedTick() && gate.acquireYellowTick(),
          "running gate blocked a worker");

  gate.pause();
  require(gate.paused(), "pause did not latch");
  require(!gate.acquireRedTick() && !gate.acquireYellowTick(),
          "paused gate allowed an unbudgeted worker tick");

  gate.grantSingleFrame(3U, 2U);
  require(gate.acquireRedTick() && gate.acquireRedTick() &&
              gate.acquireRedTick() && !gate.acquireRedTick(),
          "Red single-frame budget changed");
  require(gate.acquireYellowTick() && gate.acquireYellowTick() &&
              !gate.acquireYellowTick(),
          "Yellow single-frame budget changed");
  require(gate.paused(), "single-frame budget resumed continuous execution");

  gate.grantSingleFrame(2U, 2U);
  require(gate.acquireRedTick(), "replacement budget was not granted");
  gate.pause();
  require(!gate.acquireRedTick() && !gate.acquireYellowTick(),
          "pause did not cancel a pending single-frame budget");

  gate.resume();
  require(!gate.paused(), "resume did not clear pause");
  require(gate.acquireRedTick() && gate.acquireYellowTick(),
          "resumed gate blocked a worker");
  return EXIT_SUCCESS;
}
