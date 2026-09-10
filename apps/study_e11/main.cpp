#include "loop.hpp"
#include "options.hpp"
#include "planning.hpp"
#include "solver.hpp"

// The app owns solver topology and both planner instances.
// The loop advances an explicit ideal kinematic state, never a hardware plant.
int main(int argc, char **argv) {
  // Resolve the frozen per-unit request before constructing algorithms.
  auto options = study_e11::parse(argc, argv);
  study_e11::Solver solver(options);
  study_e11::Planning planning(options);

  // Native failures and partial evidence propagate to the process boundary.
  return study_e11::loop(options, solver, planning);
}
