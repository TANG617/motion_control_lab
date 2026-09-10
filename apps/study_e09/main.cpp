#include "loop.hpp"
#include "options.hpp"
#include "solver.hpp"

// Each study executable owns its configuration and solver composition.
// The external orchestrator owns process isolation, never numerical policy.
int main(int argc, char **argv) {
  auto options = study_e09::parseOptions(argc, argv);

  // Model and fixed topology construction are outside the timing windows.
  study_e09::Solver solver(options);

  // The app loop owns snapshot or evolving-state semantics and raw evidence.
  return study_e09::run(options, solver);
}
