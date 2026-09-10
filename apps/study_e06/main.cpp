// Independent experiment composition root.
// Parsing and evidence orchestration remain local to this app.
// Solver topology and native API calls are owned by Solver.
// No app-private implementation is imported from another app.
#include "loop.hpp"

int main(int argc, char **argv) {
  // The request is already resolved by the outer experiment orchestrator.
  const auto options = study::parse(argc, argv);

  // Construction failures retain their native exception and process failure.
  study::Solver solver(options);

  return study::loop(options, solver);
}
