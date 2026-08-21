#include "components/tui/standard_ik_tui.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace mcl = motion_control_lab;

namespace
{

void require(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

bool contains(const mcl::TuiDocument & document, const std::string & needle)
{
  if (document.title.find(needle) != std::string::npos ||
      document.subtitle.find(needle) != std::string::npos ||
      document.status.find(needle) != std::string::npos)
  {
    return true;
  }
  for (const auto & page : document.pages) {
    if (page.title.find(needle) != std::string::npos) return true;
    for (const auto & section : page.sections) {
      if (section.title.find(needle) != std::string::npos) return true;
      for (const auto & row : section.rows) {
        if (row.label.find(needle) != std::string::npos ||
            row.value.find(needle) != std::string::npos)
        {
          return true;
        }
      }
      for (const auto & line : section.lines) {
        if (line.find(needle) != std::string::npos) return true;
      }
      for (const auto & table : section.tables) {
        for (const auto & column : table.columns) {
          if (column.title.find(needle) != std::string::npos) return true;
        }
        for (const auto & table_row : table.rows) {
          for (const auto & value : table_row) {
            if (value.find(needle) != std::string::npos) return true;
          }
        }
      }
    }
  }
  return false;
}

}  // namespace

int main()
{
  mcl::IkDebugFrame snapshot;
  mcl::SolverDebug solver;
  solver.label = "PlaCo production-static baseline";
  solver.native_status = "tasks: position(x2)=scaled/9";
  solver.run_counters = mcl::SolverRunCounters{12, 11, 1};
  solver.task_scales = {{"frame_position", true, 0.75, 0.0, true, false}};
  snapshot.solvers.push_back(std::move(solver));

  mcl::InteractiveIkPresentation presentation;
  presentation.base_frame_id = "base_link";
  const auto document = mcl::makeStandardIkTuiDocument(
      snapshot, presentation, 7, "null", "Baseline", "running");

  require(document.pages.size() == 5U, "baseline projection must produce five pages");
  require(contains(document, "PlaCo production-static baseline"),
          "baseline solver identity missing from TUI document");
  require(contains(document, "position(x2)=scaled/9"),
          "baseline task summary missing from TUI document");
  require(contains(document, "frame_position"),
          "baseline task scale missing from TUI document");
  require(contains(document, "Run counters"),
          "baseline counters missing from TUI document");
  return EXIT_SUCCESS;
}
