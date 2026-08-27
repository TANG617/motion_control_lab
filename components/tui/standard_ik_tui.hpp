#pragma once

#include "contracts/presentation/ik_app_snapshot.hpp"
#include "contracts/presentation/tui_document.hpp"

#include <cstddef>
#include <string>

namespace motion_control_lab
{

TuiDocument makeStandardIkTuiDocument(
  const IkDebugFrame & frame,
  const InteractiveIkPresentation & presentation,
  std::size_t publish_count,
  const std::string & sink_status,
  const std::string & title,
  const std::string & input_status);

// Applies the shared 160x72 capability-page geometry to app-projected panels.
// Apps own the meaning of each panel; the shared TUI owns placement and chrome.
TuiPage makeStandardCapabilityPage(
  std::string title,
  std::vector<TuiSection> panels);

}  // namespace motion_control_lab
