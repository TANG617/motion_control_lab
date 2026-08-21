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

}  // namespace motion_control_lab
