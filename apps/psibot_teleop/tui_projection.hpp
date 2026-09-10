#pragma once
#include "loop.hpp"
#include "contracts/presentation/tui_document.hpp"

namespace motion_control_lab::psibot_teleop {
TuiDocument makeDocument(const Options &, const Snapshot &, const InputController &, const std::string & log_dir);
std::string plainStatus(const Snapshot &);
}  // namespace motion_control_lab::psibot_teleop
