#pragma once
#include "contracts/presentation/tui_document.hpp"
#include "loop.hpp"
namespace motion_control_lab::joint_path_planning {
TuiDocument makeDocument(const Snapshot &, const mcc::RobotModel &,
                         const Planning &);
}
