#pragma once

namespace motion_control_lab::planned_grouped_step_otg {

enum class JointTargetMode {
  FutureO1Pv,
  IkPv,
};

const char *jointTargetModeName(JointTargetMode mode);

} // namespace motion_control_lab::planned_grouped_step_otg
