#pragma once

#include <motion_control_core/motion_control_core.hpp>
#include <motion_control_sim/types.hpp>

#include <array>

#include "options.hpp"

namespace motion_control_lab::hierarchical_kinematics_step {

namespace mcc = motion_control::core;
namespace mcs = motion_control::sim;

struct ArmAdmittanceOutput {
  mcc::CartesianFrameSample nominal_link;
  mcc::CartesianFrameSample nominal_tcp;
  mcc::CartesianFrameSample command_tcp;
  mcc::CartesianFrameSample command_link;
  mcc::Pose actual_tcp_pose{mcc::Pose::Identity()};
  mcc::Twist actual_tcp_twist{mcc::Twist::Zero()};
  mcc::Wrench raw_wrench;
  mcc::Wrench filtered_wrench;
  mcc::CartesianAdmittanceDiagnostics diagnostics;
  bool drag_active{false};
};

struct DualAdmittanceOutput {
  ArmAdmittanceOutput left;
  ArmAdmittanceOutput right;
  mcc::CartesianTrajectorySample command_sample;
};

mcc::CartesianFrameSample shiftCartesianControlPoint(
    const mcc::CartesianFrameSample &source,
    const mcc::Pose &source_to_target,
    const mcc::FrameName &target_frame_name);

class DualArmAdmittance {
public:
  void configure(const AdmittanceOptions &options);
  void reset();

  DualAdmittanceOutput
  step(const mcc::CartesianTrajectorySample &nominal_link_sample,
       const mcc::Pose &left_actual_link_pose,
       const mcc::Pose &right_actual_link_pose,
       const mcc::Pose &left_tcp_offset, const mcc::Pose &right_tcp_offset,
       const std::array<mcs::DragReference, 2> &drag_references,
       double dt_seconds);

private:
  struct ArmState {
    mcc::CartesianAdmittance admittance;
    mcc::Wrench filtered_wrench;
    mcc::Pose previous_actual_tcp_pose{mcc::Pose::Identity()};
    bool has_previous_actual_pose{false};
  };

  ArmAdmittanceOutput
  stepArm(ArmState &state, const mcc::CartesianFrameSample &nominal_link,
          const mcc::Pose &actual_link_pose, const mcc::Pose &tcp_offset,
          const mcs::DragReference &drag, double dt_seconds);

  std::array<double, 7> scalar_options_{};
  bool angular_enabled_{false};
  ArmState left_;
  ArmState right_;
};

} // namespace motion_control_lab::hierarchical_kinematics_step
