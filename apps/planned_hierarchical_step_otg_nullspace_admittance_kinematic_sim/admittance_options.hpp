#pragma once

namespace motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim {

struct AdmittanceOptions {
  bool angular_enabled{false};
  double linear_environment_stiffness_n_per_m{200.0};
  double linear_environment_damping_ns_per_m{20.0};
  double maximum_force_n{20.0};
  double angular_environment_stiffness_nm_per_rad{10.0};
  double angular_environment_damping_nms_per_rad{1.0};
  double maximum_torque_nm{5.0};
  double wrench_filter_alpha{0.08};
};

} // namespace motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim
