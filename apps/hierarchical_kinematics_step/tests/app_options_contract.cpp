#include "../options.hpp"

#include <cstdlib>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace app = motion_control_lab::hierarchical_kinematics_step;

namespace {

app::Options parse(std::initializer_list<const char *> arguments) {
  std::vector<std::string> storage;
  for (const char *argument : arguments)
    storage.emplace_back(argument);
  std::vector<char *> argv;
  for (auto &argument : storage)
    argv.push_back(argument.data());
  return app::parseOptions(static_cast<int>(argv.size()), argv.data());
}

bool rejects(const std::function<void()> &call) {
  try {
    call();
    return false;
  } catch (const std::runtime_error &) {
    return true;
  }
}

} // namespace

int main() {
  const auto hierarchical = app::profileDefaults(app::Profile::Hierarchical);
  const auto planned = app::profileDefaults(app::Profile::Planned);
  const auto otg = app::profileDefaults(app::Profile::PlannedOtg);
  const auto nullspace = app::profileDefaults(app::Profile::PlannedOtgNullspace);
  const auto maximum = app::profileDefaults(
      app::Profile::PlannedOtgNullspaceAdmittanceKinematicSim);

  if (hierarchical.interactive.red_rate_hz != 1000.0 ||
      hierarchical.interactive.yellow_rate_hz != 100.0 ||
      hierarchical.interactive.solver.regularization != 1.0e-4 ||
      hierarchical.interactive.solver.legacy_cartesian_progress_weight != 3.0 ||
      hierarchical.interactive.solver
              .joint_position_braking_velocity_envelope_enabled ||
      hierarchical.interactive.solver.red_joint_acceleration_limits_enabled ||
      hierarchical.interactive.robot.inactive_joint_names.size() != 4U ||
      planned.interactive.red_rate_hz != 100.0 ||
      planned.interactive.yellow_rate_hz != 20.0 ||
      !planned.interactive.solver
           .joint_position_braking_velocity_envelope_enabled ||
      !planned.interactive.solver.red_joint_acceleration_limits_enabled ||
      planned.interactive.robot.inactive_joint_names.size() != 2U ||
      planned.interactive.robot.joint_stream.max_acceleration_rad_per_s2[10] !=
          16.2 ||
      otg.interactive.red_rate_hz != 1000.0 ||
      otg.interactive.yellow_rate_hz != 100.0 ||
      otg.interactive.robot.joint_stream.max_acceleration_rad_per_s2[10] !=
          24.3 ||
      otg.interactive.robot.joint_stream.position_upper_rad[11] != 0.9599 ||
      !nullspace.interactive.robot.inactive_joint_names.empty() ||
      nullspace.interactive.robot.joint_stream.position_lower_rad[18] !=
          -0.9599 ||
      !nullspace.interactive.solver.red_joint_acceleration_limits_enabled ||
      maximum.interactive.solver
          .joint_position_braking_velocity_envelope_enabled ||
      maximum.interactive.solver.red_joint_acceleration_limits_enabled ||
      maximum.interactive.robot.profile_provenance !=
          "planned-otg-nullspace-admittance-kinematic-sim") {
    return EXIT_FAILURE;
  }

  const auto hierarchical_capabilities =
      app::profileCapabilities(app::Profile::Hierarchical);
  const auto planned_capabilities =
      app::profileCapabilities(app::Profile::Planned);
  const auto otg_capabilities =
      app::profileCapabilities(app::Profile::PlannedOtg);
  const auto nullspace_capabilities =
      app::profileCapabilities(app::Profile::PlannedOtgNullspace);
  const auto maximum_capabilities = app::profileCapabilities(
      app::Profile::PlannedOtgNullspaceAdmittanceKinematicSim);
  if (hierarchical_capabilities.cartesian_planning ||
      hierarchical_capabilities.joint_otg ||
      !planned_capabilities.cartesian_planning ||
      planned_capabilities.joint_otg || !otg_capabilities.joint_otg ||
      otg_capabilities.nullspace || !nullspace_capabilities.nullspace ||
      nullspace_capabilities.admittance || !maximum_capabilities.admittance ||
      !maximum_capabilities.kinematic_simulation ||
      !maximum_capabilities.telemetry) {
    return EXIT_FAILURE;
  }

  if (!rejects([] {
        (void)parse({"app", "teleop", "--urdf", "/tmp/r1.urdf"});
      }) ||
      !rejects([] {
        (void)parse({"app", "--profile", "hierarchical", "teleop", "--urdf",
                     "/tmp/r1.urdf", "--max-linear-velocity-mps", "1"});
      }) ||
      !rejects([] {
        (void)parse({"app", "--profile", "planned", "teleop", "--urdf",
                     "/tmp/r1.urdf", "--joint-target-mode", "ik-pv"});
      }) ||
      !rejects([] {
        (void)parse({"app", "--profile", "planned-otg", "teleop", "--urdf",
                     "/tmp/r1.urdf",
                     "--red-tertiary-task-link4-position-weight", "5"});
      }) ||
      !rejects([] {
        (void)parse({"app", "--profile", "planned-otg-nullspace", "teleop",
                     "--urdf", "/tmp/r1.urdf", "--mujoco-model",
                     "/tmp/r1.xml"});
      })) {
    return EXIT_FAILURE;
  }

  const auto custom = parse(
      {"app", "--profile",
       "planned-otg-nullspace-admittance-kinematic-sim", "teleop", "--urdf",
       "/tmp/r1.urdf", "--mujoco-model", "/tmp/r1.xml",
       "--joint-position-braking-velocity-envelope",
       "--red-joint-acceleration-limits", "--left-tcp-offset",
       "0.1,0.2,0.3,0,0,0,1", "--inactive-joints", "knee_pitch_joint",
       "--self-collision-pair", "left_arm_link4:body_link4",
       "--yellow-task-posture-preference-joint-weight-multipliers",
       "left_arm_joint6=2.5", "--red-rate", "800", "--yellow-rate", "80"});
  if (!custom.interactive.solver
           .joint_position_braking_velocity_envelope_enabled ||
      !custom.interactive.solver.red_joint_acceleration_limits_enabled ||
      custom.interactive.robot.left_tcp_offset.translation().x() != 0.1 ||
      custom.interactive.robot.inactive_joint_names.size() != 1U ||
      custom.interactive.robot.self_collision_link_pairs.size() != 1U ||
      custom.interactive.solver
              .yellow_task_posture_preference_joint_weight_multipliers.size() !=
          1U) {
    return EXIT_FAILURE;
  }

  const auto json = app::resolvedOptionsJson(custom);
  return json.find(
             "\"profile\" : \"planned-otg-nullspace-admittance-kinematic-sim\"") !=
                 std::string::npos &&
                 json.find("\"profile_provenance\"") != std::string::npos &&
                 json.find("\"binary_argv\"") != std::string::npos
             ? EXIT_SUCCESS
             : EXIT_FAILURE;
}
