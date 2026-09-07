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
  for (const char *obsolete : {"--yellow-maximum-iterations",
                               "--minimum-position-improvement-m",
                               "--minimum-orientation-improvement-rad"}) {
    if (!rejects([&] {
          (void)parse({"app", "--profile", "planned", "teleop", obsolete, "1"});
        })) {
      return EXIT_FAILURE;
    }
  }
  for (const char *source : {"teleop", "replay"}) {
    std::vector<std::string> args{
        "app",
        "--profile",
        "planned",
        source,
        "--urdf",
        "/tmp/r1.urdf",
        "--red-secondary-task-tcp-orientation-weight",
        "75",
        "--red-secondary-task-tcp-orientation-servo-gain-per-s",
        "12",
        "--red-secondary-task-tcp-orientation-residual-normalization-radps",
        "0.5"};
    if (std::string{source} == "replay") {
      args.insert(args.end(),
                  {"--input", "/tmp/input.mcap", "--left-stream", "/left",
                   "--right-stream", "/right", "--target-period-ms", "10"});
    }
    std::vector<char *> argv;
    for (auto &arg : args)
      argv.push_back(arg.data());
    const auto parsed = app::parseOptions(argv.size(), argv.data());
    const auto &solver = parsed.interactive.solver;
    if (solver.red_secondary_task_tcp_orientation_weight != 75.0 ||
        solver.red_secondary_task_tcp_orientation_servo_gain_per_s != 12.0 ||
        solver.red_secondary_task_tcp_orientation_residual_normalization_radps !=
            0.5 ||
        app::resolvedOptionsJson(parsed).find(
            "red_secondary_task_tcp_orientation_weight") == std::string::npos) {
      return EXIT_FAILURE;
    }
  }
  constexpr const char *kDefaultUrdf =
      "/workspace/models/Psi_R1_visual_collision.urdf";
  const auto hierarchical = app::profileDefaults(app::Profile::Hierarchical);
  const auto planned = app::profileDefaults(app::Profile::Planned);
  const auto otg = app::profileDefaults(app::Profile::PlannedOtg);
  const auto nullspace =
      app::profileDefaults(app::Profile::PlannedOtgNullspace);
  const auto maximum = app::profileDefaults(
      app::Profile::PlannedOtgNullspaceAdmittanceKinematicSim);

  if (hierarchical.interactive.urdf_path != kDefaultUrdf ||
      planned.interactive.urdf_path != kDefaultUrdf ||
      otg.interactive.urdf_path != kDefaultUrdf ||
      nullspace.interactive.urdf_path != kDefaultUrdf ||
      maximum.interactive.urdf_path != kDefaultUrdf ||
      hierarchical.interactive.red_rate_hz != 1000.0 ||
      hierarchical.interactive.yellow_rate_hz != 100.0 ||
      hierarchical.interactive.solver.red_qp_regularization != 1.0e-8 ||
      hierarchical.interactive.solver.yellow_qp_regularization != 1.0e-4 ||
      hierarchical.interactive.solver.red_proxqp_maximum_iterations != 1000 ||
      !hierarchical.interactive.solver
           .red_accept_feasible_primary_maximum_iterations ||
      hierarchical.interactive.solver.legacy_cartesian_progress_weight != 3.0 ||
      hierarchical.interactive.solver
              .joint_position_braking_velocity_envelope_enabled ||
      hierarchical.interactive.solver.red_joint_acceleration_limits_enabled ||
      hierarchical.interactive.robot.inactive_joint_names.size() != 4U ||
      planned.interactive.red_rate_hz != 100.0 ||
      planned.interactive.yellow_rate_hz != 20.0 ||
      planned.interactive.solver.red_proxqp_maximum_iterations != 1000 ||
      !planned.interactive.solver
           .joint_position_braking_velocity_envelope_enabled ||
      !planned.interactive.solver.red_joint_acceleration_limits_enabled ||
      planned.interactive.robot.inactive_joint_names.size() != 2U ||
      planned.interactive.robot.joint_stream.max_acceleration_rad_per_s2[10] !=
          16.2 ||
      otg.interactive.red_rate_hz != 1000.0 ||
      otg.interactive.yellow_rate_hz != 100.0 ||
      otg.interactive.solver.red_proxqp_maximum_iterations != 1000 ||
      otg.interactive.robot.joint_stream.max_acceleration_rad_per_s2[10] !=
          24.3 ||
      otg.interactive.robot.joint_stream.position_upper_rad[11] != 0.9599 ||
      !nullspace.interactive.robot.inactive_joint_names.empty() ||
      nullspace.interactive.robot.joint_stream.position_lower_rad[18] !=
          -0.9599 ||
      nullspace.interactive.solver.red_proxqp_maximum_iterations != 200 ||
      !nullspace.interactive.solver.red_joint_acceleration_limits_enabled ||
      maximum.interactive.solver
          .joint_position_braking_velocity_envelope_enabled ||
      maximum.interactive.solver.red_joint_acceleration_limits_enabled ||
      maximum.interactive.solver.red_proxqp_maximum_iterations != 200 ||
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

  if (!rejects(
          [] { (void)parse({"app", "teleop", "--urdf", "/tmp/r1.urdf"}); }) ||
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
                     "--red-secondary-task-link4-position-weight", "5"});
      }) ||
      !rejects([] {
        (void)parse({"app", "--profile", "planned-otg-nullspace", "teleop",
                     "--urdf", "/tmp/r1.urdf", "--mujoco-model",
                     "/tmp/r1.xml"});
      }) ||
      !rejects([] {
        (void)parse({"app", "--profile", "planned-otg", "teleop",
                     "--regularization", "1e-8"});
      }) ||
      !rejects([] {
        (void)parse({"app", "--profile", "planned-otg", "teleop",
                     "--yellow-maximum-iterations", "50"});
      })) {
    return EXIT_FAILURE;
  }

  const auto default_planned =
      parse({"app", "--profile", "planned", "teleop"});
  if (default_planned.interactive.urdf_path != kDefaultUrdf) {
    return EXIT_FAILURE;
  }

  const auto custom =
      parse({"app",
             "--profile",
             "planned-otg-nullspace-admittance-kinematic-sim",
             "teleop",
             "--urdf",
             "/tmp/r1.urdf",
             "--mujoco-model",
             "/tmp/r1.xml",
             "--collision-mesh-search-paths",
             "/tmp/meshes",
       "--joint-position-braking-velocity-envelope",
             "--red-joint-acceleration-limits",
             "--red-qp-regularization",
             "3e-8",
             "--red-proxqp-maximum-iterations",
             "321",
             "--red-proxqp-absolute-tolerance",
             "4e-6",
             "--red-proxqp-relative-tolerance",
             "5e-5",
             "--red-proxqp-primal-infeasibility-tolerance",
             "6e-12",
             "--red-proxqp-warm-start",
             "--yellow-qp-regularization",
             "7e-8",
             "--yellow-proxqp-maximum-iterations",
             "654",
             "--yellow-proxqp-absolute-tolerance",
             "8e-7",
             "--yellow-proxqp-relative-tolerance",
             "9e-6",
             "--yellow-proxqp-primal-infeasibility-tolerance",
             "1e-9",
             "--no-yellow-proxqp-warm-start",
             "--no-red-accept-feasible-primary-max-iterations",
             "--left-tcp-offset",
             "0.1,0.2,0.3,0,0,0,1",
             "--inactive-joints",
             "knee_pitch_joint",
             "--self-collision-pair",
             "left_arm_link4:body_link4",
       "--yellow-task-posture-preference-joint-weight-multipliers",
             "left_arm_joint6=2.5",
             "--red-rate",
             "800",
             "--yellow-rate",
             "80",
             "--cartesian-maximum-sample-count",
             "1234",
             "--joint-maximum-sample-count",
             "5678"});
  if (custom.interactive.robot.collision_mesh_search_paths !=
          std::vector<std::string>{"/tmp/meshes"} ||
      !custom.interactive.solver
           .joint_position_braking_velocity_envelope_enabled ||
      !custom.interactive.solver.red_joint_acceleration_limits_enabled ||
      custom.interactive.solver.red_qp_regularization != 3.0e-8 ||
      custom.interactive.solver.red_proxqp_maximum_iterations != 321 ||
      custom.interactive.solver.red_proxqp_absolute_tolerance != 4.0e-6 ||
      custom.interactive.solver.red_proxqp_relative_tolerance != 5.0e-5 ||
      custom.interactive.solver.red_proxqp_primal_infeasibility_tolerance !=
          6.0e-12 ||
      !custom.interactive.solver.red_proxqp_warm_start_enabled ||
      custom.interactive.solver.yellow_qp_regularization != 7.0e-8 ||
      custom.interactive.solver.yellow_proxqp_maximum_iterations != 654 ||
      custom.interactive.solver.yellow_proxqp_absolute_tolerance != 8.0e-7 ||
      custom.interactive.solver.yellow_proxqp_relative_tolerance != 9.0e-6 ||
      custom.interactive.solver
              .yellow_proxqp_primal_infeasibility_tolerance != 1.0e-9 ||
      custom.interactive.solver.yellow_proxqp_warm_start_enabled ||
      custom.interactive.solver
          .red_accept_feasible_primary_maximum_iterations ||
      custom.planning.cartesian_maximum_sample_count != 1234U ||
      custom.planning.joint_maximum_sample_count != 5678U ||
      custom.interactive.robot.left_tcp_offset.translation().x() != 0.1 ||
      custom.interactive.robot.inactive_joint_names.size() != 1U ||
      custom.interactive.robot.self_collision_link_pairs.size() != 1U ||
      custom.interactive.solver
              .yellow_task_posture_preference_joint_weight_multipliers.size() !=
          1U) {
    return EXIT_FAILURE;
  }

  const auto reject_alias =
      parse({"app", "--profile", "hierarchical", "teleop",
             "--red-reject-primary-max-iterations"});
  if (reject_alias.interactive.solver
          .red_accept_feasible_primary_maximum_iterations) {
    return EXIT_FAILURE;
  }

  const auto json = app::resolvedOptionsJson(custom);
  return json.find("\"schema_version\" : "
                   "\"mcl.hierarchical_kinematics_step.options.v2\"") !=
                 std::string::npos &&
                 json.find("\"profile\" : "
                   "\"planned-otg-nullspace-admittance-kinematic-sim\"") !=
                 std::string::npos &&
                 json.find("\"profile_provenance\"") != std::string::npos &&
                 json.find(
                     "\"red_accept_feasible_primary_maximum_iterations\" : "
                     "false") != std::string::npos &&
                 json.find("\"red_proxqp_maximum_iterations\" : 321") !=
                     std::string::npos &&
                 json.find("\"yellow_proxqp_maximum_iterations\" : 654") !=
                     std::string::npos &&
                 json.find("\"cartesian_maximum_sample_count\" : 1234") !=
                     std::string::npos &&
                 json.find("\"joint_maximum_sample_count\" : 5678") !=
                     std::string::npos &&
                 json.find("\"binary_argv\"") != std::string::npos
             ? EXIT_SUCCESS
             : EXIT_FAILURE;
}
