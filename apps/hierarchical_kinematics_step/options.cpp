#include "options.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <initializer_list>
#include <iostream>
#include <json/json.h>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "components/tui/tui_help.hpp"

namespace motion_control_lab::hierarchical_kinematics_step {
namespace {

#ifndef MCL_R1_MUJOCO_MODEL_PATH
#define MCL_R1_MUJOCO_MODEL_PATH ""
#endif

#ifndef MCL_R1_MUJOCO_URDF_PATH
#define MCL_R1_MUJOCO_URDF_PATH ""
#endif

std::string requireValue(int &index, int argc, char **argv,
                         const std::string &option) {
  if (index + 1 >= argc) {
    throw std::runtime_error(option + " requires a value");
  }
  return argv[++index];
}

double parsePositiveDouble(const std::string &name, const std::string &value) {
  const double parsed = std::stod(value);
  if (parsed <= 0.0 || !std::isfinite(parsed)) {
    throw std::runtime_error(name + " must be a positive finite value");
  }
  return parsed;
}

double parseNonnegativeDouble(const std::string &name,
                              const std::string &value) {
  const double parsed = std::stod(value);
  if (parsed < 0.0 || !std::isfinite(parsed)) {
    throw std::runtime_error(name + " must be a non-negative finite value");
  }
  return parsed;
}

template <typename Value, typename Parser>
std::vector<Value> parseCsv(const std::string &argument,
                            const std::string &value, Parser parser) {
  std::vector<Value> result;
  std::istringstream input(value);
  std::string entry;
  while (std::getline(input, entry, ',')) {
    if (entry.empty())
      throw std::runtime_error(argument + " contains an empty CSV value");
    result.push_back(parser(entry));
  }
  if (result.empty())
    throw std::runtime_error(argument + " requires a non-empty CSV value");
  return result;
}

std::vector<std::string> parseStrings(const std::string &argument,
                                      const std::string &value) {
  return parseCsv<std::string>(argument, value,
                               [](const std::string &entry) { return entry; });
}

std::vector<double> parseNumbers(const std::string &argument,
                                 const std::string &value) {
  return parseCsv<double>(argument, value, [&](const std::string &entry) {
    const double parsed = std::stod(entry);
    if (!std::isfinite(parsed))
      throw std::runtime_error(argument + " values must be finite");
    return parsed;
  });
}

std::vector<std::size_t> parseIndices(const std::string &argument,
                                      const std::string &value) {
  return parseCsv<std::size_t>(argument, value, [&](const std::string &entry) {
    const auto parsed = std::stoull(entry);
    if (parsed > std::numeric_limits<std::size_t>::max())
      throw std::runtime_error(argument + " index is out of range");
    return static_cast<std::size_t>(parsed);
  });
}

template <typename Value, std::size_t Size>
void assignFixedArray(const std::string &argument,
                      const std::vector<Value> &values,
                      std::array<Value, Size> &destination) {
  if (values.size() != Size)
    throw std::runtime_error(argument + " requires exactly " +
                             std::to_string(Size) + " CSV values");
  std::copy(values.begin(), values.end(), destination.begin());
}

Eigen::Isometry3d parseTransform(const std::string &argument,
                                 const std::string &value) {
  const auto values = parseNumbers(argument, value);
  if (values.size() != 7U)
    throw std::runtime_error(
        argument + " requires x,y,z,qx,qy,qz,qw");
  Eigen::Quaterniond rotation{values[6], values[3], values[4], values[5]};
  if (!(rotation.norm() > 0.0) || !std::isfinite(rotation.norm()))
    throw std::runtime_error(argument + " quaternion must be finite/non-zero");
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d{values[0], values[1], values[2]};
  result.linear() = rotation.normalized().toRotationMatrix();
  return result;
}

CollisionLinkPairOptions parseCollisionPair(const std::string &argument,
                                            const std::string &value) {
  const auto separator = value.find(':');
  if (separator == std::string::npos || separator == 0U ||
      separator + 1U == value.size())
    throw std::runtime_error(argument + " requires FIRST:SECOND");
  return {value.substr(0U, separator), value.substr(separator + 1U)};
}

std::vector<std::pair<std::string, double>> parseJointWeightMultipliers(
    const std::string &argument, const std::string &value) {
  std::vector<std::pair<std::string, double>> result;
  std::istringstream input(value);
  std::string entry;
  while (std::getline(input, entry, ',')) {
    const auto separator = entry.find('=');
    if (separator == std::string::npos || separator == 0U ||
        separator + 1U == entry.size()) {
      throw std::runtime_error(
          argument + " entries must use <joint>=<non-negative-weight>");
    }
    const std::string name = entry.substr(0U, separator);
    const double weight = parseNonnegativeDouble(
        argument + " weight for " + name, entry.substr(separator + 1U));
    const auto existing = std::find_if(
        result.begin(), result.end(),
        [&](const auto &item) { return item.first == name; });
    if (existing == result.end()) {
      result.emplace_back(name, weight);
    } else {
      existing->second = weight;
    }
  }
  if (result.empty()) {
    throw std::runtime_error(argument + " requires at least one joint weight");
  }
  return result;
}

void mergeJointWeightMultipliers(
    std::vector<std::pair<std::string, double>> &destination,
    const std::vector<std::pair<std::string, double>> &values) {
  for (const auto &[name, weight] : values) {
    const auto existing = std::find_if(
        destination.begin(), destination.end(),
        [&](const auto &item) { return item.first == name; });
    if (existing == destination.end())
      destination.emplace_back(name, weight);
    else
      existing->second = weight;
  }
}

bool optionIn(const std::string &option,
              std::initializer_list<const char *> candidates) {
  return std::any_of(
      candidates.begin(), candidates.end(),
      [&](const char *candidate) { return option == candidate; });
}

PlanningSynchronization parsePlanningSynchronization(
    const std::string &argument, const std::string &value) {
  if (value == "none") {
    return PlanningSynchronization::None;
  }
  if (value == "time") {
    return PlanningSynchronization::Time;
  }
  if (value == "phase") {
    return PlanningSynchronization::Phase;
  }
  throw std::runtime_error(
      argument + " must be one of 'none', 'time', or 'phase'");
}

bool parseSolverOption(const std::string &argument, const std::string &value,
                       SolverOptions &options) {
  const double parsed = parsePositiveDouble(argument, value);
  if (argument == "--regularization")
    options.regularization = parsed;
  else if (argument == "--position-tolerance-m")
    options.position_tolerance_m = parsed;
  else if (argument == "--orientation-tolerance-rad")
    options.orientation_tolerance_rad = parsed;
  else if (argument == "--maximum-hard-violation")
    options.maximum_accepted_hard_violation = parsed;
  else if (argument == "--joint-position-margin-rad")
    options.joint_position_margin_rad = parsed;
  else if (argument == "--minimum-position-improvement-m")
    options.minimum_position_improvement_m = parsed;
  else if (argument == "--minimum-orientation-improvement-rad")
    options.minimum_orientation_improvement_rad = parsed;
  else if (argument == "--legacy-cartesian-progress-weight")
    options.legacy_cartesian_progress_weight = parsed;
  else if (argument == "--legacy-cartesian-preservation-tolerance")
    options.legacy_cartesian_preservation_tolerance = parsed;
  else if (argument == "--legacy-scale-preservation-tolerance")
    options.legacy_scale_preservation_tolerance = parsed;
  else if (argument == "--legacy-posture-preservation-tolerance")
    options.legacy_posture_preservation_tolerance = parsed;
  else if (argument == "--legacy-yellow-posture-weight")
    options.legacy_yellow_posture_weight = parsed;
  else if (argument == "--legacy-yellow-to-red-coupling-weight")
    options.legacy_yellow_to_red_coupling_weight = parsed;
  else if (argument == "--legacy-minimum-collision-distance-m")
    options.legacy_minimum_collision_distance_m = parsed;
  else if (argument == "--legacy-collision-influence-distance-m")
    options.legacy_collision_influence_distance_m = parsed;
  else if (argument == "--legacy-collision-damping-gain-per-s")
    options.legacy_collision_damping_gain_per_s = parsed;
  else if (argument == "--legacy-collision-weight")
    options.legacy_collision_weight = parsed;
  else if (argument == "--red-primary-task-tcp-position-progress-weight")
    options.red_primary_task_tcp_position_progress_weight = parsed;
  else if (argument ==
           "--red-primary-task-tcp-position-preservation-tolerance-mps")
    options.red_primary_task_tcp_position_preservation_tolerance_mps = parsed;
  else if (argument ==
           "--red-primary-task-tcp-position-progress-preservation-tolerance")
    options.red_primary_task_tcp_position_progress_preservation_tolerance =
        parsed;
  else if (argument == "--red-secondary-task-tcp-orientation-progress-weight")
    options.red_secondary_task_tcp_orientation_progress_weight = parsed;
  else if (argument ==
           "--red-secondary-task-tcp-orientation-preservation-tolerance-radps")
    options.red_secondary_task_tcp_orientation_preservation_tolerance_radps =
        parsed;
  else if (argument ==
           "--red-secondary-task-tcp-orientation-progress-preservation-tolerance")
    options
        .red_secondary_task_tcp_orientation_progress_preservation_tolerance =
        parsed;
  else if (argument ==
           "--red-tertiary-task-yellow-posture-coupling-preservation-tolerance")
    options.red_tertiary_task_yellow_posture_coupling_preservation_tolerance =
        parsed;
  else if (argument == "--red-tertiary-task-link4-position-weight")
    options.red_tertiary_task_link4_position_weight = parsed;
  else if (argument ==
           "--red-tertiary-task-link4-position-servo-gain-per-s")
    options.red_tertiary_task_link4_position_servo_gain_per_s = parsed;
  else if (argument ==
           "--red-tertiary-task-link4-position-preservation-tolerance-mps")
    options.red_tertiary_task_link4_position_preservation_tolerance_mps =
        parsed;
  else if (argument == "--red-proxqp-absolute-tolerance")
    options.red_proxqp_absolute_tolerance = parsed;
  else if (argument == "--red-proxqp-primal-infeasibility-tolerance")
    options.red_proxqp_primal_infeasibility_tolerance = parsed;
  else if (argument == "--yellow-task-posture-preference-weight")
    options.yellow_task_posture_preference_weight = parsed;
  else if (argument ==
           "--yellow-task-posture-preference-servo-gain-per-s")
    options.yellow_task_posture_preference_servo_gain_per_s = parsed;
  else if (argument ==
           "--red-tertiary-task-yellow-posture-coupling-weight")
    options.red_tertiary_task_yellow_posture_coupling_weight = parsed;
  else if (argument ==
           "--red-tertiary-task-yellow-posture-coupling-servo-gain-per-s")
    options.red_tertiary_task_yellow_posture_coupling_servo_gain_per_s = parsed;
  else if (argument ==
           "--yellow-constraints-self-collision-avoidance-minimum-distance-m")
    options.yellow_constraints_self_collision_avoidance_minimum_distance_m =
        parsed;
  else if (argument ==
           "--yellow-constraints-self-collision-avoidance-influence-distance-m")
    options.yellow_constraints_self_collision_avoidance_influence_distance_m =
        parsed;
  else if (argument ==
           "--yellow-constraints-self-collision-avoidance-damping-gain-per-s")
    options.yellow_constraints_self_collision_avoidance_damping_gain_per_s =
        parsed;
  else if (argument ==
           "--yellow-constraints-self-collision-avoidance-weight")
    options.yellow_constraints_self_collision_avoidance_weight = parsed;
  else
    return false;
  return true;
}

void validateJointWeightMultipliers(
    const std::vector<std::pair<std::string, double>> &multipliers,
    const std::string &option_name,
    const std::vector<std::string> &joint_names) {
  for (const auto &[name, weight] : multipliers) {
    static_cast<void>(weight);
    if (std::find(joint_names.begin(), joint_names.end(), name) ==
        joint_names.end()) {
      throw std::runtime_error(option_name + " contains unknown R1 joint: " +
                               name);
    }
  }
}

void validate(const HierarchicalOptions &options, Profile profile) {
  if (!(options.red_rate_hz > options.yellow_rate_hz)) {
    throw std::runtime_error("rates must satisfy red > yellow > 0");
  }
  if (options.tui.side != "left" && options.tui.side != "right") {
    throw std::runtime_error("side must be either 'left' or 'right'");
  }
  if (options.tui.max_step_m < options.tui.min_step_m) {
    throw std::runtime_error(
        "--max-step-m must be greater than or equal to --min-step-m");
  }
  if (options.tui.step_m < options.tui.min_step_m ||
      options.tui.step_m > options.tui.max_step_m) {
    throw std::runtime_error(
        "--step-m must be inside [--min-step-m, --max-step-m]");
  }
  if (options.urdf_path.empty()) {
    throw std::runtime_error("--urdf is required");
  }
  const auto joint_count = options.robot.joint_names.size();
  if (joint_count == 0U || options.robot.default_positions.size() != joint_count ||
      options.robot.effort_limits.size() != joint_count ||
      options.robot.joint_stream.joint_names.size() != joint_count) {
    throw std::runtime_error(
        "robot joint names, defaults, effort limits, and PVAJ limits must "
        "have the same non-zero length");
  }
  for (std::size_t index = 0U; index < joint_count; ++index) {
    const auto &stream = options.robot.joint_stream;
    if (stream.joint_names[index] != options.robot.joint_names[index] ||
        !(stream.position_lower_rad[index] <
          stream.position_upper_rad[index]) ||
        options.robot.default_positions[index] <
            stream.position_lower_rad[index] ||
        options.robot.default_positions[index] >
            stream.position_upper_rad[index]) {
      throw std::runtime_error(
          "robot PVAJ profile does not match joint order/defaults at index " +
          std::to_string(index));
    }
  }
  const auto validate_indices = [&](const auto &indices,
                                    const std::string &name) {
    if (std::any_of(indices.begin(), indices.end(),
                    [&](std::size_t value) { return value >= joint_count; }))
      throw std::runtime_error(name + " contains an out-of-range index");
  };
  validate_indices(options.robot.left_arm_joint_indices,
                   "--left-arm-joint-indices");
  validate_indices(options.robot.right_arm_joint_indices,
                   "--right-arm-joint-indices");
  if (profileCapabilities(profile).kinematic_simulation &&
      options.simulation.mujoco_model_path.empty()) {
    throw std::runtime_error("--mujoco-model is required for profile " +
                             std::string{profileName(profile)});
  }
  if (!(options.admittance.wrench_filter_alpha > 0.0 &&
        options.admittance.wrench_filter_alpha <= 1.0)) {
    throw std::runtime_error("--wrench-filter-alpha must be in (0, 1]");
  }
  validateJointWeightMultipliers(
      options.solver.yellow_task_posture_preference_joint_weight_multipliers,
      "--yellow-task-posture-preference-joint-weight-multipliers",
      options.robot.joint_names);
  validateJointWeightMultipliers(
      options.solver
          .red_tertiary_task_yellow_posture_coupling_joint_weight_multipliers,
      "--red-tertiary-task-yellow-posture-coupling-joint-weight-multipliers",
      options.robot.joint_names);
  if (options.solver
          .yellow_constraints_self_collision_avoidance_influence_distance_m <=
      options.solver
          .yellow_constraints_self_collision_avoidance_minimum_distance_m) {
    throw std::runtime_error(
        "--yellow-constraints-self-collision-avoidance-influence-distance-m "
        "must exceed "
        "--yellow-constraints-self-collision-avoidance-minimum-distance-m");
  }
}

std::string collisionMeshSearchRoot(const std::filesystem::path &urdf_path,
                                    Profile profile) {
  const auto canonical_urdf = std::filesystem::weakly_canonical(urdf_path);
  // The app's installed simulation URDF uses ../meshes/... paths, which
  // Pinocchio resolves from the URDF directory.  Keep the inherited
  // package://psi_r1 layout compatible for explicit production-URDF
  // overrides, where the package search root is three levels above the file.
  if (profile ==
          Profile::PlannedOtgNullspaceAdmittanceKinematicSim &&
      std::filesystem::is_directory(
          canonical_urdf.parent_path().parent_path() / "meshes")) {
    return canonical_urdf.parent_path().string();
  }
  return canonical_urdf.parent_path().parent_path().parent_path().string();
}

} // namespace

const char *profileName(Profile profile) {
  switch (profile) {
  case Profile::Hierarchical:
    return "hierarchical";
  case Profile::Planned:
    return "planned";
  case Profile::PlannedOtg:
    return "planned-otg";
  case Profile::PlannedOtgNullspace:
    return "planned-otg-nullspace";
  case Profile::PlannedOtgNullspaceAdmittanceKinematicSim:
    return "planned-otg-nullspace-admittance-kinematic-sim";
  }
  return "unknown";
}

Profile parseProfile(const std::string &value) {
  if (value == "hierarchical")
    return Profile::Hierarchical;
  if (value == "planned")
    return Profile::Planned;
  if (value == "planned-otg")
    return Profile::PlannedOtg;
  if (value == "planned-otg-nullspace")
    return Profile::PlannedOtgNullspace;
  if (value == "planned-otg-nullspace-admittance-kinematic-sim")
    return Profile::PlannedOtgNullspaceAdmittanceKinematicSim;
  throw std::runtime_error(
      "--profile must be one of hierarchical, planned, planned-otg, "
      "planned-otg-nullspace, or "
      "planned-otg-nullspace-admittance-kinematic-sim");
}

ProfileCapabilities profileCapabilities(Profile profile) {
  ProfileCapabilities result;
  result.cartesian_planning = profile != Profile::Hierarchical;
  result.joint_otg = profile == Profile::PlannedOtg ||
                     profile == Profile::PlannedOtgNullspace ||
                     profile ==
                         Profile::PlannedOtgNullspaceAdmittanceKinematicSim;
  result.nullspace = profile == Profile::PlannedOtgNullspace ||
                     profile ==
                         Profile::PlannedOtgNullspaceAdmittanceKinematicSim;
  result.admittance =
      profile == Profile::PlannedOtgNullspaceAdmittanceKinematicSim;
  result.kinematic_simulation = result.admittance;
  result.telemetry = result.joint_otg;
  return result;
}

Options profileDefaults(Profile profile) {
  Options result;
  result.profile = profile;
  auto &app = result.interactive;
  auto &solver = app.solver;
  auto &robot = app.robot;
  robot.profile_provenance = profileName(profile);

  app.simulation.viewer_enabled = false;
  solver.red_joint_acceleration_limits_enabled = false;

  switch (profile) {
  case Profile::Hierarchical:
    app.red_rate_hz = 1000.0;
    app.yellow_rate_hz = 100.0;
    solver.regularization = 1.0e-4;
    solver.maximum_accepted_hard_violation = 5.0e-4;
    solver.joint_position_braking_velocity_envelope_enabled = false;
    solver.legacy_cartesian_progress_weight = 3.0;
    solver.legacy_yellow_to_red_coupling_weight = 10.0;
    solver.legacy_minimum_collision_distance_m = 0.3;
    solver.legacy_collision_influence_distance_m = 0.35;
    solver.legacy_collision_damping_gain_per_s = 2.0;
    solver.legacy_collision_weight = 100.0;
    solver.red_proxqp_absolute_tolerance = 1.0e-6;
    robot.inactive_joint_names = {
        "torso_yaw_joint", "torso_pitch_joint", "knee_pitch_joint",
        "ankle_pitch_joint"};
    break;
  case Profile::Planned:
    app.red_rate_hz = 100.0;
    app.yellow_rate_hz = 20.0;
    solver.maximum_accepted_hard_violation = 5.0e-4;
    solver.joint_position_braking_velocity_envelope_enabled = true;
    solver.red_joint_acceleration_limits_enabled = true;
    robot.inactive_joint_names = {"knee_pitch_joint", "ankle_pitch_joint"};
    robot.joint_stream.max_acceleration_rad_per_s2 = {
        6.0,  6.0,  6.0,  4.0,   4.0,   4.0,   10.10, 10.10, 12.42, 12.48,
        16.2, 16.2, 16.2, 10.10, 10.10, 12.42, 12.48, 16.2,  16.2,  16.2};
    break;
  case Profile::PlannedOtg:
    app.red_rate_hz = 1000.0;
    app.yellow_rate_hz = 100.0;
    solver.maximum_accepted_hard_violation = 5.0e-4;
    solver.joint_position_braking_velocity_envelope_enabled = true;
    solver.red_joint_acceleration_limits_enabled = true;
    robot.inactive_joint_names = {"knee_pitch_joint", "ankle_pitch_joint"};
    robot.joint_stream.position_lower_rad[11] = -0.9599;
    robot.joint_stream.position_lower_rad[12] = -0.9599;
    robot.joint_stream.position_lower_rad[18] = -0.9599;
    robot.joint_stream.position_upper_rad[11] = 0.9599;
    robot.joint_stream.position_upper_rad[12] = 0.9599;
    robot.joint_stream.position_upper_rad[18] = 0.9599;
    break;
  case Profile::PlannedOtgNullspace:
    app.red_rate_hz = 1000.0;
    app.yellow_rate_hz = 100.0;
    solver.joint_position_braking_velocity_envelope_enabled = true;
    solver.red_joint_acceleration_limits_enabled = true;
    robot.joint_stream.position_lower_rad[11] = -0.9599;
    robot.joint_stream.position_lower_rad[12] = -0.9599;
    robot.joint_stream.position_lower_rad[18] = -0.9599;
    robot.joint_stream.position_upper_rad[11] = 0.9599;
    robot.joint_stream.position_upper_rad[12] = 0.9599;
    robot.joint_stream.position_upper_rad[18] = 0.9599;
    break;
  case Profile::PlannedOtgNullspaceAdmittanceKinematicSim:
    app.red_rate_hz = 1000.0;
    app.yellow_rate_hz = 100.0;
    app.urdf_path = MCL_R1_MUJOCO_URDF_PATH;
    app.simulation.mujoco_model_path = MCL_R1_MUJOCO_MODEL_PATH;
    app.simulation.viewer_enabled = true;
    // Preserve the pre-existing local tuning as explicit configuration.
    solver.joint_position_braking_velocity_envelope_enabled = false;
    solver.red_joint_acceleration_limits_enabled = false;
    break;
  }
  return result;
}

std::string resolvedOptionsJson(const Options &options) {
  const auto strings = [](const auto &values) {
    Json::Value result{Json::arrayValue};
    for (const auto &value : values)
      result.append(value);
    return result;
  };
  const auto numbers = [](const auto &values) {
    Json::Value result{Json::arrayValue};
    for (const auto &value : values)
      result.append(value);
    return result;
  };
  const auto transform = [](const Eigen::Isometry3d &value) {
    Json::Value result;
    const Eigen::Quaterniond rotation{value.rotation()};
    result["translation"] = Json::Value{Json::arrayValue};
    result["translation"].append(value.translation().x());
    result["translation"].append(value.translation().y());
    result["translation"].append(value.translation().z());
    result["quaternion_xyzw"] = Json::Value{Json::arrayValue};
    result["quaternion_xyzw"].append(rotation.x());
    result["quaternion_xyzw"].append(rotation.y());
    result["quaternion_xyzw"].append(rotation.z());
    result["quaternion_xyzw"].append(rotation.w());
    return result;
  };
  const auto optionalPath = [](const auto &value) {
    return value.has_value() ? Json::Value{value->string()} : Json::Value{};
  };

  Json::Value root;
  root["schema_version"] = "mcl.hierarchical_kinematics_step.options.v1";
  root["profile"] = profileName(options.profile);
  root["source_mode"] =
      options.source_mode == SourceMode::Teleop ? "teleop" : "replay";
  const auto capabilities = profileCapabilities(options.profile);
  root["capabilities"]["cartesian_planning"] =
      capabilities.cartesian_planning;
  root["capabilities"]["joint_otg"] = capabilities.joint_otg;
  root["capabilities"]["nullspace"] = capabilities.nullspace;
  root["capabilities"]["admittance"] = capabilities.admittance;
  root["capabilities"]["kinematic_simulation"] =
      capabilities.kinematic_simulation;
  root["capabilities"]["telemetry"] = capabilities.telemetry;
  root["binary_argv"] = strings(options.binary_argv);

  const auto &app = options.interactive;
  auto &runtime = root["runtime"];
  runtime["urdf_path"] = app.urdf_path;
  runtime["red_rate_hz"] = app.red_rate_hz;
  runtime["yellow_rate_hz"] = app.yellow_rate_hz;
  runtime["ui_rate_hz"] = app.ui_rate_hz;
  runtime["deadline_policy"] =
      app.deadline_policy == DeadlinePolicy::Strict ? "strict" : "monitor";
  runtime["duration_s"] = app.duration_s;
  runtime["presentation_enabled"] = app.presentation.enabled;
  runtime["teleop"]["side"] = app.tui.side;
  runtime["teleop"]["step_m"] = app.tui.step_m;
  runtime["teleop"]["min_step_m"] = app.tui.min_step_m;
  runtime["teleop"]["max_step_m"] = app.tui.max_step_m;
  runtime["teleop"]["rotation_step_deg"] = app.tui.rotation_step_deg;
  runtime["visualization"]["enabled"] = app.visualization.enabled;
  runtime["visualization"]["host"] = app.visualization.host;
  runtime["visualization"]["port"] = app.visualization.port;
  runtime["visualization"]["mcap_path"] =
      optionalPath(app.visualization.mcap_path);

  const auto &solver = app.solver;
  auto &solver_json = root["solver"];
#define MCL_SOLVER_FIELD(name) solver_json[#name] = solver.name
  MCL_SOLVER_FIELD(regularization);
  MCL_SOLVER_FIELD(position_tolerance_m);
  MCL_SOLVER_FIELD(orientation_tolerance_rad);
  MCL_SOLVER_FIELD(minimum_position_improvement_m);
  MCL_SOLVER_FIELD(minimum_orientation_improvement_rad);
  MCL_SOLVER_FIELD(maximum_accepted_hard_violation);
  MCL_SOLVER_FIELD(joint_position_margin_rad);
  MCL_SOLVER_FIELD(joint_position_braking_velocity_envelope_enabled);
  MCL_SOLVER_FIELD(red_joint_acceleration_limits_enabled);
  MCL_SOLVER_FIELD(legacy_cartesian_progress_weight);
  MCL_SOLVER_FIELD(legacy_cartesian_preservation_tolerance);
  MCL_SOLVER_FIELD(legacy_scale_preservation_tolerance);
  MCL_SOLVER_FIELD(legacy_posture_preservation_tolerance);
  MCL_SOLVER_FIELD(legacy_yellow_posture_weight);
  MCL_SOLVER_FIELD(legacy_yellow_to_red_coupling_weight);
  MCL_SOLVER_FIELD(legacy_minimum_collision_distance_m);
  MCL_SOLVER_FIELD(legacy_collision_influence_distance_m);
  MCL_SOLVER_FIELD(legacy_collision_damping_gain_per_s);
  MCL_SOLVER_FIELD(legacy_collision_weight);
  MCL_SOLVER_FIELD(red_primary_task_tcp_position_progress_weight);
  MCL_SOLVER_FIELD(red_primary_task_tcp_position_preservation_tolerance_mps);
  MCL_SOLVER_FIELD(
      red_primary_task_tcp_position_progress_preservation_tolerance);
  MCL_SOLVER_FIELD(red_secondary_task_tcp_orientation_progress_weight);
  MCL_SOLVER_FIELD(
      red_secondary_task_tcp_orientation_preservation_tolerance_radps);
  MCL_SOLVER_FIELD(
      red_secondary_task_tcp_orientation_progress_preservation_tolerance);
  MCL_SOLVER_FIELD(
      red_tertiary_task_yellow_posture_coupling_preservation_tolerance);
  MCL_SOLVER_FIELD(red_tertiary_task_link4_position_weight);
  MCL_SOLVER_FIELD(red_tertiary_task_link4_position_servo_gain_per_s);
  MCL_SOLVER_FIELD(
      red_tertiary_task_link4_position_preservation_tolerance_mps);
  MCL_SOLVER_FIELD(yellow_maximum_iterations);
  MCL_SOLVER_FIELD(red_proxqp_maximum_iterations);
  MCL_SOLVER_FIELD(red_proxqp_absolute_tolerance);
  MCL_SOLVER_FIELD(red_proxqp_primal_infeasibility_tolerance);
  MCL_SOLVER_FIELD(red_proxqp_warm_start_enabled);
  MCL_SOLVER_FIELD(yellow_task_posture_preference_weight);
  MCL_SOLVER_FIELD(yellow_task_posture_preference_servo_gain_per_s);
  MCL_SOLVER_FIELD(red_tertiary_task_yellow_posture_coupling_weight);
  MCL_SOLVER_FIELD(
      red_tertiary_task_yellow_posture_coupling_servo_gain_per_s);
  MCL_SOLVER_FIELD(
      yellow_constraints_self_collision_avoidance_minimum_distance_m);
  MCL_SOLVER_FIELD(
      yellow_constraints_self_collision_avoidance_influence_distance_m);
  MCL_SOLVER_FIELD(
      yellow_constraints_self_collision_avoidance_damping_gain_per_s);
  MCL_SOLVER_FIELD(yellow_constraints_self_collision_avoidance_weight);
#undef MCL_SOLVER_FIELD
  for (const auto &[name, weight] :
       solver.yellow_task_posture_preference_joint_weight_multipliers)
    solver_json["yellow_task_posture_preference_joint_weight_multipliers"]
               [name] = weight;
  for (const auto &[name, weight] :
       solver
           .red_tertiary_task_yellow_posture_coupling_joint_weight_multipliers)
    solver_json
        ["red_tertiary_task_yellow_posture_coupling_joint_weight_multipliers"]
        [name] = weight;

  const auto &robot = app.robot;
  auto &robot_json = root["robot"];
  robot_json["profile_provenance"] = robot.profile_provenance;
  robot_json["base_frame"] = robot.base_frame;
  robot_json["left_end_effector_frame"] = robot.left_end_effector_frame;
  robot_json["right_end_effector_frame"] = robot.right_end_effector_frame;
  robot_json["left_link4_frame"] = robot.left_link4_frame;
  robot_json["right_link4_frame"] = robot.right_link4_frame;
  robot_json["left_tcp_offset"] = transform(robot.left_tcp_offset);
  robot_json["right_tcp_offset"] = transform(robot.right_tcp_offset);
  robot_json["joint_names"] = strings(robot.joint_names);
  robot_json["default_positions"] = numbers(robot.default_positions);
  robot_json["left_arm_joint_indices"] = numbers(robot.left_arm_joint_indices);
  robot_json["right_arm_joint_indices"] =
      numbers(robot.right_arm_joint_indices);
  robot_json["effort_limits"] = numbers(robot.effort_limits);
  robot_json["inactive_joint_names"] = strings(robot.inactive_joint_names);
  robot_json["collision_mesh_search_paths"] =
      strings(robot.collision_mesh_search_paths);
  for (const auto &pair : robot.self_collision_link_pairs) {
    Json::Value value;
    value["first_link"] = pair.first_link;
    value["second_link"] = pair.second_link;
    robot_json["self_collision_link_pairs"].append(value);
  }
  const auto &stream = robot.joint_stream;
  auto &stream_json = robot_json["joint_stream"];
  stream_json["source_revision"] = stream.source_revision;
  stream_json["source_path"] = stream.source_path;
  stream_json["source_sha256"] = stream.source_sha256;
  stream_json["jerk_override_reason"] = stream.jerk_override_reason;
  stream_json["joint_names"] = strings(stream.joint_names);
  stream_json["position_lower_rad"] = numbers(stream.position_lower_rad);
  stream_json["position_upper_rad"] = numbers(stream.position_upper_rad);
  stream_json["max_velocity_rad_per_s"] =
      numbers(stream.max_velocity_rad_per_s);
  stream_json["max_acceleration_rad_per_s2"] =
      numbers(stream.max_acceleration_rad_per_s2);
  stream_json["max_jerk_rad_per_s3"] =
      numbers(stream.max_jerk_rad_per_s3);

  auto &planning = root["planning"];
  planning["max_linear_velocity_mps"] =
      options.planning.max_linear_velocity_mps;
  planning["max_linear_acceleration_mps2"] =
      options.planning.max_linear_acceleration_mps2;
  planning["max_linear_jerk_mps3"] = options.planning.max_linear_jerk_mps3;
  planning["max_angular_velocity_rps"] =
      options.planning.max_angular_velocity_rps;
  planning["max_angular_acceleration_rps2"] =
      options.planning.max_angular_acceleration_rps2;
  planning["max_angular_jerk_rps3"] =
      options.planning.max_angular_jerk_rps3;
  planning["cartesian_synchronization"] =
      planningSynchronizationName(options.planning.cartesian_synchronization);
  planning["joint_algorithm"] =
      jointPlanningAlgorithmName(options.planning.joint_algorithm);
  planning["joint_synchronization"] =
      planningSynchronizationName(options.planning.joint_synchronization);

  root["joint_target"]["mode"] = jointTargetModeName(options.joint_target.mode);
  root["joint_target"]["future_o1_velocity_deadband_rad_per_s"] =
      options.joint_target.future_o1_velocity_deadband_rad_per_s;
  root["replay_settling"]["fk_position_m"] =
      options.replay_settling.fk_position_m;
  root["replay_settling"]["fk_orientation_rad"] =
      options.replay_settling.fk_orientation_rad;
  root["replay_settling"]["velocity_rad_per_s"] =
      options.replay_settling.velocity_rad_per_s;
  root["replay_settling"]["acceleration_rad_per_s2"] =
      options.replay_settling.acceleration_rad_per_s2;
  root["replay_settling"]["required_cycles"] =
      Json::UInt64(options.replay_settling.required_cycles);

  const auto &admittance = app.admittance;
  auto &admittance_json = root["admittance"];
  admittance_json["enabled"] =
      profileCapabilities(options.profile).admittance;
  admittance_json["angular_enabled"] = admittance.angular_enabled;
  admittance_json["linear_environment_stiffness_n_per_m"] =
      admittance.linear_environment_stiffness_n_per_m;
  admittance_json["linear_environment_damping_ns_per_m"] =
      admittance.linear_environment_damping_ns_per_m;
  admittance_json["maximum_force_n"] = admittance.maximum_force_n;
  admittance_json["angular_environment_stiffness_nm_per_rad"] =
      admittance.angular_environment_stiffness_nm_per_rad;
  admittance_json["angular_environment_damping_nms_per_rad"] =
      admittance.angular_environment_damping_nms_per_rad;
  admittance_json["maximum_torque_nm"] = admittance.maximum_torque_nm;
  admittance_json["wrench_filter_alpha"] = admittance.wrench_filter_alpha;
  root["simulation"]["mujoco_model_path"] =
      app.simulation.mujoco_model_path;
  root["simulation"]["viewer_enabled"] = app.simulation.viewer_enabled;

  root["replay_trace_enabled"] = options.replay_trace_enabled;
  root["replay_elbow_teleop_enabled"] = options.replay_elbow_teleop_enabled;
  root["start_paused"] = options.start_paused;
  root["launcher_argv_json"] = options.launcher_argv_json;
  if (options.replay.has_value()) {
    const auto &replay = *options.replay;
    auto &value = root["replay"];
    value["urdf_path"] = replay.urdf_path.string();
    value["input_path"] = replay.input_path.string();
    value["input_format"] = replay::toString(replay.input_format);
    value["left_stream"] = replay.left_stream;
    value["right_stream"] = replay.right_stream;
    if (replay.initial_joint_state_stream.has_value())
      value["initial_joint_state_stream"] =
          *replay.initial_joint_state_stream;
    value["csv_mapping_path"] = optionalPath(replay.csv_mapping_path);
    value["target_period_ns"] = Json::Int64(replay.target_period_ns);
    value["timestamp_source"] = data::toString(replay.timestamp_source);
    value["timestamp_projection"]["policy"] =
        data::toString(replay.timestamp_projection.policy);
    value["timestamp_projection"]["period_ns"] =
        Json::Int64(replay.timestamp_projection.period_ns);
    value["pairing_policy"] = data::toString(replay.pairing_policy);
    value["nearest_tolerance_ns"] = Json::Int64(replay.nearest_tolerance_ns);
    value["unmatched_policy"] = data::toString(replay.unmatched_policy);
    value["execution_mode"] = data::toString(replay.execution_mode);
    value["playback_rate"] = replay.playback_rate;
    value["output_dir"] = replay.output_dir.string();
    value["output_dir_explicit"] = replay.output_dir_explicit;
    value["output_root"] = optionalPath(replay.output_root);
    if (replay.run_id.has_value())
      value["run_id"] = *replay.run_id;
    value["ui_mode"] = replay.ui_mode;
    value["terminal_input_enabled"] = replay.terminal_input_enabled;
    value["visualization_enabled"] = replay.visualization_enabled;
    value["visualization_host"] = replay.visualization_host;
    value["visualization_port"] = replay.visualization_port;
    value["visualization_mcap_path"] =
        optionalPath(replay.visualization_mcap_path);
    value["launcher"] = replay.launcher;
    value["original_argv"] = strings(replay.original_argv);
  }

  Json::StreamWriterBuilder writer;
  writer["indentation"] = "  ";
  return Json::writeString(writer, root) + "\n";
}

void printHierarchicalUsage(const char *program) {
  const HierarchicalOptions defaults;
  std::cout
      << "Usage: " << program << " [options]\n\n"
      << "Options:\n"
      << "  --side <left|right> Initial selected arm side (default: "
      << defaults.tui.side << ")\n"
      << "  --urdf <path>       Robot URDF path\n"
      << "  --mujoco-model <path> MuJoCo MJCF model\n"
      << "  --mujoco-viewer / --no-mujoco-viewer  Native kinematic viewer "
         "switch (default: on)\n"
      << "  --angular-admittance / --no-angular-admittance  Rotational "
         "compliance switch (default: off)\n"
      << "  --host <address>    WebSocket bind address (default: "
      << defaults.visualization.host << ")\n"
      << "  --port <port>       WebSocket port (default: "
      << defaults.visualization.port << ")\n"
      << "  --red-rate <hz>     Red servo rate/deadline (default: "
      << defaults.red_rate_hz << ")\n"
      << "  --yellow-rate <hz>  Yellow proposal rate/deadline (default: "
      << defaults.yellow_rate_hz << ")\n"
      << "  --ui-rate <hz>      TUI and visualization rate (default: "
      << defaults.ui_rate_hz << ")\n"
      << "  --ui <tui|none>     User interface mode (default: tui)\n"
      << "  --viz <foxglove|none> Visualization transport (default: foxglove)\n"
      << "  --deadline-policy <strict|monitor> Deadline handling (default: "
         "strict)\n"
      << "  --duration <sec>    Stop after seconds; 0 runs until Ctrl-C "
         "(default: "
      << defaults.duration_s << ")\n"
      << "  --step-m <meters>   Cartesian increment per keypress (default: "
      << defaults.tui.step_m << ")\n"
      << "  --min-step-m <m>    Minimum step size (default: "
      << defaults.tui.min_step_m << ")\n"
      << "  --max-step-m <m>    Maximum step size (default: "
      << defaults.tui.max_step_m << ")\n"
      << "  --rotation-step-deg <deg> TCP rotation step (default: "
      << defaults.tui.rotation_step_deg << ")\n"
      << "  --mcap <path>       Write MCAP from the UI thread when Foxglove is "
         "enabled\n"
      << "  --no-mcap           Disable MCAP output (default)\n"
      << "  --environment-stiffness <N/m>          Drag spring stiffness "
         "(default: 200)\n"
      << "  --environment-damping <Ns/m>           Drag damping "
         "(default: 20)\n"
      << "  --maximum-force <N>                    Drag force limit "
         "(default: 20)\n"
      << "  --rotation-environment-stiffness <Nm/rad> (default: 10)\n"
      << "  --rotation-environment-damping <Nms/rad> (default: 1)\n"
      << "  --maximum-torque <Nm>                  Drag torque limit "
         "(default: 5)\n"
      << "  --wrench-filter-alpha <value>           Low-pass alpha "
         "(default: 0.08)\n"
      << "  --regularization <value>                 QP regularization\n"
      << "  --position-tolerance-m <value>           Position tolerance\n"
      << "  --orientation-tolerance-rad <value>      Orientation tolerance\n"
      << "  --maximum-hard-violation <value>         App acceptance tolerance\n"
      << "  --joint-position-margin-rad <value>      Joint limit margin\n"
      << "  --red-primary-task-tcp-position-progress-weight <value> Primary "
         "position progress weight\n"
      << "  --red-secondary-task-tcp-orientation-progress-weight <value> "
         "Secondary orientation progress weight\n"
      << "  --red-tertiary-task-link4-position-weight <value> Tertiary "
         "link4 weight "
         "(default: "
      << defaults.solver.red_tertiary_task_link4_position_weight << ")\n"
      << "  --red-tertiary-task-link4-position-servo-gain-per-s <value> "
         "Tertiary link4 gain "
         "(default: "
      << defaults.solver.red_tertiary_task_link4_position_servo_gain_per_s
      << ")\n"
      << "  --red-tertiary-task-link4-position-preservation-tolerance-mps "
         "<value> Link4 preservation tolerance (default: "
      << defaults.solver
             .red_tertiary_task_link4_position_preservation_tolerance_mps
      << ")\n"
      << "  --red-proxqp-absolute-tolerance <value>  Red QP tolerance\n"
      << "  --red-proxqp-primal-infeasibility-tolerance <value> Red "
         "certificate tolerance\n"
      << "  --yellow-task-posture-preference-weight <value> Yellow posture "
         "preference weight\n"
      << "  --yellow-task-posture-preference-servo-gain-per-s <value> Yellow "
         "posture gain\n"
      << "  --yellow-task-posture-preference-joint-weight-multipliers "
         "<joint=value,...> Yellow posture joint weights\n"
      << "  --red-tertiary-task-yellow-posture-coupling-weight <value> "
         "Tertiary Yellow-to-Red coupling weight\n"
      << "  --red-tertiary-task-yellow-posture-coupling-servo-gain-per-s "
         "<value> Tertiary Yellow-to-Red coupling gain\n"
      << "  --red-tertiary-task-yellow-posture-coupling-joint-weight-"
         "multipliers <joint=value,...> Tertiary coupling joint weights\n"
      << "  --yellow-constraints-self-collision-avoidance-minimum-distance-m "
         "<value> Collision minimum distance\n"
      << "  --yellow-constraints-self-collision-avoidance-influence-distance-m "
         "<value> Collision influence distance\n"
      << "  --yellow-constraints-self-collision-avoidance-damping-gain-per-s "
         "<value> Collision damping gain\n"
      << "  --yellow-constraints-self-collision-avoidance-weight <value> "
         "Collision weight\n"
      << "  --help              Show this help text\n\n"
      << "Rates must satisfy red > yellow > 0. Each group period is its "
         "deadline.\n\n";
  std::cout
      << "Keyboard controls:\n"
      << "  1..7/F1..F7/Tab/BackTab: navigate pages; h or ?: help\n"
      << "  PageUp/PageDown/Home/End: scroll the current page\n"
      << "  left/right arrows: select arm; c: switch TCP/link4 focus\n"
      << "  w/s: +x/-x, a/d: +y/-y, q/e: +z/-z\n"
      << "  n: cycle TCP rotation axis, i/u: rotate TCP\n"
      << "  up/down arrows: double/halve step; m: enter step size\n"
      << "  r: reset current target; x: clear held link4 or exit; Esc: exit\n";
}

void printPlannedUsage(const char *program, SourceMode source_mode) {
  printHierarchicalUsage(program);
  const Options defaults;
  std::cout << "\nOnline Cartesian replan limits (per "
               "reference-frame/rotation-vector axis):\n"
            << "  --max-linear-velocity-mps <value>       (default: "
            << defaults.planning.max_linear_velocity_mps << ")\n"
            << "  --max-linear-acceleration-mps2 <value>  (default: "
            << defaults.planning.max_linear_acceleration_mps2 << ")\n"
            << "  --max-linear-jerk-mps3 <value>          (default: "
            << defaults.planning.max_linear_jerk_mps3 << ")\n"
            << "  --max-angular-velocity-rps <value>      (default: "
            << defaults.planning.max_angular_velocity_rps << ")\n"
            << "  --max-angular-acceleration-rps2 <value> (default: "
            << defaults.planning.max_angular_acceleration_rps2 << ")\n"
            << "  --max-angular-jerk-rps3 <value>         (default: "
            << defaults.planning.max_angular_jerk_rps3 << ")\n"
            << "  --joint-synchronization <none|time|phase> (default: "
            << planningSynchronizationName(
                   defaults.planning.joint_synchronization)
            << ")\n"
            << "  --joint-algorithm <jerk-limited> (default: "
            << jointPlanningAlgorithmName(defaults.planning.joint_algorithm)
            << ")\n"
            << "  --joint-target-mode <future-o1-pv|ik-pv> (default: "
            << jointTargetModeName(defaults.joint_target.mode) << ")\n";
  if (source_mode == SourceMode::Replay) {
    std::cout << "\nReplay startup:\n"
              << "  --start-paused  Hold the replay at timeline zero until "
                 "space is pressed\n"
              << "  --replay-elbow-teleop/--no-replay-elbow-teleop Allow realtime link4 "
                 "Secondary target editing (default: off)\n"
              << "  --replay-trace/--no-replay-trace Write detailed per-Red-tick "
                 "trace.csv (default: on)\n";
  }
}

HierarchicalOptions parseHierarchicalOptions(int argc, char **argv,
                                             HierarchicalOptions options,
                                             Profile profile) {
  bool collision_pairs_overridden = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument{argv[index]};
    if (argument == "--help" || argument == "-h") {
      printHierarchicalUsage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else if (argument == "--side") {
      options.tui.side = requireValue(index, argc, argv, argument);
    } else if (argument == "--urdf") {
      options.urdf_path = requireValue(index, argc, argv, argument);
    } else if (argument == "--mujoco-model") {
      options.simulation.mujoco_model_path =
          requireValue(index, argc, argv, argument);
    } else if (argument == "--mujoco-viewer") {
      options.simulation.viewer_enabled = true;
    } else if (argument == "--no-mujoco-viewer") {
      options.simulation.viewer_enabled = false;
    } else if (argument == "--angular-admittance") {
      options.admittance.angular_enabled = true;
    } else if (argument == "--no-angular-admittance") {
      options.admittance.angular_enabled = false;
    } else if (argument ==
               "--joint-position-braking-velocity-envelope") {
      options.solver.joint_position_braking_velocity_envelope_enabled = true;
    } else if (argument ==
               "--no-joint-position-braking-velocity-envelope") {
      options.solver.joint_position_braking_velocity_envelope_enabled = false;
    } else if (argument == "--red-joint-acceleration-limits") {
      options.solver.red_joint_acceleration_limits_enabled = true;
    } else if (argument == "--no-red-joint-acceleration-limits") {
      options.solver.red_joint_acceleration_limits_enabled = false;
    } else if (argument == "--red-proxqp-warm-start") {
      options.solver.red_proxqp_warm_start_enabled = true;
    } else if (argument == "--no-red-proxqp-warm-start") {
      options.solver.red_proxqp_warm_start_enabled = false;
    } else if (argument == "--base-frame") {
      options.robot.base_frame = requireValue(index, argc, argv, argument);
    } else if (argument == "--left-end-effector-frame") {
      options.robot.left_end_effector_frame =
          requireValue(index, argc, argv, argument);
    } else if (argument == "--right-end-effector-frame") {
      options.robot.right_end_effector_frame =
          requireValue(index, argc, argv, argument);
    } else if (argument == "--left-link4-frame") {
      options.robot.left_link4_frame =
          requireValue(index, argc, argv, argument);
    } else if (argument == "--right-link4-frame") {
      options.robot.right_link4_frame =
          requireValue(index, argc, argv, argument);
    } else if (argument == "--left-tcp-offset") {
      options.robot.left_tcp_offset = parseTransform(
          argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--right-tcp-offset") {
      options.robot.right_tcp_offset = parseTransform(
          argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--joint-names") {
      options.robot.joint_names =
          parseStrings(argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--default-joint-positions") {
      options.robot.default_positions =
          parseNumbers(argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--left-arm-joint-indices") {
      options.robot.left_arm_joint_indices =
          parseIndices(argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--right-arm-joint-indices") {
      options.robot.right_arm_joint_indices =
          parseIndices(argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--effort-limits") {
      options.robot.effort_limits =
          parseNumbers(argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--inactive-joints") {
      const auto value = requireValue(index, argc, argv, argument);
      options.robot.inactive_joint_names =
          value.empty() ? std::vector<std::string>{}
                        : parseStrings(argument, value);
    } else if (argument == "--collision-mesh-search-paths") {
      const auto value = requireValue(index, argc, argv, argument);
      options.robot.collision_mesh_search_paths =
          value.empty() ? std::vector<std::string>{}
                        : parseStrings(argument, value);
    } else if (argument == "--self-collision-pair") {
      if (!collision_pairs_overridden) {
        options.robot.self_collision_link_pairs.clear();
        collision_pairs_overridden = true;
      }
      options.robot.self_collision_link_pairs.push_back(parseCollisionPair(
          argument, requireValue(index, argc, argv, argument)));
    } else if (argument == "--joint-stream-source-revision") {
      options.robot.joint_stream.source_revision =
          requireValue(index, argc, argv, argument);
    } else if (argument == "--joint-stream-source-path") {
      options.robot.joint_stream.source_path =
          requireValue(index, argc, argv, argument);
    } else if (argument == "--joint-stream-source-sha256") {
      options.robot.joint_stream.source_sha256 =
          requireValue(index, argc, argv, argument);
    } else if (argument == "--joint-stream-jerk-override-reason") {
      options.robot.joint_stream.jerk_override_reason =
          requireValue(index, argc, argv, argument);
    } else if (argument == "--joint-stream-joint-names") {
      assignFixedArray(argument,
                       parseStrings(argument,
                                    requireValue(index, argc, argv, argument)),
                       options.robot.joint_stream.joint_names);
    } else if (argument == "--joint-stream-position-lower-rad") {
      assignFixedArray(argument,
                       parseNumbers(argument,
                                    requireValue(index, argc, argv, argument)),
                       options.robot.joint_stream.position_lower_rad);
    } else if (argument == "--joint-stream-position-upper-rad") {
      assignFixedArray(argument,
                       parseNumbers(argument,
                                    requireValue(index, argc, argv, argument)),
                       options.robot.joint_stream.position_upper_rad);
    } else if (argument == "--joint-stream-max-velocity-rad-per-s") {
      assignFixedArray(argument,
                       parseNumbers(argument,
                                    requireValue(index, argc, argv, argument)),
                       options.robot.joint_stream.max_velocity_rad_per_s);
    } else if (argument == "--joint-stream-max-acceleration-rad-per-s2") {
      assignFixedArray(argument,
                       parseNumbers(argument,
                                    requireValue(index, argc, argv, argument)),
                       options.robot.joint_stream.max_acceleration_rad_per_s2);
    } else if (argument == "--joint-stream-max-jerk-rad-per-s3") {
      assignFixedArray(argument,
                       parseNumbers(argument,
                                    requireValue(index, argc, argv, argument)),
                       options.robot.joint_stream.max_jerk_rad_per_s3);
    } else if (argument == "--host") {
      options.visualization.host = requireValue(index, argc, argv, argument);
    } else if (argument == "--port") {
      const long port = std::stol(requireValue(index, argc, argv, argument));
      if (port <= 0 || port > 65535) {
        throw std::runtime_error("port must be in [1, 65535]");
      }
      options.visualization.port = static_cast<std::uint16_t>(port);
    } else if (argument == "--red-rate") {
      options.red_rate_hz = parsePositiveDouble(
          "red rate", requireValue(index, argc, argv, argument));
    } else if (argument == "--yellow-rate") {
      options.yellow_rate_hz = parsePositiveDouble(
          "yellow rate", requireValue(index, argc, argv, argument));
    } else if (argument == "--ui-rate") {
      options.ui_rate_hz = parsePositiveDouble(
          "UI rate", requireValue(index, argc, argv, argument));
    } else if (argument == "--ui") {
      const auto value = requireValue(index, argc, argv, argument);
      if (value == "tui") {
        options.presentation.enabled = true;
      } else if (value == "none") {
        options.presentation.enabled = false;
      } else {
        throw std::runtime_error("ui must be either 'tui' or 'none'");
      }
    } else if (argument == "--viz") {
      const auto value = requireValue(index, argc, argv, argument);
      if (value == "foxglove") {
        options.visualization.enabled = true;
      } else if (value == "none") {
        options.visualization.enabled = false;
      } else {
        throw std::runtime_error("viz must be either 'foxglove' or 'none'");
      }
    } else if (argument == "--deadline-policy") {
      const auto value = requireValue(index, argc, argv, argument);
      if (value == "strict") {
        options.deadline_policy = DeadlinePolicy::Strict;
      } else if (value == "monitor") {
        options.deadline_policy = DeadlinePolicy::Monitor;
      } else {
        throw std::runtime_error(
            "--deadline-policy must be either 'strict' or 'monitor'");
      }
    } else if (argument == "--duration") {
      options.duration_s = std::stod(requireValue(index, argc, argv, argument));
      if (options.duration_s < 0.0 || !std::isfinite(options.duration_s)) {
        throw std::runtime_error("duration must be finite and non-negative");
      }
    } else if (argument == "--step-m") {
      options.tui.step_m = parsePositiveDouble(
          "step", requireValue(index, argc, argv, argument));
    } else if (argument == "--min-step-m") {
      options.tui.min_step_m = parsePositiveDouble(
          "minimum step", requireValue(index, argc, argv, argument));
    } else if (argument == "--max-step-m") {
      options.tui.max_step_m = parsePositiveDouble(
          "maximum step", requireValue(index, argc, argv, argument));
    } else if (argument == "--rotation-step-deg") {
      options.tui.rotation_step_deg = parsePositiveDouble(
          "rotation step", requireValue(index, argc, argv, argument));
    } else if (argument == "--environment-stiffness") {
      options.admittance.linear_environment_stiffness_n_per_m =
          parsePositiveDouble(argument,
                              requireValue(index, argc, argv, argument));
    } else if (argument == "--environment-damping") {
      options.admittance.linear_environment_damping_ns_per_m =
          parsePositiveDouble(argument,
                              requireValue(index, argc, argv, argument));
    } else if (argument == "--maximum-force") {
      options.admittance.maximum_force_n = parsePositiveDouble(
          argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--rotation-environment-stiffness") {
      options.admittance.angular_environment_stiffness_nm_per_rad =
          parsePositiveDouble(argument,
                              requireValue(index, argc, argv, argument));
    } else if (argument == "--rotation-environment-damping") {
      options.admittance.angular_environment_damping_nms_per_rad =
          parsePositiveDouble(argument,
                              requireValue(index, argc, argv, argument));
    } else if (argument == "--maximum-torque") {
      options.admittance.maximum_torque_nm = parsePositiveDouble(
          argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--wrench-filter-alpha") {
      options.admittance.wrench_filter_alpha = parsePositiveDouble(
          argument, requireValue(index, argc, argv, argument));
    } else if (argument ==
               "--yellow-task-posture-preference-joint-weight-multipliers") {
      mergeJointWeightMultipliers(
          options.solver.yellow_task_posture_preference_joint_weight_multipliers,
          parseJointWeightMultipliers(
              argument, requireValue(index, argc, argv, argument)));
    } else if (
        argument ==
        "--red-tertiary-task-yellow-posture-coupling-joint-weight-multipliers") {
      mergeJointWeightMultipliers(
          options.solver
              .red_tertiary_task_yellow_posture_coupling_joint_weight_multipliers,
          parseJointWeightMultipliers(
              argument, requireValue(index, argc, argv, argument)));
    } else if (argument == "--yellow-maximum-iterations") {
      const auto value = std::stoi(requireValue(index, argc, argv, argument));
      if (value <= 0)
        throw std::runtime_error(argument + " must be positive");
      options.solver.yellow_maximum_iterations = value;
    } else if (argument == "--red-proxqp-maximum-iterations") {
      const auto value = std::stoi(requireValue(index, argc, argv, argument));
      if (value <= 0)
        throw std::runtime_error(argument + " must be positive");
      options.solver.red_proxqp_maximum_iterations = value;
    } else if (optionIn(
                   argument,
                   {"--regularization", "--position-tolerance-m",
                    "--orientation-tolerance-rad", "--maximum-hard-violation",
                    "--joint-position-margin-rad",
                    "--minimum-position-improvement-m",
                    "--minimum-orientation-improvement-rad",
                    "--legacy-cartesian-progress-weight",
                    "--legacy-cartesian-preservation-tolerance",
                    "--legacy-scale-preservation-tolerance",
                    "--legacy-posture-preservation-tolerance",
                    "--legacy-yellow-posture-weight",
                    "--legacy-yellow-to-red-coupling-weight",
                    "--legacy-minimum-collision-distance-m",
                    "--legacy-collision-influence-distance-m",
                    "--legacy-collision-damping-gain-per-s",
                    "--legacy-collision-weight",
                    "--red-primary-task-tcp-position-progress-weight",
                    "--red-primary-task-tcp-position-preservation-tolerance-mps",
                    "--red-primary-task-tcp-position-progress-preservation-tolerance",
                    "--red-secondary-task-tcp-orientation-progress-weight",
                    "--red-secondary-task-tcp-orientation-preservation-tolerance-radps",
                    "--red-secondary-task-tcp-orientation-progress-preservation-tolerance",
                    "--red-tertiary-task-yellow-posture-coupling-preservation-tolerance",
                    "--red-tertiary-task-link4-position-weight",
                    "--red-tertiary-task-link4-position-servo-gain-per-s",
                    "--red-tertiary-task-link4-position-preservation-tolerance-mps",
                    "--red-proxqp-absolute-tolerance",
                    "--red-proxqp-primal-infeasibility-tolerance",
                    "--yellow-task-posture-preference-weight",
                    "--yellow-task-posture-preference-servo-gain-per-s",
                    "--red-tertiary-task-yellow-posture-coupling-weight",
                    "--red-tertiary-task-yellow-posture-coupling-servo-gain-per-s",
                    "--yellow-constraints-self-collision-avoidance-minimum-distance-m",
                    "--yellow-constraints-self-collision-avoidance-influence-distance-m",
                    "--yellow-constraints-self-collision-avoidance-damping-gain-per-s",
                    "--yellow-constraints-self-collision-avoidance-weight"})) {
      parseSolverOption(argument, requireValue(index, argc, argv, argument),
                        options.solver);
    } else if (argument == "--mcap") {
      options.visualization.mcap_path =
          std::filesystem::path{requireValue(index, argc, argv, argument)};
    } else if (argument == "--no-mcap") {
      options.visualization.mcap_path.reset();
    } else {
      throw std::runtime_error("unknown option: " + argument);
    }
  }
  validate(options, profile);
  if (options.robot.collision_mesh_search_paths.empty()) {
    options.robot.collision_mesh_search_paths = {
        collisionMeshSearchRoot(options.urdf_path, profile)};
  }
  return options;
}

Options parseOptions(int argc, char **argv) {
  if (argc < 2 || std::string{argv[1]} == "--help" ||
      std::string{argv[1]} == "-h") {
    std::cout << "Usage: " << argv[0]
              << " --profile <profile> <teleop|replay> [options]\n\n"
              << "Profiles:\n"
              << "  hierarchical\n"
              << "  planned\n"
              << "  planned-otg\n"
              << "  planned-otg-nullspace\n"
              << "  planned-otg-nullspace-admittance-kinematic-sim\n\n"
              << "  teleop  Edit source Cartesian goals and replan online "
                 "(default UI: tui)\n"
              << "  replay  Replan paired MCAP/CSV goals; --target-period-ms "
                 "is required\n";
    std::exit(EXIT_SUCCESS);
  }
  if (argc < 4 || std::string{argv[1]} != "--profile") {
    throw std::runtime_error("--profile must be specified before the subcommand");
  }
  const Profile profile = parseProfile(argv[2]);
  Options result = profileDefaults(profile);
  result.binary_argv.assign(argv, argv + argc);
  const std::string mode{argv[3]};
  if (mode != "teleop" && mode != "replay") {
    throw std::runtime_error("expected subcommand 'teleop' or 'replay'");
  }
  result.source_mode =
      mode == "replay" ? SourceMode::Replay : SourceMode::Teleop;
  if (argc >= 5 &&
      (std::string{argv[4]} == "--help" || std::string{argv[4]} == "-h")) {
    printPlannedUsage(argv[0], result.source_mode);
    if (result.source_mode == SourceMode::Replay) {
      std::cout << '\n' << replay::replayHelp(argv[0], true);
    }
    std::exit(EXIT_SUCCESS);
  }

  std::vector<char *> hierarchical_arguments{argv[0]};
  std::vector<char *> replay_arguments{argv[0]};
  bool planning_option_seen = false;
  bool joint_otg_option_seen = false;
  bool nullspace_option_seen = false;
  bool legacy_topology_option_seen = false;
  bool admittance_or_simulation_option_seen = false;
  for (int index = 4; index < argc; ++index) {
    const std::string argument{argv[index]};
    nullspace_option_seen = nullspace_option_seen || optionIn(
        argument,
        {"--red-primary-task-tcp-position-progress-weight",
         "--red-primary-task-tcp-position-preservation-tolerance-mps",
         "--red-primary-task-tcp-position-progress-preservation-tolerance",
         "--red-secondary-task-tcp-orientation-progress-weight",
         "--red-secondary-task-tcp-orientation-preservation-tolerance-radps",
         "--red-secondary-task-tcp-orientation-progress-preservation-tolerance",
         "--red-tertiary-task-yellow-posture-coupling-preservation-tolerance",
         "--red-tertiary-task-link4-position-weight",
         "--red-tertiary-task-link4-position-servo-gain-per-s",
         "--red-tertiary-task-link4-position-preservation-tolerance-mps",
         "--red-tertiary-task-yellow-posture-coupling-weight",
         "--red-tertiary-task-yellow-posture-coupling-servo-gain-per-s",
         "--red-tertiary-task-yellow-posture-coupling-joint-weight-multipliers"});
    legacy_topology_option_seen = legacy_topology_option_seen || optionIn(
        argument,
        {"--legacy-cartesian-progress-weight",
         "--legacy-cartesian-preservation-tolerance",
         "--legacy-scale-preservation-tolerance",
         "--legacy-posture-preservation-tolerance",
         "--legacy-yellow-posture-weight",
         "--legacy-yellow-to-red-coupling-weight",
         "--legacy-minimum-collision-distance-m",
         "--legacy-collision-influence-distance-m",
         "--legacy-collision-damping-gain-per-s",
         "--legacy-collision-weight"});
    auto planningValue = [&](double &destination) {
      destination = parsePositiveDouble(
          argument, requireValue(index, argc, argv, argument));
    };
    if (argument == "--max-linear-velocity-mps") {
      planning_option_seen = true;
      planningValue(result.planning.max_linear_velocity_mps);
    } else if (argument == "--max-linear-acceleration-mps2") {
      planning_option_seen = true;
      planningValue(result.planning.max_linear_acceleration_mps2);
    } else if (argument == "--max-linear-jerk-mps3") {
      planning_option_seen = true;
      planningValue(result.planning.max_linear_jerk_mps3);
    } else if (argument == "--max-angular-velocity-rps") {
      planning_option_seen = true;
      planningValue(result.planning.max_angular_velocity_rps);
    } else if (argument == "--max-angular-acceleration-rps2") {
      planning_option_seen = true;
      planningValue(result.planning.max_angular_acceleration_rps2);
    } else if (argument == "--max-angular-jerk-rps3") {
      planning_option_seen = true;
      planningValue(result.planning.max_angular_jerk_rps3);
    } else if (argument == "--cartesian-synchronization") {
      planning_option_seen = true;
      result.planning.cartesian_synchronization = parsePlanningSynchronization(
          argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--joint-synchronization") {
      joint_otg_option_seen = true;
      result.planning.joint_synchronization = parsePlanningSynchronization(
          argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--joint-algorithm") {
      joint_otg_option_seen = true;
      const auto value = requireValue(index, argc, argv, argument);
      if (value != "jerk-limited") {
        throw std::runtime_error(
            "--joint-algorithm must be 'jerk-limited'");
      }
      result.planning.joint_algorithm = JointPlanningAlgorithm::JerkLimited;
    } else if (argument == "--dump-resolved-options") {
      result.dump_resolved_options = true;
    } else if (argument == "--launcher-argv-json") {
      result.launcher_argv_json = requireValue(index, argc, argv, argument);
    } else if (argument == "--start-paused") {
      if (result.source_mode != SourceMode::Replay) {
        throw std::runtime_error("--start-paused is only valid with replay");
      }
      result.start_paused = true;
    } else if (argument == "--no-start-paused") {
      if (result.source_mode != SourceMode::Replay)
        throw std::runtime_error("--no-start-paused is only valid with replay");
      result.start_paused = false;
    } else if (argument == "--replay-trace" ||
               argument == "--no-replay-trace") {
      if (result.source_mode != SourceMode::Replay) {
        throw std::runtime_error("--replay-trace is only valid with replay");
      }
      result.replay_trace_enabled = argument == "--replay-trace";
    } else if (argument == "--replay-elbow-teleop" ||
               argument == "--no-replay-elbow-teleop") {
      nullspace_option_seen = true;
      if (result.source_mode != SourceMode::Replay) {
        throw std::runtime_error(
            "--replay-elbow-teleop is only valid with replay");
      }
      result.replay_elbow_teleop_enabled =
          argument == "--replay-elbow-teleop";
    } else if (argument == "--terminal-input" ||
               argument == "--no-terminal-input") {
      if (result.source_mode != SourceMode::Replay) {
        throw std::runtime_error("--terminal-input is only valid with replay");
      }
      replay_arguments.push_back(const_cast<char *>("--terminal-input"));
      replay_arguments.push_back(const_cast<char *>(
          argument == "--terminal-input" ? "on" : "off"));
    } else if (argument == "--joint-target-mode") {
      joint_otg_option_seen = true;
      const auto value = requireValue(index, argc, argv, argument);
      if (value == "future-o1-pv") {
        result.joint_target.mode = JointTargetMode::FutureO1Pv;
      } else if (value == "ik-pv") {
        result.joint_target.mode = JointTargetMode::IkPv;
      } else {
        throw std::runtime_error(
            "--joint-target-mode must be either 'future-o1-pv' or 'ik-pv'");
      }
    } else if (argument == "--future-o1-velocity-deadband-rad-per-s") {
      joint_otg_option_seen = true;
      result.joint_target.future_o1_velocity_deadband_rad_per_s =
          parseNonnegativeDouble(
              argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--settling-fk-position-m") {
      joint_otg_option_seen = true;
      result.replay_settling.fk_position_m = parsePositiveDouble(
          argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--settling-fk-orientation-rad") {
      joint_otg_option_seen = true;
      result.replay_settling.fk_orientation_rad = parsePositiveDouble(
          argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--settling-velocity-rad-per-s") {
      joint_otg_option_seen = true;
      result.replay_settling.velocity_rad_per_s = parsePositiveDouble(
          argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--settling-acceleration-rad-per-s2") {
      joint_otg_option_seen = true;
      result.replay_settling.acceleration_rad_per_s2 = parsePositiveDouble(
          argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--settling-required-cycles") {
      joint_otg_option_seen = true;
      const auto value =
          std::stoull(requireValue(index, argc, argv, argument));
      if (value == 0U)
        throw std::runtime_error(argument + " must be positive");
      result.replay_settling.required_cycles =
          static_cast<std::size_t>(value);
    } else if (result.source_mode == SourceMode::Teleop) {
      admittance_or_simulation_option_seen =
          admittance_or_simulation_option_seen ||
          optionIn(argument,
                   {"--mujoco-model", "--mujoco-viewer",
                    "--no-mujoco-viewer", "--angular-admittance",
                    "--no-angular-admittance", "--environment-stiffness",
                    "--environment-damping", "--maximum-force",
                    "--rotation-environment-stiffness",
                    "--rotation-environment-damping", "--maximum-torque",
                    "--wrench-filter-alpha"});
      hierarchical_arguments.push_back(argv[index]);
    } else {
      if (optionIn(argument,
                   {"--mujoco-viewer", "--no-mujoco-viewer",
                    "--angular-admittance", "--no-angular-admittance",
                    "--joint-position-braking-velocity-envelope",
                    "--no-joint-position-braking-velocity-envelope",
                    "--red-joint-acceleration-limits",
                    "--no-red-joint-acceleration-limits",
                    "--red-proxqp-warm-start",
                    "--no-red-proxqp-warm-start"})) {
        admittance_or_simulation_option_seen =
            admittance_or_simulation_option_seen ||
            optionIn(argument,
                     {"--mujoco-viewer", "--no-mujoco-viewer",
                      "--angular-admittance", "--no-angular-admittance"});
        hierarchical_arguments.push_back(argv[index]);
        continue;
      }
      const bool shared_value = optionIn(
          argument, {"--urdf", "--ui", "--viz", "--host", "--port", "--mcap"});
      const bool hierarchical_value = optionIn(
          argument,
          {"--mujoco-model", "--environment-stiffness",
           "--environment-damping", "--maximum-force",
           "--rotation-environment-stiffness",
           "--rotation-environment-damping", "--maximum-torque",
           "--wrench-filter-alpha", "--red-rate", "--yellow-rate",
           "--ui-rate", "--deadline-policy", "--joint-algorithm",
           "--duration", "--regularization", "--position-tolerance-m",
           "--orientation-tolerance-rad", "--maximum-hard-violation",
           "--joint-position-margin-rad",
           "--minimum-position-improvement-m",
           "--minimum-orientation-improvement-rad",
           "--legacy-cartesian-progress-weight",
           "--legacy-cartesian-preservation-tolerance",
           "--legacy-scale-preservation-tolerance",
           "--legacy-posture-preservation-tolerance",
           "--legacy-yellow-posture-weight",
           "--legacy-yellow-to-red-coupling-weight",
           "--legacy-minimum-collision-distance-m",
           "--legacy-collision-influence-distance-m",
           "--legacy-collision-damping-gain-per-s",
           "--legacy-collision-weight",
           "--red-primary-task-tcp-position-progress-weight",
           "--red-primary-task-tcp-position-preservation-tolerance-mps",
           "--red-primary-task-tcp-position-progress-preservation-tolerance",
           "--red-secondary-task-tcp-orientation-progress-weight",
           "--red-secondary-task-tcp-orientation-preservation-tolerance-radps",
           "--red-secondary-task-tcp-orientation-progress-preservation-tolerance",
           "--red-tertiary-task-yellow-posture-coupling-preservation-tolerance",
           "--red-tertiary-task-link4-position-weight",
           "--red-tertiary-task-link4-position-servo-gain-per-s",
           "--red-tertiary-task-link4-position-preservation-tolerance-mps",
           "--yellow-maximum-iterations",
           "--red-proxqp-maximum-iterations",
           "--red-proxqp-absolute-tolerance",
           "--red-proxqp-primal-infeasibility-tolerance",
           "--yellow-task-posture-preference-weight",
           "--yellow-task-posture-preference-servo-gain-per-s",
           "--yellow-task-posture-preference-joint-weight-multipliers",
           "--red-tertiary-task-yellow-posture-coupling-weight",
           "--red-tertiary-task-yellow-posture-coupling-servo-gain-per-s",
           "--red-tertiary-task-yellow-posture-coupling-joint-weight-multipliers",
           "--yellow-constraints-self-collision-avoidance-minimum-distance-m",
           "--yellow-constraints-self-collision-avoidance-influence-distance-m",
           "--yellow-constraints-self-collision-avoidance-damping-gain-per-s",
           "--yellow-constraints-self-collision-avoidance-weight",
           "--base-frame", "--left-end-effector-frame",
           "--right-end-effector-frame", "--left-link4-frame",
           "--right-link4-frame", "--left-tcp-offset",
           "--right-tcp-offset", "--joint-names",
           "--default-joint-positions", "--left-arm-joint-indices",
           "--right-arm-joint-indices", "--effort-limits",
           "--inactive-joints", "--self-collision-pair",
           "--collision-mesh-search-paths",
           "--joint-stream-source-revision", "--joint-stream-source-path",
           "--joint-stream-source-sha256",
           "--joint-stream-jerk-override-reason",
           "--joint-stream-joint-names",
           "--joint-stream-position-lower-rad",
           "--joint-stream-position-upper-rad",
           "--joint-stream-max-velocity-rad-per-s",
           "--joint-stream-max-acceleration-rad-per-s2",
           "--joint-stream-max-jerk-rad-per-s3"});
      admittance_or_simulation_option_seen =
          admittance_or_simulation_option_seen ||
          optionIn(argument,
                   {"--mujoco-model", "--environment-stiffness",
                    "--environment-damping", "--maximum-force",
                    "--rotation-environment-stiffness",
                    "--rotation-environment-damping", "--maximum-torque",
                    "--wrench-filter-alpha"});
      const bool replay_value =
          optionIn(argument, {"--input",
                              "--input-format",
                              "--left-stream",
                              "--right-stream",
                              "--initial-joint-state-stream",
                              "--csv-mapping",
                              "--timestamp-source",
                              "--target-period-ms",
                              "--pairing-policy",
                              "--nearest-tolerance-ms",
                              "--unmatched-policy",
                              "--execution-mode",
                              "--playback-rate",
                              "--output-dir",
                              "--output-root",
                              "--run-id",
                              "--viz-host",
                              "--viz-port",
                              "--terminal-input",
                              "--launcher"});
      if (argument == "--no-mcap") {
        hierarchical_arguments.push_back(argv[index]);
        replay_arguments.push_back(argv[index]);
        continue;
      }
      if (!shared_value && !hierarchical_value && !replay_value) {
        throw std::runtime_error("unknown option: " + argument);
      }
      if (index + 1 >= argc) {
        throw std::runtime_error(argument + " requires a value");
      }
      if (shared_value || hierarchical_value) {
        hierarchical_arguments.push_back(argv[index]);
        hierarchical_arguments.push_back(argv[index + 1]);
      }
      if (shared_value || replay_value) {
        replay_arguments.push_back(argv[index]);
        replay_arguments.push_back(argv[index + 1]);
      }
      ++index;
    }
  }
  result.interactive = parseHierarchicalOptions(
      static_cast<int>(hierarchical_arguments.size()),
      hierarchical_arguments.data(), std::move(result.interactive), profile);

  const auto capabilities = profileCapabilities(profile);
  if (planning_option_seen && !capabilities.cartesian_planning) {
    throw std::runtime_error("Cartesian planning options are not valid for profile " +
                             std::string{profileName(profile)});
  }
  if (joint_otg_option_seen && !capabilities.joint_otg) {
    throw std::runtime_error("joint OTG options are not valid for profile " +
                             std::string{profileName(profile)});
  }
  if (nullspace_option_seen && !capabilities.nullspace) {
    throw std::runtime_error("null-space options are not valid for profile " +
                             std::string{profileName(profile)});
  }
  if (legacy_topology_option_seen && capabilities.nullspace) {
    throw std::runtime_error("legacy topology options are not valid for profile " +
                             std::string{profileName(profile)});
  }
  if (admittance_or_simulation_option_seen && !capabilities.admittance) {
    throw std::runtime_error(
        "admittance/MuJoCo options are not valid for profile " +
        std::string{profileName(profile)});
  }
  if (result.source_mode == SourceMode::Replay) {
    result.replay =
        replay::parseReplayOptions(static_cast<int>(replay_arguments.size()),
                                   replay_arguments.data(), true);
    result.replay->original_argv.assign(argv, argv + argc);
    result.interactive.visualization.enabled =
        result.replay->visualization_enabled;
    if (result.start_paused && !result.replay->terminal_input_enabled) {
      throw std::runtime_error("--start-paused requires --terminal-input on");
    }
    if (result.replay_elbow_teleop_enabled &&
        !result.replay->terminal_input_enabled) {
      throw std::runtime_error(
          "--replay-elbow-teleop on requires --terminal-input on");
    }
    if (result.replay_elbow_teleop_enabled &&
        result.replay->execution_mode != data::ExecutionMode::Realtime) {
      throw std::runtime_error(
          "--replay-elbow-teleop on requires --execution-mode realtime");
    }
  }
  return result;
}

} // namespace motion_control_lab::hierarchical_kinematics_step
