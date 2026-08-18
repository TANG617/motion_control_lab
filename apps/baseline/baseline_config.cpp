#include "baseline_config.hpp"

#include <algorithm>
#include <stdexcept>

namespace motion_control_lab::baseline
{
namespace
{

Json::Value stringArray(const std::vector<std::string> & values)
{
  Json::Value result{Json::arrayValue};
  for (const auto & value : values) {
    result.append(value);
  }
  return result;
}

Json::Value doubleArray(const std::vector<double> & values)
{
  Json::Value result{Json::arrayValue};
  for (const double value : values) {
    result.append(value);
  }
  return result;
}

Json::Value taskJson(const TaskSpec & task)
{
  Json::Value result;
  result["name"] = task.name;
  result["type"] = task.type;
  result["priority"] = task.priority;
  result["weight"] = task.weight;
  result["joints"] = stringArray(task.joints);
  return result;
}

Json::Value constraintJson(const ConstraintSpec & constraint)
{
  Json::Value result;
  result["name"] = constraint.name;
  result["type"] = constraint.type;
  result["priority"] = constraint.priority;
  result["weight"] = constraint.weight;
  result["joints"] = stringArray(constraint.joints);
  return result;
}

} // namespace

const ProductionStaticConfig & productionStaticConfig()
{
  static const ProductionStaticConfig config{
      "mcl.placo_baseline_config.v1",
      "42ed3ce3a19f5a7346874a31ec659c0298751137",
      {
          {"config/profiles/teleop.yaml",
           "9d8a98d4567d7e971dc0cc64ca6c31c6aef6fdfa1000f3ea371374a4cb4e49a7"},
          {"config/robots/psi_r1.yaml",
           "895416681f8fd41138f3be280b0f7a330ca004f46951b12b3fea2a230e779d1b"},
          {"src/placo_kinematics_solver.cpp",
           "b3797327aeb7994766a56a2b8f0ce1fcc6cd4e49974648edecb756599f6b9482"},
          {"include/motion_control/placo_kinematics_solver.hpp",
           "3ef3859a048227758c6c02e5c95e694fb83a1b0514d967d87d0054824722afc6"},
      },
      "base_link",
      "left_arm_ee_link",
      "right_arm_ee_link",
      {0.0, 0.0, 0.1},
      {0.0, 0.0, 0.1},
      {
          "head_yaw_joint",   "head_pitch_joint",  "torso_yaw_joint",  "torso_pitch_joint",
          "knee_pitch_joint", "ankle_pitch_joint", "left_arm_joint1",  "left_arm_joint2",
          "left_arm_joint3",  "left_arm_joint4",   "left_arm_joint5",  "left_arm_joint6",
          "left_arm_joint7",  "right_arm_joint1",  "right_arm_joint2", "right_arm_joint3",
          "right_arm_joint4", "right_arm_joint5",  "right_arm_joint6", "right_arm_joint7",
      },
      {
          "torso_yaw_joint",
          "torso_pitch_joint",
          "left_arm_joint1",
          "left_arm_joint2",
          "left_arm_joint3",
          "left_arm_joint4",
          "left_arm_joint5",
          "left_arm_joint6",
          "left_arm_joint7",
          "right_arm_joint1",
          "right_arm_joint2",
          "right_arm_joint3",
          "right_arm_joint4",
          "right_arm_joint5",
          "right_arm_joint6",
          "right_arm_joint7",
      },
      {"head_yaw_joint", "head_pitch_joint", "ankle_pitch_joint", "knee_pitch_joint"},
      {
          0.0,    0.305, 0.0, 0.289,  0.05,  -0.05, 0.925, -1.448, -1.483, -1.401,
          -1.175, 0.0,   0.0, -0.925, 1.448, 1.483, 1.401, 1.175,  0.0,    0.0,
      },
      {
          -3.0543, -1.9199, -3.0543, -1.9199, 0.0,     -1.0821, -3.1067, -2.0944, -3.1067, -2.5307,
          -3.1067, -0.95,   -0.95,   -3.1067, -2.0944, -3.1067, -1.0472, -3.1067, -0.95,   -0.95,
      },
      {
          3.0543, 1.117, 3.0543, 1.117,  2.6529, 0.0,    3.1067, 2.0944, 3.1067, 1.0472,
          3.1067, 0.95,  0.95,   3.1067, 2.0944, 3.1067, 2.5307, 3.1067, 0.95,   0.95,
      },
      {
          1.0,
          1.0,
          3.0,
          2.0,
          1.5,
          1.5,
          3.0543261909900763,
          3.0543261909900763,
          4.71238898038469,
          5.235987755982989,
          4.1887902047863905,
          4.1887902047863905,
          4.1887902047863905,
          3.0543261909900763,
          3.0543261909900763,
          4.71238898038469,
          5.235987755982989,
          4.1887902047863905,
          4.1887902047863905,
          4.1887902047863905,
      },
      {
          0.0, 0.0, 0.9, 1.0, 3.0, 1.0, 0.6, 0.6, 0.4, 0.9,
          0.1, 0.4, 0.4, 0.6, 0.6, 0.4, 0.9, 0.1, 0.4, 0.4,
      },
      0.5,
      true,
      true,
      1.0e-8,
      1.0,
      100.0,
      0.01,
      0.08,
      20,
      8.0,
      5.0e-5,
      5.0e-5,
      1.0e-5,
      1.0e-4,
      {
          {"internal_regularization", "RegularizationTask", "soft", 1.0e-6, {}},
          {"left_frame_position", "PositionTask", "scaled", 9.0, {}},
          {"left_frame_orientation", "OrientationTask", "soft", 4.0, {}},
          {"right_frame_position", "PositionTask", "scaled", 9.0, {}},
          {"right_frame_orientation", "OrientationTask", "soft", 4.0, {}},
          {"posture_left_wrist_0", "JointsTask", "soft", 0.1, {"left_arm_joint5"}},
          {"posture_right_wrist_1", "JointsTask", "soft", 0.1, {"right_arm_joint5"}},
          {"posture_left_shoulder_2", "JointsTask", "soft", 0.4, {"left_arm_joint3"}},
          {"posture_left_wrist_3",
           "JointsTask",
           "soft",
           0.4,
           {"left_arm_joint6", "left_arm_joint7"}},
          {"posture_right_shoulder_4", "JointsTask", "soft", 0.4, {"right_arm_joint3"}},
          {"posture_right_wrist_5",
           "JointsTask",
           "soft",
           0.4,
           {"right_arm_joint6", "right_arm_joint7"}},
          {"posture_left_shoulder_6",
           "JointsTask",
           "soft",
           0.6,
           {"left_arm_joint1", "left_arm_joint2"}},
          {"posture_right_shoulder_7",
           "JointsTask",
           "soft",
           0.6,
           {"right_arm_joint1", "right_arm_joint2"}},
          {"posture_torso_8", "JointsTask", "soft", 0.9, {"torso_yaw_joint"}},
          {"posture_left_elbow_9", "JointsTask", "soft", 0.9, {"left_arm_joint4"}},
          {"posture_right_elbow_10", "JointsTask", "soft", 0.9, {"right_arm_joint4"}},
          {"posture_torso_11", "JointsTask", "soft", 1.0, {"torso_pitch_joint"}},
          {"kinetic_energy", "KineticEnergyRegularizationTask", "soft", 0.1, {}},
      },
      {
          {"absolute_joint_limits",
           "AbsoluteJointLimitsConstraint",
           "hard",
           1.0,
           {
               "torso_yaw_joint",
               "torso_pitch_joint",
               "left_arm_joint1",
               "left_arm_joint2",
               "left_arm_joint3",
               "left_arm_joint4",
               "left_arm_joint5",
               "left_arm_joint6",
               "left_arm_joint7",
               "right_arm_joint1",
               "right_arm_joint2",
               "right_arm_joint3",
               "right_arm_joint4",
               "right_arm_joint5",
               "right_arm_joint6",
               "right_arm_joint7",
           }},
      },
      {
          "adaptive_reach_weight",
          "joint_continuity",
          "elbow_pole",
          "self_collision_avoidance",
          "adaptive_self_collision",
          "manipulability",
          "waist_yaw_follow",
          "velocity_limits",
          "native_placo_joint_limits",
      },
  };
  return config;
}

std::size_t jointIndex(const ProductionStaticConfig & config, const std::string & joint_name)
{
  const auto found = std::find(config.joint_names.begin(), config.joint_names.end(), joint_name);
  if (found == config.joint_names.end()) {
    throw std::runtime_error("unknown production baseline joint: " + joint_name);
  }
  return static_cast<std::size_t>(std::distance(config.joint_names.begin(), found));
}

bool isActiveJoint(const ProductionStaticConfig & config, const std::string & joint_name)
{
  return std::find(config.active_joint_names.begin(), config.active_joint_names.end(),
                   joint_name) != config.active_joint_names.end();
}

double limitedLower(const ProductionStaticConfig & config, std::size_t joint_index)
{
  const double lower = config.lower_limits.at(joint_index);
  const double upper = config.upper_limits.at(joint_index);
  return lower + config.soft_limit_margin < upper - config.soft_limit_margin
             ? lower + config.soft_limit_margin
             : lower;
}

double limitedUpper(const ProductionStaticConfig & config, std::size_t joint_index)
{
  const double lower = config.lower_limits.at(joint_index);
  const double upper = config.upper_limits.at(joint_index);
  return lower + config.soft_limit_margin < upper - config.soft_limit_margin
             ? upper - config.soft_limit_margin
             : upper;
}

const TaskSpec & taskSpec(const ProductionStaticConfig & config, const std::string & name)
{
  const auto found = std::find_if(config.tasks.begin(), config.tasks.end(),
                                  [&](const TaskSpec & task) { return task.name == name; });
  if (found == config.tasks.end()) {
    throw std::runtime_error("unknown production baseline task: " + name);
  }
  return *found;
}

const ConstraintSpec & constraintSpec(const ProductionStaticConfig & config,
                                      const std::string & name)
{
  const auto found =
      std::find_if(config.constraints.begin(), config.constraints.end(),
                   [&](const ConstraintSpec & constraint) { return constraint.name == name; });
  if (found == config.constraints.end()) {
    throw std::runtime_error("unknown production baseline constraint: " + name);
  }
  return *found;
}

Json::Value productionStaticConfigJson()
{
  const auto & config = productionStaticConfig();
  Json::Value root;
  root["schema_version"] = config.schema_version;
  root["source"]["revision"] = config.source_revision;
  for (const auto & digest : config.source_digests) {
    Json::Value source;
    source["path"] = digest.path;
    source["sha256"] = digest.sha256;
    root["source"]["files"].append(source);
  }

  root["robot"]["base_frame"] = config.base_frame;
  root["robot"]["left_end_effector_frame"] = config.left_end_effector_frame;
  root["robot"]["right_end_effector_frame"] = config.right_end_effector_frame;
  root["robot"]["left_tcp_offset_xyz"] = doubleArray(config.left_tcp_offset_xyz);
  root["robot"]["right_tcp_offset_xyz"] = doubleArray(config.right_tcp_offset_xyz);
  root["robot"]["joint_names"] = stringArray(config.joint_names);
  root["robot"]["active_joint_names"] = stringArray(config.active_joint_names);
  root["robot"]["masked_joint_names"] = stringArray(config.masked_joint_names);
  root["robot"]["initial_positions"] = doubleArray(config.initial_positions);
  root["robot"]["lower_limits"] = doubleArray(config.lower_limits);
  root["robot"]["upper_limits"] = doubleArray(config.upper_limits);
  root["robot"]["velocity_limits"] = doubleArray(config.velocity_limits);
  root["robot"]["limited_lower"] = Json::Value{Json::arrayValue};
  root["robot"]["limited_upper"] = Json::Value{Json::arrayValue};
  for (std::size_t index = 0; index < config.joint_names.size(); ++index) {
    root["robot"]["limited_lower"].append(limitedLower(config, index));
    root["robot"]["limited_upper"].append(limitedUpper(config, index));
  }

  root["solver"]["mode"] = "target_solve";
  root["solver"]["backend"] = "eiquadprog";
  root["solver"]["use_sparsity"] = config.use_sparsity;
  root["solver"]["rewrite_equalities"] = config.rewrite_equalities;
  root["solver"]["problem_regularization"] = config.problem_regularization;
  root["solver"]["dt"] = config.solver_dt;
  root["solver"]["control_rate_hz"] = config.control_rate_hz;
  root["solver"]["control_dt_s"] = config.control_dt_s;
  root["solver"]["soft_limit_margin"] = config.soft_limit_margin;
  root["solver"]["native_joint_limits_enabled"] = false;
  root["solver"]["velocity_limits_enabled"] = false;
  root["solver"]["maximum_iterations"] = config.maximum_iterations;
  root["solver"]["soft_solve_time_budget_ms"] = config.soft_solve_time_budget_ms;
  root["solver"]["position_tolerance_m"] = config.position_tolerance_m;
  root["solver"]["orientation_tolerance_rad"] = config.orientation_tolerance_rad;
  root["solver"]["minimum_position_improvement_m"] = config.minimum_position_improvement_m;
  root["solver"]["minimum_orientation_improvement_rad"] =
      config.minimum_orientation_improvement_rad;

  root["posture"]["profile_default_weight"] = config.posture_profile_default_weight;
  root["posture"]["joint_weights_replace_default"] = true;
  root["posture"]["joint_weights"] = doubleArray(config.posture_joint_weights);

  root["tasks"] = Json::Value{Json::arrayValue};
  for (const auto & task : config.tasks) {
    root["tasks"].append(taskJson(task));
  }
  root["constraints"] = Json::Value{Json::arrayValue};
  for (const auto & constraint : config.constraints) {
    root["constraints"].append(constraintJson(constraint));
  }
  root["disabled_features"] = stringArray(config.disabled_features);
  return root;
}

std::string productionStaticConfigJsonText()
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  return Json::writeString(builder, productionStaticConfigJson()) + "\n";
}

} // namespace motion_control_lab::baseline
