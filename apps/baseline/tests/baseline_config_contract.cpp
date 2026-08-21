#include "solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "motion_control_lab/sha256.hpp"

namespace baseline = motion_control_lab::baseline;

namespace
{

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void requireNear(double actual, double expected, const std::string & message)
{
  require(std::fabs(actual - expected) <= 1.0e-15, message);
}

void requireVectorNear(const std::vector<double> & actual, const std::vector<double> & expected,
                       const std::string & message)
{
  require(actual.size() == expected.size(), message + " size mismatch");
  for (std::size_t index = 0; index < expected.size(); ++index) {
    requireNear(actual[index], expected[index], message + " at index " + std::to_string(index));
  }
}

const baseline::TaskSpec & requireTask(const baseline::ProductionStaticConfig & config,
                                       const std::string & name, const std::string & type,
                                       const std::string & priority, double weight,
                                       const std::vector<std::string> & joints = {})
{
  const auto & task = baseline::taskSpec(config, name);
  require(task.type == type, name + " type mismatch");
  require(task.priority == priority, name + " priority mismatch");
  requireNear(task.weight, weight, name + " weight mismatch");
  require(task.joints == joints, name + " joints mismatch");
  return task;
}

} // namespace

int main()
{
  try {
    const auto & config = baseline::productionStaticConfig();
    require(config.schema_version == "mcl.placo_baseline_config.v1", "schema mismatch");
    require(config.source_revision == "42ed3ce3a19f5a7346874a31ec659c0298751137",
            "source revision mismatch");
    require(config.source_digests.size() == 4U, "source digest count mismatch");
    require(config.source_digests[0].path == "config/profiles/teleop.yaml",
            "teleop source path mismatch");
    require(config.source_digests[0].sha256 ==
                "9d8a98d4567d7e971dc0cc64ca6c31c6aef6fdfa1000f3ea371374a4cb4e49a7",
            "teleop digest mismatch");
    require(config.source_digests[1].sha256 ==
                "895416681f8fd41138f3be280b0f7a330ca004f46951b12b3fea2a230e779d1b",
            "robot digest mismatch");
    require(config.source_digests[1].path == "config/robots/psi_r1.yaml",
            "robot source path mismatch");
    require(config.source_digests[2].sha256 ==
                "b3797327aeb7994766a56a2b8f0ce1fcc6cd4e49974648edecb756599f6b9482",
            "solver source digest mismatch");
    require(config.source_digests[2].path == "src/placo_kinematics_solver.cpp",
            "solver source path mismatch");
    require(config.source_digests[3].sha256 ==
                "3ef3859a048227758c6c02e5c95e694fb83a1b0514d967d87d0054824722afc6",
            "solver header digest mismatch");
    require(config.source_digests[3].path == "include/motion_control/placo_kinematics_solver.hpp",
            "solver header path mismatch");
    require(config.base_frame == "base_link", "base frame mismatch");
    require(config.left_end_effector_frame == "left_arm_ee_link", "left EE frame mismatch");
    require(config.right_end_effector_frame == "right_arm_ee_link", "right EE frame mismatch");
    requireVectorNear(config.left_tcp_offset_xyz, {0.0, 0.0, 0.1}, "left TCP offset mismatch");
    requireVectorNear(config.right_tcp_offset_xyz, {0.0, 0.0, 0.1}, "right TCP offset mismatch");
    require(
        config.joint_names ==
            std::vector<std::string>{
                "head_yaw_joint",   "head_pitch_joint",  "torso_yaw_joint",  "torso_pitch_joint",
                "knee_pitch_joint", "ankle_pitch_joint", "left_arm_joint1",  "left_arm_joint2",
                "left_arm_joint3",  "left_arm_joint4",   "left_arm_joint5",  "left_arm_joint6",
                "left_arm_joint7",  "right_arm_joint1",  "right_arm_joint2", "right_arm_joint3",
                "right_arm_joint4", "right_arm_joint5",  "right_arm_joint6", "right_arm_joint7"},
        "full joint order mismatch");
    require(config.active_joint_names ==
                std::vector<std::string>{
                    "torso_yaw_joint", "torso_pitch_joint", "left_arm_joint1", "left_arm_joint2",
                    "left_arm_joint3", "left_arm_joint4", "left_arm_joint5", "left_arm_joint6",
                    "left_arm_joint7", "right_arm_joint1", "right_arm_joint2", "right_arm_joint3",
                    "right_arm_joint4", "right_arm_joint5", "right_arm_joint6", "right_arm_joint7"},
            "active joint order mismatch");
    require(config.masked_joint_names ==
                std::vector<std::string>{"head_yaw_joint", "head_pitch_joint", "ankle_pitch_joint",
                                         "knee_pitch_joint"},
            "masked joints mismatch");
    requireVectorNear(config.initial_positions,
                      {0.0,    0.305, 0.0, 0.289,  0.05,  -0.05, 0.925, -1.448, -1.483, -1.401,
                       -1.175, 0.0,   0.0, -0.925, 1.448, 1.483, 1.401, 1.175,  0.0,    0.0},
                      "initial pose mismatch");
    requireVectorNear(config.lower_limits,
                      {-3.0543, -1.9199, -3.0543, -1.9199, 0.0,   -1.0821, -3.1067,
                       -2.0944, -3.1067, -2.5307, -3.1067, -0.95, -0.95,   -3.1067,
                       -2.0944, -3.1067, -1.0472, -3.1067, -0.95, -0.95},
                      "lower limits mismatch");
    requireVectorNear(config.upper_limits, {3.0543, 1.117,  3.0543, 1.117,  2.6529, 0.0,  3.1067,
                                            2.0944, 3.1067, 1.0472, 3.1067, 0.95,   0.95, 3.1067,
                                            2.0944, 3.1067, 2.5307, 3.1067, 0.95,   0.95},
                      "upper limits mismatch");
    requireVectorNear(config.posture_joint_weights,
                      {0.0, 0.0, 0.9, 1.0, 3.0, 1.0, 0.6, 0.6, 0.4, 0.9,
                       0.1, 0.4, 0.4, 0.6, 0.6, 0.4, 0.9, 0.1, 0.4, 0.4},
                      "posture weights mismatch");
    requireNear(config.posture_profile_default_weight, 0.5,
                "posture profile default weight mismatch");
    requireNear(config.soft_limit_margin, 0.08, "joint limit margin mismatch");
    requireNear(baseline::limitedLower(config, 2), -2.9743, "limited lower mismatch");
    requireNear(baseline::limitedUpper(config, 2), 2.9743, "limited upper mismatch");
    require(config.use_sparsity, "sparsity must be enabled");
    require(config.rewrite_equalities, "equality rewriting must be enabled");
    requireNear(config.problem_regularization, 1.0e-8, "QP regularization mismatch");
    requireNear(config.solver_dt, 1.0, "solver dt mismatch");
    requireNear(config.control_rate_hz, 100.0, "control rate mismatch");
    require(config.maximum_iterations == 20, "iteration limit mismatch");
    requireNear(config.soft_solve_time_budget_ms, 8.0, "time budget mismatch");
    requireNear(config.position_tolerance_m, 5.0e-5, "position tolerance mismatch");
    requireNear(config.orientation_tolerance_rad, 5.0e-5, "orientation tolerance mismatch");
    requireNear(config.minimum_position_improvement_m, 1.0e-5, "position improvement mismatch");
    requireNear(config.minimum_orientation_improvement_rad, 1.0e-4,
                "orientation improvement mismatch");

    require(config.tasks.size() == 18U, "task count mismatch");
    require(std::all_of(config.tasks.begin(), config.tasks.end(),
                        [](const auto & task) { return task.weight > 0.0; }),
            "zero-weight placeholder task was registered");
    requireTask(config, "internal_regularization", "RegularizationTask", "soft", 1.0e-6);
    requireTask(config, "left_frame_position", "PositionTask", "scaled", 9.0);
    requireTask(config, "right_frame_position", "PositionTask", "scaled", 9.0);
    requireTask(config, "left_frame_orientation", "OrientationTask", "soft", 4.0);
    requireTask(config, "right_frame_orientation", "OrientationTask", "soft", 4.0);
    requireTask(config, "posture_left_wrist_0", "JointsTask", "soft", 0.1, {"left_arm_joint5"});
    requireTask(config, "posture_right_wrist_1", "JointsTask", "soft", 0.1, {"right_arm_joint5"});
    requireTask(config, "posture_left_shoulder_2", "JointsTask", "soft", 0.4, {"left_arm_joint3"});
    requireTask(config, "posture_left_wrist_3", "JointsTask", "soft", 0.4,
                {"left_arm_joint6", "left_arm_joint7"});
    requireTask(config, "posture_right_shoulder_4", "JointsTask", "soft", 0.4,
                {"right_arm_joint3"});
    requireTask(config, "posture_right_wrist_5", "JointsTask", "soft", 0.4,
                {"right_arm_joint6", "right_arm_joint7"});
    requireTask(config, "posture_left_shoulder_6", "JointsTask", "soft", 0.6,
                {"left_arm_joint1", "left_arm_joint2"});
    requireTask(config, "posture_right_shoulder_7", "JointsTask", "soft", 0.6,
                {"right_arm_joint1", "right_arm_joint2"});
    requireTask(config, "posture_torso_8", "JointsTask", "soft", 0.9, {"torso_yaw_joint"});
    requireTask(config, "posture_left_elbow_9", "JointsTask", "soft", 0.9, {"left_arm_joint4"});
    requireTask(config, "posture_right_elbow_10", "JointsTask", "soft", 0.9, {"right_arm_joint4"});
    requireTask(config, "posture_torso_11", "JointsTask", "soft", 1.0, {"torso_pitch_joint"});
    requireTask(config, "kinetic_energy", "KineticEnergyRegularizationTask", "soft", 0.1);

    require(config.constraints.size() == 1U, "constraint count mismatch");
    const auto & limits = baseline::constraintSpec(config, "absolute_joint_limits");
    require(limits.type == "AbsoluteJointLimitsConstraint", "limit constraint type mismatch");
    require(limits.priority == "hard", "limit constraint priority mismatch");
    requireNear(limits.weight, 1.0, "limit constraint weight mismatch");
    require(limits.joints == config.active_joint_names, "limit constraint active joints mismatch");

    for (const auto & disabled : std::vector<std::string>{
             "adaptive_reach_weight", "joint_continuity", "elbow_pole", "self_collision_avoidance",
             "manipulability", "waist_yaw_follow"}) {
      require(std::find(config.disabled_features.begin(), config.disabled_features.end(),
                        disabled) != config.disabled_features.end(),
              disabled + " is not explicitly disabled");
      require(std::none_of(
                  config.tasks.begin(), config.tasks.end(),
                  [&](const auto & task) { return task.name.find(disabled) != std::string::npos; }),
              disabled + " task was registered");
    }

    const auto json = baseline::productionStaticConfigJson();
    require(json["tasks"].size() == 18U, "JSON task count mismatch");
    require(json["constraints"].size() == 1U, "JSON constraint count mismatch");
    require(!json["solver"]["native_joint_limits_enabled"].asBool(),
            "native PlaCo joint limits must be disabled");
    require(!json["solver"]["velocity_limits_enabled"].asBool(),
            "PlaCo velocity limits must be disabled");
    require(json["posture"]["joint_weights_replace_default"].asBool(),
            "posture joint weights must replace, not multiply, the profile default");
    const auto text = baseline::productionStaticConfigJsonText();
    require(!motion_control_lab::sha256_text(text).empty(), "config SHA-256 is empty");
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "baseline_config_contract: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
