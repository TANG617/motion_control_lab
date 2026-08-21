#include "../planning.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace app = motion_control_lab::planned_grouped_step_otg;

namespace {

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void requireNear(double actual, double expected, const std::string &message,
                 double tolerance = 1.0e-12) {
  require(std::abs(actual - expected) <= tolerance, message);
}

void testFutureO1StartupFormulaAffineAndDeadband() {
  app::JointTargetBuilder builder(app::JointTargetMode::FutureO1Pv, 0.001, 2U);
  const std::vector<double> unused_velocity{8.0, -8.0};
  auto target = builder.preview({1.0, -2.0}, unused_velocity);
  require(target.future_o1_startup && target.positions[0] == 1.0 &&
              target.velocities[0] == 0.0,
          "first startup sample mismatch");
  builder.commit({1.0, -2.0}, target);

  target = builder.preview({1.002, -1.996}, unused_velocity);
  require(target.future_o1_startup && target.positions[1] == -1.996 &&
              target.velocities[1] == 0.0,
          "second startup sample mismatch");
  builder.commit({1.002, -1.996}, target);

  target = builder.preview({1.004, -1.992}, unused_velocity);
  require(!target.future_o1_startup, "mature sample reported startup");
  requireNear(target.positions[0], 1.006, "Future-O1 position formula mismatch");
  requireNear(target.positions[1], -1.988, "Future-O1 position formula mismatch");
  requireNear(target.velocities[0], 2.0, "Future-O1 affine velocity mismatch");
  requireNear(target.velocities[1], 4.0, "Future-O1 affine velocity mismatch");
  builder.commit({1.004, -1.992}, target);

  const auto deadbanded = builder.preview(
      {1.006 + 2.0e-14, -1.988 - 2.0e-14}, unused_velocity);
  requireNear(deadbanded.velocities[0], 2.0,
              "per-joint velocity deadband did not retain prior value");
  requireNear(deadbanded.velocities[1], 4.0,
              "per-joint velocity deadband did not retain prior value");
}

void testIkPvPassThroughAndTransactionalHistory() {
  app::JointTargetBuilder direct(app::JointTargetMode::IkPv, 0.001, 2U);
  const auto target = direct.preview({1.0, 2.0}, {3.0, 4.0});
  require(target.positions == std::vector<double>({1.0, 2.0}) &&
              target.velocities == std::vector<double>({3.0, 4.0}) &&
              target.accelerations == std::vector<double>({0.0, 0.0}) &&
              !target.future_o1_startup,
          "IK-PV target was not passed through");

  app::JointTargetBuilder future(app::JointTargetMode::FutureO1Pv, 0.001, 1U);
  (void)future.preview({1.0}, {0.0});
  require(future.acceptedSampleCount() == 0U,
          "preview committed Future-O1 history");
}

void testActiveMappingAndFixedJoints() {
  std::vector<double> current(20U);
  std::vector<std::size_t> active_indices;
  std::vector<double> active_positions;
  std::vector<double> active_velocities;
  for (std::size_t index = 0U; index < current.size(); ++index) {
    current[index] = static_cast<double>(index);
    if (index != 4U && index != 5U) {
      active_indices.push_back(index);
      active_positions.push_back(100.0 + static_cast<double>(index));
      active_velocities.push_back(200.0 + static_cast<double>(index));
    }
  }
  const auto mapped = app::mapActiveIkToFull(
      current, active_indices, active_positions, active_velocities);
  require(mapped.positions.size() == 20U && mapped.velocities.size() == 20U,
          "full-order mapping did not produce 20 joints");
  for (std::size_t index = 0U; index < 20U; ++index) {
    if (index == 4U || index == 5U) {
      require(mapped.positions[index] == current[index] &&
                  mapped.velocities[index] == 0.0,
              "knee/ankle fixed-joint behavior mismatch");
    } else {
      require(mapped.positions[index] == 100.0 + static_cast<double>(index) &&
                  mapped.velocities[index] == 200.0 + static_cast<double>(index),
              "active joint mapping mismatch");
    }
  }
}

app::JointTargetLimits limits() {
  return {{-2.0, -2.0}, {2.0, 2.0}, {4.1, 4.1}, {8.2, 8.2}, {41.0, 41.0}};
}

void testConfiguredLimitProjection() {
  app::ProjectionDiagnostics diagnostics;
  app::JointTarget raw{{1.0, -1.0}, {0.5, -0.5}, {0.25, -0.25}, false};
  const auto unchanged = app::projectConfiguredLimits(raw, limits(), diagnostics);
  require(!diagnostics.projected() && unchanged.positions == raw.positions &&
              unchanged.velocities == raw.velocities &&
              unchanged.accelerations == raw.accelerations,
          "in-range target was changed");

  raw = {{1.0, -1.0}, {9.0, -9.0}, {-20.0, 20.0}, false};
  const auto projected = app::projectConfiguredLimits(raw, limits(), diagnostics);
  const double envelope = 4.1 - 8.2 * 8.2 / (2.0 * 41.0);
  require(projected.positions == raw.positions,
          "configured-limit projection changed position");
  requireNear(projected.accelerations[0], -8.2, "acceleration clamp mismatch");
  requireNear(projected.accelerations[1], 8.2, "acceleration clamp mismatch");
  requireNear(projected.velocities[0], envelope,
              "negative-acceleration stopping envelope mismatch");
  requireNear(projected.velocities[1], -envelope,
              "positive-acceleration stopping envelope mismatch");
  require(diagnostics.events.size() == 6U &&
              diagnostics.modified_joint_count == 2U,
          "projection diagnostics did not record every modification");
  for (const auto component : {app::ProjectionComponent::AccelerationLimit,
                               app::ProjectionComponent::VelocityLimit,
                               app::ProjectionComponent::JerkStoppingEnvelope}) {
    std::size_t count = 0U;
    for (const auto &event : diagnostics.events) {
      count += event.component == component ? 1U : 0U;
    }
    require(count == 2U, "projection reason diagnostics mismatch");
  }

  bool rejected = false;
  try {
    (void)app::projectConfiguredLimits(
        app::JointTarget{{2.1, 0.0}, {0.0, 0.0}, {0.0, 0.0}, false},
        limits(), diagnostics);
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  require(rejected, "out-of-range position was not rejected");
}

void testR1ProfileOrderAndReplaySettling() {
  const std::vector<std::string> expected_names{
      "head_yaw_joint",    "head_pitch_joint",  "torso_yaw_joint",
      "torso_pitch_joint", "knee_pitch_joint",  "ankle_pitch_joint",
      "left_arm_joint1",   "left_arm_joint2",   "left_arm_joint3",
      "left_arm_joint4",   "left_arm_joint5",   "left_arm_joint6",
      "left_arm_joint7",   "right_arm_joint1",  "right_arm_joint2",
      "right_arm_joint3",  "right_arm_joint4",  "right_arm_joint5",
      "right_arm_joint6",  "right_arm_joint7"};
  for (std::size_t index = 0U; index < expected_names.size(); ++index) {
    require(app::kR1StreamJointNames[index] == expected_names[index],
            "R1 stream profile joint order mismatch");
  }
  require(app::kR1StreamMaxVelocityRadPerS[6] == 5.05 &&
              app::kR1StreamMaxAccelerationRadPerS2[10] == 16.2,
          "R1 stream profile order/value mismatch");
  for (const double maximum_jerk : app::kR1StreamMaxJerkRadPerS3) {
    require(maximum_jerk == 3200.0, "OTG jerk override mismatch");
  }

  app::ReplaySettlingCounter settling;
  for (std::size_t cycle = 0U; cycle < 19U; ++cycle) {
    require(!settling.update(true, true, 1.0e-4, 1.0e-4, 1.0e-4,
                             1.0e-4, 1.0e-3, 1.0e-2),
            "replay settled too early");
  }
  require(settling.update(true, true, 1.0e-4, 1.0e-4, 1.0e-4, 1.0e-4,
                          1.0e-3, 1.0e-2),
          "replay did not settle on cycle 20");
  require(!settling.update(true, true, 1.1e-4, 0.0, 0.0, 0.0, 0.0, 0.0) &&
              settling.consecutiveCycles() == 0U,
          "settling counter did not reset on violation");
}

} // namespace

int main() {
  try {
    testFutureO1StartupFormulaAffineAndDeadband();
    testIkPvPassThroughAndTransactionalHistory();
    testActiveMappingAndFixedJoints();
    testConfiguredLimitProjection();
    testR1ProfileOrderAndReplaySettling();
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << "joint target processing test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
