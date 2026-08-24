#include "../planning.hpp"

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

namespace mcc = motion_control::core;
namespace planned = motion_control_lab::planned_hierarchical_step_otg;

constexpr double kDifferentiationStep = 1.0e-6;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool sameDoubleBits(double left, double right)
{
  return std::memcmp(&left, &right, sizeof(double)) == 0;
}

template<typename Left, typename Right>
bool sameEigenBits(const Left & left, const Right & right)
{
  if (left.size() != right.size()) {
    return false;
  }
  for (Eigen::Index index = 0; index < left.size(); ++index) {
    if (!sameDoubleBits(left.data()[index], right.data()[index])) {
      return false;
    }
  }
  return true;
}

bool sameRequestBits(
  const mcc::CartesianRetargetRequest & left, const mcc::CartesianRetargetRequest & right)
{
  if (
    left.reference_frame_name != right.reference_frame_name ||
    !sameDoubleBits(left.sample_period, right.sample_period) ||
    left.maximum_sample_count != right.maximum_sample_count ||
    left.synchronization != right.synchronization || left.segments.size() != right.segments.size()) {
    return false;
  }

  const std::array<bool, 6> same_limits{
    sameEigenBits(left.limits.max_linear_velocity, right.limits.max_linear_velocity),
    sameEigenBits(left.limits.max_linear_acceleration, right.limits.max_linear_acceleration),
    sameEigenBits(left.limits.max_linear_jerk, right.limits.max_linear_jerk),
    sameEigenBits(
      left.limits.max_rotation_vector_velocity, right.limits.max_rotation_vector_velocity),
    sameEigenBits(
      left.limits.max_rotation_vector_acceleration,
      right.limits.max_rotation_vector_acceleration),
    sameEigenBits(left.limits.max_rotation_vector_jerk, right.limits.max_rotation_vector_jerk)};
  for (bool same : same_limits) {
    if (!same) {
      return false;
    }
  }

  for (std::size_t index = 0U; index < left.segments.size(); ++index) {
    const auto & left_segment = left.segments[index];
    const auto & right_segment = right.segments[index];
    if (
      left_segment.frame_name != right_segment.frame_name ||
      !sameEigenBits(left_segment.current_pose.matrix(), right_segment.current_pose.matrix()) ||
      !sameEigenBits(left_segment.current_twist, right_segment.current_twist) ||
      !sameEigenBits(left_segment.current_acceleration, right_segment.current_acceleration) ||
      !sameEigenBits(left_segment.target_pose.matrix(), right_segment.target_pose.matrix())) {
      return false;
    }
  }
  return true;
}

mcc::CartesianRetargetRequest makeRequest()
{
  mcc::CartesianRetargetRequest request;
  request.reference_frame_name = "base_link";
  request.sample_period = 0.01;
  request.maximum_sample_count = 10000U;
  request.synchronization = mcc::TrajectorySynchronization::Time;
  request.limits.max_linear_velocity = Eigen::Vector3d{1.0, 2.0, 3.0};
  request.limits.max_linear_acceleration = Eigen::Vector3d{4.0, 5.0, 6.0};
  request.limits.max_linear_jerk = Eigen::Vector3d{12.0, 13.0, 14.0};
  request.limits.max_rotation_vector_velocity = Eigen::Vector3d{0.5, 0.6, 0.7};
  request.limits.max_rotation_vector_acceleration = Eigen::Vector3d{7.0, 8.0, 9.0};
  request.limits.max_rotation_vector_jerk = Eigen::Vector3d{21.0, 22.0, 23.0};

  mcc::CartesianRetargetSegment left;
  left.frame_name = "left_tool";
  left.current_pose.translation() = Eigen::Vector3d{0.1, -0.2, 0.3};
  left.current_pose.linear() =
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  left.current_twist << 0.25, -0.5, -0.0, 0.1, -0.2, 0.3;
  left.current_acceleration << -0.4, 0.5, -0.6, 0.7, -0.8, 0.9;
  left.target_pose = left.current_pose;
  left.target_pose.translation().x() += 0.4;

  mcc::CartesianRetargetSegment right = left;
  right.frame_name = "right_tool";
  right.current_pose.translation() = Eigen::Vector3d{-0.3, 0.2, 0.1};
  right.target_pose = right.current_pose;
  right.target_pose.translation().y() -= 0.5;
  request.segments = {left, right};
  return request;
}

void testPlanningOptionsAreApplied()
{
  const auto & robot = motion_control_lab::r1RobotConfig();
  mcc::CartesianTrajectorySample accepted;
  accepted.frames.resize(2U);
  accepted.frames[0].pose.translation() = Eigen::Vector3d{0.1, 0.2, 0.3};
  accepted.frames[1].pose.translation() = Eigen::Vector3d{-0.1, -0.2, 0.4};

  planned::PlanningOptions options;
  options.max_linear_velocity_mps = 0.31;
  options.max_linear_acceleration_mps2 = 0.62;
  options.max_linear_jerk_mps3 = 1.24;
  options.max_angular_velocity_rps = 0.45;
  options.max_angular_acceleration_rps2 = 0.90;
  options.max_angular_jerk_rps3 = 1.80;
  options.cartesian_synchronization = planned::PlanningSynchronization::Phase;
  options.joint_synchronization = planned::PlanningSynchronization::Time;

  const auto left_goal = mcc::Pose::Identity();
  const auto right_goal = mcc::Pose::Identity();
  const auto request = planned::makeRetargetRequest(
    left_goal, right_goal, accepted, robot, options, 50.0);
  const auto joint_config = planned::makeJointPlannerConfig(options);

  require(request.reference_frame_name == robot.base_frame, "wrong planning reference frame");
  require(request.sample_period == 0.02, "planning rate was not applied");
  require(
    request.synchronization == mcc::TrajectorySynchronization::Phase,
    "Cartesian synchronization was not applied");
  require(
    request.limits.max_linear_velocity == Eigen::Vector3d::Constant(0.31) &&
      request.limits.max_linear_acceleration == Eigen::Vector3d::Constant(0.62) &&
      request.limits.max_linear_jerk == Eigen::Vector3d::Constant(1.24) &&
      request.limits.max_rotation_vector_velocity == Eigen::Vector3d::Constant(0.45) &&
      request.limits.max_rotation_vector_acceleration == Eigen::Vector3d::Constant(0.90) &&
      request.limits.max_rotation_vector_jerk == Eigen::Vector3d::Constant(1.80),
    "Cartesian planning limits were not applied");
  require(
    joint_config.algorithm == mcc::JointTrajectoryAlgorithm::JerkLimited &&
      joint_config.synchronization == mcc::TrajectorySynchronization::Time,
    "joint planning options were not applied");
}

void testInRangeRequestIsBitwiseUnchanged()
{
  auto request = makeRequest();
  const auto original = request;
  const auto diagnostics = planned::clampRetargetCurrentState(request);
  require(!diagnostics.clamped(), "in-range request unexpectedly clamped");
  require(diagnostics.clamped_component_count == 0U, "in-range clamp count is not zero");
  require(
    sameDoubleBits(diagnostics.maximum_limit_ratio, 0.0),
    "in-range maximum limit ratio is not zero");
  require(sameRequestBits(request, original), "in-range request changed at the bit level");
}

bool hasEvent(
  const planned::RetargetClampDiagnostics & diagnostics, std::size_t segment_index,
  planned::RetargetClampComponent component, std::size_t axis, double original_value,
  double applied_value, double limit)
{
  for (std::size_t index = 0U; index < diagnostics.clamped_component_count; ++index) {
    const auto & event = diagnostics.events[index];
    if (
      event.segment_index == segment_index && event.component == component && event.axis == axis &&
      event.original_value == original_value && event.applied_value == applied_value &&
      event.limit == limit) {
      return true;
    }
  }
  return false;
}

void testAllPvaGroupsClampAcrossBothArms()
{
  auto request = makeRequest();
  const auto original = request;
  request.segments[0].current_twist << 1.25, -2.5, 2.5, 0.75, -0.6, -1.4;
  request.segments[0].current_acceleration << -8.0, 5.5, -6.0, -7.7, 16.0, -18.0;
  request.segments[1].current_twist << -1.1, 2.2, -3.3, -0.55, 1.2, 0.7;
  request.segments[1].current_acceleration << 4.4, -10.0, 6.6, 7.0, -8.8, 10.8;
  const auto poses_and_targets = request;

  const auto diagnostics = planned::clampRetargetCurrentState(request);
  require(diagnostics.clamped(), "out-of-range request was not clamped");
  require(diagnostics.clamped_component_count == 19U, "unexpected clamped component count");
  require(std::abs(diagnostics.maximum_limit_ratio - 2.0) < 1.0e-12, "wrong maximum ratio");

  require(
    request.segments[0].current_twist ==
      (mcc::Twist{} << 1.0, -2.0, 2.5, 0.5, -0.6, -0.7).finished(),
    "left twist clamp mismatch");
  require(
    request.segments[0].current_acceleration ==
      (mcc::SpatialAcceleration{} << -4.0, 5.0, -6.0, -7.0, 8.0, -9.0).finished(),
    "left acceleration clamp mismatch");
  require(
    request.segments[1].current_twist ==
      (mcc::Twist{} << -1.0, 2.0, -3.0, -0.5, 0.6, 0.7).finished(),
    "right twist clamp mismatch");
  require(
    request.segments[1].current_acceleration ==
      (mcc::SpatialAcceleration{} << 4.0, -5.0, 6.0, 7.0, -8.0, 9.0).finished(),
    "right acceleration clamp mismatch");
  require(
    hasEvent(
      diagnostics, 0U, planned::RetargetClampComponent::LinearVelocity, 1U, -2.5, -2.0, 2.0),
    "missing negative linear velocity event");
  require(
    hasEvent(
      diagnostics, 0U, planned::RetargetClampComponent::AngularAcceleration, 2U, -18.0, -9.0,
      9.0),
    "missing negative angular acceleration event");
  require(
    hasEvent(
      diagnostics, 1U, planned::RetargetClampComponent::AngularVelocity, 1U, 1.2, 0.6, 0.6),
    "missing right angular velocity event");
  require(
    hasEvent(
      diagnostics, 1U, planned::RetargetClampComponent::LinearAcceleration, 0U, 4.4, 4.0,
      4.0),
    "missing right linear acceleration event");

  for (std::size_t index = 0U; index < request.segments.size(); ++index) {
    require(
      sameEigenBits(
        request.segments[index].current_pose.matrix(),
        poses_and_targets.segments[index].current_pose.matrix()),
      "current pose changed during clamp");
    require(
      sameEigenBits(
        request.segments[index].target_pose.matrix(),
        poses_and_targets.segments[index].target_pose.matrix()),
      "target pose changed during clamp");
  }
  require(
    sameEigenBits(request.limits.max_linear_jerk, original.limits.max_linear_jerk) &&
      sameEigenBits(
        request.limits.max_rotation_vector_jerk,
        original.limits.max_rotation_vector_jerk),
    "jerk limits changed during clamp");
}

Eigen::Vector3d rotationVectorAcceleration(
  const Eigen::Vector3d & rotation_vector, const Eigen::Vector3d & rotation_vector_rate,
  const Eigen::Vector3d & spatial_angular_acceleration)
{
  const Eigen::Matrix3d jacobian_rate =
    (mcc::transform::rotationLeftJacobian(
       rotation_vector + kDifferentiationStep * rotation_vector_rate) -
     mcc::transform::rotationLeftJacobian(
       rotation_vector - kDifferentiationStep * rotation_vector_rate)) /
    (2.0 * kDifferentiationStep);
  return mcc::transform::rotationLeftJacobianInverse(rotation_vector) *
         (spatial_angular_acceleration - jacobian_rate * rotation_vector_rate);
}

mcc::CartesianRetargetRequest makePlannerRequest()
{
  mcc::CartesianRetargetRequest request;
  request.reference_frame_name = "base_link";
  request.sample_period = 0.01;
  request.maximum_sample_count = 10000U;
  request.synchronization = mcc::TrajectorySynchronization::Time;
  request.limits.max_linear_velocity = Eigen::Vector3d::Constant(0.4);
  request.limits.max_linear_acceleration = Eigen::Vector3d::Constant(1.0);
  request.limits.max_linear_jerk = Eigen::Vector3d::Constant(4.0);
  request.limits.max_rotation_vector_velocity = Eigen::Vector3d::Constant(0.5);
  request.limits.max_rotation_vector_acceleration = Eigen::Vector3d::Constant(1.2);
  request.limits.max_rotation_vector_jerk = Eigen::Vector3d::Constant(5.0);

  mcc::CartesianRetargetSegment left;
  left.frame_name = "left_tool";
  left.target_pose.translation() = Eigen::Vector3d{2.0, 2.0, 0.0};
  left.current_twist.x() = 0.6;
  left.current_acceleration.y() = 1.4;

  mcc::CartesianRetargetSegment right;
  right.frame_name = "right_tool";
  const Eigen::Vector3d target_rotation_vector{0.0, 1.2, 1.2};
  right.target_pose.linear() = Eigen::AngleAxisd(
    target_rotation_vector.norm(), target_rotation_vector.normalized()).toRotationMatrix();
  right.current_twist.tail<3>().z() = 0.75;
  right.current_acceleration.tail<3>().y() = 1.5;
  request.segments = {left, right};
  return request;
}

void testClampedRequestPlansWithinAllDerivativeLimits()
{
  auto request = makePlannerRequest();
  mcc::PlanningDiagnostics diagnostics;
  mcc::CartesianPlanner strict_planner;
  const auto rejected = strict_planner.replan(request, diagnostics);
  require(!rejected.ok(), "Core unexpectedly accepted the unclamped request");
  require(
    rejected.code == mcc::StatusCode::InvalidInput,
    "Core returned the wrong status for the unclamped request");

  const auto current_and_target_poses = request;
  const auto clamp = planned::clampRetargetCurrentState(request);
  require(clamp.clamped_component_count == 4U, "planner request clamp count mismatch");
  require(
    request.segments[0].current_twist.x() == 0.4 &&
      request.segments[0].current_acceleration.y() == 1.0 &&
      request.segments[1].current_twist.tail<3>().z() == 0.5 &&
      request.segments[1].current_acceleration.tail<3>().y() == 1.2,
    "planner request was not clamped to its exact limits");
  for (std::size_t index = 0U; index < request.segments.size(); ++index) {
    require(
      sameEigenBits(
        request.segments[index].current_pose.matrix(),
        current_and_target_poses.segments[index].current_pose.matrix()) &&
        sameEigenBits(
          request.segments[index].target_pose.matrix(),
          current_and_target_poses.segments[index].target_pose.matrix()),
      "planner request pose changed during clamp");
  }

  mcc::CartesianPlanner planner;
  const auto planned = planner.replan(request, diagnostics);
  require(planned.ok(), "Core rejected the clamped request: " + planned.message);

  std::array<Eigen::Vector3d, 2> previous_linear_acceleration{
    request.segments[0].current_acceleration.head<3>(),
    request.segments[1].current_acceleration.head<3>()};
  std::array<Eigen::Vector3d, 2> previous_rotation_vector_acceleration{
    request.segments[0].current_acceleration.tail<3>(),
    request.segments[1].current_acceleration.tail<3>()};
  double previous_time = 0.0;
  std::size_t sample_count = 0U;
  mcc::CartesianTrajectorySample sample;
  while (true) {
    const auto step_status = planner.step(sample, diagnostics);
    require(step_status.ok(), "Cartesian planner step failed: " + step_status.message);
    require(sample.frames.size() == 2U, "planner returned the wrong frame count");
    const double time_step = sample.time_from_start - previous_time;
    require(time_step > 0.0, "planner sample time did not advance");

    for (std::size_t index = 0U; index < sample.frames.size(); ++index) {
      const auto & frame = sample.frames[index];
      const Eigen::Vector3d rotation_vector = mcc::transform::rotationLog(
        frame.pose.linear() * request.segments[index].current_pose.linear().transpose());
      const Eigen::Vector3d rotation_vector_rate =
        mcc::transform::rotationLeftJacobianInverse(rotation_vector) * frame.twist.tail<3>();
      const Eigen::Vector3d rotation_vector_acceleration = rotationVectorAcceleration(
        rotation_vector, rotation_vector_rate, frame.acceleration.tail<3>());
      const Eigen::Vector3d linear_jerk =
        (frame.acceleration.head<3>() - previous_linear_acceleration[index]) / time_step;
      const Eigen::Vector3d rotation_vector_jerk =
        (rotation_vector_acceleration - previous_rotation_vector_acceleration[index]) / time_step;

      require(
        (frame.twist.head<3>().cwiseAbs().array() <=
         request.limits.max_linear_velocity.array() + 1.0e-9)
          .all(),
        "linear velocity limit exceeded");
      require(
        (frame.acceleration.head<3>().cwiseAbs().array() <=
         request.limits.max_linear_acceleration.array() + 1.0e-8)
          .all(),
        "linear acceleration limit exceeded");
      require(
        (linear_jerk.cwiseAbs().array() <= request.limits.max_linear_jerk.array() + 1.0e-5)
          .all(),
        "linear jerk limit exceeded");
      require(
        (rotation_vector_rate.cwiseAbs().array() <=
         request.limits.max_rotation_vector_velocity.array() + 1.0e-8)
          .all(),
        "rotation-vector velocity limit exceeded");
      require(
        (rotation_vector_acceleration.cwiseAbs().array() <=
         request.limits.max_rotation_vector_acceleration.array() + 2.0e-3)
          .all(),
        "rotation-vector acceleration limit exceeded");
      require(
        (rotation_vector_jerk.cwiseAbs().array() <=
         request.limits.max_rotation_vector_jerk.array() + 2.0e-2)
          .all(),
        "rotation-vector jerk limit exceeded");

      previous_linear_acceleration[index] = frame.acceleration.head<3>();
      previous_rotation_vector_acceleration[index] = rotation_vector_acceleration;
    }
    previous_time = sample.time_from_start;
    ++sample_count;
    require(sample_count < request.maximum_sample_count, "planner did not finish within its budget");
    if (diagnostics.state == mcc::PlanningState::Finished) {
      break;
    }
  }

  require(sample_count > 1U, "planner did not produce a trajectory");
  require(
    sample.frames[0].pose.isApprox(request.segments[0].target_pose, 1.0e-9) &&
      sample.frames[1].pose.isApprox(request.segments[1].target_pose, 1.0e-9),
    "planner did not finish at the requested stationary targets");
  require(
    sample.frames[0].twist.norm() < 1.0e-9 && sample.frames[1].twist.norm() < 1.0e-9 &&
      sample.frames[0].acceleration.norm() < 1.0e-9 &&
      sample.frames[1].acceleration.norm() < 1.0e-9,
    "planner target derivatives are not stationary");
}

}  // namespace

int main()
{
  try {
    testPlanningOptionsAreApplied();
    testInRangeRequestIsBitwiseUnchanged();
    testAllPvaGroupsClampAcrossBothArms();
    testClampedRequestPlansWithinAllDerivativeLimits();
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "planned hierarchical Step retarget clamp test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
