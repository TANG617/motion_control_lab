#include "retarget_state_clamp.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_control_lab::planned_grouped_servo_step
{
namespace
{

void clampComponent(
  double & value, double limit, std::size_t segment_index, RetargetClampComponent component,
  std::size_t axis, RetargetClampDiagnostics & diagnostics)
{
  if (!std::isfinite(value) || !std::isfinite(limit) || limit <= 0.0) {
    return;
  }
  if (value >= -limit && value <= limit) {
    return;
  }
  if (diagnostics.clamped_component_count >= diagnostics.events.size()) {
    throw std::logic_error("planned retarget clamp diagnostic capacity exceeded");
  }

  const double applied_value = std::clamp(value, -limit, limit);
  diagnostics.events[diagnostics.clamped_component_count] = RetargetClampEvent{
    segment_index, component, axis, value, applied_value, limit};
  ++diagnostics.clamped_component_count;
  diagnostics.maximum_limit_ratio =
    std::max(diagnostics.maximum_limit_ratio, std::abs(value) / limit);
  value = applied_value;
}

template<typename SpatialVector>
void clampTriplet(
  SpatialVector & values, std::size_t offset, const Eigen::Vector3d & limits,
  std::size_t segment_index, RetargetClampComponent component,
  RetargetClampDiagnostics & diagnostics)
{
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    clampComponent(
      values[static_cast<Eigen::Index>(offset + axis)], limits[static_cast<Eigen::Index>(axis)],
      segment_index, component, axis, diagnostics);
  }
}

}  // namespace

RetargetClampDiagnostics clampRetargetCurrentState(
  motion_control::core::CartesianRetargetRequest & request)
{
  if (request.segments.size() > 2U) {
    throw std::logic_error("planned grouped ServoStep retarget clamp expects at most two arms");
  }

  RetargetClampDiagnostics diagnostics;
  for (std::size_t segment_index = 0U; segment_index < request.segments.size(); ++segment_index) {
    auto & segment = request.segments[segment_index];
    clampTriplet(
      segment.current_twist, 0U, request.limits.max_linear_velocity, segment_index,
      RetargetClampComponent::LinearVelocity, diagnostics);
    clampTriplet(
      segment.current_twist, 3U, request.limits.max_rotation_vector_velocity, segment_index,
      RetargetClampComponent::AngularVelocity, diagnostics);
    clampTriplet(
      segment.current_acceleration, 0U, request.limits.max_linear_acceleration, segment_index,
      RetargetClampComponent::LinearAcceleration, diagnostics);
    clampTriplet(
      segment.current_acceleration, 3U, request.limits.max_rotation_vector_acceleration,
      segment_index, RetargetClampComponent::AngularAcceleration, diagnostics);
  }
  return diagnostics;
}

}  // namespace motion_control_lab::planned_grouped_servo_step
