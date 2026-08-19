#pragma once

#include <array>
#include <cstddef>

#include <motion_control_core/motion_control_core.hpp>

namespace motion_control_lab::planned_grouped_step_otg
{

enum class RetargetClampComponent
{
  LinearVelocity,
  AngularVelocity,
  LinearAcceleration,
  AngularAcceleration,
};

struct RetargetClampEvent
{
  std::size_t segment_index{0U};
  RetargetClampComponent component{RetargetClampComponent::LinearVelocity};
  std::size_t axis{0U};
  double original_value{0.0};
  double applied_value{0.0};
  double limit{0.0};
};

inline constexpr std::size_t kMaximumRetargetClampEvents{24U};

struct RetargetClampDiagnostics
{
  std::array<RetargetClampEvent, kMaximumRetargetClampEvents> events{};
  std::size_t clamped_component_count{0U};
  double maximum_limit_ratio{0.0};

  bool clamped() const { return clamped_component_count != 0U; }
};

/**
 * Clamp the two-arm planned app's finite current Cartesian PVA components to
 * the request limits. Poses, targets, limits, and in-range values are not
 * modified. Non-finite values remain untouched for Core validation to reject.
 */
RetargetClampDiagnostics clampRetargetCurrentState(
  motion_control::core::CartesianRetargetRequest & request);

}  // namespace motion_control_lab::planned_grouped_step_otg
