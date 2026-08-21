#pragma once

#include "contracts/presentation/ik_app_snapshot.hpp"
#include "contracts/presentation/tui_document.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace motion_control_lab
{

struct PlannerCallTuiDebug
{
  std::string state{"idle"};
  double duration_s{0.0};
  double sample_time_s{0.0};
  std::size_t sample_count{0U};
  double calculation_time_ms{0.0};
};

struct CartesianPlanningLimitsTuiDebug
{
  double linear_velocity{0.0};
  double linear_acceleration{0.0};
  double linear_jerk{0.0};
  double angular_velocity{0.0};
  double angular_acceleration{0.0};
  double angular_jerk{0.0};
};

struct JointChainTuiDebug
{
  std::string name;
  std::string compact_name;
  double ik_position{0.0};
  double ik_velocity{0.0};
  double target_position{0.0};
  double target_velocity{0.0};
  double target_acceleration{0.0};
  double execution_position{0.0};
  double execution_velocity{0.0};
  double execution_acceleration{0.0};
  double execution_jerk{0.0};
  double position_lower{0.0};
  double position_upper{0.0};
  double maximum_velocity{0.0};
  double maximum_acceleration{0.0};
  double maximum_jerk{0.0};
  std::string projection{"-"};
};

struct JointProjectionEventTuiDebug
{
  std::string joint;
  std::string component;
  double original_value{0.0};
  double applied_value{0.0};
  double limit{0.0};
};

struct CartesianClampEventTuiDebug
{
  std::string arm;
  std::string component;
  std::string axis;
  double original_value{0.0};
  double applied_value{0.0};
  double limit{0.0};
};

struct PlannedGroupedStepOtgTuiDebug
{
  std::string source_mode;
  std::string target_mode;
  std::string feedback_topology;
  double left_task_scale{1.0};
  double right_task_scale{1.0};
  CartesianPlanningLimitsTuiDebug cartesian_limits;
  PlannerCallTuiDebug cartesian_plan;
  PlannerCallTuiDebug joint_plan;
  PlannerCallTuiDebug joint_step;
  bool startup{false};
  std::uint64_t projection_event_count{0U};
  std::uint64_t projection_cycle_count{0U};
  std::size_t modified_joint_count{0U};
  double raw_otg_max_position_delta{0.0};
  double raw_otg_max_velocity_delta{0.0};
  double maximum_absolute_velocity{0.0};
  double maximum_absolute_acceleration{0.0};
  double maximum_absolute_jerk{0.0};
  std::uint64_t clamp_target_revision{0U};
  double maximum_clamp_limit_ratio{0.0};
  std::vector<JointChainTuiDebug> joints;
  std::vector<JointProjectionEventTuiDebug> projection_events;
  std::vector<CartesianClampEventTuiDebug> clamp_events;
};

TuiDocument makeAppTuiDocument(const IkDebugFrame & frame,
                               const PlannedGroupedStepOtgTuiDebug & app,
                               const InteractiveIkPresentation & presentation,
                               std::size_t publish_count, const std::string & sink_status,
                               const std::string & title, const std::string & input_status);

} // namespace motion_control_lab
