#include "components/tui/standard_ik_tui.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace motion_control_lab
{
namespace
{

using Alignment = TuiTableAlignment;

std::string fixed(double value, int precision = 3)
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(precision) << value;
  return output.str();
}

std::string signedFixed(double value, int precision = 4)
{
  std::ostringstream output;
  output << std::showpos << std::fixed << std::setprecision(precision) << value;
  return output.str();
}

std::string yesNo(bool value) { return value ? "yes" : "no"; }

std::string runtimeState(IkRuntimeState value)
{
  switch (value) {
    case IkRuntimeState::Running: return "RUNNING";
    case IkRuntimeState::RecoverableReject: return "TARGET REJECTED";
    case IkRuntimeState::FaultHold: return "FAULT HOLD";
  }
  return "UNKNOWN";
}

std::string processorList(const std::vector<unsigned int> & values)
{
  std::ostringstream output;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << values[index];
  }
  return values.empty() ? "-" : output.str();
}

TuiTableColumn textColumn(std::string title)
{
  return TuiTableColumn{std::move(title), Alignment::Left};
}

TuiTableColumn numberColumn(std::string title)
{
  return TuiTableColumn{std::move(title), Alignment::Right};
}

TuiSection sheet(
  std::string title,
  std::vector<TuiTableColumn> columns,
  std::vector<std::vector<std::string>> rows,
  std::size_t column = 0U,
  std::size_t row = 0U)
{
  TuiSection section;
  section.title = std::move(title);
  section.tables.push_back(
    TuiTable{std::move(columns), std::move(rows), TuiTableStyle::Compact});
  section.column = column;
  section.row = row;
  section.style = TuiSectionStyle::Panel;
  return section;
}

std::vector<std::string> noneRow(std::size_t width)
{
  std::vector<std::string> row(width, "-");
  row.front() = "none";
  return row;
}

std::pair<double, double> poseError(const Pose & first, const Pose & second)
{
  return {
    (first.translation() - second.translation()).norm(),
    Eigen::AngleAxisd(first.linear() * second.linear().transpose()).angle()};
}

const ArmForwardKinematics * findForwardKinematics(const IkDebugFrame & frame, ArmSide side)
{
  for (const auto & arm : frame.forward_kinematics) {
    if (arm.side == side) {
      return &arm;
    }
  }
  return nullptr;
}

const PlannedArmDebug * findPlannerArm(const IkDebugFrame & frame, ArmSide side)
{
  if (!frame.cartesian_planner.has_value()) {
    return nullptr;
  }
  for (const auto & arm : frame.cartesian_planner->arms) {
    if (arm.side == side) {
      return &arm;
    }
  }
  return nullptr;
}

std::vector<std::string> compactPoseRow(
  const std::string & arm,
  const std::string & point,
  const Pose & pose)
{
  const Eigen::Quaterniond quaternion(pose.linear());
  return {
    arm,
    point,
    signedFixed(pose.translation().x()) + " " + signedFixed(pose.translation().y()) + " " +
      signedFixed(pose.translation().z()),
    signedFixed(quaternion.x()) + " " + signedFixed(quaternion.y()) + " " +
      signedFixed(quaternion.z()) + " " + signedFixed(quaternion.w())};
}

std::vector<std::string> poseRow(
  const std::string & arm,
  const std::string & point,
  const Pose & pose)
{
  const Eigen::Quaterniond quaternion(pose.linear());
  return {
    arm,
    point,
    fixed(pose.translation().x(), 4),
    fixed(pose.translation().y(), 4),
    fixed(pose.translation().z(), 4),
    fixed(quaternion.x(), 4),
    fixed(quaternion.y(), 4),
    fixed(quaternion.z(), 4),
    fixed(quaternion.w(), 4)};
}

std::string solverRejection(const SolverDebug & solver)
{
  if (solver.grouped_attempt.has_value()) {
    return solver.grouped_attempt->rejection_reason;
  }
  return solver.disposition == "accepted" ? "none" : solver.termination_reason;
}

bool containsJointIndex(const std::vector<std::size_t> & indices, std::size_t value)
{
  return std::find(indices.begin(), indices.end(), value) != indices.end();
}

std::string jointPosition(const IkDebugFrame & frame, std::size_t index)
{
  return index < frame.positions.size() ? fixed(frame.positions[index], 5) : "-";
}

std::string jointVelocity(const IkDebugFrame & frame, std::size_t index)
{
  return index < frame.velocities.size() ? fixed(frame.velocities[index], 5) : "-";
}

TuiPage makeOverviewPage(
  const IkDebugFrame & frame,
  const InteractiveIkPresentation & presentation)
{
  TuiPage page;
  page.title = "Overview";
  page.column_weights = {3, 2};
  page.rows = {{{3, 2}, 3}, {{2, 3}, 2}};

  TuiSection cartesian;
  cartesian.title = "Cartesian tracking";
  cartesian.row = 0U;
  cartesian.column = 0U;
  cartesian.style = TuiSectionStyle::Panel;
  cartesian.rows = {
    {"Frame", presentation.base_frame_id},
    {"Runtime state", runtimeState(frame.runtime_state)},
    {"Selected arm", armSideName(frame.selected_side)},
    {"Input", frame.paused ? "paused" : "running"}};
  if (frame.cartesian_planner.has_value()) {
    cartesian.rows.push_back({"Cartesian planner", frame.cartesian_planner->state});
  }

  std::vector<std::vector<std::string>> pose_rows;
  std::vector<std::vector<std::string>> error_rows;
  if (frame.cartesian_planner.has_value()) {
    for (const auto & arm : frame.cartesian_planner->arms) {
      const std::string name = armSideName(arm.side);
      pose_rows.push_back(compactPoseRow(name, "Goal", arm.source_goal));
      pose_rows.push_back(compactPoseRow(name, "Reference", arm.reference));
      pose_rows.push_back(compactPoseRow(name, "FK", arm.forward_kinematics));
      const auto goal_reference = poseError(arm.source_goal, arm.reference);
      const auto reference_fk = poseError(arm.reference, arm.forward_kinematics);
      const auto goal_fk = poseError(arm.source_goal, arm.forward_kinematics);
      error_rows.push_back(
        {name, "Goal to Reference", fixed(goal_reference.first, 6),
         fixed(goal_reference.second, 6)});
      error_rows.push_back(
        {name, "Reference to FK", fixed(reference_fk.first, 6),
         fixed(reference_fk.second, 6)});
      error_rows.push_back(
        {name, "Goal to FK", fixed(goal_fk.first, 6), fixed(goal_fk.second, 6)});
    }
  } else {
    for (const auto & target : frame.targets) {
      const std::string name = armSideName(target.side);
      pose_rows.push_back(compactPoseRow(name, "Target", target.target_pose));
      if (const auto * fk = findForwardKinematics(frame, target.side)) {
        pose_rows.push_back(compactPoseRow(name, "FK", fk->pose));
      }
    }
    for (const auto & error : frame.target_errors) {
      error_rows.push_back(
        {armSideName(error.side), "Target to FK", fixed(error.position_m, 6),
         fixed(error.orientation_rad, 6)});
    }
  }
  if (pose_rows.empty()) {
    pose_rows.push_back(noneRow(4U));
  }
  if (error_rows.empty()) {
    error_rows.push_back(noneRow(4U));
  }
  cartesian.tables.push_back(
    TuiTable{
      {textColumn("Arm"), textColumn("State"), textColumn("Position xyz [m]"),
       textColumn("Quaternion xyzw")},
      std::move(pose_rows), TuiTableStyle::Compact});
  page.sections.push_back(std::move(cartesian));
  page.sections.push_back(sheet(
    "Cartesian errors",
    {textColumn("Arm"), textColumn("Path"), numberColumn("Position error [m]"),
     numberColumn("Orientation error [rad]")},
    std::move(error_rows), 0U, 0U));

  std::vector<std::vector<std::string>> solver_rows;
  for (const auto & solver : frame.solvers) {
    solver_rows.push_back({solver.label, "Disposition", solver.disposition});
    solver_rows.push_back({solver.label, "Termination", solver.termination_reason});
    solver_rows.push_back(
      {solver.label, "Quadratic Programming status",
       solver.backend + "/" + solver.qp_status + " " + solver.native_status});
    solver_rows.push_back(
      {solver.label, "IK calculation [ms]", fixed(solver.ik_solve_time_ms, 4)});
    solver_rows.push_back(
      {solver.label, "95th percentile [ms]", fixed(solver.ik_solve_time_percentiles.p95, 4)});
    solver_rows.push_back(
      {solver.label, "Maximum hard violation", fixed(solver.maximum_hard_violation, 9)});
    solver_rows.push_back({solver.label, "Rejection", solverRejection(solver)});
  }
  if (solver_rows.empty()) {
    solver_rows.push_back(noneRow(3U));
  }
  page.sections.push_back(
    sheet(
      "Solver health",
      {textColumn("Solver"), textColumn("Metric"), textColumn("Value")},
      std::move(solver_rows), 1U, 0U));

  std::vector<std::size_t> left_indices;
  std::vector<std::size_t> right_indices;
  if (const auto * arm = findArmPresentation(presentation, ArmSide::Left)) {
    left_indices = arm->joint_indices;
  }
  if (const auto * arm = findArmPresentation(presentation, ArmSide::Right)) {
    right_indices = arm->joint_indices;
  }
  std::vector<std::vector<std::string>> joint_rows;
  for (std::size_t index = 0U; index < frame.joint_names.size(); ++index) {
    std::string group = "Body";
    if (containsJointIndex(left_indices, index)) {
      group = "Left arm";
    } else if (containsJointIndex(right_indices, index)) {
      group = "Right arm";
    }
    joint_rows.push_back({group, frame.joint_names[index], jointPosition(frame, index)});
  }
  if (joint_rows.empty()) {
    joint_rows.push_back(noneRow(3U));
  }
  page.sections.push_back(
    sheet(
      "Joint positions",
      {textColumn("Group"), textColumn("Joint"), numberColumn("Position [rad]")},
      std::move(joint_rows), 0U, 1U));

  TuiSection runtime = sheet(
    "Worker runtime",
    {textColumn("Worker"), numberColumn("Rate [Hz]"), numberColumn("Deadline misses"),
     numberColumn("Skipped releases"), numberColumn("Recoverable rejections")},
    {}, 1U, 1U);
  for (const auto & worker : frame.workers) {
    runtime.tables.front().rows.push_back(
      {worker.label, fixed(worker.configured_rate_hz, 0),
       std::to_string(worker.deadline_miss_count),
       std::to_string(worker.skipped_release_count),
       std::to_string(worker.recoverable_rejection_count)});
  }
  if (runtime.tables.front().rows.empty()) {
    runtime.tables.front().rows.push_back(noneRow(5U));
  }
  page.sections.push_back(std::move(runtime));

  std::vector<std::vector<std::string>> processor_rows;
  for (const auto & processor : frame.cpu_affinities) {
    processor_rows.push_back(
      {processor.role, std::to_string(processor.thread_id),
       processorList(processor.requested_cpus), processorList(processor.effective_cpus)});
  }
  if (processor_rows.empty()) {
    processor_rows.push_back(noneRow(4U));
  }
  page.sections.push_back(sheet(
    "Processor affinity",
    {textColumn("Role"), numberColumn("Thread identifier"),
     textColumn("Requested processors"), textColumn("Effective processors")},
    std::move(processor_rows), 1U, 1U));

  std::vector<std::vector<std::string>> collision_rows;
  for (const auto & collision : frame.self_collisions) {
    collision_rows.push_back(
      {collision.label, fixed(collision.minimum_distance_before_m, 5),
       fixed(collision.minimum_distance_after_m, 5), fixed(collision.margin_shortfall_m, 5)});
  }
  if (collision_rows.empty()) {
    collision_rows.push_back(noneRow(4U));
  }
  page.sections.push_back(sheet(
    "Collision safety",
    {textColumn("Collision group"), numberColumn("Before [m]"),
     numberColumn("After [m]"), numberColumn("Shortfall [m]")},
    std::move(collision_rows), 1U, 1U));
  return page;
}

TuiPage makeCartesianPlanningPage(const IkDebugFrame & frame)
{
  TuiPage page;
  page.title = "Cartesian Planning";
  const auto & planner = *frame.cartesian_planner;
  page.sections.push_back(
    sheet(
      "Planner",
      {textColumn("State"), numberColumn("Sample time [s]")},
      {{planner.state, fixed(planner.sample_time_s, 6)}}));

  std::vector<std::vector<std::string>> pose_rows;
  std::vector<std::vector<std::string>> linear_rows;
  std::vector<std::vector<std::string>> angular_rows;
  std::vector<std::vector<std::string>> error_rows;
  for (const auto & arm : planner.arms) {
    const std::string name = armSideName(arm.side);
    pose_rows.push_back(poseRow(name, "Goal", arm.source_goal));
    pose_rows.push_back(poseRow(name, "Reference", arm.reference));
    pose_rows.push_back(poseRow(name, "FK", arm.forward_kinematics));
    linear_rows.push_back(
      {name, fixed(arm.reference_twist[0], 4), fixed(arm.reference_twist[1], 4),
       fixed(arm.reference_twist[2], 4), fixed(arm.reference_acceleration[0], 4),
       fixed(arm.reference_acceleration[1], 4), fixed(arm.reference_acceleration[2], 4)});
    angular_rows.push_back(
      {name, fixed(arm.reference_twist[3], 4), fixed(arm.reference_twist[4], 4),
       fixed(arm.reference_twist[5], 4), fixed(arm.reference_acceleration[3], 4),
       fixed(arm.reference_acceleration[4], 4), fixed(arm.reference_acceleration[5], 4)});
    const auto goal_reference = poseError(arm.source_goal, arm.reference);
    const auto reference_fk = poseError(arm.reference, arm.forward_kinematics);
    const auto goal_fk = poseError(arm.source_goal, arm.forward_kinematics);
    error_rows.push_back(
      {name, "Goal to Reference", fixed(goal_reference.first, 6),
       fixed(goal_reference.second, 6)});
    error_rows.push_back(
      {name, "Reference to FK", fixed(reference_fk.first, 6),
       fixed(reference_fk.second, 6)});
    error_rows.push_back(
      {name, "Goal to FK", fixed(goal_fk.first, 6), fixed(goal_fk.second, 6)});
  }
  if (pose_rows.empty()) {
    pose_rows.push_back(noneRow(9U));
    linear_rows.push_back(noneRow(7U));
    angular_rows.push_back(noneRow(7U));
    error_rows.push_back(noneRow(4U));
  }
  page.sections.push_back(
    sheet(
      "Pose: Goal to Reference to FK",
      {textColumn("Arm"), textColumn("State"), numberColumn("X [m]"), numberColumn("Y [m]"),
       numberColumn("Z [m]"), numberColumn("Quaternion x"), numberColumn("Quaternion y"),
       numberColumn("Quaternion z"), numberColumn("Quaternion w")},
      std::move(pose_rows)));
  page.sections.push_back(
    sheet(
      "Reference linear motion",
      {textColumn("Arm"), numberColumn("Velocity x [m/s]"),
       numberColumn("Velocity y [m/s]"), numberColumn("Velocity z [m/s]"),
       numberColumn("Acceleration x [m/s\xC2\xB2]"), numberColumn("Acceleration y [m/s\xC2\xB2]"),
       numberColumn("Acceleration z [m/s\xC2\xB2]")},
      std::move(linear_rows)));
  page.sections.push_back(
    sheet(
      "Reference angular motion",
      {textColumn("Arm"), numberColumn("Velocity x [rad/s]"),
       numberColumn("Velocity y [rad/s]"), numberColumn("Velocity z [rad/s]"),
       numberColumn("Acceleration x [rad/s\xC2\xB2]"),
       numberColumn("Acceleration y [rad/s\xC2\xB2]"),
       numberColumn("Acceleration z [rad/s\xC2\xB2]")},
      std::move(angular_rows)));
  page.sections.push_back(
    sheet(
      "Errors",
      {textColumn("Arm"), textColumn("Path"), numberColumn("Position error [m]"),
       numberColumn("Orientation error [rad]")},
      std::move(error_rows)));
  return page;
}

TuiPage makeSolverPage(const IkDebugFrame & frame)
{
  TuiPage page;
  page.title = "Solver and Quadratic Programming";
  std::vector<std::vector<std::string>> summary_rows;
  std::vector<std::vector<std::string>> timing_rows;
  std::vector<std::vector<std::string>> percentile_rows;
  std::vector<std::vector<std::string>> diagnostic_rows;
  std::vector<std::vector<std::string>> counter_rows;
  std::vector<std::vector<std::string>> attempt_rows;
  std::vector<std::vector<std::string>> revision_rows;
  std::vector<std::vector<std::string>> scale_rows;
  std::vector<std::vector<std::string>> requirement_rows;
  for (const auto & solver : frame.solvers) {
    summary_rows.push_back(
      {solver.label, solver.disposition, solver.termination_reason, yesNo(solver.converged),
       std::to_string(solver.ik_iterations), solver.backend, solver.qp_status,
       solver.native_status});
    timing_rows.push_back(
      {solver.label, fixed(solver.ik_solve_time_ms, 4), fixed(solver.qp_solve_time_ms, 4),
       fixed(std::max(0.0, solver.ik_solve_time_ms - solver.qp_solve_time_ms), 4)});
    percentile_rows.push_back(
      {solver.label, std::to_string(solver.ik_solve_time_percentiles.window_sample_count),
       fixed(solver.ik_solve_time_percentiles.p90, 4),
       fixed(solver.ik_solve_time_percentiles.p95, 4),
       fixed(solver.ik_solve_time_percentiles.p99, 4)});
    diagnostic_rows.push_back(
      {solver.label, std::to_string(solver.qp_iterations), fixed(solver.objective_value, 7),
       fixed(solver.primal_residual, 7), fixed(solver.dual_residual, 7),
       fixed(solver.maximum_hard_violation, 7), std::to_string(solver.active_set_size),
       yesNo(solver.warm_start_used)});
    if (solver.run_counters.has_value()) {
      counter_rows.push_back(
        {solver.label, std::to_string(solver.run_counters->attempts),
         std::to_string(solver.run_counters->accepted),
         std::to_string(solver.run_counters->rejected)});
    }
    if (solver.grouped_attempt.has_value()) {
      const auto & attempt = *solver.grouped_attempt;
      attempt_rows.push_back(
        {solver.label, attempt.rejection_reason, attempt.coupling_state});
      revision_rows.push_back(
        {solver.label, std::to_string(attempt.run_generation),
         std::to_string(attempt.attempt_revision), std::to_string(attempt.value_revision),
         std::to_string(attempt.consumed_source_value_revision),
         std::to_string(attempt.captured_state_sequence),
         std::to_string(attempt.captured_state_time_nanoseconds)});
    }
    for (const auto & scale : solver.task_scales) {
      scale_rows.push_back(
        {solver.label, scale.name, yesNo(scale.active), fixed(scale.scale, 5),
         fixed(scale.cost, 5), yesNo(scale.degraded), yesNo(scale.stuck)});
    }
    for (const auto & requirement : solver.requirements) {
      requirement_rows.push_back(
        {solver.label, requirement.name, yesNo(requirement.enabled), yesNo(requirement.active),
         fixed(requirement.maximum_violation, 7), requirement.unit, fixed(requirement.cost, 5),
         requirement.source});
    }
  }
  if (summary_rows.empty()) {
    summary_rows.push_back(noneRow(8U));
    timing_rows.push_back(noneRow(4U));
    percentile_rows.push_back(noneRow(5U));
    diagnostic_rows.push_back(noneRow(8U));
  }
  if (counter_rows.empty()) {
    counter_rows.push_back(noneRow(4U));
  }
  if (attempt_rows.empty()) {
    attempt_rows.push_back(noneRow(3U));
    revision_rows.push_back(noneRow(7U));
  }
  if (scale_rows.empty()) {
    scale_rows.push_back(noneRow(7U));
  }
  if (requirement_rows.empty()) {
    requirement_rows.push_back(noneRow(8U));
  }

  page.sections.push_back(sheet(
    "Solver summary",
    {textColumn("Solver"), textColumn("Disposition"), textColumn("Termination"),
     textColumn("Converged"), numberColumn("IK iterations"), textColumn("Backend"),
     textColumn("Quadratic Programming status"), textColumn("Native status")},
    std::move(summary_rows)));
  page.sections.push_back(sheet(
    "Calculation timing [ms]",
    {textColumn("Solver"), numberColumn("IK total"),
     numberColumn("Quadratic Programming"), numberColumn("Non-Quadratic Programming")},
    std::move(timing_rows)));
  page.sections.push_back(sheet(
    "IK calculation percentiles [ms]",
    {textColumn("Solver"), numberColumn("Window samples"), numberColumn("90th percentile"),
     numberColumn("95th percentile"), numberColumn("99th percentile")},
    std::move(percentile_rows)));
  page.sections.push_back(sheet(
    "Quadratic Programming diagnostics",
    {textColumn("Solver"), numberColumn("Iterations"), numberColumn("Objective"),
     numberColumn("Primal residual"), numberColumn("Dual residual"),
     numberColumn("Maximum hard violation"), numberColumn("Active set size"),
     textColumn("Warm start")},
    std::move(diagnostic_rows)));
  page.sections.push_back(sheet(
    "Run counters",
    {textColumn("Solver"), numberColumn("Attempts"), numberColumn("Accepted"),
     numberColumn("Rejected")},
    std::move(counter_rows)));
  page.sections.push_back(sheet(
    "Grouped attempt and coupling",
    {textColumn("Solver"), textColumn("Rejection reason"), textColumn("Coupling state")},
    std::move(attempt_rows)));
  page.sections.push_back(sheet(
    "Grouped revisions and captured state",
    {textColumn("Solver"), numberColumn("Run generation"), numberColumn("Attempt revision"),
     numberColumn("Value revision"), numberColumn("Consumed source revision"),
     numberColumn("Captured state sequence"), numberColumn("Captured time [ns]")},
    std::move(revision_rows)));
  page.sections.push_back(sheet(
    "Task scales",
    {textColumn("Solver"), textColumn("Task"), textColumn("Active"), numberColumn("Scale"),
     numberColumn("Cost"), textColumn("Degraded"), textColumn("Stuck")},
    std::move(scale_rows)));
  page.sections.push_back(sheet(
    "Requirements and constraints",
    {textColumn("Solver"), textColumn("Requirement"), textColumn("Enabled"),
     textColumn("Active"), numberColumn("Maximum violation"), textColumn("Unit"),
     numberColumn("Cost"), textColumn("Source")},
    std::move(requirement_rows)));
  return page;
}

TuiPage makeJointStatePage(const IkDebugFrame & frame)
{
  TuiPage page;
  page.title = "Joint State";
  std::vector<std::vector<std::string>> rows;
  for (std::size_t index = 0U; index < frame.joint_names.size(); ++index) {
    rows.push_back(
      {frame.joint_names[index], jointPosition(frame, index), jointVelocity(frame, index)});
  }
  if (rows.empty()) {
    rows.push_back(noneRow(3U));
  }
  page.sections.push_back(sheet(
    "Executed joint state",
    {textColumn("Joint name"), numberColumn("Position [rad]"),
     numberColumn("Velocity [rad/s]")},
    std::move(rows)));
  return page;
}

TuiPage makeRuntimePage(const IkDebugFrame & frame)
{
  TuiPage page;
  page.title = "Runtime";
  page.column_weights = {3, 2};

  std::vector<std::vector<std::string>> timing_rows;
  std::vector<std::vector<std::string>> counter_rows;
  for (const auto & worker : frame.workers) {
    const std::vector<std::pair<std::string, std::pair<double, double>>> timings{
      {"Release lateness", {worker.latest_release_lateness_ms, worker.maximum_release_lateness_ms}},
      {"Execution", {worker.latest_execution_ms, worker.maximum_execution_ms}},
      {"Solver", {worker.latest_solver_ms, worker.maximum_solver_ms}},
      {"Non-solver execution",
       {worker.latest_non_solver_execution_ms, worker.maximum_non_solver_execution_ms}},
      {"Release to finish",
       {worker.latest_release_to_finish_ms, worker.maximum_release_to_finish_ms}},
      {"Overrun", {worker.latest_overrun_ms, worker.maximum_overrun_ms}}};
    for (const auto & timing : timings) {
      timing_rows.push_back(
        {worker.label, timing.first, fixed(timing.second.first, 4),
         fixed(timing.second.second, 4)});
    }
    counter_rows.push_back(
      {worker.label, fixed(worker.configured_rate_hz, 0),
       std::to_string(worker.iteration_count), std::to_string(worker.deadline_miss_count),
       std::to_string(worker.consecutive_deadline_misses),
       std::to_string(worker.skipped_release_count),
       std::to_string(worker.recoverable_rejection_count)});
  }
  if (timing_rows.empty()) {
    timing_rows.push_back(noneRow(4U));
    counter_rows.push_back(noneRow(7U));
  }
  page.sections.push_back(sheet(
    "Worker timing [ms]",
    {textColumn("Worker"), textColumn("Metric"), numberColumn("Latest"),
     numberColumn("Maximum")},
    std::move(timing_rows)));
  page.sections.push_back(sheet(
    "Worker counters",
    {textColumn("Worker"), numberColumn("Rate [Hz]"), numberColumn("Iterations"),
     numberColumn("Deadline misses"), numberColumn("Consecutive deadline misses"),
     numberColumn("Skipped releases"), numberColumn("Recoverable rejections")},
    std::move(counter_rows)));

  std::vector<std::vector<std::string>> processor_rows;
  for (const auto & processor : frame.cpu_affinities) {
    processor_rows.push_back(
      {processor.role, yesNo(processor.enabled), std::to_string(processor.thread_id),
       processorList(processor.requested_cpus), processorList(processor.effective_cpus)});
  }
  if (processor_rows.empty()) {
    processor_rows.push_back(noneRow(5U));
  }
  page.sections.push_back(sheet(
    "Processor affinity",
    {textColumn("Role"), textColumn("Bound"), numberColumn("Thread identifier"),
     textColumn("Requested processors"), textColumn("Effective processors")},
    std::move(processor_rows), 1U));

  std::vector<std::vector<std::string>> collision_rows;
  for (const auto & collision : frame.self_collisions) {
    collision_rows.push_back(
      {collision.label, fixed(collision.minimum_distance_m, 5),
       fixed(collision.influence_distance_m, 5), fixed(collision.minimum_distance_before_m, 5),
       fixed(collision.minimum_distance_after_m, 5), fixed(collision.margin_shortfall_m, 5),
       std::to_string(collision.input_state_sequence)});
  }
  if (collision_rows.empty()) {
    collision_rows.push_back(noneRow(7U));
  }
  page.sections.push_back(sheet(
    "Collision summary",
    {textColumn("Group"), numberColumn("Minimum [m]"), numberColumn("Influence [m]"),
     numberColumn("Before [m]"), numberColumn("After [m]"),
     numberColumn("Shortfall [m]"), numberColumn("State sequence")},
    std::move(collision_rows), 1U));
  return page;
}

TuiPage makeEventsPage(
  const IkDebugFrame & frame,
  const std::string & input_status)
{
  TuiPage page;
  page.title = "Events";
  std::string solver_state = "none";
  if (!frame.solvers.empty()) {
    solver_state.clear();
    for (const auto & solver : frame.solvers) {
      if (!solver_state.empty()) {
        solver_state += " / ";
      }
      solver_state += solver.label + " " + solver.disposition;
    }
  }
  TuiSection state = sheet(
    "Current state",
    {textColumn("Metric"), textColumn("Value")},
    {{"Runtime", runtimeState(frame.runtime_state)},
     {"Input", input_status},
     {"IK disposition", solver_state},
     {"Rejected target",
      frame.rejected_target.has_value() ? std::to_string(frame.rejected_target->revision) :
                                          "none"}});
  if (!frame.ik_status.empty()) {
    state.lines.push_back("IK: " + frame.ik_status);
  }
  if (!frame.status.empty()) {
    state.lines.push_back(frame.status);
  }
  if (frame.rejected_target.has_value() && !frame.rejected_target->detail.empty()) {
    state.lines.push_back(frame.rejected_target->detail);
  }
  page.sections.push_back(std::move(state));

  std::vector<std::vector<std::string>> pair_rows;
  for (const auto & collision : frame.self_collisions) {
    for (const auto & pair : collision.pairs) {
      pair_rows.push_back(
        {collision.label, pair.first_link, pair.second_link, fixed(pair.distance_before_m, 6),
         fixed(pair.distance_after_m, 6), yesNo(pair.active)});
    }
  }
  if (pair_rows.empty()) {
    pair_rows.push_back(noneRow(6U));
  }
  page.sections.push_back(sheet(
    "Self-collision pairs",
    {textColumn("Group"), textColumn("First link"), textColumn("Second link"),
     numberColumn("Before [m]"), numberColumn("After [m]"), textColumn("Active")},
    std::move(pair_rows)));
  return page;
}

}  // namespace

TuiDocument makeStandardIkTuiDocument(
  const IkDebugFrame & frame,
  const InteractiveIkPresentation & presentation,
  std::size_t publish_count,
  const std::string & sink_status,
  const std::string & title,
  const std::string & input_status)
{
  TuiDocument document;
  document.title = title;
  document.subtitle = sink_status + "  publish sequence=" + std::to_string(publish_count);
  document.status = input_status;
  document.header_left =
    std::string{"selected arm="} + armSideName(frame.selected_side) + "   " +
    runtimeState(frame.runtime_state) + (frame.paused ? "   PAUSED" : "   LIVE");
  document.header_right = "IK status=" + frame.ik_status;

  document.pages.push_back(makeOverviewPage(frame, presentation));
  if (frame.cartesian_planner.has_value()) {
    document.pages.push_back(makeCartesianPlanningPage(frame));
  }
  document.pages.push_back(makeSolverPage(frame));
  document.pages.push_back(makeJointStatePage(frame));
  document.pages.push_back(makeRuntimePage(frame));
  document.pages.push_back(makeEventsPage(frame, input_status));

  const std::string page_count = std::to_string(document.pages.size());
  document.footer_hints =
    "Space pause · 1–" + page_count + " pages · ? help · x exit";
  document.help_lines = {
    "1.." + page_count + "/F1..F" + page_count +
      "/Tab/BackTab: pages; PageUp/PageDown/Home/End: scroll; h/?: help",
    "Arrow Left/Right: select arm; Arrow Up/Down: step size; m: set step",
    "w/s: +/-x; a/d: +/-y; q/e: +/-z; n: rotation axis; i/u: rotate; r: reset",
    "Space: pause/resume; .: replay single step; x: exit"};
  return document;
}

}  // namespace motion_control_lab
