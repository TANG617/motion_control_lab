#include "components/tui/standard_ik_tui.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <iterator>
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

std::string compactLabel(const std::string & value, std::size_t maximum = 18U)
{
  return value.size() <= maximum ? value : value.substr(0U, maximum - 3U) + "...";
}

std::string runState(const IkDebugFrame & frame)
{
  if (frame.runtime_state == IkRuntimeState::FaultHold) {
    return "FAULT · HOLD";
  }
  if (frame.runtime_state == IkRuntimeState::RecoverableReject) {
    return std::string{frame.paused ? "PAUSED" : "LIVE"} + " · REJECT · HOLD";
  }
  return frame.paused ? "PAUSED" : "LIVE";
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

std::string statusToken(const std::string & value)
{
  std::string normalized;
  normalized.reserve(value.size());
  std::transform(value.begin(), value.end(), std::back_inserter(normalized), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (normalized.find("maximum") != std::string::npos ||
      normalized.find("max_iter") != std::string::npos) {
    return "MAX_ITER";
  }
  if (normalized.find("infeasible") != std::string::npos) {
    return "INFEASIBLE";
  }
  if (normalized.find("numeric") != std::string::npos ||
      normalized.find("nan") != std::string::npos) {
    return "NUMERIC";
  }
  if (normalized.find("optimal") != std::string::npos ||
      normalized.find("solved") != std::string::npos ||
      normalized.find("accepted") != std::string::npos) {
    return "SOLVED";
  }
  return value.empty() ? "-" : value;
}

std::string solverExit(const SolverDebug & solver)
{
  for (const auto & pass : solver.qp_passes) {
    if (pass.attempted && !pass.succeeded) {
      return pass.label + " " + statusToken(pass.status + " " + pass.native_status);
    }
  }
  return statusToken(solver.qp_status + " " + solver.native_status);
}

TuiTableColumn textColumn(std::string title, int cell_width = 0)
{
  if (cell_width > static_cast<int>(title.size())) {
    title.append(static_cast<std::size_t>(cell_width) - title.size(), ' ');
  }
  return TuiTableColumn{std::move(title), Alignment::Left};
}

TuiTableColumn numberColumn(std::string title, int cell_width = 8)
{
  if (cell_width > static_cast<int>(title.size())) {
    title.insert(0U, static_cast<std::size_t>(cell_width) - title.size(), ' ');
  }
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
  section.tables.push_back(TuiTable{std::move(columns), std::move(rows), TuiTableStyle::Compact});
  section.column = column;
  section.row = row;
  section.style = TuiSectionStyle::Panel;
  return section;
}

TuiSection details(
  std::string title,
  std::vector<TuiRow> rows,
  std::size_t column = 0U,
  std::size_t row = 0U)
{
  TuiSection section;
  section.title = std::move(title);
  section.rows = std::move(rows);
  section.column = column;
  section.row = row;
  section.style = TuiSectionStyle::Panel;
  return section;
}

std::vector<std::string> noneRow(std::size_t width, std::string label = "none")
{
  std::vector<std::string> row(width, "-");
  row.front() = std::move(label);
  return row;
}

std::pair<double, double> poseError(const Pose & first, const Pose & second)
{
  return {
    (first.translation() - second.translation()).norm(),
    Eigen::AngleAxisd(first.linear() * second.linear().transpose()).angle()};
}

std::string xyz(const Pose & pose)
{
  return signedFixed(pose.translation().x()) + " " + signedFixed(pose.translation().y()) + " " +
         signedFixed(pose.translation().z());
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

std::string jointPosition(const IkDebugFrame & frame, std::size_t index)
{
  return index < frame.positions.size() ? fixed(frame.positions[index], 5) : "-";
}

std::string jointVelocity(const IkDebugFrame & frame, std::size_t index)
{
  return index < frame.velocities.size() ? fixed(frame.velocities[index], 5) : "-";
}

TuiPage makeMonitorPage(const IkDebugFrame & frame)
{
  TuiPage page;
  page.title = "Monitor";
  page.rows = {{{3, 2}, 3}, {{3, 2}, 2}};

  std::vector<std::vector<std::string>> tracking_rows;
  if (frame.cartesian_planner.has_value()) {
    for (const auto & arm : frame.cartesian_planner->arms) {
      const auto goal_reference = poseError(arm.source_goal, arm.reference);
      const auto reference_output = poseError(arm.reference, arm.forward_kinematics);
      const auto goal_output = poseError(arm.source_goal, arm.forward_kinematics);
      tracking_rows.push_back(
        {armSideName(arm.side), xyz(arm.source_goal), xyz(arm.reference),
         xyz(arm.forward_kinematics), fixed(goal_reference.first, 6),
         fixed(reference_output.first, 6), fixed(goal_output.second, 6)});
    }
  } else {
    for (const auto & target : frame.targets) {
      if (const auto * output = findForwardKinematics(frame, target.side)) {
        const auto error = poseError(target.target_pose, output->pose);
        tracking_rows.push_back(
          {armSideName(target.side), xyz(target.target_pose), "-", xyz(output->pose), "-",
           fixed(error.first, 6), fixed(error.second, 6)});
      }
    }
  }
  if (tracking_rows.empty()) {
    tracking_rows.push_back(noneRow(7U));
  }
  page.sections.push_back(sheet(
    "TCP tracking · Goal → Reference → Output",
    {textColumn("Arm", 5), textColumn("Goal xyz [m]", 23),
     textColumn("Reference xyz [m]", 23), textColumn("Output xyz [m]", 23),
     numberColumn("G→R [m]"),
     numberColumn("R→O [m]"), numberColumn("G→O [rad]")},
    std::move(tracking_rows), 0U, 0U));

  std::vector<std::vector<std::string>> solver_rows;
  for (const auto & solver : frame.solvers) {
    solver_rows.push_back(
      {compactLabel(solver.label), solver.disposition, solverExit(solver),
       std::to_string(solver.ik_iterations), fixed(solver.ik_solve_time_ms, 3),
       fixed(solver.maximum_hard_violation, 7)});
  }
  if (solver_rows.empty()) {
    solver_rows.push_back(noneRow(6U));
  }
  page.sections.push_back(sheet(
    "Solver",
    {textColumn("Level", 6), textColumn("Attempt", 8), textColumn("Exit", 18),
     numberColumn("Iter", 5), numberColumn("ms", 7), numberColumn("Hard", 10)},
    std::move(solver_rows), 1U, 0U));

  std::vector<std::vector<std::string>> worker_rows;
  for (const auto & worker : frame.workers) {
    worker_rows.push_back(
      {worker.label, fixed(worker.configured_rate_hz, 0), fixed(worker.latest_execution_ms, 3),
       fixed(worker.maximum_execution_ms, 3), std::to_string(worker.deadline_miss_count),
       std::to_string(worker.skipped_release_count),
       std::to_string(worker.recoverable_rejection_count)});
  }
  if (worker_rows.empty()) {
    worker_rows.push_back(noneRow(7U));
  }
  page.sections.push_back(sheet(
    "Workers",
    {textColumn("Worker", 6), numberColumn("Hz", 4), numberColumn("Now [ms]"),
     numberColumn("Max [ms]"), numberColumn("Miss"), numberColumn("Skip"),
     numberColumn("Reject")},
    std::move(worker_rows), 0U, 1U));

  std::vector<std::vector<std::string>> safety_rows;
  safety_rows.push_back({"Runtime", runState(frame)});
  safety_rows.push_back({"Selected arm", armSideName(frame.selected_side)});
  if (frame.replay_frame_progress.has_value()) {
    safety_rows.push_back(
      {"Replay", std::to_string(frame.replay_frame_progress->source_index + 1U) + "/" +
                   std::to_string(frame.replay_frame_progress->source_frame_count)});
  }
  safety_rows.push_back(
    {"Rejected target", frame.rejected_target.has_value()
                          ? std::to_string(frame.rejected_target->revision)
                          : "-"});
  for (const auto & collision : frame.self_collisions) {
    safety_rows.push_back(
      {collision.label + " distance", fixed(collision.minimum_distance_after_m, 5) + " m"});
    if (collision.margin_shortfall_m > 0.0) {
      safety_rows.push_back(
        {collision.label + " shortfall", fixed(collision.margin_shortfall_m, 5) + " m"});
    }
  }
  page.sections.push_back(sheet(
    "Safety and hold",
    {textColumn("Signal", 31), textColumn("Value", 42)}, std::move(safety_rows), 1U, 1U));

  struct FailurePreview
  {
    const SolverDebug * solver;
    const QpPassDebug * pass;
    const QpConstraintViolationDebug * violation;
  };
  std::vector<FailurePreview> failure_preview;
  for (const auto & solver : frame.solvers) {
    for (const auto & pass : solver.qp_passes) {
      if (!pass.attempted || pass.succeeded) {
        continue;
      }
      for (const auto & violation : pass.constraint_violations) {
        failure_preview.push_back({&solver, &pass, &violation});
      }
    }
  }
  if (!failure_preview.empty()) {
    std::sort(
      failure_preview.begin(), failure_preview.end(), [](const auto & left, const auto & right) {
        return left.violation->violation > right.violation->violation;
      });
    std::vector<std::vector<std::string>> preview_rows;
    const std::size_t preview_count = std::min<std::size_t>(failure_preview.size(), 5U);
    for (std::size_t index = 0U; index < preview_count; ++index) {
      const auto & item = failure_preview[index];
      preview_rows.push_back(
        {std::to_string(index + 1U), item.solver->label, item.pass->label,
         item.violation->source, item.violation->component, item.violation->bound_source,
         item.violation->side, fixed(item.violation->value, 9),
         "[" + fixed(item.violation->lower, 9) + ", " + fixed(item.violation->upper, 9) + "]",
         fixed(item.violation->violation, 9), item.violation->unit});
    }
    page.rows.push_back({{1}, 1});
    page.sections.push_back(sheet(
      "Failure focus · top violations from failed candidate",
      {numberColumn("#", 1), textColumn("Solver", 6), textColumn("Pass", 7),
       textColumn("Source", 32), textColumn("Comp", 8), textColumn("Bound", 18),
       textColumn("Side", 5), numberColumn("Candidate", 12), textColumn("Allowed", 29),
       numberColumn("Excess", 12), textColumn("Unit", 8)},
      std::move(preview_rows), 0U, 2U));
  }
  return page;
}

TuiPage makeMotionPage(const IkDebugFrame & frame)
{
  const auto & planner = *frame.cartesian_planner;
  TuiPage page;
  page.title = "Motion";
  page.rows = {{{2, 3}, 1}, {{1}, 3}, {{1}, 2}};

  page.sections.push_back(sheet(
    "Pipeline",
    {textColumn("Stage"), textColumn("State"), numberColumn("dt [s]")},
    {{"Cartesian", planner.state, fixed(planner.sample_time_s, 6)}}, 0U, 0U));

  std::vector<std::vector<std::string>> error_rows;
  std::vector<std::vector<std::string>> pose_rows;
  std::vector<std::vector<std::string>> reference_rows;
  for (const auto & arm : planner.arms) {
    const std::string side = armSideName(arm.side);
    const auto goal_reference = poseError(arm.source_goal, arm.reference);
    const auto reference_output = poseError(arm.reference, arm.forward_kinematics);
    const auto goal_output = poseError(arm.source_goal, arm.forward_kinematics);
    error_rows.push_back(
      {side, fixed(goal_reference.first, 6), fixed(reference_output.first, 6),
       fixed(goal_output.first, 6), fixed(goal_output.second, 6)});

    for (const auto & state : std::vector<std::pair<std::string, Pose>>{
           {"Goal", arm.source_goal}, {"Reference", arm.reference}, {"Output", arm.forward_kinematics}}) {
      const Eigen::Quaterniond q(state.second.linear());
      pose_rows.push_back(
        {side, state.first, fixed(state.second.translation().x(), 4),
         fixed(state.second.translation().y(), 4), fixed(state.second.translation().z(), 4),
         fixed(q.x(), 4), fixed(q.y(), 4), fixed(q.z(), 4), fixed(q.w(), 4)});
    }
    reference_rows.push_back(
      {side, fixed(arm.reference_twist[0], 4), fixed(arm.reference_twist[1], 4),
       fixed(arm.reference_twist[2], 4), fixed(arm.reference_twist[3], 4),
       fixed(arm.reference_twist[4], 4), fixed(arm.reference_twist[5], 4),
       fixed(arm.reference_acceleration[0], 4), fixed(arm.reference_acceleration[1], 4),
       fixed(arm.reference_acceleration[2], 4), fixed(arm.reference_acceleration[3], 4),
       fixed(arm.reference_acceleration[4], 4), fixed(arm.reference_acceleration[5], 4)});
  }
  if (error_rows.empty()) {
    error_rows.push_back(noneRow(5U));
    pose_rows.push_back(noneRow(9U));
    reference_rows.push_back(noneRow(13U));
  }
  page.sections.push_back(sheet(
    "Tracking error",
    {textColumn("Arm"), numberColumn("Goal→Ref [m]"), numberColumn("Ref→Out [m]"),
     numberColumn("Goal→Out [m]"), numberColumn("Goal→Out [rad]")},
    std::move(error_rows), 1U, 0U));
  page.sections.push_back(sheet(
    "Cartesian states",
    {textColumn("Arm"), textColumn("Domain"), numberColumn("X [m]"), numberColumn("Y [m]"),
     numberColumn("Z [m]"), numberColumn("Qx"), numberColumn("Qy"), numberColumn("Qz"),
     numberColumn("Qw")},
    std::move(pose_rows), 0U, 1U));
  page.sections.push_back(sheet(
    "Reference twist and acceleration",
    {textColumn("Arm"), numberColumn("Vx"), numberColumn("Vy"), numberColumn("Vz"),
     numberColumn("Wx"), numberColumn("Wy"), numberColumn("Wz"), numberColumn("Ax"),
     numberColumn("Ay"), numberColumn("Az"), numberColumn("Aωx"), numberColumn("Aωy"),
     numberColumn("Aωz")},
    std::move(reference_rows), 0U, 2U));
  return page;
}

struct RankedViolation
{
  const SolverDebug * solver{nullptr};
  const QpPassDebug * pass{nullptr};
  const QpConstraintViolationDebug * violation{nullptr};
};

TuiPage makeSolverPage(const IkDebugFrame & frame)
{
  TuiPage page;
  page.title = "Solver";
  page.rows = {{{1}, 1}, {{1}, 2}, {{3, 2}, 2}, {{1}, 1}};

  std::vector<std::vector<std::string>> pass_rows;
  std::vector<RankedViolation> violations;
  const QpPassDebug * first_failed_pass = nullptr;
  const SolverDebug * first_failed_solver = nullptr;
  for (const auto & solver : frame.solvers) {
    for (const auto & pass : solver.qp_passes) {
      const std::string result = !pass.attempted ? "SKIP" : pass.succeeded ? "OK" : "FAIL";
      pass_rows.push_back(
        {solver.label, pass.label, result, statusToken(pass.status + " " + pass.native_status),
         std::to_string(pass.iterations), fixed(pass.solve_time_ms, 4),
         pass.solve_time_percentiles.window_sample_count == 0U
           ? "-"
           : fixed(pass.solve_time_percentiles.p90, 4),
         pass.solve_time_percentiles.window_sample_count == 0U
           ? "-"
           : fixed(pass.solve_time_percentiles.p95, 4),
         pass.solve_time_percentiles.window_sample_count == 0U
           ? "-"
           : fixed(pass.solve_time_percentiles.p99, 4),
         yesNo(pass.warm_start_used)});
      if (pass.attempted && !pass.succeeded) {
        if (first_failed_pass == nullptr) {
          first_failed_pass = &pass;
          first_failed_solver = &solver;
        }
        for (const auto & violation : pass.constraint_violations) {
          violations.push_back({&solver, &pass, &violation});
        }
      }
    }
  }
  if (pass_rows.empty()) {
    pass_rows.push_back(noneRow(10U));
  }
  page.sections.push_back(sheet(
    "Passes",
    {textColumn("Solver", 8), textColumn("Pass", 10), textColumn("Result", 6),
     textColumn("Exit", 12), numberColumn("Iter", 5), numberColumn("Time [ms]"),
     numberColumn("P90 [ms]"),
     numberColumn("P95 [ms]"), numberColumn("P99 [ms]"), textColumn("Warm")},
    std::move(pass_rows), 0U, 0U));

  std::sort(violations.begin(), violations.end(), [](const auto & left, const auto & right) {
    return left.violation->violation > right.violation->violation;
  });
  std::vector<std::vector<std::string>> violation_rows;
  for (std::size_t index = 0U; index < violations.size(); ++index) {
    const auto & item = violations[index];
    violation_rows.push_back(
      {std::to_string(index + 1U), item.solver->label, item.pass->label,
       item.violation->source, item.violation->component,
       item.violation->kind + " / " + item.violation->bound_source,
       item.violation->side, fixed(item.violation->value, 9),
       "[" + fixed(item.violation->lower, 9) + ", " + fixed(item.violation->upper, 9) + "]",
       fixed(item.violation->violation, 9), item.violation->unit});
  }
  if (violation_rows.empty()) {
    const std::string evidence = first_failed_pass == nullptr
                                   ? "no failed pass"
                                   : first_failed_pass->last_iterate_available
                                       ? "no violation observed"
                                       : "last iterate unavailable";
    violation_rows.push_back(noneRow(11U, evidence));
  }
  page.sections.push_back(sheet(
    "Last-iterate violations · diagnostic only",
    {numberColumn("#", 3), textColumn("Solver", 8), textColumn("Pass", 10),
     textColumn("Source", 32), textColumn("Comp", 8), textColumn("Constraint / bound", 24),
     textColumn("Side", 5), numberColumn("Candidate", 12), textColumn("Allowed", 29),
     numberColumn("Excess", 12), textColumn("Unit", 8)},
    std::move(violation_rows), 0U, 1U));

  std::vector<TuiRow> context_rows;
  if (!violations.empty()) {
    const auto & selected = violations.front();
    context_rows = {
      {"Rank", "1 / " + std::to_string(violations.size())},
      {"Constraint", selected.violation->source + " / " + selected.violation->component},
      {"Bound", selected.violation->kind + " / " + selected.violation->bound_source + " / " +
                  selected.violation->side},
      {"Candidate", fixed(selected.violation->value, 9) + " " + selected.violation->unit},
      {"Allowed", "[" + fixed(selected.violation->lower, 9) + ", " +
                    fixed(selected.violation->upper, 9) + "] " + selected.violation->unit},
      {"Excess", fixed(selected.violation->violation, 9) + " " + selected.violation->unit},
      {"Residuals", "primal " + fixed(selected.pass->primal_residual, 9) + " · dual " +
                      fixed(selected.pass->dual_residual, 9)},
      {"Meaning", "failed candidate only; accepted output is unchanged"}};
  } else if (first_failed_pass != nullptr) {
    context_rows = {
      {"Pass", first_failed_solver->label + " / " + first_failed_pass->label},
      {"Exit", statusToken(first_failed_pass->status + " " + first_failed_pass->native_status)},
      {"Last iterate", yesNo(first_failed_pass->last_iterate_available)},
      {"Primal residual", fixed(first_failed_pass->primal_residual, 9)},
      {"Dual residual", fixed(first_failed_pass->dual_residual, 9)},
      {"Meaning", "failed candidate only; accepted output is unchanged"}};
  } else {
    context_rows = {{"Failure", "none"}, {"State", "all attempted passes succeeded"}};
  }
  page.sections.push_back(details("Failure context", std::move(context_rows), 0U, 2U));

  std::vector<TuiTableColumn> attempt_columns{textColumn("Metric", 12)};
  for (const auto & solver : frame.solvers) {
    attempt_columns.push_back(textColumn(solver.label, 18));
  }
  if (frame.solvers.empty()) {
    attempt_columns.push_back(textColumn("Value", 18));
  }
  std::vector<std::vector<std::string>> attempt_rows;
  const auto addAttemptMetric = [&](const std::string & label, auto value) {
    std::vector<std::string> row{label};
    for (const auto & solver : frame.solvers) {
      row.push_back(value(solver));
    }
    if (frame.solvers.empty()) {
      row.push_back("-");
    }
    attempt_rows.push_back(std::move(row));
  };
  addAttemptMetric("Attempt", [](const auto & solver) { return solver.disposition; });
  addAttemptMetric("Exit", [](const auto & solver) { return solverExit(solver); });
  addAttemptMetric("IK iter", [](const auto & solver) { return std::to_string(solver.ik_iterations); });
  addAttemptMetric("IK [ms]", [](const auto & solver) { return fixed(solver.ik_solve_time_ms, 4); });
  addAttemptMetric("QP [ms]", [](const auto & solver) { return fixed(solver.qp_solve_time_ms, 4); });
  addAttemptMetric("P90 [ms]", [](const auto & solver) {
    return fixed(solver.ik_solve_time_percentiles.p90, 4);
  });
  addAttemptMetric("P95 [ms]", [](const auto & solver) {
    return fixed(solver.ik_solve_time_percentiles.p95, 4);
  });
  addAttemptMetric("P99 [ms]", [](const auto & solver) {
    return fixed(solver.ik_solve_time_percentiles.p99, 4);
  });
  addAttemptMetric("Objective", [](const auto & solver) {
    return fixed(solver.objective_value, 7);
  });
  addAttemptMetric("Primal", [](const auto & solver) {
    return fixed(solver.primal_residual, 7);
  });
  addAttemptMetric("Dual", [](const auto & solver) {
    return fixed(solver.dual_residual, 7);
  });
  addAttemptMetric("Hard max", [](const auto & solver) {
    return fixed(solver.maximum_hard_violation, 9);
  });
  addAttemptMetric("Active set", [](const auto & solver) {
    return std::to_string(solver.active_set_size);
  });
  addAttemptMetric("Warm start", [](const auto & solver) {
    return yesNo(solver.warm_start_used);
  });
  addAttemptMetric("Runs A/O/R", [](const auto & solver) {
    if (!solver.run_counters.has_value()) {
      return std::string{"-"};
    }
    return std::to_string(solver.run_counters->attempts) + "/" +
           std::to_string(solver.run_counters->accepted) + "/" +
           std::to_string(solver.run_counters->rejected);
  });
  page.sections.push_back(sheet(
    "Attempt summary", std::move(attempt_columns), std::move(attempt_rows), 1U, 2U));

  std::vector<std::vector<std::string>> task_rows;
  for (const auto & solver : frame.solvers) {
    for (const auto & scale : solver.task_scales) {
      task_rows.push_back(
        {solver.label, "task", scale.name, scale.active ? "active" : "idle",
         fixed(scale.scale, 5), fixed(scale.cost, 5),
         scale.degraded ? "degraded" : scale.stuck ? "stuck" : "-"});
    }
    for (const auto & requirement : solver.requirements) {
      task_rows.push_back(
        {solver.label, "constraint", requirement.name,
         requirement.active ? "active" : requirement.enabled ? "enabled" : "off",
         fixed(requirement.maximum_violation, 7) + " " + requirement.unit,
         fixed(requirement.cost, 5), requirement.source});
    }
  }
  if (task_rows.empty()) {
    task_rows.push_back(noneRow(7U));
  }
  page.sections.push_back(sheet(
    "Tasks and constraints",
    {textColumn("Solver"), textColumn("Type"), textColumn("Name"), textColumn("State"),
     numberColumn("Scale / violation"), numberColumn("Cost"), textColumn("Source / flag")},
    std::move(task_rows), 0U, 3U));
  return page;
}

TuiPage makeJointsPage(const IkDebugFrame & frame)
{
  TuiPage page;
  page.title = "Joints";
  std::vector<std::vector<std::string>> rows;
  for (std::size_t index = 0U; index < frame.joint_names.size(); ++index) {
    rows.push_back({frame.joint_names[index], jointPosition(frame, index), jointVelocity(frame, index)});
  }
  if (rows.empty()) {
    rows.push_back(noneRow(3U));
  }
  page.sections.push_back(sheet(
    "Executed state",
    {textColumn("Joint"), numberColumn("Position [rad]"), numberColumn("Velocity [rad/s]")},
    std::move(rows)));
  return page;
}

TuiPage makeSystemPage(const IkDebugFrame & frame, const std::string & input_status)
{
  TuiPage page;
  page.title = "System";

  std::vector<std::vector<std::string>> worker_rows;
  for (const auto & worker : frame.workers) {
    worker_rows.push_back(
      {worker.label, fixed(worker.configured_rate_hz, 0), std::to_string(worker.iteration_count),
       fixed(worker.latest_execution_ms, 4), fixed(worker.maximum_execution_ms, 4),
       fixed(worker.maximum_solver_ms, 4), fixed(worker.maximum_non_solver_execution_ms, 4),
       fixed(worker.maximum_release_lateness_ms, 4),
       std::to_string(worker.deadline_miss_count), std::to_string(worker.skipped_release_count),
       std::to_string(worker.recoverable_rejection_count)});
  }
  if (worker_rows.empty()) {
    worker_rows.push_back(noneRow(11U));
  }
  page.sections.push_back(sheet(
    "Worker runtime",
    {textColumn("Worker"), numberColumn("Hz"), numberColumn("Iterations"),
     numberColumn("Now [ms]"), numberColumn("Max [ms]"), numberColumn("Solver max"),
     numberColumn("Non-QP max"), numberColumn("Late max"), numberColumn("Miss"),
     numberColumn("Skip"), numberColumn("Reject")},
    std::move(worker_rows)));

  std::vector<std::vector<std::string>> affinity_rows;
  for (const auto & affinity : frame.cpu_affinities) {
    affinity_rows.push_back(
      {affinity.role, yesNo(affinity.enabled), std::to_string(affinity.thread_id),
       processorList(affinity.requested_cpus), processorList(affinity.effective_cpus)});
  }
  if (affinity_rows.empty()) {
    affinity_rows.push_back(noneRow(5U));
  }
  page.sections.push_back(sheet(
    "Processor affinity",
    {textColumn("Role"), textColumn("Bound"), numberColumn("TID"), textColumn("Requested"),
     textColumn("Effective")},
    std::move(affinity_rows)));

  std::vector<TuiRow> state_rows{
    {"Run", runState(frame)}, {"Input", input_status}, {"IK", frame.ik_status},
    {"Detail", frame.status}};
  if (frame.rejected_target.has_value()) {
    state_rows.push_back({"Rejected revision", std::to_string(frame.rejected_target->revision)});
  }
  page.sections.push_back(details("Current state", std::move(state_rows)));

  std::vector<std::vector<std::string>> collision_rows;
  for (const auto & collision : frame.self_collisions) {
    for (const auto & pair : collision.pairs) {
      collision_rows.push_back(
        {collision.label, pair.first_link, pair.second_link, fixed(pair.distance_before_m, 6),
         fixed(pair.distance_after_m, 6), pair.active ? "active" : "clear"});
    }
  }
  if (collision_rows.empty()) {
    collision_rows.push_back(noneRow(6U));
  }
  page.sections.push_back(sheet(
    "Collision pairs",
    {textColumn("Group"), textColumn("Link A"), textColumn("Link B"),
     numberColumn("Before [m]"), numberColumn("After [m]"), textColumn("State")},
    std::move(collision_rows)));
  return page;
}

}  // namespace

TuiPage makeStandardCapabilityPage(std::string title, std::vector<TuiSection> panels)
{
  TuiPage page;
  page.title = std::move(title);
  for (auto & panel : panels) {
    panel.style = TuiSectionStyle::Panel;
  }
  if (panels.size() <= 1U) {
    page.rows = {{{1}, 1}};
  } else if (panels.size() == 2U) {
    page.rows = {{{1, 1}, 1}};
    panels[1].column = 1U;
  } else {
    page.rows = {{{1, 1}, 1}, {{1, 1}, 1}};
    for (std::size_t index = 0U; index < panels.size(); ++index) {
      panels[index].row = index / 2U;
      panels[index].column = index % 2U;
    }
    for (std::size_t index = 4U; index < panels.size(); ++index) {
      page.rows.push_back({{1}, 1});
      panels[index].row = index - 2U;
      panels[index].column = 0U;
    }
  }
  page.sections = std::move(panels);
  return page;
}

TuiDocument makeStandardIkTuiDocument(
  const IkDebugFrame & frame,
  const InteractiveIkPresentation & presentation,
  std::size_t publish_count,
  const std::string & sink_status,
  const std::string & title,
  const std::string & input_status)
{
  static_cast<void>(presentation);
  TuiDocument document;
  document.title = title;
  document.subtitle = "Sink " + sink_status + " · Pub " + std::to_string(publish_count);
  if (frame.replay_frame_progress.has_value()) {
    const auto & replay = *frame.replay_frame_progress;
    document.subtitle += " · Replay " + std::to_string(replay.source_index + 1U) + "/" +
                         std::to_string(replay.source_frame_count);
  }
  document.status = input_status;
  document.header_left = runState(frame) + " · arm " + armSideName(frame.selected_side);
  if (frame.rejected_target.has_value()) {
    document.header_left += " · target " + std::to_string(frame.rejected_target->revision) +
                            " rejected";
  }
  for (const auto & solver : frame.solvers) {
    if (!document.header_right.empty()) {
      document.header_right += " · ";
    }
    document.header_right += solver.label + " " + solverExit(solver);
  }

  document.pages.push_back(makeMonitorPage(frame));
  if (frame.cartesian_planner.has_value()) {
    document.pages.push_back(makeMotionPage(frame));
  }
  document.pages.push_back(makeSolverPage(frame));
  document.pages.push_back(makeJointsPage(frame));
  document.pages.push_back(makeSystemPage(frame, input_status));

  const std::string page_count = std::to_string(document.pages.size());
  document.footer_hints =
    "Space pause · 1–" + page_count + " pages · PgUp/PgDn scroll · ? help · x exit";
  document.help_lines = {
    "1.." + page_count + "/F1..F" + page_count +
      "/Tab/BackTab: pages; PageUp/PageDown/Home/End: scroll; h/?: help",
    "Arrow Left/Right: select arm; Arrow Up/Down: step size; m: set step",
    "w/s: +/-x; a/d: +/-y; q/e: +/-z; n: rotation axis; i/u: rotate; r: reset",
    "Space: pause/resume; .: replay single step; x: exit"};
  return document;
}

}  // namespace motion_control_lab
