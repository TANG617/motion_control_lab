#include "loop.hpp"

#include "components/teleop/keyboard/keyboard_target_source.hpp"
#include "components/tui/planned_grouped_tui.hpp"
#include "components/visualization/preview_projection.hpp"
#include "components/visualization/preview_transport.hpp"
#include "contracts/visualization/mcl_execution_v1.hpp"
#include "contracts/visualization/mcl_planning_v1.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <motion_control_sim/mujoco_torque_simulation.hpp>
#include <motion_control_sim/mujoco_viewer.hpp>
#include <motion_control_viz/render_batch.hpp>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace motion_control_lab::hierarchical_inverse_dynamics_torque_sim {
namespace {

namespace mcs = motion_control::sim;
namespace mcv = motion_control::viz;

constexpr const char *kProgramId =
    "mcl_hierarchical_inverse_dynamics_torque_sim";
constexpr const char *kTitle =
    "Motion Control Hierarchical Inverse Dynamics Torque Sim";

void requireOk(const mcc::Status &status, const char *operation) {
  if (!status.ok()) {
    throw std::runtime_error(std::string{operation} + ": " + status.message);
  }
}

mcc::RobotState toRobotState(const mcs::KinematicSnapshot &snapshot) {
  mcc::RobotState result;
  result.joint_positions = Eigen::Map<const Eigen::VectorXd>(
      snapshot.joints.positions.data(),
      static_cast<Eigen::Index>(snapshot.joints.positions.size()));
  result.joint_velocities = Eigen::Map<const Eigen::VectorXd>(
      snapshot.joints.velocities.data(),
      static_cast<Eigen::Index>(snapshot.joints.velocities.size()));
  return result;
}

std::uint64_t monotonicNowNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
      std::ceil(fraction * static_cast<double>(values.size())) - 1.0);
  return values[std::min(index, values.size() - 1)];
}

double orientationError(const mcc::Pose &target, const mcc::Pose &actual) {
  return Eigen::AngleAxisd(target.linear() * actual.linear().transpose())
      .angle();
}

mcc::Pose fromViewerPose(const mcs::Pose3d &source) {
  Eigen::Quaterniond orientation(
      source.orientation_wxyz[0], source.orientation_wxyz[1],
      source.orientation_wxyz[2], source.orientation_wxyz[3]);
  if (orientation.norm() <= 1.0e-12) {
    throw std::runtime_error("MuJoCo drag returned a zero quaternion");
  }
  mcc::Pose result = mcc::Pose::Identity();
  result.translation() = Eigen::Vector3d(
      source.position_m[0], source.position_m[1], source.position_m[2]);
  result.linear() = orientation.normalized().toRotationMatrix();
  return result;
}

const char *qpBackendName(mcc::QpBackend backend) {
  return backend == mcc::QpBackend::ProxQp ? "ProxQP" : "eiquadprog";
}

const char *qpStatusName(mcc::QpSolveStatus status) {
  switch (status) {
  case mcc::QpSolveStatus::Optimal:
    return "Optimal";
  case mcc::QpSolveStatus::PrimalInfeasible:
    return "Primal infeasible";
  case mcc::QpSolveStatus::DualInfeasible:
    return "Dual infeasible";
  case mcc::QpSolveStatus::MaximumIterations:
    return "Maximum iterations";
  case mcc::QpSolveStatus::NumericalFailure:
    return "Numerical failure";
  case mcc::QpSolveStatus::NotRun:
    return "Not run";
  }
  return "Unknown";
}

std::string passName(const std::optional<mcc::HierarchicalSolvePass> &pass) {
  if (!pass.has_value()) {
    return "-";
  }
  switch (*pass) {
  case mcc::HierarchicalSolvePass::Primary:
    return "primary-position";
  case mcc::HierarchicalSolvePass::Secondary:
    return "secondary-orientation";
  case mcc::HierarchicalSolvePass::Tertiary:
    return "tertiary-posture";
  case mcc::HierarchicalSolvePass::Terminal:
    return "terminal-minimum-motion";
  }
  return "-";
}

SolverDebug makeSolverDebug(
    const mcc::InverseDynamicsSolution *solution,
    const mcc::HierarchicalInverseDynamicsDiagnostics *diagnostics) {
  SolverDebug result;
  result.label = "HID";
  result.joint_limit_policy =
      "shared acceleration/velocity/position/torque bounds";
  result.backend =
      diagnostics == nullptr ? "ProxQP" : qpBackendName(diagnostics->backend);
  result.disposition =
      solution != nullptr && mcc::isAccepted(solution->disposition) ? "Accepted"
                                                                    : "Not run";
  result.termination_reason =
      diagnostics == nullptr
          ? "Awaiting first control tick"
          : "Selected " + passName(diagnostics->selected_pass);
  result.has_qp_diagnostics = diagnostics != nullptr;
  if (diagnostics == nullptr) {
    result.qp_status = "Not run";
    return result;
  }

  const mcc::InverseDynamicsPassDiagnostics *selected = nullptr;
  for (const auto &pass_diagnostics : diagnostics->passes) {
    if (!pass_diagnostics.active) {
      continue;
    }
    QpPassDebug pass;
    pass.label = passName(pass_diagnostics.pass);
    pass.attempted = pass_diagnostics.attempted;
    pass.succeeded = pass_diagnostics.succeeded;
    pass.status = qpStatusName(pass_diagnostics.backend_status);
    pass.native_status = pass_diagnostics.native_status;
    pass.solve_time_ms = pass_diagnostics.solve_time_ms;
    pass.iterations = pass_diagnostics.iterations;
    pass.warm_start_used = pass_diagnostics.warm_start_used;
    pass.objective_value = pass_diagnostics.objective_value;
    pass.primal_residual = pass_diagnostics.primal_residual;
    pass.dual_residual = pass_diagnostics.dual_residual;
    pass.last_iterate_available = pass_diagnostics.succeeded;
    result.qp_passes.push_back(std::move(pass));
    result.ik_iterations += pass_diagnostics.iterations;
    result.ik_solve_time_ms += pass_diagnostics.solve_time_ms;
    result.maximum_hard_violation = std::max(
        result.maximum_hard_violation, pass_diagnostics.maximum_hard_violation);
    if (diagnostics->selected_pass == pass_diagnostics.pass) {
      selected = &pass_diagnostics;
    }
  }
  for (const auto &constraint : diagnostics->constraints) {
    RequirementDebug item;
    item.name = constraint.name;
    item.unit = constraint.unit;
    item.source = "hierarchical-inverse-dynamics";
    item.enabled = constraint.enabled;
    item.active = constraint.enabled;
    item.maximum_violation = constraint.maximum_violation;
    item.pass = passName(diagnostics->selected_pass);
    item.evidence = constraint.name;
    item.state = constraint.enabled ? "active" : "inactive";
    item.kind = "hard/shared";
    result.requirements.push_back(std::move(item));
  }
  result.converged =
      solution != nullptr && mcc::isAccepted(solution->disposition);
  result.qp_solve_time_ms = result.ik_solve_time_ms;
  result.qp_iterations = result.ik_iterations;
  if (selected != nullptr) {
    result.qp_status = qpStatusName(selected->backend_status);
    result.native_status = selected->native_status;
    result.objective_value = selected->objective_value;
    result.primal_residual = selected->primal_residual;
    result.dual_residual = selected->dual_residual;
    result.active_set_size = selected->active_set_size;
    result.warm_start_used = selected->warm_start_used;
  }
  return result;
}

TuiPage
makeHidPage(const R1RobotConfig &robot, const Eigen::VectorXd &torque,
            const mcc::InverseDynamicsSolution *solution,
            const mcc::HierarchicalInverseDynamicsDiagnostics *diagnostics) {
  TuiPage page;
  page.title = "Dynamics";
  page.column_weights = {1, 3};
  page.rows = {{{1, 3}, 1}};

  TuiSection summary;
  summary.title = "Same-tick result";
  summary.column = 0;
  summary.row = 0;
  summary.style = TuiSectionStyle::Panel;
  summary.rows = {
      {"Disposition",
       solution != nullptr && mcc::isAccepted(solution->disposition)
           ? "Accepted"
           : "Not run"},
      {"Selected level",
       passName(diagnostics == nullptr ? std::nullopt
                                       : diagnostics->selected_pass)},
      {"Fallback level",
       passName(diagnostics == nullptr ? std::nullopt
                                       : diagnostics->same_tick_fallback_pass)},
      {"Linearization",
       diagnostics == nullptr
           ? "-"
           : std::to_string(diagnostics->workspace_update_sequence)},
      {"Model", "fixed base · torque driven"},
  };
  page.sections.push_back(std::move(summary));

  TuiTable torque_table;
  torque_table.style = TuiTableStyle::Compact;
  torque_table.columns = {{"Joint", TuiTableAlignment::Left},
                          {"Torque [Nm]", TuiTableAlignment::Right},
                          {"Limit [Nm]", TuiTableAlignment::Right},
                          {"Margin [%]", TuiTableAlignment::Right}};
  for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
    const double value = index < static_cast<std::size_t>(torque.size())
                             ? torque(static_cast<Eigen::Index>(index))
                             : 0.0;
    const double limit = robot.effort_limits[index];
    const double margin =
        limit > 0.0 ? 100.0 * (1.0 - std::abs(value) / limit) : 0.0;
    std::ostringstream torque_text;
    std::ostringstream limit_text;
    std::ostringstream margin_text;
    torque_text << std::fixed << std::setprecision(3) << value;
    limit_text << std::fixed << std::setprecision(1) << limit;
    margin_text << std::fixed << std::setprecision(1) << margin;
    torque_table.rows.push_back({robot.joint_names[index], torque_text.str(),
                                 limit_text.str(), margin_text.str()});
  }
  TuiSection torques;
  torques.title = "Actuation order · HID torque → qfrc_applied";
  torques.column = 1;
  torques.row = 0;
  torques.style = TuiSectionStyle::Panel;
  torques.tables.push_back(std::move(torque_table));
  page.sections.push_back(std::move(torques));
  return page;
}

mcv::PoseSample makePoseSample(const char *channel,
                               const std::string &reference_frame,
                               const mcc::Pose &pose) {
  const Eigen::Quaterniond orientation(pose.linear());
  mcv::PoseSample result;
  result.channel = channel;
  result.frame_id = reference_frame;
  result.pose.position_m = {pose.translation().x(), pose.translation().y(),
                            pose.translation().z()};
  result.pose.orientation_xyzw = {orientation.x(), orientation.y(),
                                  orientation.z(), orientation.w()};
  return result;
}

void appendPlanningAndExecution(
    mcv::RenderBatch &batch, const R1RobotConfig &robot,
    const std::array<mcc::CartesianFrameSample, 2> &reference,
    const mcs::KinematicSnapshot &measured, const mcc::Pose &actual_left,
    const mcc::Pose &actual_right) {
  namespace planning = contracts::mcl_planning_v1;
  namespace execution = contracts::mcl_execution_v1;
  batch.poses.push_back(makePoseSample(planning::kLeftCartesianReferenceTopic,
                                       robot.base_frame, reference[0].pose));
  batch.poses.push_back(makePoseSample(planning::kRightCartesianReferenceTopic,
                                       robot.base_frame, reference[1].pose));
  batch.poses.push_back(makePoseSample(execution::kLeftCartesianExecutionTopic,
                                       robot.base_frame, actual_left));
  batch.poses.push_back(makePoseSample(execution::kRightCartesianExecutionTopic,
                                       robot.base_frame, actual_right));
  batch.joint_states.push_back(mcv::JointStateSample{
      execution::kJointExecutionTopic, measured.joints.names,
      measured.joints.positions, measured.joints.velocities});
}

IkDebugFrame
makeDebugFrame(const R1RobotConfig &robot, const KeyboardTargetSource &input,
               const std::array<mcc::CartesianFrameSample, 2> &reference,
               const mcs::KinematicSnapshot &measured,
               const mcc::Pose &actual_left, const mcc::Pose &actual_right,
               const mcc::InverseDynamicsSolution *solution,
               const mcc::HierarchicalInverseDynamicsDiagnostics *diagnostics,
               std::size_t tick, double maximum_solve_time_ms) {
  static_cast<void>(robot);
  IkDebugFrame frame;
  frame.run_id = kProgramId;
  frame.input_targets = input.targets();
  frame.targets = input.targets();
  frame.forward_kinematics = {{ArmSide::Left, actual_left},
                              {ArmSide::Right, actual_right}};
  frame.joint_names = measured.joints.names;
  frame.positions = measured.joints.positions;
  frame.velocities = measured.joints.velocities;
  frame.paused = input.paused();
  frame.selected_side = input.selectedSide();
  frame.status =
      input.paused() ? "Torque pipeline frozen" : "Torque control live";
  frame.runtime_state = IkRuntimeState::Running;
  frame.solvers = {makeSolverDebug(solution, diagnostics)};
  frame.solve_time_ms = frame.solvers.front().ik_solve_time_ms;
  frame.iterations = frame.solvers.front().ik_iterations;
  frame.converged = frame.solvers.front().converged;
  frame.ik_status =
      solution == nullptr
          ? "Awaiting first HID tick"
          : "Accepted through " + passName(diagnostics->selected_pass);
  const auto &targets = input.targets();
  frame.target_errors = {
      {ArmSide::Left,
       (targets[0].target_pose.translation() - actual_left.translation())
           .norm(),
       orientationError(targets[0].target_pose, actual_left)},
      {ArmSide::Right,
       (targets[1].target_pose.translation() - actual_right.translation())
           .norm(),
       orientationError(targets[1].target_pose, actual_right)}};
  CartesianPlannerDebug planner;
  planner.state = input.paused() ? "paused" : "tracking";
  planner.sample_time_s = 0.001;
  planner.arms = {
      {ArmSide::Left, targets[0].target_pose, reference[0].pose, actual_left,
       reference[0].twist, reference[0].acceleration,
       (reference[0].pose.translation() - actual_left.translation()).norm(),
       orientationError(reference[0].pose, actual_left)},
      {ArmSide::Right, targets[1].target_pose, reference[1].pose, actual_right,
       reference[1].twist, reference[1].acceleration,
       (reference[1].pose.translation() - actual_right.translation()).norm(),
       orientationError(reference[1].pose, actual_right)}};
  frame.cartesian_planner = std::move(planner);
  WorkerDebug worker;
  worker.label = "HID + MuJoCo";
  worker.configured_rate_hz = 1000.0;
  worker.iteration_count = tick;
  worker.maximum_solver_ms = maximum_solve_time_ms;
  worker.latest_solver_ms = frame.solve_time_ms;
  worker.maximum_execution_ms = maximum_solve_time_ms;
  worker.latest_execution_ms = frame.solve_time_ms;
  frame.workers.push_back(std::move(worker));
  return frame;
}

} // namespace

int runLoop(const Options &options, const R1RobotConfig &robot,
            Runtime &runtime, const Handles &handles) {
  mcs::MujocoTorqueSimulation simulation;
  mcs::ModelDescription simulation_description;
  simulation_description.xml_path = options.mujoco_path;
  simulation_description.joint_names = robot.joint_names;
  simulation_description.site_names = {"left_tcp_site", "right_tcp_site"};
  simulation_description.free_joint_name = "floating_base";
  simulation_description.base_weld_name = "fixed_base_weld";
  simulation_description.base_weld_enabled = true;
  // Contact impulses are disabled in this fixed-base slice because no HID
  // contact is registered. The viewer still renders every model geometry.
  simulation_description.contact_dynamics_enabled = false;
  simulation.load(simulation_description);
  if (std::abs(simulation.timestep() - 0.001) > 1.0e-12) {
    throw std::runtime_error("R1 MuJoCo timestep must be 0.001 s");
  }
  mcs::JointState initial;
  initial.names = robot.joint_names;
  initial.positions = robot.default_positions;
  initial.velocities.assign(robot.joint_names.size(), 0.0);
  simulation.setState(initial);

  mcc::DynamicsWorkspace workspace;
  requireOk(workspace.configure(runtime.robot_model),
            "configure tracking FK workspace");
  auto measured = simulation.snapshot();
  requireOk(workspace.update(toRobotState(measured)),
            "linearize initial state");
  mcc::Pose actual_left = mcc::Pose::Identity();
  mcc::Pose actual_right = mcc::Pose::Identity();
  requireOk(workspace.modelReferenceFramePose(robot.left_end_effector_frame,
                                              actual_left),
            "left initial pose");
  requireOk(workspace.modelReferenceFramePose(robot.right_end_effector_frame,
                                              actual_right),
            "right initial pose");

  mcc::CartesianRetargetRequest plan_request;
  plan_request.reference_frame_name = robot.base_frame;
  plan_request.sample_period = simulation.timestep();
  plan_request.limits.max_linear_velocity.setConstant(0.1);
  plan_request.limits.max_linear_acceleration.setConstant(1.0);
  plan_request.limits.max_linear_jerk.setConstant(10.0);
  plan_request.limits.max_rotation_vector_velocity.setConstant(0.5);
  plan_request.limits.max_rotation_vector_acceleration.setConstant(2.0);
  plan_request.limits.max_rotation_vector_jerk.setConstant(10.0);
  mcc::CartesianRetargetSegment left;
  left.frame_name = robot.left_end_effector_frame;
  left.current_pose = actual_left;
  left.target_pose = actual_left;
  left.target_pose.translation().x() += options.retarget_x_m;
  mcc::CartesianRetargetSegment right;
  right.frame_name = robot.right_end_effector_frame;
  right.current_pose = actual_right;
  right.target_pose = actual_right;
  right.target_pose.translation().x() += options.retarget_x_m;
  plan_request.segments = {left, right};

  auto terminal = std::make_unique<TerminalFrontend>(TerminalFrontendOptions{
      options.keyboard_enabled, options.presentation.enabled});
  auto input = std::make_unique<KeyboardTargetSource>(
      *terminal, KeyboardSourceMode::Teleop, options.keyboard,
      std::vector<ArmTarget>{{ArmSide::Left, left.target_pose},
                             {ArmSide::Right, right.target_pose}},
      true);
  if (options.start_paused && options.keyboard_enabled) {
    input->setPaused(true, "Torque pipeline paused");
  }
  PlannedGroupedTui tui(options.presentation);
  auto presentation =
      makeArmPresentation(robot, foxgloveIkVisualizationChannels());
  presentation.requirements_page_enabled = true;
  auto visualization_sink =
      createPreviewSink(options.visualization, kProgramId);
  visualization_sink->open();

  std::unique_ptr<mcs::MujocoViewer> viewer;
  if (options.viewer_enabled) {
    mcs::ViewerOptions viewer_options;
    viewer_options.title = kTitle;
    viewer_options.handles = {
        {"left", "left_tcp_handle_geom", "left_tcp_site"},
        {"right", "right_tcp_handle_geom", "right_tcp_site"}};
    viewer = std::make_unique<mcs::MujocoViewer>(simulation,
                                                 std::move(viewer_options));
    viewer->open();
  }

  mcc::CartesianPlanner planner;
  mcc::PlanningDiagnostics planning_diagnostics;
  requireOk(planner.replan(plan_request, planning_diagnostics),
            "plan dual-hand retarget");
  std::uint64_t planned_target_revision = input->targetFrame().revision;
  std::array<mcc::CartesianFrameSample, 2> planner_reference;
  planner_reference[0].reference_frame_name = robot.base_frame;
  planner_reference[0].frame_name = robot.left_end_effector_frame;
  planner_reference[0].pose = left.current_pose;
  planner_reference[1].reference_frame_name = robot.base_frame;
  planner_reference[1].frame_name = robot.right_end_effector_frame;
  planner_reference[1].pose = right.current_pose;

  mcc::CartesianAdmittance left_admittance;
  mcc::CartesianAdmittance right_admittance;
  if (options.admittance_enabled) {
    requireOk(left_admittance.configure({}), "configure left admittance");
    requireOk(right_admittance.configure({}), "configure right admittance");
  }

  const std::size_t duration_ticks = static_cast<std::size_t>(
      std::ceil(options.duration_seconds / simulation.timestep()));
  const bool open_ended = options.duration_seconds == 0.0 &&
                          (options.keyboard_enabled || options.viewer_enabled);
  const std::size_t tick_budget =
      open_ended ? std::numeric_limits<std::size_t>::max()
                 : (options.start_paused && !options.keyboard_enabled
                        ? options.single_step_count
                        : duration_ticks);
  Eigen::VectorXd previous_torque = Eigen::VectorXd::Zero(20);
  Eigen::VectorXd latest_torque = Eigen::VectorXd::Zero(20);
  std::optional<mcc::InverseDynamicsSolution> latest_solution;
  std::optional<mcc::HierarchicalInverseDynamicsDiagnostics> latest_diagnostics;
  std::vector<double> solve_times_ms;
  double maximum_solve_time_ms = 0.0;
  double maximum_tracking_error = 0.0;
  double minimum_torque_margin = 1.0;
  double maximum_joint_velocity = 0.0;
  std::string maximum_joint_velocity_name;
  double maximum_joint_acceleration = 0.0;
  std::string maximum_joint_acceleration_name;
  double maximum_commanded_torque = 0.0;
  std::string maximum_commanded_torque_name;
  double maximum_acceleration_model_error = 0.0;
  std::string maximum_acceleration_model_error_name;
  double maximum_predicted_acceleration = 0.0;
  std::string maximum_predicted_acceleration_name;
  double maximum_base_speed = 0.0;
  std::size_t tick = 0;
  std::size_t publish_count = 0;
  std::array<std::uint64_t, 2> drag_sequences{0, 0};
  const auto control_period =
      std::chrono::duration<double>(simulation.timestep());
  const auto presentation_period =
      std::chrono::duration<double>(1.0 / options.ui_rate_hz);
  auto next_control = std::chrono::steady_clock::now();
  auto next_presentation = next_control;

  const auto present = [&]() {
    const auto *solution =
        latest_solution.has_value() ? &*latest_solution : nullptr;
    const auto *diagnostics =
        latest_diagnostics.has_value() ? &*latest_diagnostics : nullptr;
    const auto frame = makeDebugFrame(
        robot, *input, planner_reference, measured, actual_left, actual_right,
        solution, diagnostics, tick, maximum_solve_time_ms);
    auto batch = makeIkRenderBatch(frame, presentation, monotonicNowNs());
    appendPlanningAndExecution(batch, robot, planner_reference, measured,
                               actual_left, actual_right);
    visualization_sink->write(batch);
    ++publish_count;
    if (options.presentation.enabled) {
      PlannedGroupedTuiSnapshot snapshot;
      snapshot.frame = &frame;
      snapshot.presentation = &presentation;
      snapshot.publish_count = publish_count;
      snapshot.sink_status = visualization_sink->status();
      snapshot.title = kTitle;
      snapshot.input_status = input->status();
      snapshot.header_context = "HID → torque → mj_step · fixed base · 1 kHz";
      snapshot.footer_hints = "Space pause · ←/→ arm · wasd/qe move · n/i/u "
                              "rotate · 1–7 pages · ? help · x exit";
      snapshot.help_lines = {"Left/Right: select arm; W/S X; A/D Y; Q/E Z; "
                             "Up/Down: translation step",
                             "N: rotation axis; I/U: rotate; M: enter step; R: "
                             "reset from measured pose",
                             "MuJoCo: Ctrl+left drag hand marker in view "
                             "plane; Shift adds depth; Ctrl+right rotates",
                             "Space: freeze/resume planner, HID, torque write "
                             "and mj_step; X/Esc: exit"};
      snapshot.extra_pages = {
          makeHidPage(robot, latest_torque, solution, diagnostics)};
      tui.render(snapshot);
    }
    if (viewer) {
      viewer->render();
    }
  };

  while (tick < tick_budget) {
    const bool was_paused = input->paused();
    const auto update = input->poll(simulation.timestep());
    if (input->paused() != was_paused) {
      input->setPaused(input->paused(), input->paused()
                                            ? "Torque pipeline paused"
                                            : "Torque pipeline resumed");
    }
    for (const auto &event : update.navigation) {
      tui.handleNavigation(event);
    }
    if (const auto reset_side = input->consumeResetRequest()) {
      input->setTargetPose(*reset_side,
                           *reset_side == ArmSide::Left ? actual_left
                                                        : actual_right,
                           std::string{"Reset "} + armSideName(*reset_side) +
                               " target from measured MuJoCo state");
    }
    if (input->stopRequested()) {
      break;
    }

    if (viewer) {
      if (viewer->shouldClose()) {
        break;
      }
      for (const auto &drag : viewer->pollInteractions()) {
        const std::size_t index = drag.id == "right" ? 1U : 0U;
        if (drag.active && drag.sequence != drag_sequences[index]) {
          drag_sequences[index] = drag.sequence;
          const ArmSide side = index == 0U ? ArmSide::Left : ArmSide::Right;
          input->setTargetPose(side, fromViewerPose(drag.target_pose),
                               std::string{"MuJoCo drag updated "} +
                                   armSideName(side) + " target");
        }
      }
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= next_presentation) {
      present();
      do {
        next_presentation +=
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                presentation_period);
      } while (next_presentation <= now);
    }
    if (input->paused()) {
      next_control = std::chrono::steady_clock::now();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (input->targetFrame().revision != planned_target_revision) {
      const auto &targets = input->targets();
      for (std::size_t arm = 0; arm < plan_request.segments.size(); ++arm) {
        plan_request.segments[arm].current_pose = planner_reference[arm].pose;
        plan_request.segments[arm].current_twist = planner_reference[arm].twist;
        plan_request.segments[arm].current_acceleration =
            planner_reference[arm].acceleration;
        plan_request.segments[arm].target_pose = targets.at(arm).target_pose;
      }
      requireOk(planner.replan(plan_request, planning_diagnostics),
                "replan Cartesian target");
      planned_target_revision = input->targetFrame().revision;
    }

    mcc::CartesianTrajectorySample sample;
    requireOk(planner.step(sample, planning_diagnostics),
              "step Cartesian planner");
    if (sample.frames.size() != 2) {
      throw std::runtime_error(
          "Cartesian planner did not return both R1 hands");
    }
    std::array<mcc::CartesianFrameSample, 2> command{sample.frames[0],
                                                     sample.frames[1]};
    planner_reference = command;
    if (options.admittance_enabled) {
      for (std::size_t arm = 0; arm < command.size(); ++arm) {
        mcc::CartesianAdmittanceInput admittance_input;
        admittance_input.reference_pose = command[arm].pose;
        admittance_input.reference_twist = command[arm].twist;
        admittance_input.reference_acceleration = command[arm].acceleration;
        admittance_input.dt = simulation.timestep();
        mcc::CartesianAdmittanceOutput output;
        mcc::CartesianAdmittanceDiagnostics diagnostics;
        auto &admittance = arm == 0 ? left_admittance : right_admittance;
        requireOk(admittance.step(admittance_input, output, diagnostics),
                  "step Cartesian admittance");
        command[arm].pose = output.command_pose;
        command[arm].twist = output.command_twist;
        command[arm].acceleration = output.command_acceleration;
      }
    }

    measured = simulation.snapshot();
    mcc::InverseDynamicsRequest request;
    request.state = toRobotState(measured);
    request.reference_frame_name = robot.base_frame;
    request.previous_applied_efforts = previous_torque;
    request.position_targets = {
        {handles.left_position, command[0].pose.translation(),
         command[0].twist.head<3>(), command[0].acceleration.head<3>(), true},
        {handles.right_position, command[1].pose.translation(),
         command[1].twist.head<3>(), command[1].acceleration.head<3>(), true}};
    request.orientation_targets = {
        {handles.left_orientation, command[0].pose.linear(),
         command[0].twist.tail<3>(), command[0].acceleration.tail<3>(), true},
        {handles.right_orientation, command[1].pose.linear(),
         command[1].twist.tail<3>(), command[1].acceleration.tail<3>(), true}};
    mcc::PostureAccelerationTarget posture;
    posture.handle = handles.posture;
    posture.positions = Eigen::Map<const Eigen::VectorXd>(
        robot.default_positions.data(),
        static_cast<Eigen::Index>(robot.default_positions.size()));
    posture.velocities.setZero(
        static_cast<Eigen::Index>(robot.joint_names.size()));
    posture.accelerations.setZero(
        static_cast<Eigen::Index>(robot.joint_names.size()));
    request.posture_targets.push_back(std::move(posture));

    mcc::InverseDynamicsSolution solution;
    mcc::HierarchicalInverseDynamicsDiagnostics diagnostics;
    const mcc::Status solve_status =
        runtime.solver.solveInverseDynamics(request, solution, diagnostics);
    if (!solve_status.ok()) {
      std::string detail = solve_status.message;
      if (diagnostics.failed_pass.has_value()) {
        detail += " pass=" + passName(diagnostics.failed_pass);
        const auto &failed =
            diagnostics
                .passes[static_cast<std::size_t>(*diagnostics.failed_pass)];
        detail += " native=" + failed.native_status;
      }
      throw std::runtime_error("solve R1 HID: " + detail);
    }
    if (!mcc::isAccepted(solution.disposition) ||
        !solution.actuator_efforts.allFinite()) {
      throw std::runtime_error("R1 HID rejected or returned non-finite torque");
    }
    double solve_time = 0.0;
    for (const auto &pass : diagnostics.passes) {
      solve_time += pass.solve_time_ms;
    }
    solve_times_ms.push_back(solve_time);
    maximum_solve_time_ms = std::max(maximum_solve_time_ms, solve_time);
    previous_torque = solution.actuator_efforts;
    latest_torque = solution.actuator_efforts;
    for (Eigen::Index joint = 0; joint < solution.actuator_efforts.size();
         ++joint) {
      const double magnitude = std::abs(solution.actuator_efforts(joint));
      if (magnitude > maximum_commanded_torque) {
        maximum_commanded_torque = magnitude;
        maximum_commanded_torque_name =
            robot.joint_names[static_cast<std::size_t>(joint)];
      }
    }
    std::vector<double> efforts(solution.actuator_efforts.data(),
                                solution.actuator_efforts.data() +
                                    solution.actuator_efforts.size());
    simulation.step({robot.joint_names, efforts});
    measured = simulation.snapshot();
    const auto &generalized_indices =
        runtime.actuation_model->generalizedVelocityIndices();
    for (std::size_t joint = 0; joint < measured.joints.velocities.size();
         ++joint) {
      const double magnitude = std::abs(measured.joints.velocities[joint]);
      if (magnitude > maximum_joint_velocity) {
        maximum_joint_velocity = magnitude;
        maximum_joint_velocity_name = measured.joints.names[joint];
      }
      const double acceleration =
          std::abs(measured.joints.accelerations[joint]);
      if (acceleration > maximum_joint_acceleration) {
        maximum_joint_acceleration = acceleration;
        maximum_joint_acceleration_name = measured.joints.names[joint];
      }
      const double model_error = std::abs(
          measured.joints.accelerations[joint] -
          solution.generalized_accelerations(generalized_indices[joint]));
      if (model_error > maximum_acceleration_model_error) {
        maximum_acceleration_model_error = model_error;
        maximum_acceleration_model_error_name = measured.joints.names[joint];
      }
      const double predicted = std::abs(
          solution.generalized_accelerations(generalized_indices[joint]));
      if (predicted > maximum_predicted_acceleration) {
        maximum_predicted_acceleration = predicted;
        maximum_predicted_acceleration_name = measured.joints.names[joint];
      }
    }
    const double base_speed = std::sqrt(
        std::inner_product(measured.floating_base.twist.linear_mps.begin(),
                           measured.floating_base.twist.linear_mps.end(),
                           measured.floating_base.twist.linear_mps.begin(),
                           0.0) +
        std::inner_product(measured.floating_base.twist.angular_radps.begin(),
                           measured.floating_base.twist.angular_radps.end(),
                           measured.floating_base.twist.angular_radps.begin(),
                           0.0));
    maximum_base_speed = std::max(maximum_base_speed, base_speed);

    requireOk(workspace.update(toRobotState(measured)), "update tracking FK");
    requireOk(workspace.modelReferenceFramePose(robot.left_end_effector_frame,
                                                actual_left),
              "left tracking FK");
    requireOk(workspace.modelReferenceFramePose(robot.right_end_effector_frame,
                                                actual_right),
              "right tracking FK");
    maximum_tracking_error = std::max(
        maximum_tracking_error,
        std::max(
            (actual_left.translation() - command[0].pose.translation()).norm(),
            (actual_right.translation() - command[1].pose.translation())
                .norm()));
    for (Eigen::Index joint = 0; joint < solution.actuator_efforts.size();
         ++joint) {
      minimum_torque_margin = std::min(
          minimum_torque_margin,
          1.0 - std::abs(solution.actuator_efforts(joint)) /
                    robot.effort_limits[static_cast<std::size_t>(joint)]);
    }
    latest_solution = std::move(solution);
    latest_diagnostics = std::move(diagnostics);
    ++tick;

    if (options.keyboard_enabled || options.viewer_enabled) {
      next_control +=
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              control_period);
      const auto current = std::chrono::steady_clock::now();
      if (next_control <
          current -
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  control_period)) {
        next_control = current;
      }
      std::this_thread::sleep_until(next_control);
    }
  }

  if (viewer) {
    viewer->close();
  }
  visualization_sink->flush();
  visualization_sink->close();
  input.reset();
  terminal.reset();

  std::cout << "ticks=" << tick
            << " solve_ms_p50=" << percentile(solve_times_ms, 0.50)
            << " solve_ms_p95=" << percentile(solve_times_ms, 0.95)
            << " solve_ms_p99=" << percentile(solve_times_ms, 0.99)
            << " max_tracking_error_m=" << maximum_tracking_error
            << " min_torque_margin=" << minimum_torque_margin
            << " max_joint_velocity=" << maximum_joint_velocity
            << " max_joint_velocity_name=" << maximum_joint_velocity_name
            << " max_joint_acceleration=" << maximum_joint_acceleration
            << " max_joint_acceleration_name="
            << maximum_joint_acceleration_name
            << " max_commanded_torque=" << maximum_commanded_torque
            << " max_commanded_torque_name=" << maximum_commanded_torque_name
            << " max_acceleration_model_error="
            << maximum_acceleration_model_error
            << " max_acceleration_model_error_name="
            << maximum_acceleration_model_error_name
            << " max_predicted_acceleration=" << maximum_predicted_acceleration
            << " max_predicted_acceleration_name="
            << maximum_predicted_acceleration_name
            << " max_base_speed=" << maximum_base_speed << '\n';
  return 0;
}

} // namespace motion_control_lab::hierarchical_inverse_dynamics_torque_sim
