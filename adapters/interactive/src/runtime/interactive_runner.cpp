#include "runtime/interactive_runner.hpp"

#include "config/constants.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

namespace motion_control_lab
{

InteractiveRunner::InteractiveRunner(
  InteractiveRunnerOptions options,
  IkSolverBackend & solver_backend,
  TeleopSource & teleop_source,
  motion_control::viz::FrameSink & visualization_sink)
: options_(std::move(options)),
  solver_backend_(solver_backend),
  teleop_source_(teleop_source),
  visualization_sink_(visualization_sink)
{
  if (options_.rate_hz <= 0.0) {
    throw std::runtime_error("runner rate must be positive");
  }
}

void InteractiveRunner::run()
{
  const auto start = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / options_.rate_hz));
  auto next_publish = start;
  auto next_draw = start;
  std::size_t publish_count = 0;
  IkDebugFrame latest_frame = makeFrame(teleop_source_.command(), nullptr);

  while (!options_.stop_requested || !options_.stop_requested()) {
    const auto loop_start = std::chrono::steady_clock::now();
    const double elapsed_s =
      std::chrono::duration<double>(loop_start - start).count();
    if (options_.duration_s > 0.0 && elapsed_s >= options_.duration_s) {
      break;
    }

    teleop_source_.poll();
    if (const auto reset_side = teleop_source_.consumeResetRequest()) {
      resetTargetFromCurrentFk(*reset_side);
    }

    const TargetCommand & command = teleop_source_.command();
    if (command.stop_requested) {
      break;
    }

    if (!command.paused && loop_start >= next_publish) {
      const auto result = solver_backend_.solveTargets(
        command.targets,
        1.0 / options_.rate_hz);
      latest_frame = makeFrame(command, &result);

      try {
        const auto sample_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          loop_start - start).count();
        const auto emit_time_ns = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
        visualization_sink_.write(makeVisualizationFrame(
          latest_frame,
          publish_count,
          sample_time_ns,
          emit_time_ns));
      } catch (const std::exception & error) {
        teleop_source_.setStatus(error.what());
        latest_frame.status = error.what();
      }

      ++publish_count;
      next_publish = loop_start + period;
    }

    if (loop_start >= next_draw) {
      latest_frame.status = teleop_source_.command().status;
      latest_frame.paused = teleop_source_.command().paused;
      teleop_source_.render(latest_frame, publish_count, visualization_sink_.status());
      next_draw = loop_start + std::chrono::milliseconds(100);
    }

    const auto sleep_period = std::min(
      period,
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::milliseconds(10)));
    std::this_thread::sleep_for(sleep_period);
  }
}

IkDebugFrame InteractiveRunner::makeFrame(
  const TargetCommand & command,
  const IkSolveResult * result) const
{
  IkDebugFrame frame;
  frame.backend_id = std::string{solver_backend_.backendId()};
  frame.targets = command.targets;
  frame.joint_names = solver_backend_.jointNames();
  frame.positions = solver_backend_.positions();
  frame.velocities = solver_backend_.velocities();
  frame.status = command.status;
  frame.paused = command.paused;
  frame.selected_side = command.selected_side;

  if (result != nullptr) {
    frame.ik_status = result->status.ok() ? "ok" : result->status.message;
    frame.iterations = result->diagnostics.iterations;
    frame.converged = result->diagnostics.converged;
    frame.solve_time_ms = result->diagnostics.solve_time_ms;
    frame.target_errors = result->diagnostics.errors;
  }
  return frame;
}

motion_control::viz::VisualizationFrame InteractiveRunner::makeVisualizationFrame(
  const IkDebugFrame & frame,
  std::uint64_t sequence,
  std::int64_t sample_time_ns,
  std::uint64_t emit_time_ns) const
{
  namespace mcv = motion_control::viz;

  mcv::VisualizationFrame visualization;
  visualization.run_id = "interactive-preview";
  visualization.sequence = sequence;
  visualization.sample_time_ns = sample_time_ns;
  visualization.sample_clock = "interactive_steady";
  visualization.emit_time_ns = emit_time_ns;
  visualization.status = frame.status;
  visualization.paused = frame.paused;

  visualization.poses.reserve(frame.targets.size());
  for (const auto & target : frame.targets) {
    const Eigen::Quaterniond orientation(target.target_pose.linear());
    mcv::NamedPose pose;
    pose.entity_id = std::string{armSideName(target.side)} + "_target";
    pose.channel = target.side == ArmSide::Left
      ? kLeftTargetPoseTopic
      : kRightTargetPoseTopic;
    pose.frame_id = kBaseFrame;
    pose.pose.position_m = {
      target.target_pose.translation().x(),
      target.target_pose.translation().y(),
      target.target_pose.translation().z()};
    pose.pose.orientation_xyzw = {
      orientation.x(), orientation.y(), orientation.z(), orientation.w()};
    visualization.poses.push_back(std::move(pose));
  }

  visualization.joints = mcv::JointStateFrame{
    kJointStatesTopic,
    frame.joint_names,
    frame.positions,
    frame.velocities};
  visualization.diagnostics = {
    {"ik.iterations", static_cast<double>(frame.iterations), "count"},
    {"ik.converged", frame.converged ? 1.0 : 0.0, "bool"},
    {"ik.solve_time", frame.solve_time_ms, "ms"}};
  for (const auto & error : frame.target_errors) {
    const std::string prefix = std::string{"ik."} + armSideName(error.side);
    visualization.diagnostics.push_back(
      {prefix + ".position_error", error.position_m, "m"});
    visualization.diagnostics.push_back(
      {prefix + ".orientation_error", error.orientation_rad, "rad"});
  }
  return visualization;
}

void InteractiveRunner::resetTargetFromCurrentFk(ArmSide side)
{
  try {
    teleop_source_.setTargetPose(
      side,
      solver_backend_.currentTargetPose(side),
      std::string{"Reset "} + armSideName(side) + " target from current FK");
  } catch (const std::exception & error) {
    teleop_source_.setStatus("Reset failed: " + std::string(error.what()));
  }
}

}  // namespace motion_control_lab
