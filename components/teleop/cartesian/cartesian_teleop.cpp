#include "components/teleop/cartesian/cartesian_teleop.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace motion_control_lab
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kStepScale = 2.0;

}  // namespace

CartesianTeleop::CartesianTeleop(
  CartesianTeleopOptions options,
  std::vector<ArmTarget> initial_targets,
  bool allow_side_switching)
: options_(std::move(options)),
  selected_side_(parseArmSide(options_.side)),
  step_m_(options_.step_m),
  allow_side_switching_(allow_side_switching)
{
  if (initial_targets.empty()) {
    throw std::runtime_error("Cartesian teleop requires at least one initial target");
  }
  frame_.targets = std::move(initial_targets);
  frame_.source = "keyboard";
  if (targetFor(selected_side_) == nullptr) {
    selected_side_ = frame_.targets.front().side;
  }
  status_ = allow_side_switching_
    ? "Initialized left and right targets from FK"
    : std::string{"Initialized "} + armSideName(selected_side_) + " target from FK";
}

const MotionTargetFrame & CartesianTeleop::frame() const noexcept { return frame_; }

ArmSide CartesianTeleop::selectedSide() const noexcept { return selected_side_; }

double CartesianTeleop::stepMetres() const noexcept { return step_m_; }

ArmTarget * CartesianTeleop::targetFor(ArmSide side)
{
  const auto found = std::find_if(
    frame_.targets.begin(), frame_.targets.end(),
    [side](const ArmTarget & target) { return target.side == side; });
  return found == frame_.targets.end() ? nullptr : &*found;
}

ArmTarget * CartesianTeleop::selectedTarget() { return targetFor(selected_side_); }

std::optional<ArmSide> CartesianTeleop::apply(const TeleopIntent & intent, double dt)
{
  if (!std::isfinite(dt) || dt < 0.0) {
    throw std::runtime_error("Cartesian teleop dt must be finite and non-negative");
  }
  switch (intent.kind) {
    case TeleopIntentKind::SelectArm:
      if (allow_side_switching_ && targetFor(intent.side) != nullptr) {
        selected_side_ = intent.side;
        status_ = std::string{"Selected "} + armSideName(selected_side_);
      }
      return std::nullopt;
    case TeleopIntentKind::IncreaseStep:
      step_m_ = std::clamp(step_m_ * kStepScale, options_.min_step_m, options_.max_step_m);
      break;
    case TeleopIntentKind::DecreaseStep:
      step_m_ = std::clamp(step_m_ / kStepScale, options_.min_step_m, options_.max_step_m);
      break;
    case TeleopIntentKind::SetStep:
      if (!std::isfinite(intent.scalar) || intent.scalar < options_.min_step_m ||
          intent.scalar > options_.max_step_m) {
        std::ostringstream error;
        error << "Step must be in [" << options_.min_step_m << ", " << options_.max_step_m
              << "] m";
        throw std::runtime_error(error.str());
      }
      step_m_ = intent.scalar;
      break;
    case TeleopIntentKind::CycleRotationAxis:
      rotation_axis_index_ = (rotation_axis_index_ + 1U) % 3U;
      status_ = std::string{"Rotation axis set to TCP "} + "xyz"[rotation_axis_index_];
      ++frame_.revision;
      return std::nullopt;
    case TeleopIntentKind::ResetTarget:
      status_ = std::string{"Reset requested for "} + armSideName(selected_side_);
      return selected_side_;
    case TeleopIntentKind::Translate: {
      auto * target = selectedTarget();
      if (target == nullptr) {
        throw std::runtime_error("selected Cartesian target is unavailable");
      }
      const double scale = intent.discrete ? step_m_ : dt;
      target->target_pose.translation() += intent.translation * scale;
      std::ostringstream message;
      message << armSideName(selected_side_) << " move dx=" << std::showpos << std::fixed
              << std::setprecision(4) << intent.translation.x() * scale
              << " dy=" << intent.translation.y() * scale
              << " dz=" << intent.translation.z() * scale;
      status_ = message.str();
      break;
    }
    case TeleopIntentKind::Rotate: {
      auto * target = selectedTarget();
      if (target == nullptr) {
        throw std::runtime_error("selected Cartesian target is unavailable");
      }
      Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
      if (rotation_axis_index_ == 1U) axis = Eigen::Vector3d::UnitY();
      if (rotation_axis_index_ == 2U) axis = Eigen::Vector3d::UnitZ();
      const double angle =
        (intent.clockwise ? -1.0 : 1.0) * options_.rotation_step_deg * kPi / 180.0;
      Eigen::Quaterniond current(target->target_pose.linear());
      current.normalize();
      const Eigen::Quaterniond delta(Eigen::AngleAxisd(angle, axis));
      target->target_pose.linear() = (current * delta).normalized().toRotationMatrix();
      status_ = std::string{armSideName(selected_side_)} + " rotate around TCP " +
        "xyz"[rotation_axis_index_];
      break;
    }
  }
  if (intent.kind == TeleopIntentKind::IncreaseStep ||
      intent.kind == TeleopIntentKind::DecreaseStep ||
      intent.kind == TeleopIntentKind::SetStep) {
    std::ostringstream message;
    message << "Step set to " << std::fixed << std::setprecision(4) << step_m_ << " m";
    status_ = message.str();
  }
  ++frame_.revision;
  return std::nullopt;
}

void CartesianTeleop::setTargetPose(ArmSide side, const Pose & pose)
{
  if (auto * target = targetFor(side)) {
    target->target_pose = pose;
  } else {
    frame_.targets.push_back({side, pose});
  }
  ++frame_.revision;
}

const std::string & CartesianTeleop::status() const noexcept { return status_; }

void CartesianTeleop::setStatus(std::string status) { status_ = std::move(status); }

}  // namespace motion_control_lab
