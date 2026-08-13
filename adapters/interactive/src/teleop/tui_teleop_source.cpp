#include "teleop/tui_teleop_source.hpp"

#include "config/constants.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

namespace motion_control_lab
{
namespace
{

ArmTarget * findTarget(std::vector<ArmTarget> & targets, ArmSide side)
{
  for (auto & target : targets) {
    if (target.side == side) {
      return &target;
    }
  }
  return nullptr;
}

const ArmTarget * findTarget(const std::vector<ArmTarget> & targets, ArmSide side)
{
  for (const auto & target : targets) {
    if (target.side == side) {
      return &target;
    }
  }
  return nullptr;
}

const ArmTargetError * findError(const std::vector<ArmTargetError> & errors, ArmSide side)
{
  for (const auto & error : errors) {
    if (error.side == side) {
      return &error;
    }
  }
  return nullptr;
}

bool hasTarget(const std::vector<ArmTarget> & targets, ArmSide side)
{
  return findTarget(targets, side) != nullptr;
}

std::string targetTopicForSide(
  const InteractiveIkPresentation & presentation,
  ArmSide side)
{
  const auto * arm = findArmPresentation(presentation, side);
  return arm == nullptr ? "<unconfigured>" : arm->target_channel;
}

std::string selectedRotationAxis(std::size_t axis_index)
{
  return kRotationAxes[axis_index];
}

void adjustStep(
  TargetCommand & command,
  double & step_m,
  const TuiTeleopOptions & options,
  double scale)
{
  const double next_step = std::clamp(
    step_m * scale,
    options.min_step_m,
    options.max_step_m);
  step_m = next_step;
  std::ostringstream status;
  status << "Step set to " << std::fixed << std::setprecision(4) << step_m << " m";
  command.status = status.str();
}

void setStep(
  TargetCommand & command,
  double & step_m,
  const TuiTeleopOptions & options,
  double next_step_m)
{
  if (!std::isfinite(next_step_m)) {
    command.status = "Step must be finite";
    return;
  }
  if (next_step_m < options.min_step_m || next_step_m > options.max_step_m) {
    std::ostringstream status;
    status << "Step must be in [" << std::fixed << std::setprecision(4)
           << options.min_step_m << ", " << options.max_step_m << "] m";
    command.status = status.str();
    return;
  }
  step_m = next_step_m;
  std::ostringstream status;
  status << "Step set to " << std::fixed << std::setprecision(4) << step_m << " m";
  command.status = status.str();
}

void moveSelected(TargetCommand & command, double dx, double dy, double dz)
{
  auto * target = findTarget(command.targets, command.selected_side);
  if (target == nullptr) {
    command.status = std::string{"No target for "} + armSideName(command.selected_side);
    return;
  }

  target->target_pose.translation().x() += dx;
  target->target_pose.translation().y() += dy;
  target->target_pose.translation().z() += dz;
  std::ostringstream status;
  status << armSideName(command.selected_side) << " move dx="
         << std::showpos << std::fixed << std::setprecision(4) << dx
         << " dy=" << dy << " dz=" << dz;
  command.status = status.str();
}

void rotateSelected(
  TargetCommand & command,
  std::size_t axis_index,
  double rotation_step_rad,
  bool clockwise)
{
  auto * target = findTarget(command.targets, command.selected_side);
  if (target == nullptr) {
    command.status = std::string{"No target for "} + armSideName(command.selected_side);
    return;
  }

  const double angle = clockwise ? -rotation_step_rad : rotation_step_rad;
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
  const std::string selected_axis = selectedRotationAxis(axis_index);
  if (selected_axis == "y") {
    axis = Eigen::Vector3d::UnitY();
  } else if (selected_axis == "z") {
    axis = Eigen::Vector3d::UnitZ();
  }

  Eigen::Quaterniond current(target->target_pose.linear());
  current.normalize();
  const Eigen::Quaterniond delta(Eigen::AngleAxisd(angle, axis));
  const Eigen::Quaterniond rotated = (current * delta).normalized();
  target->target_pose.linear() = rotated.toRotationMatrix();

  std::ostringstream status;
  status << armSideName(command.selected_side) << " rotate "
         << (clockwise ? "clockwise" : "counter-clockwise")
         << " around TCP " << selected_axis
         << " by " << std::fixed << std::setprecision(2)
         << std::abs(angle) * 180.0 / kPi << " deg";
  command.status = status.str();
}

std::string formatPose(const ArmTarget * target, const std::string & base_frame_id)
{
  if (target == nullptr) {
    return "<none>";
  }

  const auto & pose = target->target_pose;
  const Eigen::Quaterniond q(pose.linear());
  std::ostringstream text;
  text << "frame=" << base_frame_id
       << " pos=(" << std::showpos << std::fixed << std::setprecision(4)
       << pose.translation().x() << ", "
       << pose.translation().y() << ", "
       << pose.translation().z() << ") quat=("
       << std::setprecision(3)
       << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << ")";
  return text.str();
}

std::string fitLine(std::string text, int width)
{
  if (width <= 0) {
    return {};
  }
  if (static_cast<int>(text.size()) > width) {
    text.resize(static_cast<std::size_t>(width));
  }
  return text;
}

void addLine(std::ostringstream & screen, int row, int col, const std::string & text, int width)
{
  if (row < 0 || col >= width) {
    return;
  }
  screen << "\033[" << (row + 1) << ";" << (col + 1) << "H"
         << fitLine(text, width - col);
}

std::vector<std::string> jointPositionLines(
  const JointNames & joint_names, const std::vector<double> & positions,
  const std::string & first_prefix, int width)
{
  std::vector<std::string> lines;
  std::string line = first_prefix;
  const std::string continuation{"  "};
  for (std::size_t index = 0; index < positions.size(); ++index) {
    std::ostringstream token;
    token << (index < joint_names.size() ? joint_names[index] : "joint_" + std::to_string(index))
          << '=' << std::fixed << std::setprecision(3) << positions[index];
    const std::string value = token.str();
    const std::size_t separator = line == first_prefix || line == continuation ? 0U : 1U;
    if (
      separator != 0U && width > 0 &&
      line.size() + separator + value.size() > static_cast<std::size_t>(width)) {
      lines.push_back(line);
      line = continuation + value;
    } else {
      if (separator != 0U) {
        line.push_back(' ');
      }
      line += value;
    }
  }
  if (line != first_prefix || positions.empty()) {
    lines.push_back(line);
  }
  return lines;
}

double collisionPairDistance(const SelfCollisionPairDebug & pair)
{
  return std::min(pair.distance_before_m, pair.distance_after_m);
}

const char * collisionPairState(
  const SelfCollisionPairDebug & pair, const SelfCollisionDebug & collision)
{
  if (pair.distance_before_m < 0.0 || pair.distance_after_m < 0.0) {
    return "COLLISION";
  }
  if (pair.distance_after_m < collision.minimum_distance_m) {
    return "SHORTFALL";
  }
  if (pair.active) {
    return "ACTIVE";
  }
  return "NEAR";
}

}  // namespace

TerminalSession::TerminalSession()
{
  if (!::isatty(STDIN_FILENO) || !::isatty(STDOUT_FILENO)) {
    throw std::runtime_error("TUI teleop requires an interactive TTY");
  }
  if (::tcgetattr(STDIN_FILENO, &original_) != 0) {
    throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
  }
  enableRawMode();
  write("\033[?25l\033[2J\033[H");
}

TerminalSession::~TerminalSession()
{
  restoreCookedMode();
  write("\033[0m\033[?25h\n");
}

void TerminalSession::enableRawMode()
{
  termios raw = original_;
  raw.c_iflag &= static_cast<tcflag_t>(~(BRKINT | ICRNL | INPCK | ISTRIP | IXON));
  raw.c_oflag &= static_cast<tcflag_t>(~(OPOST));
  raw.c_cflag |= CS8;
  raw.c_lflag &= static_cast<tcflag_t>(~(ECHO | ICANON | IEXTEN | ISIG));
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  if (::tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) != 0) {
    throw std::runtime_error("tcsetattr raw failed: " + std::string(std::strerror(errno)));
  }
  raw_enabled_ = true;
}

void TerminalSession::restoreCookedMode()
{
  if (raw_enabled_) {
    ::tcsetattr(STDIN_FILENO, TCSAFLUSH, &original_);
    raw_enabled_ = false;
  }
}

KeyInput TerminalSession::readKey()
{
  char ch = '\0';
  const ssize_t count = ::read(STDIN_FILENO, &ch, 1);
  if (count == 0 || (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
    return {};
  }
  if (count < 0) {
    return {};
  }
  if (ch != '\033') {
    return {Key::Character, ch};
  }

  char sequence[2] = {'\0', '\0'};
  if (::read(STDIN_FILENO, &sequence[0], 1) != 1) {
    return {Key::Escape, '\0'};
  }
  if (::read(STDIN_FILENO, &sequence[1], 1) != 1) {
    return {Key::Escape, '\0'};
  }
  if (sequence[0] == '[') {
    switch (sequence[1]) {
      case 'A':
        return {Key::Up, '\0'};
      case 'B':
        return {Key::Down, '\0'};
      case 'C':
        return {Key::Right, '\0'};
      case 'D':
        return {Key::Left, '\0'};
      default:
        break;
    }
  }
  return {Key::Escape, '\0'};
}

std::optional<std::string> TerminalSession::promptLine(const std::string & prompt)
{
  restoreCookedMode();
  write("\033[?25h\033[2K\r" + prompt);
  std::string value;
  if (!std::getline(std::cin, value)) {
    enableRawMode();
    write("\033[?25l");
    return std::nullopt;
  }
  enableRawMode();
  write("\033[?25l");
  return value;
}

std::pair<int, int> TerminalSession::size() const
{
  winsize window{};
  if (::ioctl(STDOUT_FILENO, TIOCGWINSZ, &window) == 0 &&
      window.ws_col > 0 &&
      window.ws_row > 0) {
    return {static_cast<int>(window.ws_row), static_cast<int>(window.ws_col)};
  }
  return {24, 100};
}

void TerminalSession::write(const std::string & text) const
{
  const char * data = text.data();
  std::size_t remaining = text.size();
  while (remaining > 0) {
    const ssize_t written = ::write(STDOUT_FILENO, data, remaining);
    if (written <= 0) {
      return;
    }
    data += written;
    remaining -= static_cast<std::size_t>(written);
  }
}

TuiTeleopSource::TuiTeleopSource(
  const TuiTeleopOptions & options,
  double rate_hz,
  std::string title,
  InteractiveIkPresentation presentation,
  std::vector<ArmTarget> initial_targets,
  bool allow_side_switching)
: options_(options),
  rate_hz_(rate_hz),
  title_(std::move(title)),
  presentation_(std::move(presentation)),
  step_m_(options.step_m),
  rotation_step_rad_(options.rotation_step_deg * kPi / 180.0),
  allow_side_switching_(allow_side_switching)
{
  if (initial_targets.empty()) {
    throw std::runtime_error("TUI teleop requires at least one initial target");
  }

  command_.targets = std::move(initial_targets);
  command_.selected_side = parseArmSide(options.side);
  if (!hasTarget(command_.targets, command_.selected_side)) {
    command_.selected_side = command_.targets.front().side;
  }
  command_.status = allow_side_switching_
    ? "Initialized left and right targets from FK"
    : std::string{"Initialized "} + armSideName(command_.selected_side) + " target from FK";
}

TuiTeleopSource::~TuiTeleopSource()
{
  terminal_.restoreCookedMode();
}

void TuiTeleopSource::poll()
{
  while (true) {
    const KeyInput key = terminal_.readKey();
    if (key.key == Key::None) {
      break;
    }
    handleKey(key);
  }
}

const TargetCommand & TuiTeleopSource::command() const
{
  return command_;
}

std::optional<ArmSide> TuiTeleopSource::consumeResetRequest()
{
  const auto requested = reset_requested_;
  reset_requested_.reset();
  return requested;
}

void TuiTeleopSource::setTargetPose(
  ArmSide side,
  const Pose & target_pose,
  const std::string & status)
{
  if (auto * target = findTarget(command_.targets, side)) {
    target->target_pose = target_pose;
  } else {
    command_.targets.push_back(ArmTarget{side, target_pose});
  }
  command_.status = status;
}

void TuiTeleopSource::setStatus(const std::string & status)
{
  command_.status = status;
}

void TuiTeleopSource::render(
  const IkDebugFrame & frame,
  std::size_t publish_count,
  const std::string & sink_status)
{
  const auto [height, width] = terminal_.size();
  std::ostringstream screen;
  screen << "\033[H\033[2J";

  if (!frame.self_collisions.empty()) {
    addLine(screen, 0, 0, title_, width);
    addLine(
      screen, 1, 0,
      "side=" + std::string{armSideName(command_.selected_side)} + "  " + sink_status +
        "  mode=" + (command_.paused ? "PAUSED" : "PUBLISHING") +
        "  rate=" + std::to_string(static_cast<int>(rate_hz_)) + " Hz" +
        "  publish_ticks=" + std::to_string(publish_count),
      width);
    addLine(
      screen, 2, 0,
      "targets: L=" + targetTopicForSide(presentation_, ArmSide::Left) +
        " R=" + targetTopicForSide(presentation_, ArmSide::Right),
      width);
    {
      std::ostringstream line;
      line << "step=" << std::fixed << std::setprecision(4) << step_m_
           << " m  rot_axis=tcp_" << selectedRotationAxis(rotation_axis_index_)
           << "  rot_step=" << std::setprecision(2) << options_.rotation_step_deg << " deg";
      addLine(screen, 3, 0, line.str(), width);
    }
    addLine(
      screen, 4, 0,
      "target left : " +
        formatPose(findTarget(command_.targets, ArmSide::Left), presentation_.base_frame_id),
      width);
    addLine(
      screen, 5, 0,
      "target right: " +
        formatPose(findTarget(command_.targets, ArmSide::Right), presentation_.base_frame_id),
      width);
    {
      std::ostringstream line;
      line << "IK: " << frame.ik_status << "  iterations=" << frame.iterations
           << "  converged=" << std::boolalpha << frame.converged
           << "  solve_ms=" << std::fixed << std::setprecision(3) << frame.solve_time_ms;
      addLine(screen, 6, 0, line.str(), width);
    }
    int row = 7;
    for (const auto side : {ArmSide::Left, ArmSide::Right}) {
      const auto * error = findError(frame.target_errors, side);
      if (error == nullptr) {
        continue;
      }
      std::ostringstream line;
      line << "error " << armSideName(side) << ": position=" << std::fixed
           << std::setprecision(6) << error->position_m << " m  orientation="
           << error->orientation_rad << " rad";
      addLine(screen, row++, 0, line.str(), width);
    }

    const auto & collision = frame.self_collisions.front();
    std::size_t active_count = 0;
    std::size_t penetrating_before_count = 0;
    std::size_t penetrating_after_count = 0;
    std::vector<const SelfCollisionPairDebug *> relevant_pairs;
    relevant_pairs.reserve(collision.pairs.size());
    for (const auto & pair : collision.pairs) {
      active_count += pair.active ? 1U : 0U;
      penetrating_before_count += pair.distance_before_m < 0.0 ? 1U : 0U;
      penetrating_after_count += pair.distance_after_m < 0.0 ? 1U : 0U;
      if (pair.active || pair.distance_after_m < collision.influence_distance_m) {
        relevant_pairs.push_back(&pair);
      }
    }
    std::sort(
      relevant_pairs.begin(), relevant_pairs.end(),
      [](const auto * left, const auto * right) {
        return collisionPairDistance(*left) < collisionPairDistance(*right);
      });
    if (relevant_pairs.empty() && !collision.pairs.empty()) {
      relevant_pairs.push_back(&*std::min_element(
        collision.pairs.begin(), collision.pairs.end(), [](const auto & left, const auto & right) {
          return collisionPairDistance(left) < collisionPairDistance(right);
        }));
    }
    {
      std::ostringstream line;
      line << collision.label << ": input_seq=" << collision.input_state_sequence
           << " active=" << active_count << '/' << collision.pairs.size()
           << " penetrating(before/after)=" << penetrating_before_count << '/'
           << penetrating_after_count;
      addLine(screen, row++, 0, line.str(), width);
    }
    {
      std::ostringstream line;
      line << "distance: min_before=" << std::fixed << std::setprecision(4)
           << collision.minimum_distance_before_m
           << " m  min_after=" << collision.minimum_distance_after_m << " m";
      addLine(screen, row++, 0, line.str(), width);
    }
    {
      std::ostringstream line;
      line << "threshold: safe=" << std::fixed << std::setprecision(4)
           << collision.minimum_distance_m << " m  influence=" << collision.influence_distance_m
           << " m  shortfall=" << collision.margin_shortfall_m << " m";
      addLine(screen, row++, 0, line.str(), width);
    }

    const auto q_lines = jointPositionLines(
      frame.joint_names, collision.input_joint_positions, "all q (before): ", width);
    for (const auto & line : q_lines) {
      if (row >= height - 1) {
        break;
      }
      addLine(screen, row++, 0, line, width);
    }

    std::size_t shown_pair_count = 0;
    for (const auto * pair : relevant_pairs) {
      if (row >= height - 2) {
        break;
      }
      std::ostringstream line;
      line << "collision[" << collisionPairState(*pair, collision) << "] " << pair->first_link
           << " <-> " << pair->second_link << " before=" << std::showpos << std::fixed
           << std::setprecision(4) << pair->distance_before_m
           << " after=" << pair->distance_after_m << " m";
      addLine(screen, row++, 0, line.str(), width);
      ++shown_pair_count;
    }
    if (shown_pair_count < relevant_pairs.size() && row < height - 1) {
      addLine(
        screen, row, 0,
        "... " + std::to_string(relevant_pairs.size() - shown_pair_count) +
          " additional active/near collision pairs not shown",
        width);
    }

    addLine(screen, std::max(0, height - 1), 0, "status: " + command_.status, width);
    terminal_.write(screen.str());
    return;
  }

  addLine(screen, 0, 0, title_, width);
  addLine(
    screen,
    1,
    0,
    "side=" + std::string{armSideName(command_.selected_side)} +
      "  " + sink_status +
      "  mode=" + (command_.paused ? "PAUSED" : "PUBLISHING") +
      "  rate=" + std::to_string(static_cast<int>(rate_hz_)) + " Hz" +
      "  publish_ticks=" + std::to_string(publish_count),
    width);

  if (allow_side_switching_) {
    addLine(
      screen, 3, 0,
      "IK target left : " + targetTopicForSide(presentation_, ArmSide::Left), width);
    addLine(
      screen, 4, 0,
      "IK target right: " + targetTopicForSide(presentation_, ArmSide::Right), width);
  } else {
    addLine(
      screen,
      3,
      0,
      std::string{"IK target "} + armSideName(command_.selected_side) + ": " +
        targetTopicForSide(presentation_, command_.selected_side),
      width);
  }

  {
    std::ostringstream line;
    line << "step=" << std::fixed << std::setprecision(4) << step_m_
         << " m  rot_axis=tcp_" << selectedRotationAxis(rotation_axis_index_)
         << "  rot_step=" << std::setprecision(2)
         << options_.rotation_step_deg << " deg";
    addLine(screen, 6, 0, line.str(), width);
  }

  if (allow_side_switching_) {
    addLine(
      screen,
      8,
      0,
      "target left : " + formatPose(
        findTarget(command_.targets, ArmSide::Left), presentation_.base_frame_id),
      width);
    addLine(
      screen,
      9,
      0,
      "target right: " + formatPose(
        findTarget(command_.targets, ArmSide::Right), presentation_.base_frame_id),
      width);
  } else {
    addLine(
      screen,
      8,
      0,
      std::string{"target "} + armSideName(command_.selected_side) + ": " +
        formatPose(
          findTarget(command_.targets, command_.selected_side),
          presentation_.base_frame_id),
      width);
  }

  {
    std::ostringstream line;
    line << "IK: " << frame.ik_status
         << "  iterations=" << frame.iterations
         << "  converged=" << std::boolalpha << frame.converged
         << "  solve_ms=" << std::fixed << std::setprecision(3) << frame.solve_time_ms;
    addLine(screen, 11, 0, line.str(), width);
  }
  int error_row = 12;
  for (const auto side : {ArmSide::Left, ArmSide::Right}) {
    const auto * error = findError(frame.target_errors, side);
    if (error == nullptr) {
      continue;
    }
    std::ostringstream line;
    line << "error " << armSideName(side)
         << ": position=" << std::fixed << std::setprecision(6)
         << error->position_m << " m  orientation="
         << error->orientation_rad << " rad";
    addLine(screen, error_row++, 0, line.str(), width);
  }

  if (!frame.positions.empty()) {
    std::ostringstream line;
    line << armSideName(command_.selected_side) << " arm q: ";
    const auto * arm = findArmPresentation(presentation_, command_.selected_side);
    if (arm != nullptr) {
      for (const auto index : arm->joint_indices) {
        if (index >= frame.positions.size()) {
          continue;
        }
        const std::string joint_name = index < frame.joint_names.size()
          ? frame.joint_names[index]
          : ("joint_" + std::to_string(index));
        line << joint_name << "="
             << std::fixed << std::setprecision(3) << frame.positions[index] << " ";
      }
    }
    addLine(screen, 15, 0, line.str(), width);
  }

  if (show_help_) {
    addLine(screen, 17, 0, "Keys", width);
    addLine(screen, 18, 0, "w/s: +x/-x    a/d: +y/-y    q/e: +z/-z", width);
    addLine(screen, 19, 0, "n: cycle TCP rotation axis    i: clockwise    u: counter-clockwise", width);
    addLine(
      screen,
      20,
      0,
      allow_side_switching_
        ? "LEFT/RIGHT: switch arm    UP/DOWN: step x2 / step /2"
        : "UP/DOWN: step x2 / step /2",
      width);
    addLine(screen, 21, 0, "m: manual step    r: reset from FK    space: pause    h: hide help", width);
    addLine(screen, 22, 0, "x or Esc: exit", width);
  } else {
    addLine(screen, 17, 0, "h: show help", width);
  }

  addLine(screen, std::max(0, height - 1), 0, "status: " + command_.status, width);
  terminal_.write(screen.str());
}

void TuiTeleopSource::handleKey(const KeyInput & input)
{
  if (input.key == Key::Escape) {
    command_.status = "Exiting";
    command_.stop_requested = true;
    return;
  }
  if (input.key == Key::Left || input.key == Key::Right) {
    if (allow_side_switching_) {
      const auto side = input.key == Key::Left ? ArmSide::Left : ArmSide::Right;
      if (hasTarget(command_.targets, side)) {
        command_.selected_side = side;
        command_.status = std::string{"Selected "} + armSideName(side);
      }
    }
    return;
  }
  if (input.key == Key::Up) {
    adjustStep(command_, step_m_, options_, kStepScale);
    return;
  }
  if (input.key == Key::Down) {
    adjustStep(command_, step_m_, options_, 1.0 / kStepScale);
    return;
  }
  if (input.key != Key::Character) {
    return;
  }

  const char key = static_cast<char>(std::tolower(
    static_cast<unsigned char>(input.character)));
  switch (key) {
    case 'x':
      command_.status = "Exiting";
      command_.stop_requested = true;
      break;
    case 'h':
      show_help_ = !show_help_;
      break;
    case ' ':
      command_.paused = !command_.paused;
      command_.status = command_.paused ? "Publishing paused" : "Publishing resumed";
      break;
    case 'm': {
      std::ostringstream prompt;
      prompt << "New step [" << std::fixed << std::setprecision(4)
             << options_.min_step_m << ", " << options_.max_step_m << "] m: ";
      const auto raw_value = terminal_.promptLine(prompt.str());
      if (!raw_value.has_value() || raw_value->empty()) {
        command_.status = "Step unchanged";
        break;
      }
      try {
        setStep(command_, step_m_, options_, std::stod(*raw_value));
      } catch (const std::exception &) {
        command_.status = "Invalid step input: " + *raw_value;
      }
      break;
    }
    case 'r':
      reset_requested_ = command_.selected_side;
      command_.status = std::string{"Reset requested for "} + armSideName(command_.selected_side);
      break;
    case 'n':
      rotation_axis_index_ = (rotation_axis_index_ + 1) % kRotationAxes.size();
      command_.status = "Rotation axis set to TCP " + selectedRotationAxis(rotation_axis_index_);
      break;
    case 'i':
      rotateSelected(command_, rotation_axis_index_, rotation_step_rad_, true);
      break;
    case 'u':
      rotateSelected(command_, rotation_axis_index_, rotation_step_rad_, false);
      break;
    case 'w':
      moveSelected(command_, step_m_, 0.0, 0.0);
      break;
    case 's':
      moveSelected(command_, -step_m_, 0.0, 0.0);
      break;
    case 'a':
      moveSelected(command_, 0.0, step_m_, 0.0);
      break;
    case 'd':
      moveSelected(command_, 0.0, -step_m_, 0.0);
      break;
    case 'q':
      moveSelected(command_, 0.0, 0.0, step_m_);
      break;
    case 'e':
      moveSelected(command_, 0.0, 0.0, -step_m_);
      break;
    default:
      break;
  }
}

}  // namespace motion_control_lab
