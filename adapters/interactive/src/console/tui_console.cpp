#include "console/tui_console.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <deque>
#include <ftxui/component/app.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/component_options.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/loop.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/dom/table.hpp>
#include <ftxui/screen/color.hpp>
#include <ftxui/screen/terminal.hpp>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "config/constants.hpp"

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

const ArmForwardKinematics * findForwardKinematics(
  const std::vector<ArmForwardKinematics> & poses, ArmSide side)
{
  for (const auto & pose : poses) {
    if (pose.side == side) {
      return &pose;
    }
  }
  return nullptr;
}

bool hasTarget(const std::vector<ArmTarget> & targets, ArmSide side)
{
  return findTarget(targets, side) != nullptr;
}

std::string targetTopicForSide(const InteractiveIkPresentation & presentation, ArmSide side)
{
  const auto * arm = findArmPresentation(presentation, side);
  return arm == nullptr ? "<unconfigured>" : arm->target_channel;
}

std::string selectedRotationAxis(std::size_t axis_index) { return kRotationAxes[axis_index]; }

void adjustStep(
  TargetCommand & command, double & step_m, const TuiTeleopOptions & options, double scale)
{
  step_m = std::clamp(step_m * scale, options.min_step_m, options.max_step_m);
  std::ostringstream status;
  status << "Step set to " << std::fixed << std::setprecision(4) << step_m << " m";
  command.status = status.str();
}

void setStep(
  TargetCommand & command, double & step_m, const TuiTeleopOptions & options, double next_step_m)
{
  if (!std::isfinite(next_step_m)) {
    command.status = "Step must be finite";
    return;
  }
  if (next_step_m < options.min_step_m || next_step_m > options.max_step_m) {
    std::ostringstream status;
    status << "Step must be in [" << std::fixed << std::setprecision(4) << options.min_step_m
           << ", " << options.max_step_m << "] m";
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
  status << armSideName(command.selected_side) << " move dx=" << std::showpos << std::fixed
         << std::setprecision(4) << dx << " dy=" << dy << " dz=" << dz;
  command.status = status.str();
}

void rotateSelected(
  TargetCommand & command, std::size_t axis_index, double rotation_step_rad, bool clockwise)
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
  target->target_pose.linear() = (current * delta).normalized().toRotationMatrix();

  std::ostringstream status;
  status << armSideName(command.selected_side) << " rotate "
         << (clockwise ? "clockwise" : "counter-clockwise") << " around TCP " << selected_axis
         << " by " << std::fixed << std::setprecision(2) << std::abs(angle) * 180.0 / kPi << " deg";
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
  text << "frame=" << base_frame_id << " pos=(" << std::showpos << std::fixed
       << std::setprecision(4) << pose.translation().x() << ", " << pose.translation().y() << ", "
       << pose.translation().z() << ") quat=(" << std::setprecision(3) << q.x() << ", " << q.y()
       << ", " << q.z() << ", " << q.w() << ")";
  return text.str();
}

std::string formatTargetError(const ArmTargetError & error)
{
  std::ostringstream line;
  line << "error " << armSideName(error.side) << ": position=" << std::fixed << std::setprecision(6)
       << error.position_m << " m  orientation=" << error.orientation_rad << " rad";
  return line.str();
}

std::string formatStep(double step_m, std::size_t rotation_axis_index, double rotation_step_deg)
{
  std::ostringstream line;
  line << "step=" << std::fixed << std::setprecision(4) << step_m << " m  rot_axis=tcp_"
       << selectedRotationAxis(rotation_axis_index) << "  rot_step=" << std::setprecision(2)
       << rotation_step_deg << " deg";
  return line.str();
}

std::string formatIk(const IkDebugFrame & frame)
{
  std::ostringstream line;
  line << "IK: " << frame.ik_status << "  iterations=" << frame.iterations
       << "  converged=" << std::boolalpha << frame.converged << "  solve_ms=" << std::fixed
       << std::setprecision(3) << frame.solve_time_ms;
  return line.str();
}

std::string jointPositionText(
  const JointNames & joint_names, const std::vector<double> & positions, const std::string & prefix)
{
  std::ostringstream line;
  line << prefix;
  for (std::size_t index = 0; index < positions.size(); ++index) {
    if (index != 0U) {
      line << ' ';
    }
    line << (index < joint_names.size() ? joint_names[index] : "joint_" + std::to_string(index))
         << '=' << std::fixed << std::setprecision(3) << positions[index];
  }
  return line.str();
}

std::string selectedJointPositionText(
  const InteractiveIkPresentation & presentation, ArmSide side, const JointNames & joint_names,
  const std::vector<double> & positions)
{
  std::ostringstream line;
  line << armSideName(side) << " arm q: ";
  const auto * arm = findArmPresentation(presentation, side);
  if (arm == nullptr) {
    return line.str();
  }
  bool first = true;
  for (const auto index : arm->joint_indices) {
    if (index >= positions.size()) {
      continue;
    }
    if (!first) {
      line << ' ';
    }
    first = false;
    line << (index < joint_names.size() ? joint_names[index] : "joint_" + std::to_string(index))
         << '=' << std::fixed << std::setprecision(3) << positions[index];
  }
  return line.str();
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

ftxui::Element labelledParagraph(const std::string & label, const std::string & value)
{
  using namespace ftxui;
  return hbox({text(label) | bold, paragraph(value) | flex});
}

ftxui::Element modeBadge(IkRuntimeState runtime_state, bool paused)
{
  using namespace ftxui;
  if (runtime_state == IkRuntimeState::FaultHold) {
    return text(" FAULT HOLD ") | bold | color(Color::Red);
  }
  if (runtime_state == IkRuntimeState::RecoverableReject) {
    return text(" TARGET REJECTED ") | bold | color(Color::Yellow);
  }
  return text(paused ? " PAUSED " : " PUBLISHING ") | bold |
         color(paused ? Color::Yellow : Color::Green);
}

ftxui::Element collisionPairElement(
  const SelfCollisionPairDebug & pair, const SelfCollisionDebug & collision)
{
  using namespace ftxui;
  std::ostringstream line;
  const std::string state = collisionPairState(pair, collision);
  line << "[" << state << "] " << pair.first_link << " <-> " << pair.second_link
       << "  before=" << std::showpos << std::fixed << std::setprecision(4)
       << pair.distance_before_m << "  after=" << pair.distance_after_m << " m";
  auto element = paragraph(line.str());
  if (state == "COLLISION") {
    return element | bold | color(Color::Red);
  }
  if (state == "SHORTFALL" || state == "ACTIVE") {
    return element | color(Color::Yellow);
  }
  return element | dim;
}

std::string formatFixed(double value, int precision)
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(precision) << value;
  return output.str();
}

std::string formatTimingPair(double latest, double maximum)
{
  return formatFixed(latest, 3) + "/" + formatFixed(maximum, 3);
}

std::string formatPercentileTriplet(const RollingPercentilesSnapshot & percentiles)
{
  if (percentiles.window_sample_count == 0U) {
    return "-";
  }
  return formatFixed(percentiles.p90, 3) + "/" + formatFixed(percentiles.p95, 3) + "/" +
         formatFixed(percentiles.p99, 3);
}

std::string formatPercentileWindow(const RollingPercentilesSnapshot & percentiles)
{
  return std::to_string(percentiles.window_sample_count) + "/" +
         std::to_string(percentiles.window_capacity) + " (" +
         std::to_string(percentiles.total_sample_count) + " total)";
}

std::string formatScientific(double value)
{
  std::ostringstream output;
  output << std::scientific << std::setprecision(3) << value;
  return output.str();
}

std::string formatPosition(const Pose & pose)
{
  std::ostringstream output;
  output << std::showpos << std::fixed << std::setprecision(4) << pose.translation().x() << ' '
         << pose.translation().y() << ' ' << pose.translation().z();
  return output.str();
}

std::string formatQuaternion(const Pose & pose)
{
  const Eigen::Quaterniond quaternion(pose.linear());
  std::ostringstream output;
  output << std::showpos << std::fixed << std::setprecision(4) << quaternion.x() << ' '
         << quaternion.y() << ' ' << quaternion.z() << ' ' << quaternion.w();
  return output.str();
}

std::string formatSpatial(const Eigen::Matrix<double, 6, 1> & value)
{
  std::ostringstream output;
  output << std::showpos << std::scientific << std::setprecision(3);
  for (Eigen::Index index = 0; index < value.size(); ++index) {
    if (index != 0) {
      output << ' ';
    }
    output << value[index];
  }
  return output.str();
}

std::string joinStrings(const std::vector<std::string> & values)
{
  if (values.empty()) {
    return "-";
  }
  std::ostringstream output;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      output << ", ";
    }
    output << values[index];
  }
  return output.str();
}

std::string joinCpus(const std::vector<unsigned int> & cpus)
{
  if (cpus.empty()) {
    return "-";
  }
  std::ostringstream output;
  for (std::size_t index = 0; index < cpus.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << cpus[index];
  }
  return output.str();
}

ftxui::Element renderTable(std::vector<std::vector<std::string>> rows)
{
  using namespace ftxui;
  if (rows.empty()) {
    return text("<none>") | dim;
  }
  Table table(std::move(rows));
  table.SelectRow(0).Decorate(bold);
  table.SelectRow(0).SeparatorHorizontal(LIGHT);
  table.SelectAll().SeparatorVertical(LIGHT);
  return table.Render();
}

ftxui::Element debugWindow(const std::string & title, ftxui::Element content)
{
  using namespace ftxui;
  return window(text(" " + title + " ") | bold, std::move(content));
}

}  // namespace

class TuiConsole::Impl
{
public:
  explicit Impl(TuiConsole & source)
  : source_(source), app_(ftxui::App::FullscreenAlternateScreen())
  {
    using namespace ftxui;

    InputOption input_options;
    input_options.multiline = false;
    step_input_ = Input(&step_input_value_, "translation step in metres", input_options);

    auto main_component = Renderer([this] { return renderMain(); });
    tabs_ = Container::Tab({main_component, step_input_}, &active_tab_);
    root_ = Renderer(tabs_, [this] {
      auto main = renderMain();
      if (step_input_active_) {
        return dbox({main, renderStepDialog()});
      }
      if (source_.show_help_) {
        return dbox({main, renderHelpDialog()});
      }
      return main;
    });
    root_ |= CatchEvent([this](Event event) { return handleEvent(event); });

    app_.TrackMouse(false);
    loop_ = std::make_unique<Loop>(&app_, root_);
  }

  void poll()
  {
    if (!loop_->HasQuitted()) {
      loop_->RunOnce();
    }
    if (loop_->HasQuitted()) {
      source_.command_.status = "Exiting";
      source_.command_.stop_requested = true;
    }
  }

  void render(
    const IkDebugFrame & frame, std::size_t publish_count, const std::string & sink_status)
  {
    if (loop_->HasQuitted()) {
      return;
    }
    recordFrameEvents(frame, publish_count);
    frame_ = frame;
    publish_count_ = publish_count;
    sink_status_ = sink_status;
    has_frame_ = true;
    app_.PostEvent(ftxui::Event::Custom);
    loop_->RunOnce();
  }

  void setMotionInputEnabled(bool enabled, const std::string & status)
  {
    source_.motion_input_enabled_ = enabled;
    if (!enabled) {
      step_input_active_ = false;
      active_tab_ = 0;
    }
    source_.motion_input_disabled_status_ = status;
    source_.command_.status = status;
  }

private:
  ftxui::Element renderHeader() const
  {
    using namespace ftxui;
    std::ostringstream rate;
    rate << "loop=" << static_cast<int>(source_.rate_hz_) << " Hz  publish_seq=" << publish_count_;
    return vbox({
      hbox({text(source_.title_) | bold, filler(), text(sink_status_) | dim}),
      hbox({
        text(std::string{"side="} + armSideName(source_.command_.selected_side) + "  "),
        modeBadge(frame_.runtime_state, source_.command_.paused),
        filler(),
        text(rate.str()) | dim,
      }),
    });
  }

  ftxui::Element renderTabs() const
  {
    using namespace ftxui;
    static const std::vector<std::string> labels = {
      "1 Overview", "2 Solver/QP", "3 Joints", "4 Runtime", "5 Events"};
    Elements tabs;
    for (std::size_t index = 0; index < labels.size(); ++index) {
      auto label = text(" " + labels[index] + " ");
      if (static_cast<int>(index) == page_index_) {
        label = label | bold | color(Color::Black) | bgcolor(Color::Cyan);
      } else {
        label = label | dim;
      }
      tabs.push_back(std::move(label));
    }
    return hbox(std::move(tabs));
  }

  ftxui::Element renderMain() const
  {
    using namespace ftxui;
    if (!has_frame_) {
      return vbox({
               text(source_.title_) | bold | center,
               separator(),
               text("Waiting for the first IK frame") | center | flex,
               separator(),
               labelledParagraph("status: ", source_.command_.status),
             }) |
             border;
    }

    auto body =
      renderPage() | focusPositionRelative(0.0F, scroll_y_) | vscroll_indicator | yframe | flex;
    Elements footer;
    footer.push_back(labelledParagraph("command: ", source_.command_.status));
    if (!frame_.status.empty() && frame_.status != source_.command_.status) {
      footer.push_back(labelledParagraph("solver: ", frame_.status));
    }
    return vbox({
             renderHeader(),
             separator(),
             renderTabs(),
             separator(),
             std::move(body),
             separator(),
             vbox(std::move(footer)),
           }) |
           border;
  }

  ftxui::Element renderPage() const
  {
    switch (page_index_) {
      case 0:
        return renderOverview();
      case 1:
        return renderSolverPage();
      case 2:
        return renderJointsPage();
      case 3:
        return renderRuntimePage();
      case 4:
        return renderEventsPage();
      default:
        return ftxui::text("Unknown page");
    }
  }

  ftxui::Element renderOverview() const
  {
    using namespace ftxui;
    const int width = Terminal::Size().dimx;
    if (width >= 200) {
      return vbox({
        hbox({
          renderCartesianPanel(false) | flex,
          separator(),
          renderSolverSummaryPanel() | flex,
          separator(),
          renderRuntimeSummaryPanel() | flex,
        }),
        renderAllJointsCompactPanel(false),
      });
    }
    if (width >= 100) {
      return vbox({
        hbox({
          renderCartesianPanel(false) | flex,
          separator(),
          renderSolverSummaryPanel() | flex,
        }),
        hbox({
          renderAllJointsCompactPanel(false) | flex,
          separator(),
          renderRuntimeSummaryPanel() | flex,
        }),
      });
    }
    return vbox({
      renderCartesianPanel(true),
      renderSolverSummaryPanel(),
      renderAllJointsCompactPanel(false),
      renderRuntimeSummaryPanel(),
    });
  }

  ftxui::Element renderCartesianPanel(bool compact) const
  {
    using namespace ftxui;
    Elements content;
    content.push_back(text(formatStep(
      source_.step_m_, source_.rotation_axis_index_, source_.options_.rotation_step_deg)));
    content.push_back(labelledParagraph("frame: ", source_.presentation_.base_frame_id));

    std::vector<std::vector<std::string>> rows = {
      {"arm", "kind", "position xyz [m]", "quaternion xyzw"}};
    for (const auto & arm : source_.presentation_.arms) {
      if (compact && arm.side != source_.command_.selected_side) {
        continue;
      }
      if (const auto * target = findTarget(source_.command_.targets, arm.side)) {
        if (compact) {
          content.push_back(labelledParagraph(
            std::string{armSideName(arm.side)} + " target p: ",
            formatPosition(target->target_pose)));
          content.push_back(labelledParagraph(
            std::string{armSideName(arm.side)} + " target q: ",
            formatQuaternion(target->target_pose)));
        } else {
          rows.push_back(
            {armSideName(arm.side), "target", formatPosition(target->target_pose),
             formatQuaternion(target->target_pose)});
        }
      }
      if (frame_.rejected_target.has_value()) {
        if (const auto * rejected = findTarget(frame_.rejected_target->targets, arm.side)) {
          if (compact) {
            content.push_back(
              labelledParagraph(
                std::string{armSideName(arm.side)} + " rejected p: ",
                formatPosition(rejected->target_pose)) |
              color(Color::Yellow));
            content.push_back(
              labelledParagraph(
                std::string{armSideName(arm.side)} + " rejected q: ",
                formatQuaternion(rejected->target_pose)) |
              color(Color::Yellow));
          } else {
            rows.push_back(
              {armSideName(arm.side), "rejected", formatPosition(rejected->target_pose),
               formatQuaternion(rejected->target_pose)});
          }
        }
      }
      if (const auto * fk = findForwardKinematics(frame_.forward_kinematics, arm.side)) {
        if (compact) {
          content.push_back(labelledParagraph(
            std::string{armSideName(arm.side)} + " FK p: ", formatPosition(fk->pose)));
          content.push_back(labelledParagraph(
            std::string{armSideName(arm.side)} + " FK q: ", formatQuaternion(fk->pose)));
        } else {
          rows.push_back(
            {armSideName(arm.side), "FK", formatPosition(fk->pose), formatQuaternion(fk->pose)});
        }
      }
      if (const auto * error = findError(frame_.target_errors, arm.side)) {
        std::ostringstream line;
        line << armSideName(arm.side) << " error: position=" << formatScientific(error->position_m)
             << " m  orientation=" << formatScientific(error->orientation_rad) << " rad";
        content.push_back(text(line.str()));
      }
    }
    if (!compact) {
      content.insert(content.begin() + 2, renderTable(std::move(rows)));
    }
    if (frame_.rejected_target.has_value()) {
      content.push_back(
        labelledParagraph(
          "rejected target revision: ", std::to_string(frame_.rejected_target->revision)) |
        color(Color::Yellow));
      if (!frame_.rejected_target->detail.empty()) {
        content.push_back(paragraph(frame_.rejected_target->detail) | color(Color::Yellow));
      }
    }
    if (frame_.cartesian_planner.has_value()) {
      const auto & planner = *frame_.cartesian_planner;
      content.push_back(separator());
      content.push_back(labelledParagraph(
        "planner: ", planner.state + " sample=" + formatFixed(planner.sample_time_s, 6) + " s"));
      for (const auto & arm : planner.arms) {
        content.push_back(labelledParagraph(
          std::string{armSideName(arm.side)} + " goal p: ", formatPosition(arm.source_goal)));
        content.push_back(labelledParagraph(
          std::string{armSideName(arm.side)} + " reference p: ", formatPosition(arm.reference)));
        content.push_back(labelledParagraph(
          std::string{armSideName(arm.side)} + " reference twist: ",
          formatSpatial(arm.reference_twist)));
        content.push_back(labelledParagraph(
          std::string{armSideName(arm.side)} + " reference acceleration: ",
          formatSpatial(arm.reference_acceleration)));
        content.push_back(labelledParagraph(
          std::string{armSideName(arm.side)} + " tracking error: ",
          "position=" + formatScientific(arm.tracking_position_error_m) +
            " m orientation=" + formatScientific(arm.tracking_orientation_error_rad) + " rad"));
      }
    }

    std::ostringstream topics;
    for (std::size_t index = 0; index < source_.presentation_.arms.size(); ++index) {
      if (index != 0U) {
        topics << "  ";
      }
      const auto & arm = source_.presentation_.arms[index];
      topics << armSideName(arm.side) << '=' << arm.target_channel;
    }
    content.push_back(paragraph("targets: " + topics.str()) | dim);
    return debugWindow("Cartesian", vbox(std::move(content)));
  }

  ftxui::Element renderSolverSummaryPanel() const
  {
    using namespace ftxui;
    std::vector<std::vector<std::string>> rows = {
      {"solver", "result", "termination", "QP", "IK total/QP/non-QP ms", "hard max"}};
    for (const auto & solver : frame_.solvers) {
      const std::string qp_timing =
        solver.has_qp_diagnostics ? formatFixed(solver.qp_solve_time_ms, 3) : "-";
      const std::string non_qp_timing =
        solver.has_qp_diagnostics
          ? formatFixed(std::max(0.0, solver.ik_solve_time_ms - solver.qp_solve_time_ms), 3)
          : "-";
      rows.push_back(
        {solver.label, solver.disposition, solver.termination_reason, solver.qp_status,
         formatFixed(solver.ik_solve_time_ms, 3) + "/" + qp_timing + "/" + non_qp_timing,
         solver.has_qp_diagnostics ? formatScientific(solver.maximum_hard_violation) : "-"});
    }
    if (frame_.solvers.empty()) {
      rows.push_back({"IK", frame_.ik_status, "-", "-", formatFixed(frame_.solve_time_ms, 3), "-"});
    }
    return debugWindow("Solver health", renderTable(std::move(rows)));
  }

  ftxui::Element renderRuntimeSummaryPanel() const
  {
    using namespace ftxui;
    Elements content;
    if (frame_.workers.empty()) {
      content.push_back(text("single-loop runtime") | dim);
    } else {
      std::vector<std::vector<std::string>> rows = {
        {"worker", "Hz", "iterations", "miss", "skipped", "recoverable rejects", "solver now/max",
         "sched now/max", "exec now/max"}};
      for (const auto & worker : frame_.workers) {
        rows.push_back(
          {worker.label, formatFixed(worker.configured_rate_hz, 0),
           std::to_string(worker.iteration_count), std::to_string(worker.deadline_miss_count),
           std::to_string(worker.skipped_release_count),
           std::to_string(worker.recoverable_rejection_count),
           formatTimingPair(worker.latest_solver_ms, worker.maximum_solver_ms),
           formatTimingPair(worker.latest_release_lateness_ms, worker.maximum_release_lateness_ms),
           formatTimingPair(worker.latest_execution_ms, worker.maximum_execution_ms)});
      }
      content.push_back(renderTable(std::move(rows)));
    }
    for (const auto & collision : frame_.self_collisions) {
      std::ostringstream line;
      line << collision.label << ": seq=" << collision.input_state_sequence
           << " min=" << formatFixed(collision.minimum_distance_before_m, 4) << " -> "
           << formatFixed(collision.minimum_distance_after_m, 4)
           << " m  shortfall=" << formatFixed(collision.margin_shortfall_m, 4) << " m";
      content.push_back(paragraph(line.str()));
    }
    return debugWindow("Runtime", vbox(std::move(content)));
  }

  ftxui::Element renderSolverPage() const
  {
    using namespace ftxui;
    Elements content;
    if (frame_.solvers.empty()) {
      return debugWindow("Solver/QP", paragraph(formatIk(frame_)));
    }
    for (const auto & solver : frame_.solvers) {
      Elements solver_content;
      std::vector<std::vector<std::string>> summary = {
        {"field", "value"},
        {"disposition", solver.disposition},
        {"termination", solver.termination_reason},
        {"converged", solver.converged ? "true" : "false"},
        {"joint-limit policy", solver.joint_limit_policy},
        {"IK iterations", std::to_string(solver.ik_iterations)},
        {"IK total [ms]", formatFixed(solver.ik_solve_time_ms, 6)},
        {"IK percentile window", formatPercentileWindow(solver.ik_solve_time_percentiles)},
        {"IK P90/P95/P99 [ms]", formatPercentileTriplet(solver.ik_solve_time_percentiles)},
        {"QP backend/status", solver.backend + "/" + solver.qp_status},
        {"QP native status", solver.native_status.empty() ? "-" : solver.native_status},
        {"QP iterations", solver.has_qp_diagnostics ? std::to_string(solver.qp_iterations) : "-"},
        {"QP backend [ms]",
         solver.has_qp_diagnostics ? formatFixed(solver.qp_solve_time_ms, 6) : "-"},
        {"IK non-QP [ms]",
         solver.has_qp_diagnostics
           ? formatFixed(std::max(0.0, solver.ik_solve_time_ms - solver.qp_solve_time_ms), 6)
           : "-"},
        {"objective", solver.has_qp_diagnostics ? formatScientific(solver.objective_value) : "-"},
        {"primal residual",
         solver.has_qp_diagnostics ? formatScientific(solver.primal_residual) : "-"},
        {"dual residual", solver.has_qp_diagnostics ? formatScientific(solver.dual_residual) : "-"},
        {"maximum hard violation",
         solver.has_qp_diagnostics ? formatScientific(solver.maximum_hard_violation) : "-"},
        {"active set size",
         solver.has_qp_diagnostics ? std::to_string(solver.active_set_size) : "-"},
        {"warm start",
         solver.has_qp_diagnostics ? (solver.warm_start_used ? "true" : "false") : "-"},
        {"saturated joints", joinStrings(solver.saturated_joints)},
      };
      if (solver.run_counters.has_value()) {
        summary.push_back(
          {"attempts/accepted/rejected", std::to_string(solver.run_counters->attempts) + "/" +
                                           std::to_string(solver.run_counters->accepted) + "/" +
                                           std::to_string(solver.run_counters->rejected)});
      }
      solver_content.push_back(renderTable(std::move(summary)));
      if (solver.grouped_attempt.has_value()) {
        const auto & attempt = *solver.grouped_attempt;
        solver_content.push_back(text("Grouped attempt") | bold);
        if (Terminal::Size().dimx < 120) {
          solver_content.push_back(renderTable({
            {"field", "value"},
            {"attempt revision", std::to_string(attempt.attempt_revision)},
            {"value revision", std::to_string(attempt.value_revision)},
            {"attempt accepted", attempt.attempt_accepted ? "true" : "false"},
            {"has accepted value", attempt.has_accepted_value ? "true" : "false"},
            {"coupling", attempt.coupling_state},
            {"source value revision", std::to_string(attempt.consumed_source_value_revision)},
            {"captured state sequence", std::to_string(attempt.captured_state_sequence)},
            {"captured state time [ns]", std::to_string(attempt.captured_state_time_nanoseconds)},
            {"run generation", std::to_string(attempt.run_generation)},
          }));
        } else {
          solver_content.push_back(renderTable({
            {"attempt", "value", "accepted", "coupling", "source", "state", "run"},
            {std::to_string(attempt.attempt_revision), std::to_string(attempt.value_revision),
             attempt.attempt_accepted ? "true" : "false", attempt.coupling_state,
             std::to_string(attempt.consumed_source_value_revision),
             std::to_string(attempt.captured_state_sequence),
             std::to_string(attempt.run_generation)},
          }));
        }
        if (attempt.rejection_reason != "none") {
          solver_content.push_back(
            labelledParagraph("rejection: ", attempt.rejection_reason) | color(Color::Red));
        }
      }
      if (!solver.task_scales.empty()) {
        if (Terminal::Size().dimx < 120) {
          for (const auto & scale : solver.task_scales) {
            std::ostringstream line;
            line << scale.name << ": active=" << std::boolalpha << scale.active
                 << " scale=" << formatFixed(scale.scale, 6)
                 << " cost=" << formatScientific(scale.cost) << " degraded=" << scale.degraded
                 << " stuck=" << scale.stuck;
            solver_content.push_back(paragraph(line.str()));
          }
        } else {
          std::vector<std::vector<std::string>> rows = {
            {"task scale", "active", "scale", "cost", "degraded", "stuck"}};
          for (const auto & scale : solver.task_scales) {
            rows.push_back(
              {scale.name, scale.active ? "true" : "false", formatFixed(scale.scale, 6),
               formatScientific(scale.cost), scale.degraded ? "true" : "false",
               scale.stuck ? "true" : "false"});
          }
          solver_content.push_back(renderTable(std::move(rows)));
        }
      }
      if (!solver.requirements.empty()) {
        if (Terminal::Size().dimx < 120) {
          solver_content.push_back(text("Requirements") | bold);
          for (const auto & requirement : solver.requirements) {
            std::ostringstream line;
            line << requirement.name << ": enabled=" << std::boolalpha << requirement.enabled
                 << " active=" << requirement.active
                 << " violation=" << formatScientific(requirement.maximum_violation)
                 << " cost=" << formatScientific(requirement.cost) << " unit=" << requirement.unit
                 << " source=" << requirement.source;
            solver_content.push_back(paragraph(line.str()));
          }
        } else {
          std::vector<std::vector<std::string>> rows = {
            {"requirement", "enabled", "active", "violation", "cost", "unit", "source"}};
          for (const auto & requirement : solver.requirements) {
            rows.push_back(
              {requirement.name, requirement.enabled ? "true" : "false",
               requirement.active ? "true" : "false",
               formatScientific(requirement.maximum_violation), formatScientific(requirement.cost),
               requirement.unit, requirement.source});
          }
          solver_content.push_back(renderTable(std::move(rows)));
        }
      }
      content.push_back(debugWindow(solver.label, vbox(std::move(solver_content))));
    }
    return vbox(std::move(content));
  }

  std::vector<std::size_t> armJointIndices(ArmSide side) const
  {
    const auto * arm = findArmPresentation(source_.presentation_, side);
    return arm == nullptr ? std::vector<std::size_t>{} : arm->joint_indices;
  }

  std::vector<std::size_t> nonArmJointIndices() const
  {
    std::vector<std::size_t> indices;
    for (std::size_t index = 0; index < frame_.positions.size(); ++index) {
      bool arm_joint = false;
      for (const auto & arm : source_.presentation_.arms) {
        if (
          std::find(arm.joint_indices.begin(), arm.joint_indices.end(), index) !=
          arm.joint_indices.end()) {
          arm_joint = true;
          break;
        }
      }
      if (!arm_joint) {
        indices.push_back(index);
      }
    }
    return indices;
  }

  std::vector<std::string> saturatedJointNames() const
  {
    std::vector<std::string> saturated;
    for (const auto & solver : frame_.solvers) {
      saturated.insert(
        saturated.end(), solver.saturated_joints.begin(), solver.saturated_joints.end());
    }
    return saturated;
  }

  std::string jointName(std::size_t index) const
  {
    return index < frame_.joint_names.size() ? frame_.joint_names[index]
                                             : "joint_" + std::to_string(index);
  }

  bool isJointSaturated(std::size_t index, const std::vector<std::string> & saturated) const
  {
    const std::string name = jointName(index);
    return std::find(saturated.begin(), saturated.end(), name) != saturated.end();
  }

  ftxui::Element renderJointGroupPanel(
    const std::string & title, const std::vector<std::size_t> & indices) const
  {
    using namespace ftxui;
    std::vector<std::vector<std::string>> rows = {{"joint", "q", "dq", "saturated"}};
    const auto saturated = saturatedJointNames();
    for (const auto index : indices) {
      if (index >= frame_.positions.size()) {
        continue;
      }
      rows.push_back(
        {jointName(index), formatFixed(frame_.positions[index], 6),
         index < frame_.velocities.size() ? formatFixed(frame_.velocities[index], 6) : "-",
         isJointSaturated(index, saturated) ? "YES" : ""});
    }
    return debugWindow(title, renderTable(std::move(rows)));
  }

  ftxui::Element renderAllJointsCompactPanel(bool include_velocity) const
  {
    using namespace ftxui;
    const auto body = nonArmJointIndices();
    const auto left = armJointIndices(ArmSide::Left);
    const auto right = armJointIndices(ArmSide::Right);
    const auto saturated = saturatedJointNames();
    const std::size_t row_count = std::max({body.size(), left.size(), right.size()});

    std::vector<std::vector<std::string>> rows;
    rows.push_back(
      include_velocity
        ? std::vector<std::string>{"body", "q", "dq", "left", "q", "dq", "right", "q", "dq"}
        : std::vector<std::string>{"body", "q", "left", "q", "right", "q"});

    auto shortBodyName = [this](std::size_t index) {
      std::string name = jointName(index);
      constexpr std::string_view suffix = "_joint";
      if (
        name.size() >= suffix.size() &&
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
        name.erase(name.size() - suffix.size());
      }
      return name;
    };
    auto appendJoint = [this, &saturated, include_velocity](
                         std::vector<std::string> & row, const std::vector<std::size_t> & indices,
                         std::size_t display_row, const std::string & label) {
      if (display_row >= indices.size() || indices[display_row] >= frame_.positions.size()) {
        row.push_back("");
        row.push_back("");
        if (include_velocity) {
          row.push_back("");
        }
        return;
      }
      const std::size_t index = indices[display_row];
      row.push_back(label + (isJointSaturated(index, saturated) ? "!" : ""));
      row.push_back(formatFixed(frame_.positions[index], 4));
      if (include_velocity) {
        row.push_back(
          index < frame_.velocities.size() ? formatFixed(frame_.velocities[index], 4) : "-");
      }
    };

    for (std::size_t display_row = 0; display_row < row_count; ++display_row) {
      std::vector<std::string> row;
      appendJoint(
        row, body, display_row, display_row < body.size() ? shortBodyName(body[display_row]) : "");
      appendJoint(row, left, display_row, "J" + std::to_string(display_row + 1));
      appendJoint(row, right, display_row, "J" + std::to_string(display_row + 1));
      rows.push_back(std::move(row));
    }
    return debugWindow(
      include_velocity ? "All joint state [rad, rad/s]" : "All joint positions [rad]",
      vbox({renderTable(std::move(rows)), text("! saturated") | dim}));
  }

  ftxui::Element renderJointsPage() const
  {
    using namespace ftxui;
    if (Terminal::Size().dimx >= 145) {
      return hbox({
        renderJointGroupPanel("Body state", nonArmJointIndices()) | flex,
        separator(),
        renderJointGroupPanel("Left arm state", armJointIndices(ArmSide::Left)) | flex,
        separator(),
        renderJointGroupPanel("Right arm state", armJointIndices(ArmSide::Right)) | flex,
      });
    }
    return renderAllJointsCompactPanel(Terminal::Size().dimx >= 95);
  }

  ftxui::Element renderRuntimePage() const
  {
    using namespace ftxui;
    Elements content;
    if (!frame_.cpu_affinities.empty()) {
      std::vector<std::vector<std::string>> rows = {
        {"role", "status", "TID", "requested CPUs", "effective CPUs"}};
      for (const auto & affinity : frame_.cpu_affinities) {
        const bool bound = affinity.enabled && !affinity.effective_cpus.empty();
        rows.push_back(
          {affinity.role,
           !affinity.enabled ? "disabled"
           : bound           ? "bound"
                             : "pending",
           affinity.thread_id > 0 ? std::to_string(affinity.thread_id) : "-",
           joinCpus(affinity.requested_cpus), joinCpus(affinity.effective_cpus)});
      }
      content.push_back(debugWindow("CPU affinity", renderTable(std::move(rows))));
    }
    std::vector<std::vector<std::string>> percentile_rows = {
      {"solver", "window/capacity", "total", "P90 [ms]", "P95 [ms]", "P99 [ms]"}};
    for (const auto & solver : frame_.solvers) {
      const auto & percentiles = solver.ik_solve_time_percentiles;
      if (percentiles.window_sample_count == 0U) {
        continue;
      }
      percentile_rows.push_back(
        {solver.label,
         std::to_string(percentiles.window_sample_count) + "/" +
           std::to_string(percentiles.window_capacity),
         std::to_string(percentiles.total_sample_count), formatFixed(percentiles.p90, 3),
         formatFixed(percentiles.p95, 3), formatFixed(percentiles.p99, 3)});
    }
    if (percentile_rows.size() > 1U) {
      content.push_back(
        debugWindow("IK solve-time percentiles", renderTable(std::move(percentile_rows))));
    }
    if (!frame_.workers.empty()) {
      if (Terminal::Size().dimx < 120) {
        Elements workers;
        for (const auto & worker : frame_.workers) {
          workers.push_back(debugWindow(
            worker.label,
            renderTable({
              {"metric", "value"},
              {"configured rate [Hz]", formatFixed(worker.configured_rate_hz, 0)},
              {"solver latest/max [ms]",
               formatTimingPair(worker.latest_solver_ms, worker.maximum_solver_ms)},
              {"sched delay latest/max [ms]",
               formatTimingPair(
                 worker.latest_release_lateness_ms, worker.maximum_release_lateness_ms)},
              {"non-solver execution latest/max [ms]",
               formatTimingPair(
                 worker.latest_non_solver_execution_ms, worker.maximum_non_solver_execution_ms)},
              {"worker execution latest/max [ms]",
               formatTimingPair(worker.latest_execution_ms, worker.maximum_execution_ms)},
              {"release-to-finish latest/max [ms]",
               formatTimingPair(
                 worker.latest_release_to_finish_ms, worker.maximum_release_to_finish_ms)},
              {"overrun latest/max [ms]",
               formatTimingPair(worker.latest_overrun_ms, worker.maximum_overrun_ms)},
              {"iterations", std::to_string(worker.iteration_count)},
              {"deadline misses", std::to_string(worker.deadline_miss_count)},
              {"consecutive misses", std::to_string(worker.consecutive_deadline_misses)},
              {"skipped releases", std::to_string(worker.skipped_release_count)},
              {"recoverable rejections", std::to_string(worker.recoverable_rejection_count)},
            })));
        }
        content.push_back(debugWindow("Periodic workers", vbox(std::move(workers))));
      } else {
        std::vector<std::vector<std::string>> rows = {
          {"worker", "Hz", "iterations", "miss/consecutive", "skipped", "recoverable rejects",
           "solver latest/max", "sched delay latest/max", "non-solver latest/max",
           "execution latest/max", "release-finish latest/max", "overrun latest/max"}};
        for (const auto & worker : frame_.workers) {
          rows.push_back(
            {worker.label, formatFixed(worker.configured_rate_hz, 0),
             std::to_string(worker.iteration_count),
             std::to_string(worker.deadline_miss_count) + "/" +
               std::to_string(worker.consecutive_deadline_misses),
             std::to_string(worker.skipped_release_count),
             std::to_string(worker.recoverable_rejection_count),
             formatTimingPair(worker.latest_solver_ms, worker.maximum_solver_ms),
             formatTimingPair(
               worker.latest_release_lateness_ms, worker.maximum_release_lateness_ms),
             formatTimingPair(
               worker.latest_non_solver_execution_ms, worker.maximum_non_solver_execution_ms),
             formatTimingPair(worker.latest_execution_ms, worker.maximum_execution_ms),
             formatTimingPair(
               worker.latest_release_to_finish_ms, worker.maximum_release_to_finish_ms),
             formatTimingPair(worker.latest_overrun_ms, worker.maximum_overrun_ms)});
        }
        content.push_back(debugWindow("Periodic workers [ms]", renderTable(std::move(rows))));
      }
    }
    for (const auto & collision : frame_.self_collisions) {
      Elements collision_content;
      collision_content.push_back(renderTable({
        {"input seq", "safe", "influence", "before min", "after min", "shortfall"},
        {std::to_string(collision.input_state_sequence),
         formatFixed(collision.minimum_distance_m, 6),
         formatFixed(collision.influence_distance_m, 6),
         formatFixed(collision.minimum_distance_before_m, 6),
         formatFixed(collision.minimum_distance_after_m, 6),
         formatFixed(collision.margin_shortfall_m, 6)},
      }));
      std::vector<const SelfCollisionPairDebug *> pairs;
      pairs.reserve(collision.pairs.size());
      for (const auto & pair : collision.pairs) {
        pairs.push_back(&pair);
      }
      std::sort(pairs.begin(), pairs.end(), [](const auto * left, const auto * right) {
        return collisionPairDistance(*left) < collisionPairDistance(*right);
      });
      for (const auto * pair : pairs) {
        collision_content.push_back(collisionPairElement(*pair, collision));
      }
      collision_content.push_back(
        paragraph(
          jointPositionText(frame_.joint_names, collision.input_joint_positions, "input q: ")) |
        dim);
      content.push_back(debugWindow(collision.label, vbox(std::move(collision_content))));
    }
    if (content.empty()) {
      content.push_back(
        text("No CPU affinity, multi-rate worker, or collision diagnostics") | dim | center);
    }
    return vbox(std::move(content));
  }

  ftxui::Element renderEventsPage() const
  {
    using namespace ftxui;
    std::vector<std::vector<std::string>> rows = {{"publish seq", "source", "event"}};
    for (const auto & event : events_) {
      rows.push_back({std::to_string(event.publish_sequence), event.source, event.message});
    }
    return debugWindow("Recent state changes", renderTable(std::move(rows)));
  }

  ftxui::Element renderHelpDialog() const
  {
    using namespace ftxui;
    return window(
             text(" Help ") | bold,
             vbox({
               text("1..5 / F1..F5 / Tab: switch Overview, Solver/QP, "
                    "Joints, Runtime, Events"),
               text("PageUp/PageDown/Home/End: scroll current page"),
               separator(),
               text("w/s: +x/-x    a/d: +y/-y    q/e: +z/-z"),
               text("n: cycle TCP rotation axis    i/u: rotate "
                    "clockwise/counter-clockwise"),
               text(
                 source_.allow_side_switching_ ? "LEFT/RIGHT: select arm    UP/DOWN: step x2 / "
                                                 "step /2"
                                               : "UP/DOWN: step x2 / step /2"),
               text("m: manual step    r: reset from FK    space: pause"),
               text("h or Esc: close help    x: exit"),
             })) |
           clear_under | center;
  }

  void appendEvent(std::size_t publish_sequence, std::string source, std::string message)
  {
    events_.push_back(DebugEvent{publish_sequence, std::move(source), std::move(message)});
    constexpr std::size_t kMaximumEventCount = 64U;
    while (events_.size() > kMaximumEventCount) {
      events_.pop_front();
    }
  }

  void recordFrameEvents(const IkDebugFrame & frame, std::size_t publish_sequence)
  {
    if (source_.command_.status != last_command_status_) {
      last_command_status_ = source_.command_.status;
      appendEvent(publish_sequence, "command", last_command_status_);
    }
    if (!frame.status.empty() && frame.status != last_frame_status_) {
      last_frame_status_ = frame.status;
      appendEvent(publish_sequence, "solver", last_frame_status_);
    }

    std::vector<std::string> solver_states;
    solver_states.reserve(frame.solvers.size());
    for (const auto & solver : frame.solvers) {
      std::string state =
        solver.disposition + " / " + solver.termination_reason + " / " + solver.qp_status;
      if (!solver.native_status.empty()) {
        state += " / " + solver.native_status;
      }
      solver_states.push_back(std::move(state));
    }
    for (std::size_t index = 0; index < solver_states.size(); ++index) {
      if (
        index >= last_solver_states_.size() || solver_states[index] != last_solver_states_[index]) {
        const std::string label = index < frame.solvers.size() ? frame.solvers[index].label : "IK";
        appendEvent(publish_sequence, label, solver_states[index]);
      }
    }
    last_solver_states_ = std::move(solver_states);
  }

  void selectPage(int page)
  {
    page_index_ = std::clamp(page, 0, 4);
    scroll_y_ = 0.0F;
  }

  ftxui::Element renderStepDialog() const
  {
    using namespace ftxui;
    std::ostringstream range;
    range << "Allowed range: [" << std::fixed << std::setprecision(4) << source_.options_.min_step_m
          << ", " << source_.options_.max_step_m << "] m";
    return window(
             text(" Manual translation step ") | bold,
             vbox({
               text(range.str()),
               hbox({text("value: "), step_input_->Render() | flex}),
               separatorEmpty(),
               text("Enter: apply    Esc: cancel") | dim,
             })) |
           clear_under | center;
  }

  bool handleEvent(const ftxui::Event & event)
  {
    using namespace ftxui;
    if (event == Event::Custom) {
      return true;
    }
    if (step_input_active_) {
      if (event == Event::Return) {
        applyStepInput();
        return true;
      }
      if (event == Event::Escape) {
        closeStepInput("Step unchanged");
        return true;
      }
      return false;
    }

    if (source_.show_help_) {
      if (event == Event::Escape || event == Event::h || event == Event::H) {
        source_.show_help_ = false;
      } else if (event == Event::q || event == Event::Q || event == Event::x || event == Event::X) {
        requestExit();
      }
      return true;
    }

    if (event == Event::Tab) {
      selectPage((page_index_ + 1) % 5);
      return true;
    }
    if (event == Event::TabReverse) {
      selectPage((page_index_ + 4) % 5);
      return true;
    }
    if (
      event == Event::F1 || event == Event::F2 || event == Event::F3 || event == Event::F4 ||
      event == Event::F5) {
      selectPage(
        event == Event::F1   ? 0
        : event == Event::F2 ? 1
        : event == Event::F3 ? 2
        : event == Event::F4 ? 3
                             : 4);
      return true;
    }
    if (event == Event::PageUp) {
      scroll_y_ = std::max(0.0F, scroll_y_ - 0.2F);
      return true;
    }
    if (event == Event::PageDown) {
      scroll_y_ = std::min(1.0F, scroll_y_ + 0.2F);
      return true;
    }
    if (event == Event::Home) {
      scroll_y_ = 0.0F;
      return true;
    }
    if (event == Event::End) {
      scroll_y_ = 1.0F;
      return true;
    }

    if (event == Event::Escape) {
      requestExit();
      return true;
    }
    if (event == Event::ArrowLeft || event == Event::ArrowRight) {
      if (source_.allow_side_switching_) {
        const auto side = event == Event::ArrowLeft ? ArmSide::Left : ArmSide::Right;
        if (hasTarget(source_.command_.targets, side)) {
          source_.command_.selected_side = side;
          source_.command_.status = std::string{"Selected "} + armSideName(side);
        }
      }
      return true;
    }
    if (!source_.motion_input_enabled_ && (event == Event::ArrowUp || event == Event::ArrowDown)) {
      source_.command_.status = source_.motion_input_disabled_status_;
      return true;
    }
    if (event == Event::ArrowUp) {
      adjustStep(source_.command_, source_.step_m_, source_.options_, kStepScale);
      return true;
    }
    if (event == Event::ArrowDown) {
      adjustStep(source_.command_, source_.step_m_, source_.options_, 1.0 / kStepScale);
      return true;
    }
    if (!event.is_character() || event.character().size() != 1U) {
      return false;
    }

    const char key =
      static_cast<char>(std::tolower(static_cast<unsigned char>(event.character().front())));
    if (source_.source_controls_.mode() == TuiControlMode::Replay) {
      if (key >= '1' && key <= '5') {
        selectPage(key - '1');
        return true;
      }
      const auto action = source_.source_controls_.handleCharacter(
        key, source_.command_, source_.single_step_requested_);
      if (action == TuiSourceControlAction::Exit) {
        requestExit();
      } else if (action == TuiSourceControlAction::ToggleHelp) {
        source_.show_help_ = !source_.show_help_;
      }
      return true;
    }
    const bool diagnostic_key = (key >= '1' && key <= '5') || key == 'x' || key == 'h';
    if (!source_.motion_input_enabled_ && !diagnostic_key) {
      source_.command_.status = source_.motion_input_disabled_status_;
      return true;
    }
    switch (key) {
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
        selectPage(key - '1');
        break;
      case 'x':
        requestExit();
        break;
      case 'h':
        source_.show_help_ = !source_.show_help_;
        break;
      case ' ':
        source_.command_.paused = !source_.command_.paused;
        source_.command_.status =
          source_.command_.paused ? "Publishing paused" : "Publishing resumed";
        break;
      case 'm':
        step_input_value_.clear();
        step_input_active_ = true;
        active_tab_ = 1;
        step_input_->TakeFocus();
        break;
      case 'r':
        source_.reset_requested_ = source_.command_.selected_side;
        source_.command_.status =
          std::string{"Reset requested for "} + armSideName(source_.command_.selected_side);
        break;
      case 'n':
        source_.rotation_axis_index_ = (source_.rotation_axis_index_ + 1) % kRotationAxes.size();
        source_.command_.status =
          "Rotation axis set to TCP " + selectedRotationAxis(source_.rotation_axis_index_);
        break;
      case 'i':
        rotateSelected(
          source_.command_, source_.rotation_axis_index_, source_.rotation_step_rad_, true);
        break;
      case 'u':
        rotateSelected(
          source_.command_, source_.rotation_axis_index_, source_.rotation_step_rad_, false);
        break;
      case 'w':
        moveSelected(source_.command_, source_.step_m_, 0.0, 0.0);
        break;
      case 's':
        moveSelected(source_.command_, -source_.step_m_, 0.0, 0.0);
        break;
      case 'a':
        moveSelected(source_.command_, 0.0, source_.step_m_, 0.0);
        break;
      case 'd':
        moveSelected(source_.command_, 0.0, -source_.step_m_, 0.0);
        break;
      case 'q':
        moveSelected(source_.command_, 0.0, 0.0, source_.step_m_);
        break;
      case 'e':
        moveSelected(source_.command_, 0.0, 0.0, -source_.step_m_);
        break;
      default:
        return false;
    }
    return true;
  }

  void applyStepInput()
  {
    if (step_input_value_.empty()) {
      closeStepInput("Step unchanged");
      return;
    }
    try {
      setStep(source_.command_, source_.step_m_, source_.options_, std::stod(step_input_value_));
    } catch (const std::exception &) {
      source_.command_.status = "Invalid step input: " + step_input_value_;
    }
    closeStepInput(source_.command_.status);
  }

  void closeStepInput(const std::string & status)
  {
    source_.command_.status = status;
    step_input_active_ = false;
    active_tab_ = 0;
  }

  void requestExit()
  {
    source_.command_.status = "Exiting";
    source_.command_.stop_requested = true;
    app_.Exit();
  }

  TuiConsole & source_;
  struct DebugEvent
  {
    std::size_t publish_sequence{0};
    std::string source;
    std::string message;
  };

  ftxui::App app_;
  ftxui::Component step_input_;
  ftxui::Component tabs_;
  ftxui::Component root_;
  std::unique_ptr<ftxui::Loop> loop_;
  std::string step_input_value_;
  bool step_input_active_{false};
  int active_tab_{0};
  int page_index_{0};
  float scroll_y_{0.0F};
  bool has_frame_{false};
  IkDebugFrame frame_;
  std::size_t publish_count_{0};
  std::string sink_status_{"visualization not opened"};
  std::deque<DebugEvent> events_;
  std::string last_command_status_;
  std::string last_frame_status_;
  std::vector<std::string> last_solver_states_;
};

TuiConsole::TuiConsole(
  const TuiTeleopOptions & options, double rate_hz, std::string title,
  InteractiveIkPresentation presentation, std::vector<ArmTarget> initial_targets,
  bool allow_side_switching, bool console_enabled, TuiControlMode control_mode)
: options_(options),
  rate_hz_(rate_hz),
  title_(std::move(title)),
  presentation_(std::move(presentation)),
  step_m_(options.step_m),
  rotation_step_rad_(options.rotation_step_deg * kPi / 180.0),
  allow_side_switching_(allow_side_switching),
  console_enabled_(console_enabled),
  source_controls_(control_mode)
{
  if (initial_targets.empty()) {
    throw std::runtime_error("TUI teleop requires at least one initial target");
  }

  command_.targets = std::move(initial_targets);
  command_.selected_side = parseArmSide(options.side);
  if (!hasTarget(command_.targets, command_.selected_side)) {
    command_.selected_side = command_.targets.front().side;
  }
  command_.status =
    allow_side_switching_
      ? "Initialized left and right targets from FK"
      : std::string{"Initialized "} + armSideName(command_.selected_side) + " target from FK";
  if (console_enabled_) {
    impl_ = std::make_unique<Impl>(*this);
  }
}

TuiConsole::~TuiConsole() = default;

void TuiConsole::poll()
{
  if (impl_ != nullptr) {
    impl_->poll();
  }
}

const TargetCommand & TuiConsole::command() const { return command_; }

std::optional<ArmSide> TuiConsole::consumeResetRequest()
{
  const auto requested = reset_requested_;
  reset_requested_.reset();
  return requested;
}

bool TuiConsole::consumeSingleStepRequest()
{
  const bool requested = single_step_requested_;
  single_step_requested_ = false;
  return requested;
}

void TuiConsole::setTargetPose(ArmSide side, const Pose & target_pose, const std::string & status)
{
  if (auto * target = findTarget(command_.targets, side)) {
    target->target_pose = target_pose;
  } else {
    command_.targets.push_back(ArmTarget{side, target_pose});
  }
  command_.status = status;
}

void TuiConsole::setStatus(const std::string & status) { command_.status = status; }

void TuiConsole::setMotionInputEnabled(bool enabled, const std::string & status)
{
  motion_input_enabled_ = enabled;
  motion_input_disabled_status_ = status;
  command_.status = status;
  if (impl_ != nullptr) {
    impl_->setMotionInputEnabled(enabled, status);
  }
}

void TuiConsole::render(
  const IkDebugFrame & frame, std::size_t publish_count, const std::string & sink_status)
{
  if (impl_ != nullptr) {
    impl_->render(frame, publish_count, sink_status);
  }
}

}  // namespace motion_control_lab
