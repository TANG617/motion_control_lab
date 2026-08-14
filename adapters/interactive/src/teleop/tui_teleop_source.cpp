#include "teleop/tui_teleop_source.hpp"

#include "config/constants.hpp"

#include <Eigen/Geometry>

#include <ftxui/component/app.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/component_options.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/loop.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/dom/table.hpp>
#include <ftxui/screen/color.hpp>
#include <ftxui/screen/terminal.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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
  const std::vector<ArmForwardKinematics> & poses,
  ArmSide side)
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
  step_m = std::clamp(step_m * scale, options.min_step_m, options.max_step_m);
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
  target->target_pose.linear() = (current * delta).normalized().toRotationMatrix();

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

std::string formatTargetError(const ArmTargetError & error)
{
  std::ostringstream line;
  line << "error " << armSideName(error.side)
       << ": position=" << std::fixed << std::setprecision(6)
       << error.position_m << " m  orientation=" << error.orientation_rad << " rad";
  return line.str();
}

std::string formatStep(
  double step_m,
  std::size_t rotation_axis_index,
  double rotation_step_deg)
{
  std::ostringstream line;
  line << "step=" << std::fixed << std::setprecision(4) << step_m
       << " m  rot_axis=tcp_" << selectedRotationAxis(rotation_axis_index)
       << "  rot_step=" << std::setprecision(2) << rotation_step_deg << " deg";
  return line.str();
}

std::string formatIk(const IkDebugFrame & frame)
{
  std::ostringstream line;
  line << "IK: " << frame.ik_status
       << "  iterations=" << frame.iterations
       << "  converged=" << std::boolalpha << frame.converged
       << "  solve_ms=" << std::fixed << std::setprecision(3) << frame.solve_time_ms;
  return line.str();
}

std::string jointPositionText(
  const JointNames & joint_names,
  const std::vector<double> & positions,
  const std::string & prefix)
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
  const InteractiveIkPresentation & presentation,
  ArmSide side,
  const JointNames & joint_names,
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
  const SelfCollisionPairDebug & pair,
  const SelfCollisionDebug & collision)
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

ftxui::Element modeBadge(bool paused)
{
  using namespace ftxui;
  return text(paused ? " PAUSED " : " PUBLISHING ") |
         bold |
         color(paused ? Color::Yellow : Color::Green);
}

ftxui::Element collisionPairElement(
  const SelfCollisionPairDebug & pair,
  const SelfCollisionDebug & collision)
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

std::string formatScientific(double value)
{
  std::ostringstream output;
  output << std::scientific << std::setprecision(3) << value;
  return output.str();
}

std::string formatPosition(const Pose & pose)
{
  std::ostringstream output;
  output << std::showpos << std::fixed << std::setprecision(4)
         << pose.translation().x() << ' ' << pose.translation().y() << ' '
         << pose.translation().z();
  return output.str();
}

std::string formatQuaternion(const Pose & pose)
{
  const Eigen::Quaterniond quaternion(pose.linear());
  std::ostringstream output;
  output << std::showpos << std::fixed << std::setprecision(4)
         << quaternion.x() << ' ' << quaternion.y() << ' '
         << quaternion.z() << ' ' << quaternion.w();
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

class TuiTeleopSource::Impl
{
public:
  explicit Impl(TuiTeleopSource & source)
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
    const IkDebugFrame & frame,
    std::size_t publish_count,
    const std::string & sink_status)
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

private:
  ftxui::Element renderHeader() const
  {
    using namespace ftxui;
    std::ostringstream rate;
    rate << "loop=" << static_cast<int>(source_.rate_hz_)
         << " Hz  publish_seq=" << publish_count_;
    return vbox({
      hbox({text(source_.title_) | bold, filler(), text(sink_status_) | dim}),
      hbox({
        text(std::string{"side="} + armSideName(source_.command_.selected_side) + "  "),
        modeBadge(source_.command_.paused),
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
      }) | border;
    }

    auto body = renderPage() |
      focusPositionRelative(0.0F, scroll_y_) |
      vscroll_indicator |
      yframe |
      flex;
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
    }) | border;
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
        renderJointsPanel(true),
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
          renderJointsPanel(true) | flex,
          separator(),
          renderRuntimeSummaryPanel() | flex,
        }),
      });
    }
    return vbox({
      renderCartesianPanel(true),
      renderSolverSummaryPanel(),
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
          rows.push_back({
            armSideName(arm.side), "target", formatPosition(target->target_pose),
            formatQuaternion(target->target_pose)});
        }
      }
      if (const auto * fk = findForwardKinematics(frame_.forward_kinematics, arm.side)) {
        if (compact) {
          content.push_back(labelledParagraph(
            std::string{armSideName(arm.side)} + " FK p: ", formatPosition(fk->pose)));
          content.push_back(labelledParagraph(
            std::string{armSideName(arm.side)} + " FK q: ", formatQuaternion(fk->pose)));
        } else {
          rows.push_back({
            armSideName(arm.side), "FK", formatPosition(fk->pose), formatQuaternion(fk->pose)});
        }
      }
      if (const auto * error = findError(frame_.target_errors, arm.side)) {
        std::ostringstream line;
        line << armSideName(arm.side) << " error: position="
             << formatScientific(error->position_m) << " m  orientation="
             << formatScientific(error->orientation_rad) << " rad";
        content.push_back(text(line.str()));
      }
    }
    if (!compact) {
      content.insert(content.begin() + 2, renderTable(std::move(rows)));
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
      {"solver", "result", "termination", "QP", "IK/QP ms", "hard max"}};
    for (const auto & solver : frame_.solvers) {
      rows.push_back({
        solver.label,
        solver.disposition,
        solver.termination_reason,
        solver.qp_status,
        formatFixed(solver.ik_solve_time_ms, 3) + "/" +
          formatFixed(solver.qp_solve_time_ms, 3),
        formatScientific(solver.maximum_hard_violation)});
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
        {"worker", "Hz", "iterations", "miss", "consecutive", "skipped", "max solver ms"}};
      for (const auto & worker : frame_.workers) {
        rows.push_back({
          worker.label,
          formatFixed(worker.configured_rate_hz, 0),
          std::to_string(worker.iteration_count),
          std::to_string(worker.deadline_miss_count),
          std::to_string(worker.consecutive_deadline_misses),
          std::to_string(worker.skipped_release_count),
          formatFixed(worker.maximum_solver_ms, 3)});
      }
      content.push_back(renderTable(std::move(rows)));
    }
    for (const auto & collision : frame_.self_collisions) {
      std::ostringstream line;
      line << collision.label << ": seq=" << collision.input_state_sequence
           << " min=" << formatFixed(collision.minimum_distance_before_m, 4)
           << " -> " << formatFixed(collision.minimum_distance_after_m, 4)
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
        {"IK solve ms", formatFixed(solver.ik_solve_time_ms, 6)},
        {"QP backend/status", solver.backend + "/" + solver.qp_status},
        {"QP native status", solver.native_status.empty() ? "-" : solver.native_status},
        {"QP iterations", std::to_string(solver.qp_iterations)},
        {"QP solve ms", formatFixed(solver.qp_solve_time_ms, 6)},
        {"objective", formatScientific(solver.objective_value)},
        {"primal residual", formatScientific(solver.primal_residual)},
        {"dual residual", formatScientific(solver.dual_residual)},
        {"maximum hard violation", formatScientific(solver.maximum_hard_violation)},
        {"active set size", std::to_string(solver.active_set_size)},
        {"warm start", solver.warm_start_used ? "true" : "false"},
        {"saturated joints", joinStrings(solver.saturated_joints)},
      };
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
            {"source value revision",
             std::to_string(attempt.consumed_source_value_revision)},
            {"captured state sequence", std::to_string(attempt.captured_state_sequence)},
            {"captured state time [ns]",
             std::to_string(attempt.captured_state_time_nanoseconds)},
            {"run generation", std::to_string(attempt.run_generation)},
          }));
        } else {
          solver_content.push_back(renderTable({
            {"attempt", "value", "accepted", "coupling", "source", "state", "run"},
            {std::to_string(attempt.attempt_revision),
             std::to_string(attempt.value_revision),
             attempt.attempt_accepted ? "true" : "false",
             attempt.coupling_state,
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
                 << " cost=" << formatScientific(scale.cost)
                 << " degraded=" << scale.degraded << " stuck=" << scale.stuck;
            solver_content.push_back(paragraph(line.str()));
          }
        } else {
          std::vector<std::vector<std::string>> rows = {
            {"task scale", "active", "scale", "cost", "degraded", "stuck"}};
          for (const auto & scale : solver.task_scales) {
            rows.push_back({
              scale.name,
              scale.active ? "true" : "false",
              formatFixed(scale.scale, 6),
              formatScientific(scale.cost),
              scale.degraded ? "true" : "false",
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
                 << " cost=" << formatScientific(requirement.cost)
                 << " unit=" << requirement.unit << " source=" << requirement.source;
            solver_content.push_back(paragraph(line.str()));
          }
        } else {
          std::vector<std::vector<std::string>> rows = {
            {"requirement", "enabled", "active", "violation", "cost", "unit", "source"}};
          for (const auto & requirement : solver.requirements) {
            rows.push_back({
              requirement.name,
              requirement.enabled ? "true" : "false",
              requirement.active ? "true" : "false",
              formatScientific(requirement.maximum_violation),
              formatScientific(requirement.cost),
              requirement.unit,
              requirement.source});
          }
          solver_content.push_back(renderTable(std::move(rows)));
        }
      }
      content.push_back(debugWindow(solver.label, vbox(std::move(solver_content))));
    }
    return vbox(std::move(content));
  }

  ftxui::Element renderJointsPanel(bool selected_only) const
  {
    using namespace ftxui;
    std::vector<std::vector<std::string>> rows = {{"joint", "q", "dq", "saturated"}};
    std::vector<std::size_t> indices;
    if (selected_only) {
      if (const auto * arm = findArmPresentation(
          source_.presentation_, source_.command_.selected_side))
      {
        indices = arm->joint_indices;
      }
    } else {
      indices.resize(frame_.positions.size());
      for (std::size_t index = 0; index < indices.size(); ++index) {
        indices[index] = index;
      }
    }
    std::vector<std::string> saturated;
    for (const auto & solver : frame_.solvers) {
      saturated.insert(
        saturated.end(), solver.saturated_joints.begin(), solver.saturated_joints.end());
    }
    for (const auto index : indices) {
      if (index >= frame_.positions.size()) {
        continue;
      }
      const std::string name = index < frame_.joint_names.size()
        ? frame_.joint_names[index]
        : "joint_" + std::to_string(index);
      const bool is_saturated =
        std::find(saturated.begin(), saturated.end(), name) != saturated.end();
      rows.push_back({
        name,
        formatFixed(frame_.positions[index], 6),
        index < frame_.velocities.size() ? formatFixed(frame_.velocities[index], 6) : "-",
        is_saturated ? "YES" : ""});
    }
    return debugWindow(
      selected_only
      ? std::string{armSideName(source_.command_.selected_side)} + " arm state"
      : "All joint state",
      renderTable(std::move(rows)));
  }

  ftxui::Element renderJointsPage() const
  {
    return renderJointsPanel(false);
  }

  ftxui::Element renderRuntimePage() const
  {
    using namespace ftxui;
    Elements content;
    if (!frame_.workers.empty()) {
      if (Terminal::Size().dimx < 120) {
        Elements workers;
        for (const auto & worker : frame_.workers) {
          workers.push_back(debugWindow(worker.label, renderTable({
            {"metric", "value"},
            {"configured rate [Hz]", formatFixed(worker.configured_rate_hz, 0)},
            {"iterations", std::to_string(worker.iteration_count)},
            {"deadline misses", std::to_string(worker.deadline_miss_count)},
            {"consecutive misses", std::to_string(worker.consecutive_deadline_misses)},
            {"skipped releases", std::to_string(worker.skipped_release_count)},
            {"maximum release lateness [ms]",
             formatFixed(worker.maximum_release_lateness_ms, 3)},
            {"maximum execution [ms]", formatFixed(worker.maximum_execution_ms, 3)},
            {"maximum release-to-finish [ms]",
             formatFixed(worker.maximum_release_to_finish_ms, 3)},
            {"maximum overrun [ms]", formatFixed(worker.maximum_overrun_ms, 3)},
            {"maximum solver [ms]", formatFixed(worker.maximum_solver_ms, 3)},
          })));
        }
        content.push_back(debugWindow("Periodic workers", vbox(std::move(workers))));
      } else {
        std::vector<std::vector<std::string>> rows = {
          {"worker", "Hz", "iterations", "miss", "consecutive", "skipped",
           "late max", "exec max", "finish max", "overrun max", "solver max"}};
        for (const auto & worker : frame_.workers) {
          rows.push_back({
            worker.label,
            formatFixed(worker.configured_rate_hz, 0),
            std::to_string(worker.iteration_count),
            std::to_string(worker.deadline_miss_count),
            std::to_string(worker.consecutive_deadline_misses),
            std::to_string(worker.skipped_release_count),
            formatFixed(worker.maximum_release_lateness_ms, 3),
            formatFixed(worker.maximum_execution_ms, 3),
            formatFixed(worker.maximum_release_to_finish_ms, 3),
            formatFixed(worker.maximum_overrun_ms, 3),
            formatFixed(worker.maximum_solver_ms, 3)});
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
      collision_content.push_back(paragraph(jointPositionText(
        frame_.joint_names, collision.input_joint_positions, "input q: ")) | dim);
      content.push_back(debugWindow(collision.label, vbox(std::move(collision_content))));
    }
    if (content.empty()) {
      content.push_back(text("No multi-rate worker or collision diagnostics") | dim | center);
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
        text("1..5 / F1..F5 / Tab: switch Overview, Solver/QP, Joints, Runtime, Events"),
        text("PageUp/PageDown/Home/End: scroll current page"),
        separator(),
        text("w/s: +x/-x    a/d: +y/-y    q/e: +z/-z"),
        text("n: cycle TCP rotation axis    i/u: rotate clockwise/counter-clockwise"),
        text(source_.allow_side_switching_
          ? "LEFT/RIGHT: select arm    UP/DOWN: step x2 / step /2"
          : "UP/DOWN: step x2 / step /2"),
        text("m: manual step    r: reset from FK    space: pause"),
        text("h or Esc: close help    x: exit"),
      })) | clear_under | center;
  }

  void appendEvent(
    std::size_t publish_sequence,
    std::string source,
    std::string message)
  {
    events_.push_back(DebugEvent{
      publish_sequence, std::move(source), std::move(message)});
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
      std::string state = solver.disposition + " / " + solver.termination_reason +
        " / " + solver.qp_status;
      if (!solver.native_status.empty()) {
        state += " / " + solver.native_status;
      }
      solver_states.push_back(std::move(state));
    }
    for (std::size_t index = 0; index < solver_states.size(); ++index) {
      if (index >= last_solver_states_.size() ||
          solver_states[index] != last_solver_states_[index])
      {
        const std::string label = index < frame.solvers.size()
          ? frame.solvers[index].label
          : "IK";
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
    range << "Allowed range: [" << std::fixed << std::setprecision(4)
          << source_.options_.min_step_m << ", " << source_.options_.max_step_m << "] m";
    return window(
      text(" Manual translation step ") | bold,
      vbox({
        text(range.str()),
        hbox({text("value: "), step_input_->Render() | flex}),
        separatorEmpty(),
        text("Enter: apply    Esc: cancel") | dim,
      })) | clear_under | center;
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
      } else if (event == Event::x || event == Event::X) {
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
    if (event == Event::F1 || event == Event::F2 || event == Event::F3 ||
        event == Event::F4 || event == Event::F5)
    {
      selectPage(
        event == Event::F1 ? 0 : event == Event::F2 ? 1 : event == Event::F3 ? 2 :
        event == Event::F4 ? 3 : 4);
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

    const char key = static_cast<char>(std::tolower(
      static_cast<unsigned char>(event.character().front())));
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
        source_.rotation_axis_index_ =
          (source_.rotation_axis_index_ + 1) % kRotationAxes.size();
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
      setStep(
        source_.command_, source_.step_m_, source_.options_,
        std::stod(step_input_value_));
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

  TuiTeleopSource & source_;
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
  impl_ = std::make_unique<Impl>(*this);
}

TuiTeleopSource::~TuiTeleopSource() = default;

void TuiTeleopSource::poll()
{
  impl_->poll();
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
  impl_->render(frame, publish_count, sink_status);
}

}  // namespace motion_control_lab
