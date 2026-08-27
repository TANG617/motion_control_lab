#include "components/tui/planned_grouped_tui.hpp"

#include "components/tui/standard_ik_tui.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
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

std::vector<std::string> noneRow(std::size_t width)
{
  std::vector<std::string> row(width, "-");
  row.front() = "none";
  return row;
}

double utilization(double value, double limit)
{
  return limit > 0.0 ? 100.0 * std::abs(value) / limit : 0.0;
}

double positionMargin(const JointChainTuiDebug & joint)
{
  return std::min(
    joint.execution_position - joint.position_lower,
    joint.position_upper - joint.execution_position);
}

TuiPage & page(TuiDocument & document, const std::string & title)
{
  const auto result = std::find_if(
    document.pages.begin(), document.pages.end(),
    [&title](const TuiPage & candidate) { return candidate.title == title; });
  if (result == document.pages.end()) {
    throw std::logic_error("standard IK TUI is missing page: " + title);
  }
  return *result;
}

TuiSection & section(TuiPage & source, const std::string & title)
{
  const auto result = std::find_if(
    source.sections.begin(), source.sections.end(),
    [&title](const TuiSection & candidate) { return candidate.title == title; });
  if (result == source.sections.end()) {
    throw std::logic_error("standard IK TUI is missing section: " + source.title + "/" + title);
  }
  return *result;
}

struct JointGroupRisk
{
  double velocity_percent{0.0};
  double acceleration_percent{0.0};
  double jerk_percent{0.0};
  double position_margin{std::numeric_limits<double>::infinity()};
};

JointGroupRisk jointGroupRisk(
  const std::vector<JointChainTuiDebug> & joints,
  std::size_t begin,
  std::size_t end)
{
  JointGroupRisk result;
  for (std::size_t index = begin; index < std::min(end, joints.size()); ++index) {
    const auto & joint = joints[index];
    result.velocity_percent = std::max(
      result.velocity_percent, utilization(joint.execution_velocity, joint.maximum_velocity));
    result.acceleration_percent = std::max(
      result.acceleration_percent,
      utilization(joint.execution_acceleration, joint.maximum_acceleration));
    result.jerk_percent = std::max(
      result.jerk_percent, utilization(joint.execution_jerk, joint.maximum_jerk));
    result.position_margin = std::min(result.position_margin, positionMargin(joint));
  }
  if (!std::isfinite(result.position_margin)) {
    result.position_margin = 0.0;
  }
  return result;
}

void enrichMonitor(TuiPage & monitor, const PlannedGroupedJointOtgTuiDebug & app)
{
  auto & safety = section(monitor, "Safety and hold").tables.front().rows;
  safety.push_back({"Task scale L/R", fixed(app.left_task_scale, 3) + " / " +
                                          fixed(app.right_task_scale, 3)});
  safety.push_back({"Projection cycle / total",
                    std::to_string(app.projection_events.size()) + " / " +
                      std::to_string(app.projection_event_count)});
  safety.push_back({"Retarget clamps", std::to_string(app.clamp_events.size())});
  safety.push_back({"Clamp max ratio", fixed(app.maximum_clamp_limit_ratio, 3)});
  safety.push_back({"Motion max |v| / |a| / |j|",
                    fixed(app.maximum_absolute_velocity, 3) + " / " +
                      fixed(app.maximum_absolute_acceleration, 3) + " / " +
                      fixed(app.maximum_absolute_jerk, 2)});
}

void enrichMotion(TuiPage & motion, const PlannedGroupedJointOtgTuiDebug & app)
{
  motion.rows = {{{3, 2}, 2}, {{1}, 1}, {{1}, 3}, {{1}, 2}};
  auto & pipeline = section(motion, "Pipeline");
  pipeline.tables.front() = TuiTable{
    {textColumn("Stage"), textColumn("State"), numberColumn("Duration [s]"),
     numberColumn("dt [s]"), numberColumn("Samples"), numberColumn("Calc [ms]")},
    {{"Cartesian", app.cartesian_plan.state, fixed(app.cartesian_plan.duration_s, 6),
      fixed(app.cartesian_plan.sample_time_s, 6),
      std::to_string(app.cartesian_plan.sample_count),
      fixed(app.cartesian_plan.calculation_time_ms, 4)},
     {"Joint plan", app.joint_plan.state, fixed(app.joint_plan.duration_s, 6),
      fixed(app.joint_plan.sample_time_s, 6), std::to_string(app.joint_plan.sample_count),
      fixed(app.joint_plan.calculation_time_ms, 4)},
     {"Joint step", app.joint_step.state, fixed(app.joint_step.duration_s, 6),
      fixed(app.joint_step.sample_time_s, 6), std::to_string(app.joint_step.sample_count),
      fixed(app.joint_step.calculation_time_ms, 4)}},
    TuiTableStyle::Compact};

  auto & error = section(motion, "Tracking error");
  error.column = 0U;
  error.row = 1U;
  auto & states = section(motion, "Cartesian states");
  states.row = 2U;
  auto & reference = section(motion, "Reference twist and acceleration");
  reference.row = 3U;
  motion.sections.push_back(sheet(
    "Limits",
    {textColumn("Domain"), numberColumn("Velocity"), numberColumn("Acceleration"),
     numberColumn("Jerk")},
    {{"Linear", fixed(app.cartesian_limits.linear_velocity),
      fixed(app.cartesian_limits.linear_acceleration), fixed(app.cartesian_limits.linear_jerk)},
     {"Angular", fixed(app.cartesian_limits.angular_velocity),
      fixed(app.cartesian_limits.angular_acceleration), fixed(app.cartesian_limits.angular_jerk)}},
    1U, 0U));
}

void replaceJointsPage(TuiPage & joints_page, const PlannedGroupedJointOtgTuiDebug & app)
{
  joints_page.sections.clear();
  joints_page.rows = {{{1, 1}, 1}, {{1}, 4}};
  const std::vector<std::pair<std::string, std::pair<std::size_t, std::size_t>>> groups{
    {"Head", {0U, 2U}}, {"Body", {2U, 6U}},
    {"Left arm", {6U, 13U}}, {"Right arm", {13U, 20U}}};
  std::vector<std::vector<std::string>> risk_rows;
  for (const auto & group : groups) {
    const auto risk = jointGroupRisk(app.joints, group.second.first, group.second.second);
    risk_rows.push_back(
      {group.first, fixed(risk.position_margin, 4), fixed(risk.velocity_percent, 1),
       fixed(risk.acceleration_percent, 1), fixed(risk.jerk_percent, 1)});
  }
  joints_page.sections.push_back(sheet(
    "Group risk",
    {textColumn("Group"), numberColumn("q margin [rad]"), numberColumn("V [%]"),
     numberColumn("A [%]"), numberColumn("J [%]")},
    std::move(risk_rows), 0U, 0U));
  joints_page.sections.push_back(details(
    "Target pipeline",
    {{"Target mode", app.target_mode}, {"Feedback", app.feedback_topology},
     {"Raw target Δq max", fixed(app.raw_otg_max_position_delta, 5)},
     {"Raw target Δdq max", fixed(app.raw_otg_max_velocity_delta, 5)},
     {"Modified joints", std::to_string(app.modified_joint_count)},
     {"Projection cycle / total", std::to_string(app.projection_events.size()) + " / " +
                                      std::to_string(app.projection_event_count)}},
    1U, 0U));

  std::vector<std::vector<std::string>> chain_rows;
  for (const auto & joint : app.joints) {
    chain_rows.push_back(
      {joint.name, fixed(joint.ik_position, 5), fixed(joint.target_position, 5),
       fixed(joint.execution_position, 5), fixed(joint.target_velocity, 5),
       fixed(joint.execution_velocity, 5), fixed(joint.execution_acceleration, 4),
       fixed(joint.execution_jerk, 2),
       "[" + fixed(joint.position_lower, 3) + ", " + fixed(joint.position_upper, 3) + "]",
       fixed(positionMargin(joint), 4),
       fixed(utilization(joint.execution_velocity, joint.maximum_velocity), 1),
       fixed(utilization(joint.execution_acceleration, joint.maximum_acceleration), 1),
       fixed(utilization(joint.execution_jerk, joint.maximum_jerk), 1), joint.projection});
  }
  if (chain_rows.empty()) {
    chain_rows.push_back(noneRow(14U));
  }
  joints_page.sections.push_back(sheet(
    "Joint chain · Raw IK → Target → Executed",
    {textColumn("Joint", 18), numberColumn("Raw q"), numberColumn("Target q"),
     numberColumn("Exec q"), numberColumn("Target dq"), numberColumn("Exec dq"),
     numberColumn("Exec ddq"), numberColumn("Exec jerk"), textColumn("q limits", 18),
     numberColumn("q margin"), numberColumn("V%"), numberColumn("A%"),
     numberColumn("J%"), textColumn("Projection", 10)},
    std::move(chain_rows), 0U, 1U));
}

void enrichSystem(TuiPage & system, const PlannedGroupedJointOtgTuiDebug & app)
{
  system.sections.push_back(sheet(
    "Planner runtime",
    {textColumn("Stage"), textColumn("State"), numberColumn("Duration [s]"),
     numberColumn("Samples"), numberColumn("Calc [ms]")},
    {{"Cartesian", app.cartesian_plan.state, fixed(app.cartesian_plan.duration_s, 6),
      std::to_string(app.cartesian_plan.sample_count), fixed(app.cartesian_plan.calculation_time_ms, 4)},
     {"Joint plan", app.joint_plan.state, fixed(app.joint_plan.duration_s, 6),
      std::to_string(app.joint_plan.sample_count), fixed(app.joint_plan.calculation_time_ms, 4)},
     {"Joint step", app.joint_step.state, fixed(app.joint_step.duration_s, 6),
      std::to_string(app.joint_step.sample_count), fixed(app.joint_step.calculation_time_ms, 4)}}));

  std::vector<std::vector<std::string>> projection_rows;
  for (const auto & event : app.projection_events) {
    projection_rows.push_back(
      {event.joint, event.component, fixed(event.original_value, 6),
       fixed(event.applied_value, 6), fixed(event.limit, 6)});
  }
  if (projection_rows.empty()) {
    projection_rows.push_back(noneRow(5U));
  }
  system.sections.push_back(sheet(
    "Projection events",
    {textColumn("Joint"), textColumn("Component"), numberColumn("Original", 9),
     numberColumn("Applied", 9), numberColumn("Limit", 9)},
    std::move(projection_rows)));

  std::vector<std::vector<std::string>> clamp_rows;
  for (const auto & event : app.clamp_events) {
    clamp_rows.push_back(
      {event.arm, event.component, event.axis, fixed(event.original_value, 6),
       fixed(event.applied_value, 6), fixed(event.limit, 6)});
  }
  if (clamp_rows.empty()) {
    clamp_rows.push_back(noneRow(6U));
  }
  system.sections.push_back(sheet(
    "Retarget clamps",
    {textColumn("Arm"), textColumn("Component"), textColumn("Axis"),
     numberColumn("Original", 9), numberColumn("Applied", 9), numberColumn("Limit", 9)},
    std::move(clamp_rows)));
}

}  // namespace

TuiDocument makePlannedGroupedTuiDocument(const PlannedGroupedTuiSnapshot & snapshot)
{
  if (snapshot.frame == nullptr || snapshot.presentation == nullptr) {
    throw std::logic_error("planned-grouped TUI snapshot is missing required presentation data");
  }
  TuiDocument document = makeStandardIkTuiDocument(
    *snapshot.frame, *snapshot.presentation, snapshot.publish_count, snapshot.sink_status,
    snapshot.title, snapshot.input_status);
  if (snapshot.joint_otg.has_value()) {
    const auto & app = *snapshot.joint_otg;
    enrichMonitor(page(document, "Monitor"), app);
    enrichMotion(page(document, "Motion"), app);
    replaceJointsPage(page(document, "Joints"), app);
    enrichSystem(page(document, "System"), app);
    document.header_left += " · source " + app.source_mode;
  }

  document.pages.insert(document.pages.end(), snapshot.extra_pages.begin(), snapshot.extra_pages.end());
  if (!snapshot.header_context.empty()) {
    document.header_left += " · " + snapshot.header_context;
  }
  const std::string page_count = std::to_string(document.pages.size());
  if (snapshot.footer_hints.has_value()) {
    document.footer_hints = *snapshot.footer_hints;
  } else if (snapshot.joint_otg.has_value()) {
    document.footer_hints = snapshot.joint_otg->source_mode.find("replay") != std::string::npos
                              ? "Space pause · . step · 1–" + page_count +
                                  " pages · ? help · x exit"
                              : "Space pause · 1–" + page_count + " pages · ? help · x exit";
  }
  document.help_lines.front() =
    "1.." + page_count +
    (document.pages.size() <= 7U ? "/F1..F" + page_count : "/F1..F7") +
    "/Tab/BackTab: pages; PageUp/PageDown/Home/End: scroll; h/?: help";
  if (!snapshot.help_lines.empty()) {
    document.help_lines.resize(1U);
    document.help_lines.insert(
      document.help_lines.end(), snapshot.help_lines.begin(), snapshot.help_lines.end());
  }
  return document;
}

PlannedGroupedTui::PlannedGroupedTui(PlannedGroupedTuiConfig config)
: renderer_(config.enabled)
{
}

void PlannedGroupedTui::handleNavigation(const KeyEvent & event)
{
  renderer_.handleNavigation(event);
}

void PlannedGroupedTui::render(const PlannedGroupedTuiSnapshot & snapshot)
{
  renderer_.render(makePlannedGroupedTuiDocument(snapshot));
}

}  // namespace motion_control_lab
