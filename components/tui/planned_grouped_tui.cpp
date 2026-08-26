#include "components/tui/planned_grouped_tui.hpp"

#include "components/tui/standard_ik_tui.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace motion_control_lab {
namespace {

using Alignment = TuiTableAlignment;

std::string fixed(double value, int precision = 3) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(precision) << value;
  return output.str();
}

std::string yesNo(bool value) { return value ? "yes" : "no"; }

TuiTableColumn textColumn(std::string title) {
  return TuiTableColumn{std::move(title), Alignment::Left};
}

TuiTableColumn numberColumn(std::string title) {
  return TuiTableColumn{std::move(title), Alignment::Right};
}

TuiSection sheet(std::string title, std::vector<TuiTableColumn> columns,
                 std::vector<std::vector<std::string>> rows,
                 std::size_t column = 0U, std::size_t row = 0U) {
  TuiSection section;
  section.title = std::move(title);
  section.tables.push_back(
      TuiTable{std::move(columns), std::move(rows), TuiTableStyle::Compact});
  section.column = column;
  section.row = row;
  section.style = TuiSectionStyle::Panel;
  return section;
}

std::vector<std::string> noneRow(std::size_t width) {
  std::vector<std::string> row(width, "-");
  row.front() = "none";
  return row;
}

double utilization(double value, double limit) {
  return limit > 0.0 ? 100.0 * std::abs(value) / limit : 0.0;
}

TuiPage &page(TuiDocument &document, const std::string &title) {
  const auto result = std::find_if(
      document.pages.begin(), document.pages.end(),
      [&title](const TuiPage &candidate) { return candidate.title == title; });
  if (result == document.pages.end()) {
    throw std::logic_error("standard IK TUI is missing page: " + title);
  }
  return *result;
}

TuiSection &section(TuiPage &source, const std::string &title) {
  const auto result =
      std::find_if(source.sections.begin(), source.sections.end(),
                   [&title](const TuiSection &candidate) {
                     return candidate.title == title;
                   });
  if (result == source.sections.end()) {
    throw std::logic_error(
        "standard IK TUI is missing section: " + source.title + "/" + title);
  }
  return *result;
}

struct JointGroupMaximum {
  double velocity{0.0};
  double acceleration{0.0};
  double jerk{0.0};
  double velocity_utilization{0.0};
  double acceleration_utilization{0.0};
  double jerk_utilization{0.0};
};

JointGroupMaximum
jointGroupMaximum(const std::vector<JointChainTuiDebug> &joints,
                  std::size_t begin, std::size_t end) {
  JointGroupMaximum result;
  for (std::size_t index = begin; index < std::min(end, joints.size());
       ++index) {
    const auto &joint = joints[index];
    result.velocity =
        std::max(result.velocity, std::abs(joint.execution_velocity));
    result.acceleration =
        std::max(result.acceleration, std::abs(joint.execution_acceleration));
    result.jerk = std::max(result.jerk, std::abs(joint.execution_jerk));
    result.velocity_utilization =
        std::max(result.velocity_utilization,
                 utilization(joint.execution_velocity, joint.maximum_velocity));
    result.acceleration_utilization = std::max(
        result.acceleration_utilization,
        utilization(joint.execution_acceleration, joint.maximum_acceleration));
    result.jerk_utilization =
        std::max(result.jerk_utilization,
                 utilization(joint.execution_jerk, joint.maximum_jerk));
  }
  return result;
}

TuiPage makeJointPlanningPage(const PlannedGroupedJointOtgTuiDebug &app) {
  TuiPage page;
  page.title = "Joint Planning";
  page.sections.push_back(sheet(
      "Summary", {textColumn("Metric"), textColumn("Value")},
      {{"Target mode", app.target_mode},
       {"Feedback topology", app.feedback_topology},
       {"Startup", yesNo(app.startup)},
       {"Projection events this cycle",
        std::to_string(app.projection_events.size())},
       {"Projection events total", std::to_string(app.projection_event_count)},
       {"Projection cycles", std::to_string(app.projection_cycle_count)},
       {"Modified joints", std::to_string(app.modified_joint_count)}}));
  page.sections.push_back(sheet(
      "Maximum target and execution motion",
      {textColumn("Metric"), numberColumn("Value")},
      {{"Raw target position delta", fixed(app.raw_otg_max_position_delta, 5)},
       {"Raw target velocity delta", fixed(app.raw_otg_max_velocity_delta, 5)},
       {"Absolute velocity", fixed(app.maximum_absolute_velocity, 5)},
       {"Absolute acceleration", fixed(app.maximum_absolute_acceleration, 5)},
       {"Absolute jerk", fixed(app.maximum_absolute_jerk, 3)}}));
  page.sections.push_back(sheet(
      "Planner calls",
      {textColumn("Call"), textColumn("State"), numberColumn("Duration [s]"),
       numberColumn("Sample time [s]"), numberColumn("Samples"),
       numberColumn("Calculation [ms]")},
      {{"Plan", app.joint_plan.state, fixed(app.joint_plan.duration_s, 6),
        fixed(app.joint_plan.sample_time_s, 6),
        std::to_string(app.joint_plan.sample_count),
        fixed(app.joint_plan.calculation_time_ms, 4)},
       {"First step", app.joint_step.state, fixed(app.joint_step.duration_s, 6),
        fixed(app.joint_step.sample_time_s, 6),
        std::to_string(app.joint_step.sample_count),
        fixed(app.joint_step.calculation_time_ms, 4)}}));

  std::vector<std::vector<std::string>> target_rows;
  std::vector<std::vector<std::string>> execution_rows;
  for (const auto &joint : app.joints) {
    target_rows.push_back(
        {joint.name, fixed(joint.ik_position, 5), fixed(joint.ik_velocity, 5),
         fixed(joint.target_position, 5), fixed(joint.target_velocity, 5),
         fixed(joint.target_acceleration, 4), joint.projection});
    execution_rows.push_back(
        {joint.name, fixed(joint.execution_position, 5),
         fixed(joint.execution_velocity, 5),
         fixed(joint.execution_acceleration, 4), fixed(joint.execution_jerk, 2),
         fixed(utilization(joint.execution_velocity, joint.maximum_velocity),
               1),
         fixed(utilization(joint.execution_acceleration,
                           joint.maximum_acceleration),
               1),
         fixed(utilization(joint.execution_jerk, joint.maximum_jerk), 1)});
  }
  if (target_rows.empty()) {
    target_rows.push_back(noneRow(7U));
    execution_rows.push_back(noneRow(8U));
  }
  page.sections.push_back(
      sheet("IK and projected target",
            {textColumn("Joint name"), numberColumn("IK position [rad]"),
             numberColumn("IK velocity [rad/s]"),
             numberColumn("Target position [rad]"),
             numberColumn("Target velocity [rad/s]"),
             numberColumn("Target acceleration [rad/s\xC2\xB2]"),
             textColumn("Projection")},
            std::move(target_rows)));
  page.sections.push_back(
      sheet("Execution and limit utilization",
            {textColumn("Joint name"), numberColumn("Position [rad]"),
             numberColumn("Velocity [rad/s]"),
             numberColumn("Acceleration [rad/s\xC2\xB2]"),
             numberColumn("Jerk [rad/s\xC2\xB3]"),
             numberColumn("Velocity utilization [%]"),
             numberColumn("Acceleration utilization [%]"),
             numberColumn("Jerk utilization [%]")},
            std::move(execution_rows)));
  return page;
}

void replaceJointStatePage(TuiPage &joint_page,
                           const PlannedGroupedJointOtgTuiDebug &app) {
  joint_page.sections.clear();
  joint_page.rows = {{{1, 1}, 1}, {{1}, 3}, {{1}, 3}};
  const std::vector<std::pair<std::string, std::pair<std::size_t, std::size_t>>>
      groups{{"Head", {0U, 2U}},
             {"Body", {2U, 6U}},
             {"Left arm", {6U, 13U}},
             {"Right arm", {13U, 20U}}};
  std::vector<std::vector<std::string>> maximum_rows;
  std::vector<std::vector<std::string>> utilization_rows;
  for (const auto &group : groups) {
    const auto maximum =
        jointGroupMaximum(app.joints, group.second.first, group.second.second);
    maximum_rows.push_back({group.first, fixed(maximum.velocity, 4),
                            fixed(maximum.acceleration, 4),
                            fixed(maximum.jerk, 2)});
    utilization_rows.push_back({group.first,
                                fixed(maximum.velocity_utilization, 1),
                                fixed(maximum.acceleration_utilization, 1),
                                fixed(maximum.jerk_utilization, 1)});
  }
  joint_page.sections.push_back(
      sheet("Group maximum motion",
            {textColumn("Group"), numberColumn("Velocity [rad/s]"),
             numberColumn("Acceleration [rad/s\xC2\xB2]"),
             numberColumn("Jerk [rad/s\xC2\xB3]")},
            std::move(maximum_rows), 0U, 0U));
  joint_page.sections.push_back(
      sheet("Group maximum limit utilization",
            {textColumn("Group"), numberColumn("Velocity [%]"),
             numberColumn("Acceleration [%]"), numberColumn("Jerk [%]")},
            std::move(utilization_rows), 1U, 0U));

  std::vector<std::vector<std::string>> execution_rows;
  std::vector<std::vector<std::string>> limit_rows;
  for (const auto &joint : app.joints) {
    execution_rows.push_back({joint.name, fixed(joint.execution_position, 5),
                              fixed(joint.execution_velocity, 5),
                              fixed(joint.execution_acceleration, 4),
                              fixed(joint.execution_jerk, 2)});
    limit_rows.push_back(
        {joint.name, fixed(joint.position_lower, 4),
         fixed(joint.position_upper, 4),
         fixed(utilization(joint.execution_velocity, joint.maximum_velocity),
               1),
         fixed(utilization(joint.execution_acceleration,
                           joint.maximum_acceleration),
               1),
         fixed(utilization(joint.execution_jerk, joint.maximum_jerk), 1)});
  }
  if (execution_rows.empty()) {
    execution_rows.push_back(noneRow(5U));
    limit_rows.push_back(noneRow(6U));
  }
  joint_page.sections.push_back(
      sheet("Executed joint state",
            {textColumn("Joint name"), numberColumn("Position [rad]"),
             numberColumn("Velocity [rad/s]"),
             numberColumn("Acceleration [rad/s\xC2\xB2]"),
             numberColumn("Jerk [rad/s\xC2\xB3]")},
            std::move(execution_rows), 0U, 1U));
  joint_page.sections.push_back(
      sheet("Position limits and utilization",
            {textColumn("Joint name"), numberColumn("Lower position [rad]"),
             numberColumn("Upper position [rad]"),
             numberColumn("Velocity utilization [%]"),
             numberColumn("Acceleration utilization [%]"),
             numberColumn("Jerk utilization [%]")},
            std::move(limit_rows), 0U, 2U));
}

} // namespace

TuiDocument
makePlannedGroupedTuiDocument(const PlannedGroupedTuiSnapshot &snapshot) {
  if (snapshot.frame == nullptr || snapshot.presentation == nullptr) {
    throw std::logic_error(
        "planned-grouped TUI snapshot is missing required presentation data");
  }
  TuiDocument document = makeStandardIkTuiDocument(
      *snapshot.frame, *snapshot.presentation, snapshot.publish_count,
      snapshot.sink_status, snapshot.title, snapshot.input_status);
  if (snapshot.joint_otg.has_value()) {
    const auto &app = *snapshot.joint_otg;
    document.header_right = "source=" + app.source_mode +
                            "  Cartesian planning=" + app.cartesian_plan.state +
                            "  Joint execution=" + app.joint_step.state;

    auto &overview = page(document, "Overview");
    overview.sections.push_back(sheet(
        "Safety", {textColumn("Safety metric"), textColumn("Value")},
        {{"Left task scale", fixed(app.left_task_scale, 3)},
         {"Right task scale", fixed(app.right_task_scale, 3)},
         {"Projection events this cycle",
          std::to_string(app.projection_events.size())},
         {"Projection events total",
          std::to_string(app.projection_event_count)},
         {"Retarget clamps this cycle",
          std::to_string(app.clamp_events.size())},
         {"Maximum clamp limit ratio", fixed(app.maximum_clamp_limit_ratio, 3)},
         {"Maximum absolute velocity", fixed(app.maximum_absolute_velocity, 3)},
         {"Maximum absolute acceleration",
          fixed(app.maximum_absolute_acceleration, 3)},
         {"Maximum absolute jerk", fixed(app.maximum_absolute_jerk, 2)}},
        1U, 0U));
    auto planning = sheet("Planning",
                          {textColumn("Planning stage"), textColumn("State"),
                           numberColumn("Calculation [ms]")},
                          {{"Cartesian", app.cartesian_plan.state,
                            fixed(app.cartesian_plan.calculation_time_ms, 4)},
                           {"Joint plan", app.joint_plan.state,
                            fixed(app.joint_plan.calculation_time_ms, 4)},
                           {"Joint first step", app.joint_step.state,
                            fixed(app.joint_step.calculation_time_ms, 4)}},
                          1U, 1U);
    planning.lines.push_back("Target mode: " + app.target_mode);
    planning.lines.push_back("Feedback topology: " + app.feedback_topology);
    overview.sections.push_back(std::move(planning));

    auto &cartesian_page = page(document, "Cartesian Planning");
    cartesian_page.sections.front() = sheet(
        "Planner",
        {textColumn("State"), numberColumn("Duration [s]"),
         numberColumn("Sample time [s]"), numberColumn("Samples"),
         numberColumn("Calculation [ms]")},
        {{app.cartesian_plan.state, fixed(app.cartesian_plan.duration_s, 6),
          fixed(app.cartesian_plan.sample_time_s, 6),
          std::to_string(app.cartesian_plan.sample_count),
          fixed(app.cartesian_plan.calculation_time_ms, 4)}});
    cartesian_page.sections.insert(
        cartesian_page.sections.begin() + 1,
        sheet("Configured limits",
              {textColumn("Metric"), numberColumn("Value"), textColumn("Unit")},
              {{"Linear velocity", fixed(app.cartesian_limits.linear_velocity),
                "m/s"},
               {"Linear acceleration",
                fixed(app.cartesian_limits.linear_acceleration), "m/s\xC2\xB2"},
               {"Linear jerk", fixed(app.cartesian_limits.linear_jerk),
                "m/s\xC2\xB3"},
               {"Angular velocity",
                fixed(app.cartesian_limits.angular_velocity), "rad/s"},
               {"Angular acceleration",
                fixed(app.cartesian_limits.angular_acceleration),
                "rad/s\xC2\xB2"},
               {"Angular jerk", fixed(app.cartesian_limits.angular_jerk),
                "rad/s\xC2\xB3"}}));

    const auto cartesian_position =
        std::find_if(document.pages.begin(), document.pages.end(),
                     [](const TuiPage &candidate) {
                       return candidate.title == "Cartesian Planning";
                     });
    document.pages.insert(cartesian_position + 1, makeJointPlanningPage(app));

    replaceJointStatePage(page(document, "Joint State"), app);

    auto &runtime = page(document, "Runtime");
    runtime.sections.push_back(
        sheet("Planner timing",
              {textColumn("Planner"), textColumn("State"),
               numberColumn("Duration [s]"), numberColumn("Sample time [s]"),
               numberColumn("Samples"), numberColumn("Calculation [ms]")},
              {{"Cartesian", app.cartesian_plan.state,
                fixed(app.cartesian_plan.duration_s, 6),
                fixed(app.cartesian_plan.sample_time_s, 6),
                std::to_string(app.cartesian_plan.sample_count),
                fixed(app.cartesian_plan.calculation_time_ms, 4)},
               {"Joint plan", app.joint_plan.state,
                fixed(app.joint_plan.duration_s, 6),
                fixed(app.joint_plan.sample_time_s, 6),
                std::to_string(app.joint_plan.sample_count),
                fixed(app.joint_plan.calculation_time_ms, 4)},
               {"Joint first step", app.joint_step.state,
                fixed(app.joint_step.duration_s, 6),
                fixed(app.joint_step.sample_time_s, 6),
                std::to_string(app.joint_step.sample_count),
                fixed(app.joint_step.calculation_time_ms, 4)}},
              1U));

    auto &events = page(document, "Events");
    auto &current_state = section(events, "Current state");
    current_state.tables.front().rows.push_back(
        {"Projection modified joints",
         std::to_string(app.modified_joint_count)});
    current_state.tables.front().rows.push_back(
        {"Clamp target revision", std::to_string(app.clamp_target_revision)});
    current_state.tables.front().rows.push_back(
        {"Maximum clamp limit ratio", fixed(app.maximum_clamp_limit_ratio, 3)});

    std::vector<std::vector<std::string>> projection_rows;
    for (const auto &event : app.projection_events) {
      projection_rows.push_back(
          {event.joint, event.component, fixed(event.original_value, 6),
           fixed(event.applied_value, 6), fixed(event.limit, 6)});
    }
    if (projection_rows.empty()) {
      projection_rows.push_back(noneRow(5U));
    }
    events.sections.insert(
        events.sections.begin() + 1,
        sheet("Joint projection events",
              {textColumn("Joint"), textColumn("Component"),
               numberColumn("Original"), numberColumn("Applied"),
               numberColumn("Limit")},
              std::move(projection_rows)));

    std::vector<std::vector<std::string>> clamp_rows;
    for (const auto &event : app.clamp_events) {
      clamp_rows.push_back({event.arm, event.component, event.axis,
                            fixed(event.original_value, 6),
                            fixed(event.applied_value, 6),
                            fixed(event.limit, 6)});
    }
    if (clamp_rows.empty()) {
      clamp_rows.push_back(noneRow(6U));
    }
    events.sections.insert(
        events.sections.begin() + 2,
        sheet("Cartesian retarget clamp events",
              {textColumn("Arm"), textColumn("Component"), textColumn("Axis"),
               numberColumn("Original"), numberColumn("Applied"),
               numberColumn("Limit")},
              std::move(clamp_rows)));
  }

  document.pages.insert(document.pages.end(), snapshot.extra_pages.begin(),
                        snapshot.extra_pages.end());
  if (!snapshot.header_context.empty()) {
    document.header_left += "   " + snapshot.header_context;
  }
  const std::string page_count = std::to_string(document.pages.size());
  if (snapshot.footer_hints.has_value()) {
    document.footer_hints = *snapshot.footer_hints;
  } else if (snapshot.joint_otg.has_value()) {
    document.footer_hints =
        snapshot.joint_otg->source_mode == "mcap replay"
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
    document.help_lines.insert(document.help_lines.end(),
                               snapshot.help_lines.begin(),
                               snapshot.help_lines.end());
  }
  return document;
}

PlannedGroupedTui::PlannedGroupedTui(PlannedGroupedTuiConfig config)
    : renderer_(config.enabled) {}

void PlannedGroupedTui::handleNavigation(const KeyEvent &event) {
  renderer_.handleNavigation(event);
}

void PlannedGroupedTui::render(const PlannedGroupedTuiSnapshot &snapshot) {
  renderer_.render(makePlannedGroupedTuiDocument(snapshot));
}

} // namespace motion_control_lab
