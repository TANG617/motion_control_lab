#include "tui_projection.hpp"
#include <cmath>
#include <iomanip>
#include <sstream>

namespace motion_control_lab::psibot_teleop {
namespace {
std::string number(double value, int precision = 3) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}
std::string age(Clock::time_point point) {
  if (point == Clock::time_point{}) return "unavailable";
  return number(std::chrono::duration<double, std::milli>(Clock::now() - point).count()) + " ms";
}
const char * partName(int part) {
  constexpr const char * names[] = {"all", "left arm", "right arm", "waist", "head"};
  return names[part];
}
std::string profileName(sdk::ArmProfile profile) {
  if (profile == sdk::ArmProfile::JointPosition) return "JointPosition";
  if (profile == sdk::ArmProfile::CartesianPosition) return "CartesianPosition";
  if (profile == sdk::ArmProfile::Idle) return "Idle";
  return "SDK profile " + std::to_string(static_cast<int>(profile));
}
void wideColumns(TuiPage & page, std::size_t split) {
  TuiPageLayout wide;
  wide.minimum_width = 121;
  wide.column_weights = {1, 1};
  wide.sections = page.sections;
  for (std::size_t i = split; i < wide.sections.size(); ++i) wide.sections[i].column = 1;
  page.responsive_layouts.push_back(std::move(wide));
}
}

std::string plainStatus(const Snapshot & state) {
  return std::string("connected=") + (state.connected ? "yes" : "no") +
    " mode=" + modeName(state.mode) + " sending=" + (state.publishing ? "yes" : "no") +
    " requests=" + std::to_string(state.requests) + " sends=" + std::to_string(state.sends) +
    " reads=" + std::to_string(state.reads) + " last_rpc=" + state.last_rpc +
    " code=" + std::to_string(state.last_code) + " rpc_ms=" + number(state.rpc_ms) +
    " activity=" + state.activity;
}

TuiDocument makeDocument(const Options & options, const Snapshot & state,
                         const InputController & input, const std::string & log_dir) {
  TuiDocument doc;
  doc.title = "PSI Teleop";
  doc.subtitle = options.address;
  doc.minimum_width = 48;
  doc.minimum_height = 12;
  doc.dim_secondary_text = false;
  const std::string control = !state.ready ? "Connecting" : state.mode == Mode::Wbc ? "Dual-arm WBC" :
    input.jointControl() ? "Joint" : "Cartesian";
  const std::string target = input.jointControl() ? std::string(partName(input.controlPart())) +
    " J" + std::to_string(input.selectedJoint() + 1) : partName(input.teleop().selectedSide() == ArmSide::Left ? 1 : 2);
  const std::string sending = input.applying() ? "SWITCHING" : input.starting() ? "STARTING" :
    input.sending() ? "SENDING" : "PAUSED";
  doc.header_left = control + " | " + target + " | " + sending;
  // Keep the footer readable in a narrow split; use wrapping rather than an
  // unbounded status + right-aligned hints on the same line.
  doc.status = input.menu() ? "Up/Down mode | Left/Right target\nEnter select | Esc cancel" :
    input.applying() || input.starting() ? input.status() + " | " + state.activity :
    input.status() + "\n" + (input.sending() ? "Space pause" : "Space start") + " | c mode | h help | x exit";
  doc.help_lines = {
    "c: choose Joint control, Single-arm Cartesian or Dual-arm WBC. Up/Down selects mode; Left/Right selects target; Enter applies; Esc cancels.",
    "Choosing a mode pauses sending, ends this app's WBC if active, and selects the required single-part profile. Space begins after completion.",
    "Space starts/pauses sending. Begin reads actual feedback; paused motion keys do not accumulate targets.",
    "Joint: Left/Right joint; w/s +/- angle; Up/Down step x2 /2; r reset from actual. Use c to select left/right arm, waist or head.",
    "Cartesian: Left/Right arm; switching arms selects its Cartesian profile and pauses. Space begins again.",
    "Cartesian / WBC: w/s +/-X; a/d +/-Y; q/e +/-Z (Base). n selects TCP local rotation axis; u/i rotates. q is a motion key.",
    "Cartesian / WBC: Up/Down step x2 /2; m enters metres; r resets selected target from actual feedback.",
    "WBC: Left/Right selects which target to edit without pausing. Both targets are retained; only changed sides are submitted.",
    "1..5 / F1..F5 / Tab / Shift-Tab pages; PgUp/PgDn/Home/End scroll; x/Esc exit; Ctrl-C coordinated exit.",
    "Pausing stops new submissions; accepted motion may continue. RPC success means accepted, not completed.",
    "WBC starts on Space, uses server frequency, and ends on mode change or app exit. One client should own motion commands.",
    "SDK RPCs retain their built-in timeout. Exit waits for pending RPCs before cleanup. Runtime shows call duration, not solver time.",
    "Position is metres; quaternion order is xyzw. Read completion age is not source feedback age.",
    "Logs: " + log_dir + "/sdk.log"};

  TuiPage overview; overview.title = "Home";
  TuiSection connection;
  connection.title = "Control";
  connection.rows = {{"Step", input.jointControl() ? number(input.jointStepDeg(), 2) + " deg" :
      number(input.teleop().stepMetres() * 1000, 2) + " mm"}};
  connection.lines = input.jointControl() ?
    std::vector<std::string>{"Left/Right joint   w/s +/- angle", "Up/Down step       r reset from actual"} :
    std::vector<std::string>{"w/s X   a/d Y   q/e Z   Left/Right arm", "n axis  u/i rotate  Up/Down step  r reset"};
  if (state.mode == Mode::Wbc)
    connection.lines.push_back("Both arm targets retained; edit one at a time.");
  overview.sections.push_back(connection);
  TuiSection jog;
  if (input.jointControl() && !input.jointTargets().empty()) {
    jog.title = "Selected joint";
    const auto i = jointOffset(input.controlPart()) + input.selectedJoint();
    const auto & name = state.feedback.names.at(i);
    jog.rows = {{"Selected", name}, {"Edited", number(input.jointTargets().at(i), 6) + " rad"},
                {"Actual", number(state.feedback.positions.at(i), 6) + " rad"}};
    if (state.submitted_joints && state.submitted_joints->component == input.controlPart()) {
      const auto accepted = state.submitted_joints->positions.at(input.selectedJoint());
      jog.rows.push_back({"Accepted", number(accepted, 6) + " rad"});
      jog.rows.push_back({"Error", number(accepted - state.feedback.positions.at(i), 6) + " rad"});
    } else jog.rows.push_back({"Accepted", "none in this sending session"});
    for (const auto & limit : state.limits.joints) if (limit.name == name)
      jog.rows.push_back({"Limits", number(limit.position_lower, 4) + " .. " + number(limit.position_upper, 4) + " rad"});
    overview.sections.push_back(jog);
  }
  if (!input.jointControl() && state.ready) {
    TuiSection tcp;
    tcp.title = "Selected TCP | Base xyz [m]";
    const auto side = input.teleop().selectedSide() == ArmSide::Left ? 0 : 1;
    const auto xyz = [](const Pose & pose) {
      return number(pose.translation().x(), 4) + "  " + number(pose.translation().y(), 4) + "  " + number(pose.translation().z(), 4);
    };
    tcp.rows = {{"Edited", xyz(input.teleop().frame().targets[side].target_pose)},
                {"Accepted", state.submitted[side] ? xyz(*state.submitted[side]) : "none this session"},
                {"Actual", xyz(state.feedback.poses[side])}};
    overview.sections.push_back(tcp);
  }
  TuiSection connection_detail;
  connection_detail.title = "Connection";
  connection_detail.rows = {{"Endpoint", options.address},
    {"SDK", state.connected ? "Connected (Psi)" : "Connecting"}, {"Activity", state.activity}};
  connection_detail.lines = {"Pause stops new targets; accepted motion may continue."};
  overview.sections.push_back(connection_detail);
  wideColumns(overview, 1);
  doc.pages.push_back(std::move(overview));

  TuiPage poses; poses.title = "Targets";
  if (!jog.rows.empty()) poses.sections.push_back(jog);
  for (std::size_t i = 0; i < 2; ++i) {
    TuiSection comparison;
    comparison.title = std::string(i == 0 ? "Left" : "Right") + " TCP | Base / xyzw";
    TuiTable table; table.style = TuiTableStyle::Compact;
    table.columns = {{"Axis", TuiTableAlignment::Left}, {"Edited", TuiTableAlignment::Right},
                     {"Accepted", TuiTableAlignment::Right}, {"Actual", TuiTableAlignment::Right}};
    const auto edit = toSdkPose(input.teleop().frame().targets[i].target_pose);
    const auto actual = toSdkPose(state.feedback.poses[i]);
    const auto accepted = toSdkPose(state.submitted[i].value_or(Pose::Identity()));
    const char * axes[] = {"x [m]", "y [m]", "z [m]", "qx", "qy", "qz", "qw"};
    for (std::size_t j = 0; j < 7; ++j)
      table.rows.push_back({axes[j], state.ready && !input.jointControl() ? number(edit[j], 5) : "--",
        state.submitted[i] ? number(accepted[j], 5) : "--", state.ready ? number(actual[j], 5) : "--"});
    comparison.tables.push_back(table);
    if (state.ready && state.submitted[i]) {
      const auto & target = *state.submitted[i];
      const auto & actual_pose = state.feedback.poses[i];
      const double angle = Eigen::Quaterniond(target.rotation()).angularDistance(Eigen::Quaterniond(actual_pose.rotation()));
      comparison.lines = {"Accepted - actual: " + number((target.translation() - actual_pose.translation()).norm() * 1000) +
        " mm / " + number(angle * 180.0 / 3.141592653589793) + " deg"};
    } else comparison.lines = {"Accepted: no target submitted this session"};
    poses.sections.push_back(comparison);
  }
  wideColumns(poses, poses.sections.size() - 1);
  doc.pages.push_back(std::move(poses));

  TuiPage joints; joints.title = "Joints";
  TuiSection joint_section; joint_section.title = "SDK actual joint state";
  TuiTable table; table.style = TuiTableStyle::Compact;
  table.columns = {{"Joint", TuiTableAlignment::Left}, {"  q [rad]", TuiTableAlignment::Right},
                   {"    v", TuiTableAlignment::Right}, {"    a", TuiTableAlignment::Right}};
  for (std::size_t i = 0; i < state.feedback.names.size(); ++i) {
    const auto value = [i](const std::vector<double> & values) { return i < values.size() ? number(values[i]) : "n/a"; };
    table.rows.push_back({state.feedback.names[i], value(state.feedback.positions), value(state.feedback.velocities), value(state.feedback.accelerations)});
  }
  joint_section.tables.push_back(table);
  joint_section.lines = {"Velocity: rad/s; acceleration: rad/s^2.", "Torque: unavailable (current Psi server supplies zero placeholders).",
                         "Joint read completed " + age(state.joint_read_at) + " ago."};
  joints.sections.push_back(joint_section);
  TuiSection limits; limits.title = "SDK joint limits";
  TuiTable limit_table; limit_table.style = TuiTableStyle::Compact;
  limit_table.columns = {{"Joint", TuiTableAlignment::Left}, {"  lower", TuiTableAlignment::Right},
                         {"  upper", TuiTableAlignment::Right}, {"  vmax", TuiTableAlignment::Right}};
  for (const auto & limit : state.limits.joints)
    limit_table.rows.push_back({limit.name, number(limit.position_lower), number(limit.position_upper), number(limit.max_velocity)});
  limits.tables.push_back(limit_table);
  joints.sections.push_back(limits);
  wideColumns(joints, 1);
  doc.pages.push_back(std::move(joints));

  TuiPage runtime; runtime.title = "RPC";
  TuiSection rpc; rpc.title = "Client RPC and polling";
  rpc.rows = {{"Last RPC", state.last_rpc}, {"Return code", std::to_string(state.last_code)},
    {"Last / max RPC", number(state.rpc_ms) + " / " + number(state.maximum_rpc_ms) + " ms"},
    {"Requests", std::to_string(state.requests)}, {"Accepted sends", std::to_string(state.sends)},
    {"Actual sends", state.elapsed_s > 0 ? number(state.sends / state.elapsed_s) + " Hz (session average)" : "n/a"},
    {"Actual polling", state.elapsed_s > 0 ? number(state.reads / state.elapsed_s) + " Hz (session average)" : "n/a"},
    {"Configured send / read", number(options.send_hz, 1) + " / " + number(options.feedback_hz, 1) + " Hz"},
    {"Pose read completed", age(state.pose_read_at) + " ago"}, {"Joint read completed", age(state.joint_read_at) + " ago"}};
  rpc.lines = {"Only changed targets are sent. RPC durations include transport and server response time.",
               "SDK does not expose source timestamps. Joint and pose reads are separate RPCs.",
               "HQP/Yellow diagnostics and server motion completion are not available through this SDK."};
  runtime.sections.push_back(rpc);
  TuiSection server;
  server.title = "SDK component state";
  for (int part = 1; part <= 4; ++part)
    server.rows.push_back({partName(part), state.ready ?
      profileName(state.profiles[part - 1]) + (state.enabled[part - 1] ? " / enable=true" : " / enable=false") : "unavailable"});
  server.rows.push_back({"App WBC session", state.owns_wbc ? "started" : "not started"});
  server.lines = {"Enable is a software flag; current Cortex Mock does not gate motion on it.",
                  "Server session and emergency stop are not exposed by this SDK.", "SDK log: " + log_dir + "/sdk.log"};
  runtime.sections.push_back(server);
  doc.pages.push_back(std::move(runtime));

  TuiPage events; events.title = "Events";
  TuiSection recent; recent.title = "Recent app events (newest first, capacity 128)";
  recent.lines.assign(state.events.rbegin(), state.events.rend());
  if (recent.lines.empty()) recent.lines.push_back("No events yet");
  events.sections.push_back(std::move(recent));
  doc.pages.push_back(std::move(events));

  if (input.menu()) {
    TuiSection menu;
    menu.title = "Control mode | " + std::string(input.menuRow() == 2 ? "both arms" : partName(input.component()));
    const char * actions[] = {"Joint control", "Single-arm Cartesian", "Dual-arm WBC"};
    for (int i = 0; i < 3; ++i)
      menu.lines.push_back(std::string(i == input.menuRow() ? "> " : "  ") + actions[i]);
    const char * descriptions[] = {"w/s jogs one joint; Left/Right selects joint.",
      "Move one TCP in Base; rotate about local axes.",
      "Edit either TCP; solve both arms together."};
    menu.lines.push_back("");
    menu.lines.push_back(descriptions[input.menuRow()]);
    // A modal owns motion/navigation keys regardless of the previously selected page.
    doc.modal_sections = {menu};

  }
  return doc;
}
}  // namespace motion_control_lab::psibot_teleop
