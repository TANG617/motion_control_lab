#include "tui_projection.hpp"
#include <iomanip>
#include <sstream>
namespace motion_control_lab::joint_path_planning {
namespace {
std::string n(double x) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(4) << x;
  return s.str();
}
std::string xyz(const Pose &p) {
  return n(p.translation().x()) + "  " + n(p.translation().y()) + "  " +
         n(p.translation().z());
}
TuiSection section(std::string title, std::vector<TuiRow> rows) {
  TuiSection s;
  s.title = std::move(title);
  s.rows = std::move(rows);
  return s;
}
TuiPage page(std::string title, TuiSection s) {
  TuiPage p;
  p.title = std::move(title);
  p.sections.push_back(std::move(s));
  return p;
}
} // namespace
TuiDocument makeDocument(const Snapshot &s, const mcc::RobotModel &model,
                         const Planning &p) {
  TuiDocument d;
  d.title = "R1 joint path planning";
  d.status = stageName(s.stage);
  d.subtitle = s.message;
  d.header_left = (s.whole_body ? "16 DOF | target: " : "7 DOF | target: ") +
                  std::string(armSideName(s.side));
  d.header_right = "Kinematic execution | static scene";
  d.minimum_width = 45;
  d.minimum_height = 16;
  d.footer_hints = "Enter plan+go | Space pause | c cancel plan | 1-7 pages | "
                   "? help | x exit";
  d.help_lines = {
      "1..7/F1..F7/Tab: pages; PgUp/PgDn/Home/End: scroll; h/?: help",
      "Left/Right: select arm; w/s: X; a/d: Y; q/e: Z (base frame)",
      "Up/Down: double/halve step; m: enter step; n: rotation axis; i/u: "
      "rotate",
      "r: reset target from current TCP; p: load configured demo goal",
      "Enter: IK -> joint search -> timing -> execute (no confirmation)",
      "Space: freeze/resume execution time, not braking; c: cancel planning",
      "While busy: motion editing and resubmission disabled; navigation "
      "remains available",
      "x/Esc/Ctrl+C: exit. Static scene, discrete validation, no hardware "
      "control."};
  auto overview = section("Target and execution",
                          {{"State", stageName(s.stage)},
                           {"Current TCP [m]", xyz(s.actual)},
                           {"Draft TCP [m]", xyz(s.draft)},
                           {"Translation step [m]", n(s.step_m)},
                           {"Elapsed trajectory [s]", n(s.progress_s)}});
  if (s.attempt)
    overview.rows.push_back({"Submitted TCP [m]", xyz(s.attempt->target)});
  if (s.attempt && s.attempt->ik.pose_constraint) {
    const auto values = diagnosticsJson(s);
    overview.rows.push_back({"Held TCP", values["held_frame"].asString()});
    overview.rows.push_back(
        {"Hold error [mm / rad]",
         n(values["held_position_error_m"].asDouble() * 1000.) + " / " +
             n(values["held_orientation_error_rad"].asDouble())});
    overview.rows.push_back({"Hold tolerance", "1 mm / 0.01 rad (base_link)"});
  }
  overview.lines.push_back(s.message);
  d.pages.push_back(page("Overview", overview));
  TuiSection ik = section("IK endpoint acceptance", {}),
             path = section("Joint path search", {}),
             collision = section("Full-body collision", {}),
             trajectory = section("Trajectory timing", {});
  if (s.attempt) {
    const auto &a = *s.attempt;
    const auto &m = a.motion;
    ik.rows = {{"Accepted endpoint", a.ik.accepted ? "yes" : "no"},
               {"Iterations", std::to_string(a.ik.diagnostics.iterations)},
               {"IK [ms]", n(a.ik.diagnostics.solve_time_ms)}};
    for (const auto &e : a.ik.diagnostics.position_errors)
      ik.rows.push_back({"Position error [m]", n(e.norm_m)});
    for (const auto &e : a.ik.diagnostics.orientation_errors)
      ik.rows.push_back({"Orientation error [rad]", n(e.norm_rad)});
    path.rows = {{"Accepted motion", m.accepted ? "yes" : "no"},
                 {"Termination", terminationName(m.planning.termination)},
                 {"Native backend", m.planning.backend_status},
                 {"Straight joint path",
                  m.direct.valid ? "valid" : "invalid / not checked"},
                 {"Waypoints", std::to_string(m.path.path.waypoints.size())},
                 {"Checked states", std::to_string(m.planning.checked_states)},
                 {"Planner [ms]", n(m.planning.total_time_ms)}};
    path.rows.push_back(
        {"Shortening",
         simplificationTerminationName(m.planning.simplification_termination)});
    path.rows.push_back(
        {"Shortening [ms]", n(m.planning.simplification_time_ms)});
    path.rows.push_back(
        {"Shortcuts / attempts",
         std::to_string(m.planning.accepted_shortcuts) + " / " +
             std::to_string(m.planning.simplification_attempts)});
    const auto quality = diagnosticsJson(s);
    if (quality.isMember("path_length_before_simplification")) {
      path.rows.push_back(
          {"Normalized length",
           n(quality["path_length_before_simplification"].asDouble()) + " -> " +
               n(quality["path_length_after_simplification"].asDouble())});
      path.rows.push_back(
          {"Shorter [%]", n(quality["path_shortening_percent"].asDouble())});
    }
    if (m.verified.minimum_distance_m)
      collision.rows.push_back({"Final path sampled distance [m]",
                                n(*m.verified.minimum_distance_m)});
    const auto &v = m.accepted ? m.verified : m.planning.last_validation;
    collision.rows.push_back({"Validation", validationName(v.reason)});
    if (v.violating_pair)
      collision.rows.push_back(
          {"Violating pair", v.violating_pair->first.name + " / " +
                                 v.violating_pair->second.name});
    if (v.nearest_pair)
      collision.rows.push_back(
          {"Nearest checked pair",
           v.nearest_pair->first.name + " / " + v.nearest_pair->second.name});
    trajectory.rows = {
        {"Duration [s]", n(m.timing.duration)},
        {"Timing mode", s.timing_mode},
        {"Smooth validation", s.smooth_validation},
        {"Trajectory validation", m.trajectory_validation_status},
        {"Motion segments", std::to_string(m.timing.segment_count)},
        {"Through waypoints", std::to_string(m.timing.through_waypoint_count)},
        {"Stops (incl. endpoints)",
         std::to_string(m.curve ? m.smoothing.stop_times.size()
                                : m.timing.stop_waypoint_indices.size())},
        {"Timing [ms]", n(m.timing.calculation_time_ms)},
        {"Pruned waypoints", std::to_string(m.planning.removed_vertices)},
        {"Trajectory verification [ms]", n(m.trajectory_validation_ms)},
        {"Certified trajectory intervals",
         std::to_string(
             m.trajectory_validation.geometry.certified_interval_count)},
        {"Smooth deviation bound [rad]",
         m.trajectory_validation.maximum_deviation_bound.size()
             ? n(m.trajectory_validation.maximum_deviation_bound.maxCoeff())
             : "-"},
        {"Request [ms]", n(a.request_time_ms)},
        {"Samples", std::to_string(m.timing.sample_count)},
        {"Peak velocity/limit", n(m.timing.maximum_velocity_ratio)},
        {"Peak acceleration/limit", n(m.timing.maximum_acceleration_ratio)},
        {"Peak jerk/limit", n(m.timing.maximum_jerk_ratio)}};
  } else {
    ik.lines = {"No submitted target"};
    path.lines = {"No plan"};
    trajectory.lines = {"No timed trajectory"};
  }
  collision.rows.push_back(
      {"Collision geometries", std::to_string(p.geometry.geometry_count)});
  collision.rows.push_back(
      {"Allowed self pairs",
       std::to_string(p.scene->description().allowed_collisions.size())});
  collision.lines = {
      "Clearance 5 mm; final revolute step 0.005 rad",
      "Sampled evidence, not continuous collision certification"};
  for (const auto &name : p.geometry.links_without_geometry)
    collision.lines.push_back("No collision geometry: " + name);
  d.pages.push_back(page("IK", ik));
  d.pages.push_back(page("Path", path));
  d.pages.push_back(page("Collision", collision));
  d.pages.push_back(page("Trajectory", trajectory));
  TuiSection joints;
  joints.title = "Full model positions [rad]";
  TuiTable t;
  t.style = TuiTableStyle::Compact;
  t.columns = {{"Joint"}, {"q", TuiTableAlignment::Right}, {"Role"}};
  for (std::size_t i = 0; i < model.jointNames().size(); ++i) {
    const auto &name = model.jointNames()[i];
    t.rows.push_back(
        {name, n(s.state.joint_positions[i]),
         name.find(std::string(armSideName(s.side)) + "_arm_joint") == 0
             ? "target arm"
             : (s.whole_body && (name == "torso_pitch_joint" ||
                                 name == "torso_yaw_joint")
                    ? "waist"
                    : (s.whole_body && (name.find("left_arm_joint") == 0 ||
                                        name.find("right_arm_joint") == 0)
                           ? "held TCP"
                           : "fixed q"))});
  }
  joints.tables.push_back(t);
  d.pages.push_back(page("Joints", joints));
  auto events = section(
      "Runtime and events",
      {{"Scene", p.scene->description().id},
       {"Scene revision", std::to_string(p.scene->description().revision)},
       {"State", stageName(s.stage)}});
  events.lines = s.events;
  d.pages.push_back(page("Runtime", events));
  return d;
}
} // namespace motion_control_lab::joint_path_planning
