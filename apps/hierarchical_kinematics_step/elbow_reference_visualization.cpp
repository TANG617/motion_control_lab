#include "elbow_reference_visualization.hpp"

#include <cmath>
#include <cstring>
#include <json/json.h>

namespace motion_control_lab::hierarchical_kinematics_step {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegrees = 180.0 / kPi;

std::vector<std::byte> bytes(const std::string &text) {
  std::vector<std::byte> result(text.size());
  std::memcpy(result.data(), text.data(), text.size());
  return result;
}
std::string encode(const Json::Value &value) {
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  return Json::writeString(writer, value);
}
const auto &schema() {
  static const auto data = [] {
    Json::Value value;
    value["type"] = "object";
    value["title"] = "Dual-arm posture reference angle and task participation";
    for (const char *key :
         {"cos_phi", "sin_phi", "angle_rad", "angle_deg", "angle_unwrapped_deg",
          "sample_time_s", "consume_time_s", "reference_age_ms",
          "inference_ms"})
      value["properties"][key]["type"] = "number";
    for (const char *key : {"generation", "input_sequence", "target_revision",
                            "selected_priority"})
      value["properties"][key]["type"] = "integer";
    for (const char *key :
         {"task_enabled", "secondary_attempted", "secondary_succeeded", "secondary_selected", "mirror_tcp_input"})
      value["properties"][key]["type"] = "boolean";
    for (const char *key : {"left_scale", "right_scale", "left_tcp_position_error_m",
         "right_tcp_position_error_m", "left_tcp_orientation_error_rad", "right_tcp_orientation_error_rad"})
      value["properties"][key]["type"] = "number";
    value["properties"]["source"]["type"] = "string";
    value["properties"]["side"]["type"] = "string";
    value["properties"]["participation"]["type"] = "string";
    for (const char *key : {"elbow_raw_error_m", "elbow_executed_error_m"}) {
      value["properties"][key]["type"].append("number");
      value["properties"][key]["type"].append("null");
    }
    return std::make_shared<const std::vector<std::byte>>(bytes(encode(value)));
  }();
  return data;
}
std::array<double, 3> point(const Eigen::Vector3d &p) {
  return {p.x(), p.y(), p.z()};
}
} // namespace

double ElbowReferenceAngleTracker::update(const ElbowPrediction &p, std::size_t side) {
  const double angle = std::atan2(p.arm_angles[side][1], p.arm_angles[side][0]);
  if (!initialized_ || p.generation != generation_) {
    unwrapped_ = angle;
  } else if (p.sequence == sequence_) {
    return unwrapped_;
  } else {
    unwrapped_ += std::remainder(angle - wrapped_, 2.0 * kPi);
  }
  initialized_ = true;
  generation_ = p.generation;
  sequence_ = p.sequence;
  wrapped_ = angle;
  return unwrapped_;
}

ElbowReferenceVisualizationSnapshot makeElbowReferenceVisualizationSnapshot(
    const ElbowConsumption &consumed, const std::array<ElbowGeometry,2> &geometry,
    const std::array<Eigen::Vector3d,2> &shoulder, const ArmPoses &ee_reference,
    const Eigen::Isometry3d &ref, std::array<ElbowReferenceAngleTracker,2> &tracker) {
  ElbowReferenceVisualizationSnapshot result;
  result.valid = true;
  result.consumed = consumed;
  for (int a=0;a<2;++a) {
    auto &arm=result.arms[a];
    arm.angle_unwrapped_rad=tracker[a].update(consumed.prediction,a);
    arm.shoulder=shoulder[a];
    arm.wrist=ee_reference[a]*geometry[a].wrist_in_ee;
    if (consumed.arms[a].enabled) {
      const auto k=geometry[a].referenceCircle(ref*shoulder[a],ref*ee_reference[a]);
      arm.circle={ref.inverse()*k.center,ref.linear().transpose()*k.u,
                  ref.linear().transpose()*k.v,k.radius};
    }
  }
  return result;
}

void appendElbowReferenceVisualization(motion_control::viz::RenderBatch &batch,
                            const std::string &frame, const std::string &source,
                            const ElbowReferenceVisualizationSnapshot &s) {
  if (!s.valid || source == "manual")
    return;
  namespace mcv = motion_control::viz;
  const auto &record = s.consumed;
  const auto &p = record.prediction;
  for (int a=0;a<2;++a) {
  const auto &arm=s.arms[a];
  const auto &consumed_arm=record.arms[a];
  const std::string side=a==0?"left":"right";
  const std::string prefix=source=="harp"?"/mcl/harp/":"/mcl/elbow_reference/";
  const double phi = std::atan2(p.arm_angles[a][1], p.arm_angles[a][0]);
  Json::Value message;
  message["generation"] = Json::UInt64(p.generation);
  message["input_sequence"] = Json::UInt64(p.sequence);
  message["target_revision"] = Json::UInt64(record.target_revision);
  message["sample_time_s"] = p.sample_time_s;
  message["consume_time_s"] = record.time_s;
  message["cos_phi"] = p.arm_angles[a][0];
  message["sin_phi"] = p.arm_angles[a][1];
  message["angle_rad"] = phi;
  message["angle_deg"] = phi * kDegrees;
  message["angle_unwrapped_deg"] = arm.angle_unwrapped_rad * kDegrees;
  message["reference_age_ms"] = (record.time_s - p.sample_time_s) * 1000.0;
  message["inference_ms"] = p.inference_ms;
  message["source"] = source;
  message["side"] = side;
  message["task_enabled"] = consumed_arm.enabled;
  message["participation"] = consumed_arm.enabled ? "task reference" : "prediction only; task disabled";
  message["elbow_raw_error_m"] = consumed_arm.enabled ? Json::Value((Eigen::Map<const Eigen::Vector3d>(consumed_arm.raw.data())-Eigen::Map<const Eigen::Vector3d>(consumed_arm.target.data())).norm()) : Json::Value();
  message["elbow_executed_error_m"] = consumed_arm.enabled ? Json::Value((Eigen::Map<const Eigen::Vector3d>(consumed_arm.executed.data())-Eigen::Map<const Eigen::Vector3d>(consumed_arm.target.data())).norm()) : Json::Value();
  message["mirror_tcp_input"] = record.mirror_tcp_input;
  message["left_scale"] = s.left_scale;
  message["right_scale"] = s.right_scale;
  message["left_tcp_position_error_m"] = s.left_tcp_position_error_m;
  message["right_tcp_position_error_m"] = s.right_tcp_position_error_m;
  message["left_tcp_orientation_error_rad"] = s.left_tcp_orientation_error_rad;
  message["right_tcp_orientation_error_rad"] = s.right_tcp_orientation_error_rad;

  message["secondary_attempted"] = record.secondary_executed;
  message["secondary_succeeded"] = record.secondary_succeeded;
  message["selected_priority"] = record.selected_priority;
  message["secondary_selected"] =
      record.secondary_succeeded && record.selected_priority == 2;
  batch.encoded_messages.push_back(
      {prefix + side + "/angle", "json", source == "harp" ? "mcl.harp.Angle" : "mcl.elbow_reference.Angle", "jsonschema", schema(),
       bytes(encode(message)), batch.timestamp_ns});

  if (!consumed_arm.enabled) continue;
  const mcv::ColorRgba gray{0.6, 0.6, 0.6, 0.6};
  const mcv::ColorRgba white{1, 1, 1, 0.9};
  const mcv::ColorRgba green{0.15, 1, 0.2, 0.95};
  const auto add = [&](const char *id, const mcv::ColorRgba &color,
                       std::vector<std::array<double, 3>> points,
                       double width) {
    batch.line_strips.push_back(
        {id, prefix + side + "/scene", frame, std::move(points), color, width, true});
  };
  const auto &circle = arm.circle;
  const auto onCircle = [&](double angle) -> Eigen::Vector3d {
    return circle.center + circle.radius * (std::cos(angle) * circle.u +
                                            std::sin(angle) * circle.v);
  };
  add("shoulder_wrist_axis", gray, {point(arm.shoulder), point(arm.wrist)}, 2);
  std::vector<std::array<double, 3>> ring, arc;
  constexpr int segments = 64;
  ring.reserve(segments + 1);
  arc.reserve(segments + 1);
  for (int i = 0; i <= segments; ++i) {
    ring.push_back(point(onCircle(2.0 * kPi * i / segments)));
    arc.push_back(point(onCircle(phi * i / segments)));
  }
  add("elbow_circle", gray, std::move(ring), 1.5);
  add("zero_direction", white, {point(circle.center), point(onCircle(0))}, 2);
  // Use the exact accepted control target as the endpoint, not a later FK.
  add(source == "harp" ? "harp_direction" : "elbow_reference_direction", green, {point(circle.center), consumed_arm.target}, 3);
  add(source == "harp" ? "harp_angle_arc" : "elbow_reference_angle_arc", green, std::move(arc), 3);
  const Eigen::Vector3d end = onCircle(phi);
  const Eigen::Vector3d radial =
      std::cos(phi) * circle.u + std::sin(phi) * circle.v;
  const Eigen::Vector3d tangent =
      (phi < 0 ? -1.0 : 1.0) *
      (-std::sin(phi) * circle.u + std::cos(phi) * circle.v);
  const double length = circle.radius * 0.12;
  auto arrow_color = green;
  if (std::abs(phi) < 1e-9)
    arrow_color.alpha = 0;
  add(source == "harp" ? "harp_angle_arrow" : "elbow_reference_angle_arrow", arrow_color,
      {point(end - length * tangent + length * 0.5 * radial), point(end),
       point(end - length * tangent - length * 0.5 * radial)},
      3);
  add("raw_elbow", mcv::ColorRgba{1,.65,.1,1}, {consumed_arm.target,consumed_arm.raw}, 2);
  add("executed_elbow", mcv::ColorRgba{.2,.6,1,1}, {consumed_arm.target,consumed_arm.executed}, 2);
  }
}
} // namespace motion_control_lab::hierarchical_kinematics_step
