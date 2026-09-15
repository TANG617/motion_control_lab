#include "../elbow_reference_visualization.hpp"
#include <cmath>
#include <iostream>
#include <json/json.h>
#include <stdexcept>
#include <set>
#include <type_traits>

namespace app = motion_control_lab::hierarchical_kinematics_step;
constexpr double pi = 3.14159265358979323846;
void require(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
Json::Value decode(const motion_control::viz::EncodedMessageSample &sample) {
  Json::CharReaderBuilder builder;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  Json::Value value;
  std::string error;
  auto *begin = reinterpret_cast<const char *>(sample.data.data());
  require(reader->parse(begin, begin + sample.data.size(), &value, &error),
          "JSON decode");
  return value;
}
int main() {
  try {
    static_assert(std::is_standard_layout_v<app::ElbowReferenceVisualizationSnapshot>);
    app::ElbowReferenceAngleTracker tracker;
    app::ElbowPrediction p;
    for (double sign : {1.0, -1.0}) {
      ++p.generation;
      p.sequence = 0;
      p.cosine = std::cos(sign * (pi - .01));
      p.sine = std::sin(sign * (pi - .01));
      require(std::abs(tracker.update(p) - sign * (pi - .01)) < 1e-12,
              "run reset");
      ++p.sequence;
      p.cosine = std::cos(-sign * (pi - .01));
      p.sine = std::sin(-sign * (pi - .01));
      const double angle = tracker.update(p);
      require(std::abs(angle - sign * (pi + .01)) < 1e-12,
              "bidirectional unwrap");
      require(tracker.update(p) == angle, "repeat/pause holds unwrapped angle");
    }
    Eigen::Isometry3d shoulder = Eigen::Isometry3d::Identity(),
                      elbow = shoulder, wrist = shoulder;
    elbow.translation() << .2, .2, 0;
    wrist.translation() << .4, 0, 0;
    const auto geometry =
        app::ElbowGeometry::fromFk(shoulder, elbow, wrist, wrist);
    for (const std::string source : {"harp", "recorded"})
    for (double angle : {0.0, pi / 2, -pi / 2, pi, -pi}) {
      app::ElbowConsumption record;
      record.prediction.generation = ++p.generation;
      record.prediction.cosine = std::cos(angle);
      record.prediction.sine = std::sin(angle);
      record.time_s = .01;
      record.mirror_tcp_input = true;
      record.secondary_executed = true;
      record.secondary_succeeded = false;
      record.selected_priority = 1;
      const Eigen::Vector3d target = geometry.target(
          shoulder.translation(), wrist, std::cos(angle), std::sin(angle));
      record.target = {target.x(), target.y(), target.z()};
      auto snapshot = app::makeElbowReferenceVisualizationSnapshot(
          record, geometry, shoulder.translation(), wrist, tracker);
      snapshot.left_scale = .3; snapshot.right_scale = .8;
      snapshot.left_tcp_position_error_m = .001;
      snapshot.right_tcp_position_error_m = .002;
      snapshot.left_tcp_orientation_error_rad = .01;
      snapshot.right_tcp_orientation_error_rad = .02;
      motion_control::viz::RenderBatch batch;
      batch.timestamp_ns = 123;
      app::appendElbowReferenceVisualization(batch, "base_link", source, snapshot);
      require(batch.encoded_messages.size() == 1 &&
                  batch.line_strips.size() == 6,
              "topics/scene primitives");
      const auto &sample = batch.encoded_messages.front();
      require(sample.channel == (source == "harp" ? "/mcl/harp/left/angle" : app::kRecordedElbowAngleTopic) &&
                  sample.schema_name == (source == "harp" ? "mcl.harp.Angle" : "mcl.elbow_reference.Angle") &&
                  sample.message_encoding == "json" &&
                  sample.schema_encoding == "jsonschema" &&
                  sample.timestamp_ns == 123,
              "numeric channel contract");
      const auto json = decode(sample);
      require(json["mirror_tcp_input"].asBool() && json["left_scale"].asDouble() == .3 &&
          json["right_scale"].asDouble() == .8 &&
          json["left_tcp_position_error_m"].asDouble() == .001 &&
          json["right_tcp_position_error_m"].asDouble() == .002 &&
          json["left_tcp_orientation_error_rad"].asDouble() == .01 &&
          json["right_tcp_orientation_error_rad"].asDouble() == .02, "mirror comparison metadata");
      require(std::abs(json["angle_rad"].asDouble() - angle) < 1e-12,
              "atan2 angle");
      require(std::abs(json["angle_deg"].asDouble() - angle * 180 / pi) < 1e-10,
              "degrees");
      require(!json["secondary_selected"].asBool() &&
                  json["selected_priority"].asInt() == 1,
              "Primary-only is not learning completion");
      require(json["reference_age_ms"].asDouble() == 10,
              "active reference age");
      std::set<std::string> marker_ids;
      for (const auto &line : batch.line_strips) {
        marker_ids.insert(line.entity_id);
        require(line.channel == (source == "harp" ? "/mcl/harp/left/scene" : app::kRecordedElbowSceneTopic) &&
                    line.frame_id == "base_link",
                "scene frame/channel");
        if (line.entity_id == (source == "harp" ? "harp_direction" : "elbow_reference_direction"))
          require(line.points_m.back() == record.target,
                  "exact accepted elbow target endpoint");
        if (line.entity_id == "elbow_circle") {
          for (const auto &point : line.points_m) {
            Eigen::Vector3d x(point[0], point[1], point[2]);
            require(std::abs((x - shoulder.translation()).norm() -
                             geometry.upper_length) < 1e-12,
                    "ring upper length");
            require(std::abs((x - wrist.translation()).norm() -
                             geometry.lower_length) < 1e-12,
                    "ring lower length");
          }
        }
        if (line.entity_id == (source == "harp" ? "harp_angle_arc" : "elbow_reference_angle_arc")) {
          const auto &last = line.points_m.back();
          require((Eigen::Vector3d(last[0], last[1], last[2]) - target).norm() <
                      1e-12,
                  "arc ends at predicted angle");
        }
      }
      const std::string prefix = source == "harp" ? "harp_" : "elbow_reference_";
      require(marker_ids == std::set<std::string>{"shoulder_wrist_axis", "elbow_circle", "zero_direction", prefix + "direction", prefix + "angle_arc", prefix + "angle_arrow"}, "exact source-specific marker identities");
      motion_control::viz::RenderBatch disabled;
      app::appendElbowReferenceVisualization(disabled, "base_link", "manual", snapshot);
      app::appendElbowReferenceVisualization(disabled, "base_link", "recorded", {});
      require(disabled.encoded_messages.empty() && disabled.line_strips.empty(),
              "manual/uninitialized have no elbow-reference display");
      app::appendElbowReferenceVisualization(disabled, "base_link", "recorded", snapshot);
      require(decode(disabled.encoded_messages.front())["source"].asString() ==
                  "recorded",
              "recorded source");
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
