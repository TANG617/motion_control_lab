#include "visualization.hpp"
#include "contracts/visualization/mcl_execution_v1.hpp"
#include "contracts/visualization/mcl_planning_v1.hpp"
#include "contracts/visualization/mcl_state_v1.hpp"
#include "scene_schema.hpp"
#include <cstring>
namespace motion_control_lab::joint_path_planning {
namespace {
namespace v = motion_control::viz;
Json::Value vector(double x, double y, double z) {
  Json::Value r;
  r["x"] = x;
  r["y"] = y;
  r["z"] = z;
  return r;
}
Json::Value color(double r, double g, double b, double a) {
  auto c = vector(r, g, b);
  c.removeMember("x");
  c.removeMember("y");
  c.removeMember("z");
  c["r"] = r;
  c["g"] = g;
  c["b"] = b;
  c["a"] = a;
  return c;
}
Json::Value pose(const Pose &p) {
  Json::Value r;
  r["position"] =
      vector(p.translation().x(), p.translation().y(), p.translation().z());
  Eigen::Quaterniond q(p.rotation());
  r["orientation"] = vector(q.x(), q.y(), q.z());
  r["orientation"]["w"] = q.w();
  return r;
}
v::EncodedMessageSample encoded(std::string topic, std::string name,
                                const Json::Value &data, std::uint64_t stamp) {
  v::EncodedMessageSample x;
  x.channel = std::move(topic);
  x.schema_name = std::move(name);
  x.message_encoding = "json";
  x.schema_encoding = "jsonschema";
  x.timestamp_ns = stamp;
  const auto bytes = [](const std::string &s) {
    return std::make_shared<const std::vector<std::byte>>(
        reinterpret_cast<const std::byte *>(s.data()),
        reinterpret_cast<const std::byte *>(s.data() + s.size()));
  };
  static const auto scene_schema = bytes(kSceneSchema),
                    status_schema = bytes(kStatusSchema);
  x.schema_data =
      x.schema_name == "foxglove.SceneUpdate" ? scene_schema : status_schema;
  Json::StreamWriterBuilder b;
  b["indentation"] = "";
  const auto text = Json::writeString(b, data);
  x.data.resize(text.size());
  std::memcpy(x.data.data(), text.data(), text.size());
  return x;
}
} // namespace
motion_control::viz::RenderBatch
Visualization::render(const Snapshot &s, const Options &o,
                      const mcc::RobotModel &model, std::uint64_t stamp) {
  namespace v = motion_control::viz;
  v::RenderBatch batch;
  batch.timestamp_ns = stamp;
  std::vector<double> q(s.state.joint_positions.data(),
                        s.state.joint_positions.data() +
                            s.state.joint_positions.size());
  namespace state_topics = contracts::mcl_state_v1;
  namespace execution_topics = contracts::mcl_execution_v1;
  namespace planning_topics = contracts::mcl_planning_v1;
  batch.joint_states.push_back({execution_topics::kJointExecutionTopic,
                                model.jointNames(),
                                q,
                                {},
                                stamp});
  const bool has_ik = s.attempt && s.attempt->ik.accepted &&
                      s.attempt->motion.goal.joint_positions.size();
  // The common IK topic contains accepted IK results only. Between requests,
  // Foxglove retains its last accepted sample, matching the shared contract.
  if (has_ik) {
    const auto &g = s.attempt->motion.goal.joint_positions;
    std::vector<double> positions(g.data(), g.data() + g.size());
    batch.joint_states.push_back({state_topics::kJointIkTopic,
                                  model.jointNames(),
                                  positions,
                                  {},
                                  stamp});
  }
  const auto pv = [&](const Pose &p) {
    Eigen::Quaterniond q(p.rotation());
    return v::Pose3d{
        {p.translation().x(), p.translation().y(), p.translation().z()},
        {q.x(), q.y(), q.z(), q.w()}};
  };
  const std::array<const char *, 2> input_topics{
      state_topics::kLeftCartesianInputTopic,
      state_topics::kRightCartesianInputTopic};
  const std::array<const char *, 2> goal_topics{
      state_topics::kLeftCartesianGoalTopic,
      state_topics::kRightCartesianGoalTopic};
  const std::array<const char *, 2> execution_pose_topics{
      execution_topics::kLeftCartesianExecutionTopic,
      execution_topics::kRightCartesianExecutionTopic};
  const std::array<const char *, 2> ik_topics{
      state_topics::kLeftCartesianIkTopic,
      state_topics::kRightCartesianIkTopic};
  for (std::size_t i = 0; i < 2; ++i) {
    // Keyboard input is already a base-frame TCP goal; no further
    // normalization.
    batch.poses.push_back(
        {input_topics[i], "base_link", pv(s.draft_tcp[i]), stamp});
    batch.poses.push_back(
        {goal_topics[i], "base_link", pv(s.draft_tcp[i]), stamp});
    batch.poses.push_back(
        {execution_pose_topics[i], "base_link", pv(s.execution_tcp[i]), stamp});
    if (has_ik)
      batch.poses.push_back(
          {ik_topics[i], "base_link", pv(s.attempt->ik_tcp[i]), stamp});
  }
  if (s.attempt)
    batch.poses.push_back({s.attempt->side == ArmSide::Left
                               ? planning_topics::kLeftCartesianReferenceTopic
                               : planning_topics::kRightCartesianReferenceTopic,
                           "base_link", pv(s.attempt->target), stamp});
  if (s.attempt && s.attempt->ik.pose_constraint) {
    const auto &c = *s.attempt->ik.pose_constraint;
    batch.poses.push_back({"/joint_path/held_tcp", "base_link",
                           pv(c.model_root_T_target), stamp});
  }
  Json::Value scene;
  scene["entities"] = Json::arrayValue;
  scene["deletions"] = Json::arrayValue;
  const auto refresh = o.scene_refresh->load();
  const bool snapshot = !initialized_ || refresh != refresh_;
  const bool changed = snapshot || s.attempt != attempt_;
  Json::Value timestamp;
  timestamp["sec"] = Json::UInt64(stamp / 1000000000);
  timestamp["nsec"] = Json::UInt(stamp % 1000000000);
  const auto erase = [&](const std::string &id) {
    Json::Value deletion;
    deletion["type"] = 0;
    deletion["id"] = id;
    deletion["timestamp"] = timestamp;
    scene["deletions"].append(deletion);
  };
  if (changed) {
    for (const auto &id : path_entities_)
      erase(id);
    path_entities_.clear();
  }
  bool path_entity = false;
  const auto entity = [&](const std::string &id) {
    Json::Value e;
    e["id"] = id;
    e["frame_id"] = "base_link";
    e["timestamp"] = timestamp;
    if (path_entity)
      path_entities_.push_back(id);
    e["lifetime"]["sec"] = 0;
    e["lifetime"]["nsec"] = 0;
    e["frame_locked"] = false;
    for (const char *k : {"metadata", "arrows", "cubes", "spheres", "cylinders",
                          "lines", "triangles", "texts", "models"})
      e[k] = Json::arrayValue;
    return e;
  };
  if (snapshot)
    for (const auto &box : o.scene["obstacles"]) {
      auto e = entity(box["id"].asString());
      Json::Value cube;
      Pose p = Pose::Identity();
      for (int k = 0; k < 3; ++k)
        p.translation()[k] = box["xyz"][k].asDouble();
      cube["pose"] = pose(p);
      cube["size"] =
          vector(box["size"][0].asDouble(), box["size"][1].asDouble(),
                 box["size"][2].asDouble());
      cube["color"] = color(1, .45, .1, .55);
      e["cubes"].append(cube);
      Json::Value label;
      p.translation().z() += box["size"][2].asDouble() * .5 + .04;
      label["pose"] = pose(p);
      label["billboard"] = true;
      label["font_size"] = 14;
      label["scale_invariant"] = true;
      label["color"] = color(1, .7, .3, 1);
      label["text"] = box["id"].asString() +
                      " [m]: " + std::to_string(box["size"][0].asDouble()) +
                      " x " + std::to_string(box["size"][1].asDouble()) +
                      " x " + std::to_string(box["size"][2].asDouble());
      e["texts"].append(label);
      scene["entities"].append(e);
    }
  const auto line = [&](const char *id,
                        const std::vector<std::array<double, 3>> &points,
                        Json::Value c) {
    if (points.size() < 2)
      return;
    auto e = entity(id);
    Json::Value l;
    l["type"] = 0;
    l["pose"] = pose(Pose::Identity());
    l["thickness"] = .006;
    l["scale_invariant"] = false;
    l["color"] = c;
    l["colors"] = Json::arrayValue;
    l["indices"] = Json::arrayValue;
    l["points"] = Json::arrayValue;
    for (const auto &p : points)
      l["points"].append(vector(p[0], p[1], p[2]));
    e["lines"].append(l);
    scene["entities"].append(e);
  };
  path_entity = true;
  if (changed && s.attempt) {
    line("joint-direct", s.attempt->direct_curve,
         s.attempt->motion.direct.valid ? color(.6, .6, .6, .7)
                                        : color(1, .15, .15, .8));
    if (s.attempt->motion.accepted) {
      line("planned-path", s.attempt->path_curve, color(0, .8, 1, 1));
      for (const auto waypoint :
           s.attempt->motion.timing.stop_waypoint_indices) {
        const std::size_t i = waypoint ? waypoint * 81 - 1 : 0;
        const auto &p = s.attempt->path_curve.at(i);
        auto e = entity("waypoint-" + std::to_string(i));
        Json::Value sphere;
        Pose t = Pose::Identity();
        t.translation() = Eigen::Vector3d(p[0], p[1], p[2]);
        sphere["pose"] = pose(t);
        sphere["size"] = vector(.025, .025, .025);
        sphere["color"] = color(0, .8, 1, 1);
        e["spheres"].append(sphere);
        scene["entities"].append(e);
      }
    }
  }
  path_entity = false;
  initialized_ = true;
  refresh_ = refresh;
  attempt_ = s.attempt;
  if (s.trail.size() < 2)
    erase("executed-trace");
  line("executed-trace", s.trail, color(.1, 1, .3, 1));
  batch.encoded_messages.push_back(
      encoded("/joint_path/scene", "foxglove.SceneUpdate", scene, stamp));
  batch.encoded_messages.push_back(encoded(
      "/joint_path/status", "mcl.JointPathStatus", diagnosticsJson(s), stamp));
  return batch;
}
} // namespace motion_control_lab::joint_path_planning
