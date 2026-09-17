#include "visualization.hpp"
#include <sstream>
#include <stdexcept>
using namespace motion_control_lab::joint_path_planning;
Json::Value scene(const motion_control::viz::RenderBatch &batch) {
  for (const auto &sample : batch.encoded_messages)
    if (sample.channel == "/joint_path/scene") {
      Json::Value value;
      std::istringstream in(
          std::string(reinterpret_cast<const char *>(sample.data.data()),
                      sample.data.size()));
      in >> value;
      return value;
    }
  throw std::runtime_error("missing scene");
}
void expect(bool value) {
  if (!value)
    throw std::runtime_error("scene update contract failed");
}
int main(int argc, char **argv) {
  if (argc != 3)
    return 1;
  Options o;
  o.scene = readJson(argv[2]);
  mcc::RobotModelDescription md;
  md.urdf_path = argv[1];
  md.kinematics_reference_frame = "base_link";
  std::shared_ptr<const mcc::RobotModel> model;
  require(mcc::RobotModel::load(md, model));
  Snapshot s;
  s.state = initialState(*model, o);
  Visualization renderer;
  auto first = scene(renderer.render(s, o, *model, 1));
  expect(first["entities"].size() == o.scene["obstacles"].size());
  expect(scene(renderer.render(s, o, *model, 2))["entities"].empty());
  auto a = std::make_shared<Attempt>();
  a->id = 1;
  a->motion.accepted = true;
  a->motion.timing.stop_waypoint_indices = {0, 1};
  a->path_curve.resize(81);
  a->direct_curve.resize(81);
  s.attempt = a;
  auto accepted = scene(renderer.render(s, o, *model, 3));
  expect(accepted["entities"].size() == 4);
  expect(scene(renderer.render(s, o, *model, 4))["entities"].empty());
  ++*o.scene_refresh;
  auto subscribed = scene(renderer.render(s, o, *model, 5));
  expect(subscribed["entities"].size() == 4 + o.scene["obstacles"].size());
  s.attempt.reset();
  auto cleared = scene(renderer.render(s, o, *model, 6));
  expect(cleared["entities"].empty());
  expect(cleared["deletions"].size() == 5);
  for (const auto &deletion : cleared["deletions"])
    expect(deletion["type"].asInt() == 0 && !deletion["id"].asString().empty());
}
