#include "../elbow_reference.hpp"
#include <cmath>
#include <json/json.h>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>
using namespace motion_control_lab::hierarchical_kinematics_step;
void require(bool x, const char *m) {
  if (!x)
    throw std::runtime_error(m);
}
int main(int argc, char **argv) {
  try {
    Eigen::Isometry3d s = Eigen::Isometry3d::Identity(), e = s, w = s, ee = s;
    e.translation() << .2, .2, 0;
    w.translation() << .4, 0, 0;
    ee = w;
    ee.translation().z() = .1;
    auto g = ElbowGeometry::fromFk(s, e, w, ee);
    auto initial = g.angle(s.translation(), w.translation(), e.translation());
    require((g.target(s.translation(), ee, initial.x(), initial.y()) -
             e.translation())
                    .norm() < 1e-12,
            "FK swivel roundtrip");
    for (double phi = -3.14; phi < 3.15; phi += .02) {
      auto target = g.target(s.translation(), ee, std::cos(phi), std::sin(phi));
      require(std::abs((target - s.translation()).norm() - g.upper_length) <
                  1e-12,
              "upper arm length");
      require(std::abs((target - w.translation()).norm() - g.lower_length) <
                  1e-12,
              "lower arm length");
      auto angle = g.angle(s.translation(), w.translation(), target);
      require((angle - Eigen::Vector2d(std::cos(phi), std::sin(phi))).norm() <
                  1e-12,
              "angle roundtrip");
    }
    require((g.target(s.translation(), ee, std::cos(M_PI - 1e-7),
                      std::sin(M_PI - 1e-7)) -
             g.target(s.translation(), ee, std::cos(-M_PI + 1e-7),
                      std::sin(-M_PI + 1e-7)))
                    .norm() < 1e-6,
            "pi wrap continuity");
    bool rejected = false;
    auto unreachable = ee;
    unreachable.translation().x() = 5;
    try {
      g.target(s.translation(), unreachable, 1, 0);
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "unreachable geometry must fail");
    // Reject an output alias before opening it, including a symlink alias.
    const auto directory = std::filesystem::temp_directory_path() /
        ("mcl-elbow-record-test-" + std::to_string(::getpid()));
    std::filesystem::create_directory(directory);
    const auto input = directory / "input.jsonl";
    const auto alias = directory / "output.jsonl";
    { std::ofstream file(input); file << "preserve original input\n"; }
    std::filesystem::create_symlink(input, alias);
    ElbowReferenceOptions recorded_options;
    recorded_options.source = "recorded";
    recorded_options.recorded_path = input.string();
    recorded_options.record_path = alias.string();
    rejected = false;
    try {
      ElbowReference reference(recorded_options, 1000);
      reference.initialize(ee);
    } catch (const std::runtime_error &error) {
      rejected = std::string(error.what()).find("must differ") != std::string::npos;
    }
    std::ifstream preserved(input);
    std::string contents;
    std::getline(preserved, contents);
    std::filesystem::remove_all(directory);
    require(rejected && contents == "preserve original input",
            "recorded output must not truncate its input");
    // Old recordings omit the mirror flag; new rows preserve mode transitions.
    std::filesystem::create_directory(directory);
    {
      std::ofstream file(input);
      file << R"({"generation":1,"sequence":0,"sample_time_s":0,"cos":1,"sin":0,"inference_ms":0,"consume_time_s":0,"left_goal":[0,0,0,0,0,0,1],"right_goal":[0,0,0,0,0,0,1]})" << '\n';
      file << R"({"generation":1,"sequence":1,"sample_time_s":0.001,"cos":1,"sin":0,"inference_ms":0,"consume_time_s":0.001,"left_goal":[0,0,0,0,0,0,1],"right_goal":[0,0,0,0,0,0,1],"mirror_tcp_input":true})" << '\n';
    }
    recorded_options.record_path.clear();
    {
      ElbowReference reference(recorded_options, 1000);
      reference.initialize(ee, 1);
      reference.consume(0);
      require(!reference.recordedConsumption().mirror_tcp_input, "legacy mirror defaults off");
      reference.consume(.001);
      require(reference.recordedConsumption().mirror_tcp_input, "recorded mirror transition");
      reference.finish();
    }
    std::filesystem::remove_all(directory);
    if (argc == 2) {
      std::ifstream fixture(argv[1]); std::string line; std::getline(fixture, line);
      Json::Value first; std::istringstream input_row(line); input_row >> first;
      recorded_options.recorded_path = argv[1];
      ElbowReference reference(recorded_options, 1000);
      reference.initialize(ee, first["generation"].asUInt64());
      const auto initial = reference.consume(0);
      const auto paused = reference.consume(0);
      require(initial.cosine == paused.cosine && initial.sequence == paused.sequence, "recorded pause is unchanged");
      reference.consume(reference.recordedEndTime());
      bool exhausted = false;
      try { reference.consume(reference.recordedEndTime() + 1.); }
      catch (const std::runtime_error &error) { exhausted = std::string(error.what()).find("exhausted") != std::string::npos; }
      require(exhausted, "recorded exhaustion remains failure");
      reference.finish();
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
