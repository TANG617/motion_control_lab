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
    e.translation() << .2, .1, .2;
    w.translation() << .4, .2, 0;
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
    auto straight_elbow=s, straight_wrist=s;
    straight_elbow.translation().z()=.2;
    straight_wrist.translation().z()=.4;
    const auto straight=ElbowGeometry::fromFk(s,straight_elbow,straight_wrist,straight_wrist);
    require(std::abs(straight.upper_length-.2)<1e-12,"inactive straight arm still provides geometry");
    rejected=false;
    try { straight.target(s.translation(),straight_wrist,1,0); }
    catch(const std::runtime_error &) { rejected=true; }
    require(rejected,"enabled straight arm must reject a degenerate elbow circle");
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
      reference.initialize({ee,ee});
    } catch (const std::runtime_error &error) {
      rejected = std::string(error.what()).find("must differ") != std::string::npos;
    }
    std::ifstream preserved(input);
    std::string contents;
    std::getline(preserved, contents);
    std::filesystem::remove_all(directory);
    require(rejected && contents == "preserve original input",
            "recorded output must not truncate its input");
    // Legacy records fail even when they carry otherwise valid angles.
    std::filesystem::create_directory(directory);
    { std::ofstream file(input); file << R"({"cos":1,"sin":0})" << '\n'; }
    { std::ofstream manifest(input.string()+".model.json"); manifest << "{}"; }
    recorded_options.record_path.clear();
    rejected=false;
    try { ElbowReference reference(recorded_options,1000); reference.initialize({ee,ee}); }
    catch(const std::runtime_error &error) { rejected=std::string(error.what()).find("legacy single-arm")!=std::string::npos; }
    require(rejected,"legacy record rejection");
    std::filesystem::remove_all(directory);
    auto singular=ee;
    singular.translation()=Eigen::Vector3d(.4,0,.1);
    rejected=false;
    try { g.target(s.translation(),singular,1,0); }
    catch(const std::runtime_error &) { rejected=true; }
    require(rejected,"singular axis has no alternate-axis fallback");
    RobotOptions robot;
    ArmPoses poses{ee,ee};
    poses[1].translation().y()=-.2;
    Eigen::Isometry3d torso=Eigen::Isometry3d::Identity();
    torso.linear()=Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    torso.translation()<<.1,.2,.3;
    const auto wrists=referenceWrists(robot,poses,torso);
    const auto ref=referenceFromBase(robot,torso);
    for(int a=0;a<2;++a) {
      require((ref.inverse()*wrists[a].translation()-poses[a]*robot.wrist_in_ee).norm()<1e-12,"torso/wrist point roundtrip");
      require((ref.linear().transpose()*wrists[a].linear()*robot.ee_to_model_wrist_axes[a].transpose()-poses[a].linear()).norm()<1e-12,"wrist rotation roundtrip");
      require((robot.ee_to_model_wrist_axes[a].transpose()*robot.ee_to_model_wrist_axes[a]-Eigen::Matrix3d::Identity()).norm()<1e-12,"calibrated proper axes");
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
