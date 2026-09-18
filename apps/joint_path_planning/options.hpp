#pragma once
#include "components/robot/r1/r1_robot_config.hpp"
#include "contracts/input/input_contract.hpp"
#include "motion_control_core/model/types.hpp"
#include <atomic>
#include <filesystem>
#include <json/json.h>
#include <memory>
#include <string>
namespace motion_control_lab::joint_path_planning {
namespace mcc = motion_control::core;
inline constexpr double kIkProgressWeight = 100.0;
inline constexpr double kIkPostureWeight = 10.0;
inline constexpr const char *kIkPolicy = "scaled-pose-initial-posture-v1";
struct Options {
  std::shared_ptr<std::atomic<std::uint64_t>> scene_refresh{
      std::make_shared<std::atomic<std::uint64_t>>(0)};
  std::filesystem::path config, urdf, output;
  Json::Value scene;
  Json::Value argv{Json::arrayValue};
  std::string side{"left"}, host{"127.0.0.1"};
  std::string planning_mode{"whole-body"};
  std::string timing_mode{"straight-through"};
  std::string smooth_validation{"full"};
  int port{8765};
  bool headless{false}, realtime{false}, viz{true}, record{true};
  bool help{false}, describe{false}, dump{false};
  double simplification_budget{1.0};
  double smoothing_budget{5.0}, smoothing_revolute_deviation{.005};
  double budget{50.0}, period{0.01}, acceleration{2.0}, jerk{10.0};
  unsigned seed{42};
  Json::Value goal;
  std::string launcher{"native"};
};
Options parseOptions(int argc, char **argv);
Json::Value resolved(const Options &);
Json::Value readJson(const std::filesystem::path &);
void writeJson(const std::filesystem::path &, const Json::Value &);
void require(const mcc::Status &);
} // namespace motion_control_lab::joint_path_planning
