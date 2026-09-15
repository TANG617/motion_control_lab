#pragma once
#include "motion_control_lab/execution_request.hpp"
#include <Eigen/Geometry>
#include <filesystem>
#include <json/json.h>
#include <motion_control_core/motion_control_core.hpp>
namespace motion_control_lab::planned_kinematics_step {
namespace mcc = motion_control::core;
struct Options {
  execution::Request request;
  Json::Value input, config, original_argv;
  std::filesystem::path output;
  bool dump_only{};
  std::int64_t period_ns{}, duration_ns{};
  int planned_releases{};
};
Json::Value capabilities();
Options parse(int argc, char **argv);
void prepareInput(Options &);
std::vector<double> numbers(const Json::Value &);
std::vector<std::string> strings(const Json::Value &);
Json::Value array(const std::vector<double> &);
mcc::Pose pose(const Json::Value &);
Json::Value jsonPose(const mcc::Pose &);
void write(const std::filesystem::path &, const Json::Value &);
void require(const mcc::Status &);
} // namespace motion_control_lab::planned_kinematics_step
