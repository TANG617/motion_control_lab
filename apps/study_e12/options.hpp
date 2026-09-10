#pragma once
#include <Eigen/Geometry>
#include <filesystem>
#include <json/json.h>
#include <motion_control_core/motion_control_core.hpp>
namespace study_e12 {
namespace mcc = motion_control::core;
struct Options {
  Json::Value request, input, config;
  std::filesystem::path output;
};
Options parse(int argc, char **argv);
std::vector<double> numbers(const Json::Value &);
std::vector<std::string> strings(const Json::Value &);
Json::Value array(const std::vector<double> &);
mcc::Pose pose(const Json::Value &);
Json::Value jsonPose(const mcc::Pose &);
void write(const std::filesystem::path &, const Json::Value &);
void require(const mcc::Status &);
} // namespace study_e12
