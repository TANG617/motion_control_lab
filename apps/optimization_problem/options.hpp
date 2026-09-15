#pragma once
#include <json/json.h>
namespace motion_control_lab::optimization_problem {
Json::Value resolveOptions(const Json::Value &config);
Json::Value capabilities();
} // namespace motion_control_lab::optimization_problem
