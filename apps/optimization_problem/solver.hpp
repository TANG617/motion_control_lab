#pragma once
#include <json/json.h>
namespace motion_control_lab::optimization_problem {
Json::Value solve(const Json::Value &problem, const Json::Value &options);
}
