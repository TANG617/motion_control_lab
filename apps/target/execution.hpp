#pragma once
#include "options.hpp"
#include <json/json.h>
namespace motion_control_lab::target {
Json::Value capabilities();
Json::Value resolvedOptions(const AppOptions &options);
int executePublic(int argc, char **argv);
bool publicExecution(int argc, char **argv);
} // namespace motion_control_lab::target
