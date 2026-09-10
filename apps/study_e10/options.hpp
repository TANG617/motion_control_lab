#pragma once
#include <json/json.h>
#include <string>
namespace study_e10 {
struct Options {
  Json::Value request, config, input;
  std::string output;
};
Options parseOptions(int argc, char **argv);
void writeJson(const std::string &path, const Json::Value &value);
long long nowNs();
double threadCpuSeconds();
} // namespace study_e10
