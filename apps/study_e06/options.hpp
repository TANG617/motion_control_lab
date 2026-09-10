#pragma once
#include <Eigen/Core>
#include <json/json.h>
#include <string>
namespace study {
struct Options {
  Json::Value request, input, config;
  std::string output, method;
};
Options parse(int argc, char **argv);
Eigen::VectorXd vector(const Json::Value &j);
Eigen::MatrixXd matrix(const Json::Value &j);
Json::Value json(const Eigen::VectorXd &v);
Json::Value jsonMatrix(const Eigen::MatrixXd &v);
void write(const std::string &path, const Json::Value &j);
long long now();
} // namespace study
