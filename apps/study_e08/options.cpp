#include "options.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
namespace study {
Options parse(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "--request /absolute/study_request.json\n";
    std::exit(0);
  }
  if (argc != 3 || std::string(argv[1]) != "--request")
    throw std::runtime_error("expected --request FILE");
  Options o;
  std::ifstream f(argv[2]);
  f >> o.request;
  o.input = o.request["input"];
  o.config = o.request["config"];
  o.output = o.request["output_dir"].asString();
  o.method = o.request["identity"]["method_id"].asString();
  return o;
}
Eigen::VectorXd vector(const Json::Value &j) {
  Eigen::VectorXd v(j.size());
  for (unsigned i = 0; i < j.size(); ++i)
    v[i] = j[i].asDouble();
  return v;
}
Eigen::MatrixXd matrix(const Json::Value &j) {
  Eigen::MatrixXd v(j.size(), j[0].size());
  for (unsigned i = 0; i < j.size(); ++i)
    for (unsigned k = 0; k < j[i].size(); ++k)
      v(i, k) = j[i][k].asDouble();
  return v;
}
Json::Value json(const Eigen::VectorXd &v) {
  Json::Value j(Json::arrayValue);
  for (int i = 0; i < v.size(); ++i)
    j.append(v[i]);
  return j;
}
Json::Value jsonMatrix(const Eigen::MatrixXd &v) {
  Json::Value j(Json::arrayValue);
  for (int i = 0; i < v.rows(); ++i) {
    Json::Value row(Json::arrayValue);
    for (int k = 0; k < v.cols(); ++k)
      row.append(v(i, k));
    j.append(row);
  }
  return j;
}
void write(const std::string &p, const Json::Value &j) {
  std::ofstream f(p);
  f.exceptions(std::ios::badbit | std::ios::failbit);
  f << j;
}
long long now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
} // namespace study
