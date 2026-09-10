#include "options.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
namespace study_e12 {
Options parse(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "mcl_study_e12 --request PATH\n";
    std::exit(0);
  }
  if (argc != 3 || std::string(argv[1]) != "--request")
    throw std::runtime_error("expected --request PATH");
  Options o;
  std::ifstream f(argv[2]);
  f >> o.request;
  o.input = o.request["input"];
  o.config = o.request["config"];
  o.output = o.request["output_dir"].asString();
  return o;
}
std::vector<double> numbers(const Json::Value &v) {
  std::vector<double> r;
  for (auto &x : v)
    r.push_back(x.asDouble());
  return r;
}
std::vector<std::string> strings(const Json::Value &v) {
  std::vector<std::string> r;
  for (auto &x : v)
    r.push_back(x.asString());
  return r;
}
Json::Value array(const std::vector<double> &v) {
  Json::Value r(Json::arrayValue);
  for (double x : v)
    r.append(x);
  return r;
}
mcc::Pose pose(const Json::Value &v) {
  auto p = mcc::Pose::Identity();
  for (int i = 0; i < 3; i++) {
    p.translation()[i] = v["position"][i].asDouble();
    for (int j = 0; j < 3; j++)
      p.linear()(i, j) = v["rotation"][i][j].asDouble();
  }
  return p;
}
Json::Value jsonPose(const mcc::Pose &p) {
  Json::Value v;
  for (int i = 0; i < 3; i++) {
    v["position"].append(p.translation()[i]);
    for (int j = 0; j < 3; j++)
      v["rotation"][i].append(p.linear()(i, j));
  }
  return v;
}
void write(const std::filesystem::path &p, const Json::Value &v) {
  std::ofstream f(p);
  f.exceptions(std::ios::badbit | std::ios::failbit);
  f << v << '\n';
}
void require(const mcc::Status &s) {
  if (!s.ok())
    throw std::runtime_error(s.message);
}
} // namespace study_e12
