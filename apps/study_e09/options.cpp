#include "options.hpp"
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <stdexcept>
namespace study_e09 {
Options parseOptions(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "mcl_study_e09 --request /absolute/request.json\n";
    std::exit(0);
  }
  if (argc != 3 || std::string(argv[1]) != "--request")
    throw std::runtime_error("expected --request filename");
  Options o;
  std::ifstream stream(argv[2]);
  stream >> o.request;
  o.config = o.request["config"];
  o.input = o.request["input"];
  o.output = o.request["output_dir"].asString();
  return o;
}
void writeJson(const std::string &p, const Json::Value &v) {
  std::ofstream f(p);
  f << v << '\n';
}
long long nowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
double threadCpuSeconds() {
  timespec t{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
  return t.tv_sec + t.tv_nsec * 1e-9;
}
} // namespace study_e09
