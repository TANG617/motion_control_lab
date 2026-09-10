#pragma once
#include "options.hpp"
#include <memory>
namespace study {
class Solver {
public:
  explicit Solver(const Options &);
  ~Solver();
  Json::Value solve(const Json::Value &, const Json::Value &, int);

private:
  class Impl;
  std::unique_ptr<Impl> impl;
};
} // namespace study
