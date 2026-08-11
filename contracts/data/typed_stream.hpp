#pragma once

#include "contracts/data/data_error.hpp"

#include <string>
#include <utility>
#include <vector>

namespace motion_control_lab::data
{

template<typename T>
struct TypedStream
{
  std::string logical_name;
  std::string decoder_id;
  std::vector<T> samples;
  std::vector<Diagnostic> diagnostics;
};

template<typename T>
struct DecodedSample
{
  T sample;
  std::vector<Diagnostic> diagnostics;
};

}  // namespace motion_control_lab::data
