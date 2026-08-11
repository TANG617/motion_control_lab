#pragma once

#include "contracts/data/sample_time.hpp"

#include <string>
#include <vector>

namespace motion_control_lab::data
{

struct StampedJointState
{
  SampleTime time;
  std::vector<std::string> names;
  std::vector<double> positions;
  std::vector<double> velocities;
};

}  // namespace motion_control_lab::data
