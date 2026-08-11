#pragma once

#include "contracts/data/sample_time.hpp"

#include <Eigen/Geometry>

#include <string>

namespace motion_control_lab::data
{

struct StampedPose
{
  SampleTime time;
  std::string frame_id;
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
};

}  // namespace motion_control_lab::data
