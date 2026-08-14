#pragma once

#include "adapters/data/source/data_source.hpp"
#include "contracts/data/pose_sample.hpp"
#include "contracts/data/typed_stream.hpp"

namespace motion_control_lab::data
{

class Ros2PoseStampedCdrDecoder final
{
public:
  using Sample = StampedPose;

  const std::string & id() const;
  bool supports(const StreamDescriptor & stream) const;
  DecodedSample<Sample> decode(const EncodedRecord & record) const;
};

}  // namespace motion_control_lab::data
