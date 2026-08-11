#pragma once

#include "adapters/data/decoder/decoder_registry.hpp"
#include "contracts/data/joint_state_sample.hpp"

namespace motion_control_lab::data
{

class Ros2JointStateCdrDecoder final : public RecordDecoder
{
public:
  const std::string & id() const override;
  std::type_index outputType() const override;
  bool supports(const StreamDescriptor & stream) const override;
  AnyDecodedSample decode(const EncodedRecord & record) const override;
};

}  // namespace motion_control_lab::data
