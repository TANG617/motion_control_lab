#pragma once

#include "adapters/data/decoder/decoder_registry.hpp"
#include "contracts/data/joint_state_sample.hpp"

#include <optional>
#include <string>

namespace motion_control_lab::data
{

struct CsvJointStateMapping
{
  std::string decoder_id{"csv_joint_state"};
  std::string timestamp_column{"timestamp_ns"};
  TimestampSource timestamp_target{TimestampSource::ConfiguredColumn};
  std::optional<std::string> header_stamp_column;
  std::optional<std::string> log_time_column;
  std::optional<std::string> publish_time_column;
  std::string names_column{"names"};
  std::string positions_column{"positions"};
  std::optional<std::string> velocities_column{"velocities"};
  char vector_delimiter{';'};
};

class CsvJointStateDecoder final : public RecordDecoder
{
public:
  explicit CsvJointStateDecoder(CsvJointStateMapping mapping);

  const std::string & id() const override;
  std::type_index outputType() const override;
  bool supports(const StreamDescriptor & stream) const override;
  AnyDecodedSample decode(const EncodedRecord & record) const override;

private:
  CsvJointStateMapping mapping_;
};

}  // namespace motion_control_lab::data
