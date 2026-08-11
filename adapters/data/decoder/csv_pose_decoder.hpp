#pragma once

#include "adapters/data/decoder/decoder_registry.hpp"
#include "contracts/data/pose_sample.hpp"

#include <optional>
#include <string>

namespace motion_control_lab::data
{

struct CsvPoseMapping
{
  std::string decoder_id{"csv_pose"};
  std::string timestamp_column{"timestamp_ns"};
  TimestampSource timestamp_target{TimestampSource::ConfiguredColumn};
  std::optional<std::string> header_stamp_column;
  std::optional<std::string> log_time_column;
  std::optional<std::string> publish_time_column;
  std::optional<std::string> frame_id_column{"frame_id"};
  std::string fixed_frame_id;
  std::string x_column{"x"};
  std::string y_column{"y"};
  std::string z_column{"z"};
  std::string qx_column{"qx"};
  std::string qy_column{"qy"};
  std::string qz_column{"qz"};
  std::string qw_column{"qw"};
};

class CsvPoseDecoder final : public RecordDecoder
{
public:
  explicit CsvPoseDecoder(CsvPoseMapping mapping);

  const std::string & id() const override;
  std::type_index outputType() const override;
  bool supports(const StreamDescriptor & stream) const override;
  AnyDecodedSample decode(const EncodedRecord & record) const override;

private:
  CsvPoseMapping mapping_;
};

}  // namespace motion_control_lab::data
