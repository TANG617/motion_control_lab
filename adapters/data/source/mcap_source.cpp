#include "adapters/data/source/mcap_source.hpp"

#include <mcap/reader.hpp>

#include <algorithm>
#include <limits>
#include <utility>

namespace motion_control_lab::data
{
namespace
{

void requireMcapStatus(const mcap::Status & status, const std::string & operation)
{
  if (!status.ok()) {
    throw DataError(DataErrorCode::InvalidFormat, operation + ": " + status.message);
  }
}

StreamDescriptor makeDescriptor(
  const mcap::Channel & channel,
  const mcap::SchemaPtr & schema)
{
  StreamDescriptor descriptor;
  descriptor.format = PhysicalFormat::Mcap;
  descriptor.logical_name = channel.topic;
  descriptor.channel_id = channel.id;
  descriptor.topic = channel.topic;
  descriptor.message_encoding = channel.messageEncoding;
  descriptor.metadata.insert(channel.metadata.begin(), channel.metadata.end());
  if (schema) {
    descriptor.schema_name = schema->name;
    descriptor.schema_encoding = schema->encoding;
    descriptor.schema_data = schema->data;
  }
  return descriptor;
}

class VectorCursor final : public SourceCursor
{
public:
  explicit VectorCursor(std::vector<EncodedRecord> records)
    : records_(std::move(records))
  {
  }

  bool next(EncodedRecord & record) override
  {
    if (next_ >= records_.size()) {
      return false;
    }
    record = std::move(records_[next_]);
    ++next_;
    return true;
  }

private:
  std::vector<EncodedRecord> records_;
  std::size_t next_{};
};

}  // namespace

struct McapSource::Impl
{
  std::filesystem::path path;
  SourceCatalog catalog;
};

McapSource::McapSource(std::filesystem::path path)
  : impl_(std::make_unique<Impl>())
{
  impl_->path = std::move(path);
  mcap::McapReader reader;
  requireMcapStatus(reader.open(impl_->path.string()), "failed to open MCAP");
  std::vector<std::string> problems;
  const auto summary_status = reader.readSummary(
    mcap::ReadSummaryMethod::AllowFallbackScan,
    [&](const mcap::Status & problem) {problems.push_back(problem.message);});
  requireMcapStatus(summary_status, "failed to inspect MCAP summary");

  impl_->catalog.source_path =
    std::filesystem::absolute(impl_->path).lexically_normal().string();
  if (reader.header().has_value()) {
    impl_->catalog.metadata["profile"] = reader.header()->profile;
    impl_->catalog.metadata["library"] = reader.header()->library;
  }
  impl_->catalog.metadata["summary_problem_count"] = std::to_string(problems.size());
  impl_->catalog.metadata["chunk_count"] = std::to_string(reader.chunkIndexes().size());

  auto channels = reader.channels();
  impl_->catalog.streams.reserve(channels.size());
  for (const auto & [channel_id, channel] : channels) {
    (void)channel_id;
    impl_->catalog.streams.push_back(makeDescriptor(*channel, reader.schema(channel->schemaId)));
  }
  std::sort(
    impl_->catalog.streams.begin(), impl_->catalog.streams.end(),
    [](const StreamDescriptor & left, const StreamDescriptor & right) {
      return left.channel_id.value_or(0) < right.channel_id.value_or(0);
    });

  for (const auto & [name, index] : reader.metadataIndexes()) {
    mcap::Record record{};
    const auto record_status =
      mcap::McapReader::ReadRecord(*reader.dataSource(), index.offset, &record);
    if (!record_status.ok()) {
      continue;
    }
    mcap::Metadata metadata;
    const auto parse_status = mcap::McapReader::ParseMetadata(record, &metadata);
    if (!parse_status.ok()) {
      continue;
    }
    for (const auto & [key, value] : metadata.metadata) {
      impl_->catalog.metadata["metadata." + name + "." + key] = value;
    }
  }
  reader.close();
}

McapSource::~McapSource() = default;
McapSource::McapSource(McapSource &&) noexcept = default;
McapSource & McapSource::operator=(McapSource &&) noexcept = default;

const SourceCatalog & McapSource::catalog() const
{
  return impl_->catalog;
}

std::unique_ptr<SourceCursor> McapSource::select(const SourceSelector & selector) const
{
  bool found = false;
  for (const auto & stream : impl_->catalog.streams) {
    if (stream.logical_name == selector.logical_name) {
      found = true;
      break;
    }
  }
  if (!found) {
    throw DataError(
            DataErrorCode::InvalidArgument,
            "MCAP topic does not exist: " + selector.logical_name);
  }
  if (selector.start_log_time_ns.has_value() && selector.end_log_time_ns.has_value() &&
      selector.start_log_time_ns.value() >= selector.end_log_time_ns.value()) {
    throw DataError(DataErrorCode::InvalidArgument, "MCAP time range must be [start, end)");
  }

  mcap::McapReader reader;
  requireMcapStatus(reader.open(impl_->path.string()), "failed to open MCAP");
  mcap::ReadMessageOptions options;
  options.startTime = selector.start_log_time_ns.value_or(0);
  options.endTime = selector.end_log_time_ns.value_or(mcap::MaxTime);
  options.readOrder = mcap::ReadMessageOptions::ReadOrder::FileOrder;
  options.topicFilter = [topic = selector.logical_name](std::string_view candidate) {
      return candidate == topic;
    };

  std::vector<std::string> problems;
  std::vector<EncodedRecord> records;
  for (const auto & view : reader.readMessages(
      [&](const mcap::Status & problem) {problems.push_back(problem.message);}, options)) {
    if (view.channel->topic != selector.logical_name) {
      throw DataError(DataErrorCode::InvalidFormat, "MCAP reader violated topic filter");
    }
    BinaryRecord record;
    record.stream = makeDescriptor(*view.channel, view.schema);
    record.sequence = view.message.sequence;
    record.log_time_ns = view.message.logTime;
    record.publish_time_ns = view.message.publishTime;
    if (view.message.dataSize > 0) {
      if (view.message.data == nullptr) {
        throw DataError(DataErrorCode::InvalidFormat, "MCAP message payload pointer is null");
      }
      record.payload.assign(view.message.data, view.message.data + view.message.dataSize);
    }
    records.emplace_back(std::move(record));
  }
  reader.close();
  if (!problems.empty()) {
    throw DataError(
            DataErrorCode::InvalidFormat,
            "MCAP read reported " + std::to_string(problems.size()) +
            " problem(s); first: " + problems.front());
  }
  return std::make_unique<VectorCursor>(std::move(records));
}

}  // namespace motion_control_lab::data
