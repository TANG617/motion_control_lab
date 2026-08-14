#include "adapters/data/source/mcap_source.hpp"

#include <mcap/reader.hpp>

#include <algorithm>
#include <limits>
#include <optional>
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

class McapCursor final : public SourceCursor
{
public:
  McapCursor(std::filesystem::path path, const SourceSelector & selector)
    : topic_(selector.logical_name)
  {
    requireMcapStatus(reader_.open(path.string()), "failed to open MCAP");
    mcap::ReadMessageOptions options;
    options.startTime = selector.start_log_time_ns.value_or(0);
    options.endTime = selector.end_log_time_ns.value_or(mcap::MaxTime);
    options.readOrder = mcap::ReadMessageOptions::ReadOrder::FileOrder;
    options.topicFilter = [topic = topic_](std::string_view candidate) {
        return candidate == topic;
      };
    view_.emplace(reader_.readMessages(
        [this](const mcap::Status & problem) {problems_.push_back(problem.message);},
        options));
    iterator_.emplace(view_->begin());
    requireNoProblems();
  }

  bool next(EncodedRecord & record) override
  {
    requireNoProblems();
    if (*iterator_ == view_->end()) {
      return false;
    }

    const auto & message_view = **iterator_;
    if (message_view.channel->topic != topic_) {
      throw DataError(DataErrorCode::InvalidFormat, "MCAP reader violated topic filter");
    }
    BinaryRecord binary;
    binary.stream = makeDescriptor(*message_view.channel, message_view.schema);
    binary.sequence = message_view.message.sequence;
    binary.log_time_ns = message_view.message.logTime;
    binary.publish_time_ns = message_view.message.publishTime;
    if (message_view.message.dataSize > 0) {
      if (message_view.message.data == nullptr) {
        throw DataError(DataErrorCode::InvalidFormat, "MCAP message payload pointer is null");
      }
      binary.payload.assign(
        message_view.message.data,
        message_view.message.data + message_view.message.dataSize);
    }
    record = std::move(binary);
    ++*iterator_;
    requireNoProblems();
    return true;
  }

private:
  void requireNoProblems() const
  {
    if (!problems_.empty()) {
      throw DataError(
              DataErrorCode::InvalidFormat,
              "MCAP read reported " + std::to_string(problems_.size()) +
              " problem(s); first: " + problems_.front());
    }
  }

  std::string topic_;
  mcap::McapReader reader_;
  std::vector<std::string> problems_;
  std::optional<mcap::LinearMessageView> view_;
  std::optional<mcap::LinearMessageView::Iterator> iterator_;
};

}  // namespace

McapSource::McapSource(std::filesystem::path path)
  : path_(std::move(path))
{
  mcap::McapReader reader;
  requireMcapStatus(reader.open(path_.string()), "failed to open MCAP");
  std::vector<std::string> problems;
  const auto summary_status = reader.readSummary(
    mcap::ReadSummaryMethod::AllowFallbackScan,
    [&](const mcap::Status & problem) {problems.push_back(problem.message);});
  requireMcapStatus(summary_status, "failed to inspect MCAP summary");

  catalog_.source_path = std::filesystem::absolute(path_).lexically_normal().string();
  if (reader.header().has_value()) {
    catalog_.metadata["profile"] = reader.header()->profile;
    catalog_.metadata["library"] = reader.header()->library;
  }
  catalog_.metadata["summary_problem_count"] = std::to_string(problems.size());
  catalog_.metadata["chunk_count"] = std::to_string(reader.chunkIndexes().size());

  auto channels = reader.channels();
  catalog_.streams.reserve(channels.size());
  for (const auto & [channel_id, channel] : channels) {
    (void)channel_id;
    catalog_.streams.push_back(makeDescriptor(*channel, reader.schema(channel->schemaId)));
  }
  std::sort(
    catalog_.streams.begin(), catalog_.streams.end(),
    [](const StreamDescriptor & left, const StreamDescriptor & right) {
      return left.channel_id.value_or(0) < right.channel_id.value_or(0);
    });
  reader.close();
}

const SourceCatalog & McapSource::catalog() const
{
  return catalog_;
}

std::unique_ptr<SourceCursor> McapSource::select(const SourceSelector & selector) const
{
  bool found = false;
  for (const auto & stream : catalog_.streams) {
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

  return std::make_unique<McapCursor>(path_, selector);
}

}  // namespace motion_control_lab::data
