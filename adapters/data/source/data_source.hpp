#pragma once

#include "contracts/data/data_error.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace motion_control_lab::data
{

enum class PhysicalFormat
{
  Mcap,
  Csv
};

struct ColumnDescriptor
{
  std::size_t index{};
  std::string name;
};

struct StreamDescriptor
{
  PhysicalFormat format{PhysicalFormat::Mcap};
  std::string logical_name;
  std::optional<std::uint16_t> channel_id;
  std::string topic;
  std::string schema_name;
  std::string schema_encoding;
  std::string message_encoding;
  std::vector<std::byte> schema_data;
  std::vector<ColumnDescriptor> columns;
  std::map<std::string, std::string> metadata;
};

struct SourceCatalog
{
  std::string source_path;
  std::map<std::string, std::string> metadata;
  std::vector<StreamDescriptor> streams;
};

struct SourceSelector
{
  std::string logical_name;
  std::optional<std::uint64_t> start_log_time_ns;
  std::optional<std::uint64_t> end_log_time_ns;
};

struct BinaryRecord
{
  StreamDescriptor stream;
  std::uint64_t sequence{};
  std::uint64_t log_time_ns{};
  std::uint64_t publish_time_ns{};
  std::vector<std::byte> payload;
};

struct RowRecord
{
  StreamDescriptor stream;
  std::size_t row_number{};
  std::vector<std::string> values;

  const std::string & at(const std::string & column) const;
  bool has(const std::string & column) const;
};

using EncodedRecord = std::variant<BinaryRecord, RowRecord>;

class SourceCursor
{
public:
  virtual ~SourceCursor() = default;
  virtual bool next(EncodedRecord & record) = 0;
};

class DataSource
{
public:
  virtual ~DataSource() = default;
  virtual const SourceCatalog & catalog() const = 0;
  virtual std::unique_ptr<SourceCursor> select(const SourceSelector & selector) const = 0;
};

}  // namespace motion_control_lab::data
