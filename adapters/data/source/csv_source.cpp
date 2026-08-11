#include "adapters/data/source/csv_source.hpp"

#include <fstream>
#include <set>
#include <utility>

namespace motion_control_lab::data
{
namespace
{

std::vector<std::string> parseCsvRecord(std::istream & input, std::size_t row_number)
{
  std::vector<std::string> fields;
  std::string field;
  bool in_quotes = false;
  bool consumed = false;

  while (true) {
    const int next = input.get();
    if (next == EOF) {
      if (in_quotes) {
        throw DataError(
                DataErrorCode::InvalidFormat,
                "unterminated quoted CSV field at row " + std::to_string(row_number));
      }
      if (!consumed && field.empty() && fields.empty()) {
        return {};
      }
      fields.push_back(field);
      return fields;
    }

    consumed = true;
    const char value = static_cast<char>(next);
    if (in_quotes) {
      if (value == '"') {
        if (input.peek() == '"') {
          input.get();
          field.push_back('"');
        } else {
          in_quotes = false;
        }
      } else {
        field.push_back(value);
      }
      continue;
    }

    if (value == '"') {
      if (!field.empty()) {
        throw DataError(
                DataErrorCode::InvalidFormat,
                "quote appeared inside an unquoted CSV field at row " +
                std::to_string(row_number));
      }
      in_quotes = true;
    } else if (value == ',') {
      fields.push_back(field);
      field.clear();
    } else if (value == '\n') {
      fields.push_back(field);
      return fields;
    } else if (value == '\r') {
      if (input.peek() == '\n') {
        input.get();
      }
      fields.push_back(field);
      return fields;
    } else {
      field.push_back(value);
    }
  }
}

class CsvCursor final : public SourceCursor
{
public:
  CsvCursor(std::filesystem::path path, StreamDescriptor stream)
    : input_(std::move(path)), stream_(std::move(stream))
  {
    if (!input_) {
      throw DataError(DataErrorCode::Io, "failed to open CSV source");
    }
    const auto header = parseCsvRecord(input_, 1);
    if (header.size() != stream_.columns.size()) {
      throw DataError(DataErrorCode::InvalidFormat, "CSV header changed after inspection");
    }
  }

  bool next(EncodedRecord & record) override
  {
    auto values = parseCsvRecord(input_, row_number_);
    if (values.empty()) {
      return false;
    }
    if (values.size() != stream_.columns.size()) {
      throw DataError(
              DataErrorCode::InvalidFormat,
              "CSV row " + std::to_string(row_number_) + " has " +
              std::to_string(values.size()) + " values; expected " +
              std::to_string(stream_.columns.size()));
    }
    record = RowRecord{stream_, row_number_, std::move(values)};
    ++row_number_;
    return true;
  }

private:
  std::ifstream input_;
  StreamDescriptor stream_;
  std::size_t row_number_{2};
};

}  // namespace

CsvSource::CsvSource(
  std::filesystem::path path,
  std::vector<std::string> logical_stream_names)
  : path_(std::move(path))
{
  if (logical_stream_names.empty()) {
    throw DataError(DataErrorCode::InvalidArgument, "CSV source requires a logical stream name");
  }
  std::ifstream input(path_);
  if (!input) {
    throw DataError(DataErrorCode::Io, "failed to open CSV: " + path_.string());
  }
  const auto header = parseCsvRecord(input, 1);
  if (header.empty()) {
    throw DataError(DataErrorCode::InvalidFormat, "CSV header is empty: " + path_.string());
  }
  std::set<std::string> unique_columns;
  std::vector<ColumnDescriptor> columns;
  columns.reserve(header.size());
  for (std::size_t index = 0; index < header.size(); ++index) {
    if (header[index].empty() || !unique_columns.insert(header[index]).second) {
      throw DataError(
              DataErrorCode::InvalidFormat,
              "CSV column names must be non-empty and unique");
    }
    columns.push_back({index, header[index]});
  }

  std::set<std::string> unique_streams;
  catalog_.source_path = std::filesystem::absolute(path_).lexically_normal().string();
  catalog_.metadata["column_count"] = std::to_string(columns.size());
  for (auto & name : logical_stream_names) {
    if (name.empty() || !unique_streams.insert(name).second) {
      throw DataError(
              DataErrorCode::InvalidArgument,
              "CSV logical stream names must be non-empty and unique");
    }
    StreamDescriptor descriptor;
    descriptor.format = PhysicalFormat::Csv;
    descriptor.logical_name = std::move(name);
    descriptor.columns = columns;
    catalog_.streams.push_back(std::move(descriptor));
  }
}

const SourceCatalog & CsvSource::catalog() const
{
  return catalog_;
}

std::unique_ptr<SourceCursor> CsvSource::select(const SourceSelector & selector) const
{
  if (selector.start_log_time_ns.has_value() || selector.end_log_time_ns.has_value()) {
    throw DataError(
            DataErrorCode::InvalidArgument,
            "CSV source cannot apply a physical log-time filter; filter typed samples instead");
  }
  for (const auto & stream : catalog_.streams) {
    if (stream.logical_name == selector.logical_name) {
      return std::make_unique<CsvCursor>(path_, stream);
    }
  }
  throw DataError(
          DataErrorCode::InvalidArgument,
          "CSV logical stream does not exist: " + selector.logical_name);
}

}  // namespace motion_control_lab::data
