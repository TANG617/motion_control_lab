#pragma once

#include "adapters/data/source/data_source.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace motion_control_lab::data
{

class CsvSource final : public DataSource
{
public:
  explicit CsvSource(
    std::filesystem::path path,
    std::vector<std::string> logical_stream_names);

  const SourceCatalog & catalog() const override;
  std::unique_ptr<SourceCursor> select(const SourceSelector & selector) const override;

private:
  std::filesystem::path path_;
  SourceCatalog catalog_;
};

}  // namespace motion_control_lab::data
