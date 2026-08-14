#pragma once

#include "adapters/data/source/data_source.hpp"

#include <filesystem>

namespace motion_control_lab::data
{

class McapSource final : public DataSource
{
public:
  explicit McapSource(std::filesystem::path path);

  const SourceCatalog & catalog() const override;
  std::unique_ptr<SourceCursor> select(const SourceSelector & selector) const override;

private:
  std::filesystem::path path_;
  SourceCatalog catalog_;
};

}  // namespace motion_control_lab::data
