#pragma once

#include "adapters/data/source/data_source.hpp"

#include <filesystem>
#include <memory>

namespace motion_control_lab::data
{

class McapSource final : public DataSource
{
public:
  explicit McapSource(std::filesystem::path path);
  ~McapSource() override;

  McapSource(McapSource &&) noexcept;
  McapSource & operator=(McapSource &&) noexcept;
  McapSource(const McapSource &) = delete;
  McapSource & operator=(const McapSource &) = delete;

  const SourceCatalog & catalog() const override;
  std::unique_ptr<SourceCursor> select(const SourceSelector & selector) const override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace motion_control_lab::data
