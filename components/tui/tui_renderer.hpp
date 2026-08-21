#pragma once

#include "contracts/input/input_contract.hpp"
#include "contracts/presentation/tui_document.hpp"

#include <cstddef>

namespace motion_control_lab
{

class TuiRenderer
{
public:
  explicit TuiRenderer(bool enabled);

  bool enabled() const noexcept;
  bool handleNavigation(const KeyEvent & event);
  void render(const TuiDocument & document);

private:
  bool enabled_{false};
  std::size_t page_index_{0};
  std::size_t page_count_{1};
  std::size_t scroll_offset_{0};
  bool show_help_{false};
};

} // namespace motion_control_lab
