#pragma once

#include "contracts/input/input_contract.hpp"

#include <string>
#include <vector>

namespace motion_control_lab
{

struct TerminalFrontendOptions
{
  bool input_enabled{true};
  bool alternate_screen{false};
};

class TerminalFrontend
{
public:
  explicit TerminalFrontend(TerminalFrontendOptions options);
  ~TerminalFrontend();

  TerminalFrontend(const TerminalFrontend &) = delete;
  TerminalFrontend & operator=(const TerminalFrontend &) = delete;

  std::vector<KeyEvent> poll();
  bool inputEnabled() const noexcept;

private:
  std::vector<KeyEvent> decodeBufferedInput();
  void restore() noexcept;

  TerminalFrontendOptions options_;
  bool configured_{false};
  std::string input_buffer_;
  bool escape_pending_{false};

  struct TerminalState;
  TerminalState * state_{nullptr};
};

}  // namespace motion_control_lab
