#include "components/terminal_frontend/terminal_frontend.hpp"

#include <cerrno>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace motion_control_lab
{

struct TerminalFrontend::TerminalState
{
  termios attributes{};
};

namespace
{

bool startsWith(const std::string & value, const char * prefix, std::size_t size)
{
  return value.size() >= size && value.compare(0, size, prefix, size) == 0;
}

} // namespace

TerminalFrontend::TerminalFrontend(TerminalFrontendOptions options) : options_(options)
{
  if (!options_.input_enabled && !options_.alternate_screen) {
    return;
  }
  if (!::isatty(STDIN_FILENO) || !::isatty(STDOUT_FILENO)) {
    throw std::runtime_error("terminal input/rendering requires an interactive TTY");
  }

  state_ = new TerminalState;
  if (::tcgetattr(STDIN_FILENO, &state_->attributes) != 0) {
    const std::string detail = std::strerror(errno);
    delete state_;
    state_ = nullptr;
    throw std::runtime_error("tcgetattr failed: " + detail);
  }
  termios raw = state_->attributes;
  raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  if (::tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) != 0) {
    const std::string detail = std::strerror(errno);
    delete state_;
    state_ = nullptr;
    throw std::runtime_error("tcsetattr failed: " + detail);
  }

  configured_ = true;
  if (options_.alternate_screen) {
    std::cout << "\x1b[?1049h\x1b[?25l" << std::flush;
  }
}

TerminalFrontend::~TerminalFrontend() { restore(); }

void TerminalFrontend::restore() noexcept
{
  if (!configured_) {
    delete state_;
    state_ = nullptr;
    return;
  }
  if (options_.alternate_screen) {
    std::cout << "\x1b[?25h\x1b[?1049l" << std::flush;
  }
  ::tcsetattr(STDIN_FILENO, TCSAFLUSH, &state_->attributes);
  configured_ = false;
  delete state_;
  state_ = nullptr;
}

bool TerminalFrontend::inputEnabled() const noexcept { return options_.input_enabled; }

std::vector<KeyEvent> TerminalFrontend::poll()
{
  if (!options_.input_enabled) {
    return {};
  }
  char buffer[128];
  while (true) {
    const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
    if (count > 0) {
      input_buffer_.append(buffer, static_cast<std::size_t>(count));
      continue;
    }
    if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    throw std::runtime_error(std::string{"terminal read failed: "} + std::strerror(errno));
  }
  const bool had_pending_escape = escape_pending_;
  auto events = decodeBufferedInput();
  if (input_buffer_ == "\x1b") {
    if (had_pending_escape) {
      events.push_back({KeyCode::Escape, '\0'});
      input_buffer_.clear();
      escape_pending_ = false;
    } else {
      escape_pending_ = true;
    }
  } else {
    escape_pending_ = false;
  }
  return events;
}

std::vector<KeyEvent> TerminalFrontend::decodeBufferedInput()
{
  std::vector<KeyEvent> events;
  while (!input_buffer_.empty()) {
    if (input_buffer_.front() != '\x1b') {
      const char value = input_buffer_.front();
      input_buffer_.erase(0, 1);
      if (value == '\r' || value == '\n') {
        events.push_back({KeyCode::Enter, '\0'});
      } else if (value == '\t') {
        events.push_back({KeyCode::Tab, '\0'});
      } else {
        events.push_back(KeyEvent::characterKey(value));
      }
      continue;
    }

    struct Sequence
    {
      const char * bytes;
      std::size_t size;
      KeyCode code;
    };
    static constexpr Sequence sequences[] = {
        {"\x1b[A", 3, KeyCode::ArrowUp},     {"\x1b[B", 3, KeyCode::ArrowDown},
        {"\x1b[C", 3, KeyCode::ArrowRight},  {"\x1b[D", 3, KeyCode::ArrowLeft},
        {"\x1b[5~", 4, KeyCode::PageUp},     {"\x1b[6~", 4, KeyCode::PageDown},
        {"\x1b[H", 3, KeyCode::Home},        {"\x1b[F", 3, KeyCode::End},
        {"\x1b[Z", 3, KeyCode::BackTab},     {"\x1bOP", 3, KeyCode::Function1},
        {"\x1bOQ", 3, KeyCode::Function2},   {"\x1bOR", 3, KeyCode::Function3},
        {"\x1bOS", 3, KeyCode::Function4},   {"\x1b[15~", 5, KeyCode::Function5},
        {"\x1b[17~", 5, KeyCode::Function6}, {"\x1b[18~", 5, KeyCode::Function7},
    };
    bool matched = false;
    bool incomplete = false;
    for (const auto & sequence : sequences) {
      if (startsWith(input_buffer_, sequence.bytes, sequence.size)) {
        events.push_back({sequence.code, '\0'});
        input_buffer_.erase(0, sequence.size);
        matched = true;
        break;
      }
      if (input_buffer_.size() < sequence.size &&
          std::string{sequence.bytes, input_buffer_.size()} == input_buffer_) {
        incomplete = true;
      }
    }
    if (matched) {
      continue;
    }
    if (incomplete) {
      break;
    }
    events.push_back({KeyCode::Escape, '\0'});
    input_buffer_.erase(0, 1);
  }
  return events;
}

} // namespace motion_control_lab
