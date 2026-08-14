#pragma once

#include "config/interactive_ik_options.hpp"
#include "runtime/interactive_types.hpp"

#include <optional>
#include <string>
#include <termios.h>
#include <utility>
#include <vector>

namespace motion_control_lab
{

enum class Key
{
  None,
  Escape,
  Up,
  Down,
  Left,
  Right,
  Character,
};

struct KeyInput
{
  Key key{Key::None};
  char character{'\0'};
};

class TerminalSession
{
public:
  TerminalSession();
  TerminalSession(const TerminalSession &) = delete;
  TerminalSession & operator=(const TerminalSession &) = delete;
  ~TerminalSession();

  void enableRawMode();
  void restoreCookedMode();

  KeyInput readKey();

  std::optional<std::string> promptLine(const std::string & prompt);

  std::pair<int, int> size() const;

  void write(const std::string & text) const;

private:
  termios original_{};
  int original_input_flags_{};
  bool raw_enabled_{false};
  bool nonblocking_enabled_{false};
};

class TuiTeleopSource
{
public:
  TuiTeleopSource(
    const TuiTeleopOptions & options,
    double rate_hz,
    std::string title,
    InteractiveIkPresentation presentation,
    std::vector<ArmTarget> initial_targets,
    bool allow_side_switching);
  ~TuiTeleopSource();

  void poll();

  const TargetCommand & command() const;

  std::optional<ArmSide> consumeResetRequest();

  void setTargetPose(
    ArmSide side,
    const Pose & target_pose,
    const std::string & status);

  void setStatus(const std::string & status);

  void render(
    const IkDebugFrame & frame,
    std::size_t publish_count,
    const std::string & sink_status);

private:
  void handleKey(const KeyInput & input);

  TuiTeleopOptions options_;
  double rate_hz_{20.0};
  std::string title_;
  InteractiveIkPresentation presentation_;
  TerminalSession terminal_;
  TargetCommand command_;
  double step_m_{0.01};
  std::size_t rotation_axis_index_{0};
  double rotation_step_rad_{0.0};
  bool show_help_{true};
  bool allow_side_switching_{false};
  std::optional<ArmSide> reset_requested_;
};

}  // namespace motion_control_lab
