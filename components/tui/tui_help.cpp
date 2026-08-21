#include "components/tui/tui_help.hpp"

#include <ostream>

namespace motion_control_lab
{

void printTuiControlsHelp(std::ostream & output)
{
  output << "Keyboard controls:\n"
         << "  1..7/F1..F7/Tab/BackTab: navigate pages; h or ?: help\n"
         << "  PageUp/PageDown/Home/End: scroll the current page\n"
         << "  w/s: +x/-x, a/d: +y/-y, q/e: +z/-z\n"
         << "  n: cycle TCP rotation axis, i: clockwise, u: counter-clockwise\n"
         << "  left/right arrows: switch arm when the app controls both arms\n"
         << "  up/down arrows: double/halve step size\n"
         << "  m: enter step size, r: reset target from current FK\n"
         << "  space: pause/resume input, x or Esc: exit\n";
}

} // namespace motion_control_lab
