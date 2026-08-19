#include "console/tui_help.hpp"

#include <ostream>

namespace motion_control_lab {

void printTuiControlsHelp(std::ostream &output) {
  output << "TUI controls:\n"
         << "  1..5/F1..F5/Tab: Overview, Solver/QP, Joints, Runtime, Events\n"
         << "  PageUp/PageDown/Home/End: scroll the current page\n"
         << "  w/s: +x/-x, a/d: +y/-y, q/e: +z/-z\n"
         << "  n: cycle TCP rotation axis, i: clockwise, u: counter-clockwise\n"
         << "  left/right arrows: switch arm when the app controls both arms\n"
         << "  up/down arrows: double/halve step size\n"
         << "  m: enter step size, r: reset target from current FK\n"
         << "  space: pause/resume publishing, h: show/hide help, x or Esc: "
            "exit\n";
}

} // namespace motion_control_lab
