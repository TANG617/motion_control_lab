#include "components/terminal_frontend/key_router.hpp"

#include <cctype>

namespace motion_control_lab
{

KeyRoute KeyRouter::route(const KeyEvent & event) const noexcept
{
  switch (event.code) {
  case KeyCode::PageUp:
  case KeyCode::PageDown:
  case KeyCode::Home:
  case KeyCode::End:
  case KeyCode::Tab:
  case KeyCode::BackTab:
  case KeyCode::Function1:
  case KeyCode::Function2:
  case KeyCode::Function3:
  case KeyCode::Function4:
  case KeyCode::Function5:
  case KeyCode::Function6:
  case KeyCode::Function7: return KeyRoute::Navigation;
  case KeyCode::Character: {
    const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(event.character)));
    if ((key >= '1' && key <= '9') || key == 'h' || key == '?') {
      return KeyRoute::Navigation;
    }
    return KeyRoute::Source;
  }
  default: return KeyRoute::Source;
  }
}

} // namespace motion_control_lab
