#pragma once

#include "contracts/input/input_contract.hpp"

namespace motion_control_lab
{

enum class KeyRoute
{
  Navigation,
  Source,
};

class KeyRouter
{
public:
  KeyRoute route(const KeyEvent & event) const noexcept;
};

}  // namespace motion_control_lab
