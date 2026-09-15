#pragma once
#include "planning.hpp"
#include "solver.hpp"
namespace motion_control_lab::planned_kinematics_step {
int loop(const Options &, Solver &, Planning &);
}
