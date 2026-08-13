#include "dual_arm_ik_app.hpp"

#include <motion_control_core/motion_control_core.hpp>

int main(int argc, char ** argv)
{
  return motion_control_lab::runDualArmIkApp(
    argc,
    argv,
    {
      "mcl_dual_arm_ik_target_solve",
      "Dual-arm IK — TargetSolve",
      motion_control::core::IkSolveMode::TargetSolve,
    });
}
