#!/usr/bin/env bash
set -euo pipefail

lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
install_prefix=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}
binary=${MCL_BINARY:-"${install_prefix}/bin/mcl_hierarchical_inverse_dynamics_torque_sim"}
asset_root=${MCL_R1_SIM_ASSET_ROOT:-"${install_prefix}/share/motion-control-lab/robots/r1/mujoco"}
urdf=${MCL_URDF:-"${asset_root}/urdf/r1.urdf"}
mujoco_model=${MCL_MUJOCO_MODEL:-"${asset_root}/mjcf/r1.xml"}

exec "${lab_root}/scripts/launch_app.sh" "${binary}" \
  --urdf "${urdf}" --mujoco-model "${mujoco_model}" \
  --ui none --viz none --no-mujoco-viewer "$@"
