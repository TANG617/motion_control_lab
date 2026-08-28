#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
install_prefix=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}
binary=${MCL_BINARY:-"${install_prefix}/bin/mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim"}
asset_root=${MCL_R1_SIM_ASSET_ROOT:-"${install_prefix}/share/motion-control-lab/robots/r1/mujoco"}
urdf=${MCL_URDF:-"${asset_root}/urdf/r1.urdf"}
mujoco_model=${MCL_MUJOCO_MODEL:-"${asset_root}/mjcf/r1.xml"}
viewer_option=--mujoco-viewer
if [[ "${MCL_MUJOCO_VIEWER:-on}" == "off" ]]; then
  viewer_option=--no-mujoco-viewer
fi

export MCL_CPU_SET=${MCL_CPU_SET-5-7}
export MCL_RT_PRIORITY=${MCL_RT_PRIORITY-60}

exec "${lab_root}/scripts/launch_app.sh" "${binary}" teleop --urdf "${urdf}" \
  --mujoco-model "${mujoco_model}" "${viewer_option}" \
  --red-rate "${MCL_RED_RATE_HZ:-1000}" --yellow-rate "${MCL_YELLOW_RATE_HZ:-100}" \
  --deadline-policy "${MCL_DEADLINE_POLICY:-monitor}" \
  --joint-target-mode "${MCL_JOINT_TARGET_MODE:-future-o1-pv}" \
  --max-linear-velocity-mps "${MCL_MAX_LINEAR_VELOCITY_MPS:-0.9}" \
  --max-linear-acceleration-mps2 "${MCL_MAX_LINEAR_ACCELERATION_MPS2:-5.0}" \
  --max-linear-jerk-mps3 "${MCL_MAX_LINEAR_JERK_MPS3:-80.0}" \
  --max-angular-velocity-rps "${MCL_MAX_ANGULAR_VELOCITY_RPS:-3.0}" \
  --max-angular-acceleration-rps2 "${MCL_MAX_ANGULAR_ACCELERATION_RPS2:-20.0}" \
  --max-angular-jerk-rps3 "${MCL_MAX_ANGULAR_JERK_RPS3:-200.0}" \
  --ui "${MCL_UI:-tui}" --viz "${MCL_VIZ:-foxglove}" \
  --host "${MCL_VIZ_HOST:-127.0.0.1}" --port "${MCL_VIZ_PORT:-8765}" \
  --no-mcap "$@"
