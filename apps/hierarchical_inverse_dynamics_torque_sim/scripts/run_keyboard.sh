#!/usr/bin/env bash
set -euo pipefail

lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
install_prefix=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}
binary=${MCL_BINARY:-"${install_prefix}/bin/mcl_hierarchical_inverse_dynamics_torque_sim"}
asset_root=${MCL_R1_SIM_ASSET_ROOT:-"${install_prefix}/share/motion-control-lab/robots/r1/mujoco"}
urdf=${MCL_URDF:-"${asset_root}/urdf/r1.urdf"}
mujoco_model=${MCL_MUJOCO_MODEL:-"${asset_root}/mjcf/r1.xml"}

export MCL_CPU_SET=${MCL_CPU_SET-5-7}
export MCL_RT_PRIORITY=${MCL_RT_PRIORITY-60}

viewer_flag=--mujoco-viewer
if [[ ${MCL_MUJOCO_VIEWER:-on} == off ]]; then
  viewer_flag=--no-mujoco-viewer
fi

extra_args=()
if [[ -n ${MCL_MCAP:-} ]]; then
  extra_args+=(--mcap "${MCL_MCAP}")
fi

exec "${lab_root}/scripts/launch_app.sh" "${binary}" teleop \
  --urdf "${urdf}" --mujoco-model "${mujoco_model}" \
  --duration "${MCL_DURATION:-0}" --retarget-x "${MCL_RETARGET_X_M:-0}" \
  --side "${MCL_SIDE:-left}" --step-m "${MCL_STEP_M:-0.005}" \
  --min-step-m "${MCL_MIN_STEP_M:-0.001}" \
  --max-step-m "${MCL_MAX_STEP_M:-0.5}" \
  --rotation-step-deg "${MCL_ROTATION_STEP_DEG:-5.0}" \
  --ui "${MCL_UI:-tui}" --viz "${MCL_VIZ:-foxglove}" \
  --host "${MCL_VIZ_HOST:-127.0.0.1}" --port "${MCL_VIZ_PORT:-8765}" \
  --ui-rate "${MCL_UI_RATE_HZ:-100}" "${viewer_flag}" \
  "${extra_args[@]}" "$@"
