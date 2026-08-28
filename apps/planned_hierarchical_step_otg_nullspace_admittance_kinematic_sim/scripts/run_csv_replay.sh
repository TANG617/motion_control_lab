#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
install_prefix=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}
binary=${MCL_BINARY:-"${install_prefix}/bin/mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim"}
asset_root=${MCL_R1_SIM_ASSET_ROOT:-"${install_prefix}/share/motion-control-lab/robots/r1/mujoco"}
urdf=${MCL_URDF:-"${asset_root}/urdf/r1.urdf"}
mujoco_model=${MCL_MUJOCO_MODEL:-"${asset_root}/mjcf/r1.xml"}
input=${MCL_INPUT:?set MCL_INPUT to the source CSV}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" replay --urdf "${urdf}" \
  --mujoco-model "${mujoco_model}" --no-mujoco-viewer \
  --input "${input}" --input-format csv --left-stream "${MCL_LEFT_STREAM:-left}" \
  --right-stream "${MCL_RIGHT_STREAM:-right}" --timestamp-source "${MCL_TIMESTAMP_SOURCE:-csv_timestamp}" \
  --target-period-ms "${MCL_TARGET_PERIOD_MS:-1}" --execution-mode "${MCL_EXECUTION_MODE:-batch}" \
  --red-rate "${MCL_RED_RATE_HZ:-1000}" --yellow-rate "${MCL_YELLOW_RATE_HZ:-100}" \
  --deadline-policy "${MCL_DEADLINE_POLICY:-strict}" \
  --joint-target-mode "${MCL_JOINT_TARGET_MODE:-future-o1-pv}" --ui "${MCL_UI:-none}" \
  --terminal-input "${MCL_TERMINAL_INPUT:-off}" --viz "${MCL_VIZ:-none}" \
  --output-root "${MCL_OUTPUT_ROOT:-${lab_root}/runs/planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim}" --launcher "$0" "$@"
