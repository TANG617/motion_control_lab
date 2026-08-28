#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
install_prefix=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}
binary=${MCL_BINARY:-"${install_prefix}/bin/mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim"}
asset_root=${MCL_R1_SIM_ASSET_ROOT:-"${install_prefix}/share/motion-control-lab/robots/r1/mujoco"}
urdf=${MCL_URDF:-"${asset_root}/urdf/r1.urdf"}
mujoco_model=${MCL_MUJOCO_MODEL:-"${asset_root}/mjcf/r1.xml"}
input=${MCL_INPUT:-/workspace/fixtures/raw/sliced-RW1AZHYCSEFT5_RW1AZHYCSEFT5260310002_20260813164525_0.mcap}
viewer_option=--mujoco-viewer
if [[ "${MCL_MUJOCO_VIEWER:-on}" == "off" ]]; then
  viewer_option=--no-mujoco-viewer
fi

start_paused=()
case "${MCL_START_PAUSED:-on}" in
  on) start_paused=(--start-paused) ;;
  off) ;;
  *) echo "MCL_START_PAUSED must be 'on' or 'off'" >&2; exit 64 ;;
esac

# Preserve the production-faithful replay scheduling preset. Assign an empty
# value explicitly to disable either wrapper for a non-realtime smoke run.
export MCL_CPU_SET=${MCL_CPU_SET-5-7}
export MCL_RT_PRIORITY=${MCL_RT_PRIORITY-60}

exec "${lab_root}/scripts/launch_app.sh" "${binary}" replay --urdf "${urdf}" \
  --mujoco-model "${mujoco_model}" "${viewer_option}" \
  --input "${input}" \
  --input-format mcap \
  --left-stream "${MCL_LEFT_STREAM:-/hal/tracker/htc/left/calib_target_pose}" \
  --right-stream "${MCL_RIGHT_STREAM:-/hal/tracker/htc/right/calib_target_pose}" \
  --initial-joint-state-stream "${MCL_INITIAL_JOINT_STATE_STREAM:-/mc/ik/joint_states}" \
  --timestamp-source "${MCL_TIMESTAMP_SOURCE:-header_stamp}" \
  --target-period-ms "${MCL_TARGET_PERIOD_MS:-10}" \
  --pairing-policy "${MCL_PAIRING_POLICY:-nearest}" \
  --nearest-tolerance-ms "${MCL_NEAREST_TOLERANCE_MS:-5}" \
  --unmatched-policy "${MCL_UNMATCHED_POLICY:-drop_with_diagnostics}" \
  --execution-mode "${MCL_EXECUTION_MODE:-realtime}" \
  --playback-rate "${MCL_PLAYBACK_RATE:-1}" \
  --red-rate "${MCL_RED_RATE_HZ:-1000}" \
  --yellow-rate "${MCL_YELLOW_RATE_HZ:-500}" \
  --deadline-policy "${MCL_DEADLINE_POLICY:-monitor}" \
  --joint-target-mode "${MCL_JOINT_TARGET_MODE:-future-o1-pv}" \
  --max-linear-velocity-mps "${MCL_MAX_LINEAR_VELOCITY_MPS:-3.0}" \
  --max-linear-acceleration-mps2 "${MCL_MAX_LINEAR_ACCELERATION_MPS2:-20.0}" \
  --max-linear-jerk-mps3 "${MCL_MAX_LINEAR_JERK_MPS3:-400.0}" \
  --max-angular-velocity-rps "${MCL_MAX_ANGULAR_VELOCITY_RPS:-3.0}" \
  --max-angular-acceleration-rps2 "${MCL_MAX_ANGULAR_ACCELERATION_RPS2:-20.0}" \
  --max-angular-jerk-rps3 "${MCL_MAX_ANGULAR_JERK_RPS3:-300.0}" \
  --ui "${MCL_UI:-tui}" \
  --terminal-input "${MCL_TERMINAL_INPUT:-on}" \
  --replay-elbow-teleop "${MCL_REPLAY_ELBOW_TELEOP:-on}" \
  --replay-trace "${MCL_REPLAY_TRACE:-on}" \
  --viz "${MCL_VIZ:-foxglove}" \
  --host "${MCL_VIZ_HOST:-127.0.0.1}" \
  --port "${MCL_VIZ_PORT:-8765}" \
  --no-mcap \
  --output-root "${MCL_OUTPUT_ROOT:-/workspace/runs/mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim}" \
  --launcher "$0" \
  "${start_paused[@]}" \
  "$@"
