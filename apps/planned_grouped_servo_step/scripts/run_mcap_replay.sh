#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
binary=${MCL_BINARY:-"${lab_root}/build/mcl_planned_grouped_servo_step"}
urdf=${MCL_URDF:-/workspace/models/Psi_R1_visual_collision.urdf}
input=${MCL_INPUT:-/workspace/fixtures/raw/sliced_RW1AZHYCSEFT5_RW1AZHYCSEFT5260310002_20260731015829_0.mcap}

start_paused=()
case "${MCL_START_PAUSED:-on}" in
  on) start_paused=(--start-paused) ;;
  off) ;;
  *) echo "MCL_START_PAUSED must be 'on' or 'off'" >&2; exit 64 ;;
esac

export MCL_CPU_SET=${MCL_CPU_SET-5-7}
export MCL_RT_PRIORITY=${MCL_RT_PRIORITY-60}

exec "${lab_root}/scripts/launch_app.sh" "${binary}" replay --urdf "${urdf}" \
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
  --yellow-rate "${MCL_YELLOW_RATE_HZ:-100}" \
  --deadline-policy "${MCL_DEADLINE_POLICY:-monitor}" \
  --max-linear-velocity-mps "${MCL_MAX_LINEAR_VELOCITY_MPS:-0.9}" \
  --max-linear-acceleration-mps2 "${MCL_MAX_LINEAR_ACCELERATION_MPS2:-5.0}" \
  --max-linear-jerk-mps3 "${MCL_MAX_LINEAR_JERK_MPS3:-80.0}" \
  --max-angular-velocity-rps "${MCL_MAX_ANGULAR_VELOCITY_RPS:-3.0}" \
  --max-angular-acceleration-rps2 "${MCL_MAX_ANGULAR_ACCELERATION_RPS2:-20.0}" \
  --max-angular-jerk-rps3 "${MCL_MAX_ANGULAR_JERK_RPS3:-150.0}" \
  --ui "${MCL_UI:-tui}" \
  --terminal-input "${MCL_TERMINAL_INPUT:-on}" \
  --viz "${MCL_VIZ:-foxglove}" \
  --host "${MCL_VIZ_HOST:-127.0.0.1}" \
  --port "${MCL_VIZ_PORT:-8765}" \
  --no-mcap \
  --output-root "${MCL_OUTPUT_ROOT:-/workspace/runs/mcl_planned_grouped_servo_step}" \
  --launcher "$0" \
  "${start_paused[@]}" \
  "$@"
