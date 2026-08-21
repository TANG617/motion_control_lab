#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
binary=${MCL_BINARY:-"${lab_root}/build/mcl_planned_grouped_servo_step"}
urdf=${MCL_URDF:-/workspace/models/Psi_R1_visual_collision.urdf}
input=${MCL_INPUT:?set MCL_INPUT to the source CSV}

export MCL_CPU_SET=${MCL_CPU_SET-5-7}
export MCL_RT_PRIORITY=${MCL_RT_PRIORITY-60}

exec "${lab_root}/scripts/launch_app.sh" "${binary}" replay --urdf "${urdf}" \
  --input "${input}" --input-format csv --left-stream "${MCL_LEFT_STREAM:-left}" \
  --right-stream "${MCL_RIGHT_STREAM:-right}" --timestamp-source "${MCL_TIMESTAMP_SOURCE:-csv_timestamp}" \
  --target-period-ms "${MCL_TARGET_PERIOD_MS:-10}" --execution-mode "${MCL_EXECUTION_MODE:-batch}" \
  --red-rate "${MCL_RED_RATE_HZ:-1000}" --yellow-rate "${MCL_YELLOW_RATE_HZ:-100}" \
  --deadline-policy "${MCL_DEADLINE_POLICY:-monitor}" \
  --max-linear-velocity-mps "${MCL_MAX_LINEAR_VELOCITY_MPS:-0.9}" \
  --max-linear-acceleration-mps2 "${MCL_MAX_LINEAR_ACCELERATION_MPS2:-5.0}" \
  --max-linear-jerk-mps3 "${MCL_MAX_LINEAR_JERK_MPS3:-80.0}" \
  --max-angular-velocity-rps "${MCL_MAX_ANGULAR_VELOCITY_RPS:-3.0}" \
  --max-angular-acceleration-rps2 "${MCL_MAX_ANGULAR_ACCELERATION_RPS2:-20.0}" \
  --max-angular-jerk-rps3 "${MCL_MAX_ANGULAR_JERK_RPS3:-150.0}" \
  --ui "${MCL_UI:-none}" \
  --terminal-input "${MCL_TERMINAL_INPUT:-off}" --viz "${MCL_VIZ:-none}" \
  --no-mcap \
  --output-root "${MCL_OUTPUT_ROOT:-/workspace/runs/mcl_planned_grouped_servo_step}" \
  --launcher "$0" "$@"
