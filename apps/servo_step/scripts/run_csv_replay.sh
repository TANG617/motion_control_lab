#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
binary=${MCL_BINARY:-"${lab_root}/build/mcl_servo_step"}
urdf=${MCL_URDF:-/workspace/models/r1.cos.urdf}
input=${MCL_INPUT:?set MCL_INPUT to the source CSV}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" replay \
  --urdf "${urdf}" --input "${input}" --input-format csv \
  --left-stream "${MCL_LEFT_STREAM:-left}" --right-stream "${MCL_RIGHT_STREAM:-right}" \
  --timestamp-source "${MCL_TIMESTAMP_SOURCE:-csv_timestamp}" \
  --target-period-ms "${MCL_TARGET_PERIOD_MS:-10}" --execution-mode "${MCL_EXECUTION_MODE:-batch}" \
  --rate "${MCL_RATE_HZ:-100}" --solver "${MCL_SOLVER:-mcc}" --backend "${MCL_BACKEND:-proxqp}" \
  --ui "${MCL_UI:-none}" --terminal-input "${MCL_TERMINAL_INPUT:-off}" --viz "${MCL_VIZ:-none}" \
  --output-root "${MCL_OUTPUT_ROOT:-${lab_root}/runs/servo_step}" --launcher "$0" "$@"
