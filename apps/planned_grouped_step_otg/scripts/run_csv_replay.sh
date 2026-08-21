#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
binary=${MCL_BINARY:-"${lab_root}/build/mcl_planned_grouped_step_otg"}
urdf=${MCL_URDF:-/workspace/models/r1.cos.urdf}
input=${MCL_INPUT:?set MCL_INPUT to the source CSV}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" replay --urdf "${urdf}" \
  --input "${input}" --input-format csv --left-stream "${MCL_LEFT_STREAM:-left}" \
  --right-stream "${MCL_RIGHT_STREAM:-right}" --timestamp-source "${MCL_TIMESTAMP_SOURCE:-csv_timestamp}" \
  --target-period-ms "${MCL_TARGET_PERIOD_MS:-1}" --execution-mode "${MCL_EXECUTION_MODE:-batch}" \
  --red-rate "${MCL_RED_RATE_HZ:-1000}" --yellow-rate "${MCL_YELLOW_RATE_HZ:-100}" \
  --deadline-policy "${MCL_DEADLINE_POLICY:-strict}" \
  --joint-target-mode "${MCL_JOINT_TARGET_MODE:-future-o1-pv}" --ui "${MCL_UI:-none}" \
  --terminal-input "${MCL_TERMINAL_INPUT:-off}" --viz "${MCL_VIZ:-none}" \
  --output-root "${MCL_OUTPUT_ROOT:-${lab_root}/runs/planned_grouped_step_otg}" --launcher "$0" "$@"
