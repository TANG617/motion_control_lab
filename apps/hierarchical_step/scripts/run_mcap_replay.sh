#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
binary=${MCL_BINARY:-"${lab_root}/build/mcl_hierarchical_step"}
urdf=${MCL_URDF:-/workspace/models/r1.cos.urdf}
input=${MCL_INPUT:?set MCL_INPUT to the source MCAP}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" replay --urdf "${urdf}" \
  --input "${input}" --input-format mcap --left-stream "${MCL_LEFT_STREAM:-/mc/ik/left_target}" \
  --right-stream "${MCL_RIGHT_STREAM:-/mc/ik/right_target}" --target-period-ms "${MCL_TARGET_PERIOD_MS:-1}" \
  --execution-mode "${MCL_EXECUTION_MODE:-batch}" --red-rate "${MCL_RED_RATE_HZ:-1000}" \
  --yellow-rate "${MCL_YELLOW_RATE_HZ:-100}" --deadline-policy "${MCL_DEADLINE_POLICY:-strict}" \
  --ui "${MCL_UI:-none}" --terminal-input "${MCL_TERMINAL_INPUT:-off}" --viz "${MCL_VIZ:-none}" \
  --output-root "${MCL_OUTPUT_ROOT:-${lab_root}/runs/hierarchical_step}" --launcher "$0" "$@"
