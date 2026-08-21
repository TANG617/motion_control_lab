#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
binary=${MCL_BINARY:-"${lab_root}/build/mcl_replay_plan"}
input=${MCL_INPUT:?set MCL_INPUT to the source MCAP}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" --input "${input}" --input-format mcap \
  --left-stream "${MCL_LEFT_STREAM:-/mc/ik/left_target}" \
  --right-stream "${MCL_RIGHT_STREAM:-/mc/ik/right_target}" \
  --target-period-ms "${MCL_TARGET_PERIOD_MS:-10}" --execution-mode "${MCL_EXECUTION_MODE:-batch}" \
  --output-dir "${MCL_OUTPUT_DIR:-${lab_root}/runs/replay_plan_mcap}" --launcher "$0" "$@"
