#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
install_prefix=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}
binary=${MCL_BINARY:-"${install_prefix}/bin/mcl_replay_plan"}
input=${MCL_INPUT:?set MCL_INPUT to the source CSV}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" --input "${input}" --input-format csv \
  --left-stream "${MCL_LEFT_STREAM:-left}" --right-stream "${MCL_RIGHT_STREAM:-right}" \
  --timestamp-source "${MCL_TIMESTAMP_SOURCE:-csv_timestamp}" \
  --target-period-ms "${MCL_TARGET_PERIOD_MS:-10}" --execution-mode "${MCL_EXECUTION_MODE:-batch}" \
  --output-dir "${MCL_OUTPUT_DIR:-${lab_root}/runs/replay_plan_csv}" --launcher "$0" "$@"
