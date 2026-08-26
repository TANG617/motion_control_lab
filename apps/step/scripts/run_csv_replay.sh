#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
install_prefix=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}
binary=${MCL_BINARY:-"${install_prefix}/bin/mcl_step"}
urdf=${MCL_URDF:-/workspace/models/r1.cos.urdf}
input=${MCL_INPUT:?set MCL_INPUT to the source CSV}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" replay \
  --urdf "${urdf}" --input "${input}" --input-format csv \
  --left-stream "${MCL_LEFT_STREAM:-left}" --right-stream "${MCL_RIGHT_STREAM:-right}" \
  --timestamp-source "${MCL_TIMESTAMP_SOURCE:-csv_timestamp}" \
  --target-period-ms "${MCL_TARGET_PERIOD_MS:-10}" --execution-mode "${MCL_EXECUTION_MODE:-batch}" \
  --rate "${MCL_RATE_HZ:-100}" --solver "${MCL_SOLVER:-mcc}" --backend "${MCL_BACKEND:-proxqp}" \
  --ui "${MCL_UI:-none}" --terminal-input "${MCL_TERMINAL_INPUT:-off}" --viz "${MCL_VIZ:-none}" \
  --output-root "${MCL_OUTPUT_ROOT:-${lab_root}/runs/step}" --launcher "$0" "$@"
