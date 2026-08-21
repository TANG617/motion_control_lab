#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
binary=${MCL_BINARY:-"${lab_root}/build/mcl_baseline"}
urdf=${MCL_URDF:-/workspace/models/r1.cos.urdf}
input=${MCL_INPUT:?set MCL_INPUT to the source CSV}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" replay --urdf "${urdf}" \
  --input "${input}" --input-format csv --left-stream "${MCL_LEFT_STREAM:-left}" \
  --right-stream "${MCL_RIGHT_STREAM:-right}" --timestamp-source "${MCL_TIMESTAMP_SOURCE:-csv_timestamp}" \
  --target-period-ms 10 --execution-mode "${MCL_EXECUTION_MODE:-batch}" --ui "${MCL_UI:-none}" \
  --terminal-input "${MCL_TERMINAL_INPUT:-off}" --viz "${MCL_VIZ:-none}" \
  --output-root "${MCL_OUTPUT_ROOT:-${lab_root}/runs/baseline}" --launcher "$0" "$@"
