#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
binary=${MCL_BINARY:-"${lab_root}/build/mcl_cartesian_planning"}
request=${MCL_REQUEST:-"${lab_root}/apps/cartesian_planning/example_request.json"}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" \
  --request "${request}" --output-dir "${MCL_OUTPUT_DIR:-${lab_root}/runs/cartesian_planning}" \
  --no-live --once "$@"
