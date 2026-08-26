#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
install_prefix=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}
binary=${MCL_BINARY:-"${install_prefix}/bin/mcl_cartesian_planning"}
request=${MCL_REQUEST:-"${lab_root}/apps/cartesian_planning/example_request.json"}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" \
  --request "${request}" --output-dir "${MCL_OUTPUT_DIR:-${lab_root}/runs/cartesian_planning}" \
  --no-live --once "$@"
