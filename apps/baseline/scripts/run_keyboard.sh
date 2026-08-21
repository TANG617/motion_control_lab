#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
binary=${MCL_BINARY:-"${lab_root}/build/mcl_baseline"}
urdf=${MCL_URDF:-/workspace/models/r1.cos.urdf}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" teleop --urdf "${urdf}" \
  --ui "${MCL_UI:-tui}" --viz "${MCL_VIZ:-foxglove}" "$@"
