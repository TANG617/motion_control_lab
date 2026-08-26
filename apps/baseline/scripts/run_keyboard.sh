#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
install_prefix=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}
binary=${MCL_BINARY:-"${install_prefix}/bin/mcl_baseline"}
urdf=${MCL_URDF:-/workspace/models/r1.cos.urdf}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" teleop --urdf "${urdf}" \
  --ui "${MCL_UI:-tui}" --viz "${MCL_VIZ:-foxglove}" "$@"
