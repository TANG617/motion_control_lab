#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
install_prefix=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}
binary=${MCL_BINARY:-"${install_prefix}/bin/mcl_single_arm_step"}
urdf=${MCL_URDF:-/workspace/models/r1.cos.urdf}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" \
  --urdf "${urdf}" --rate "${MCL_RATE_HZ:-100}" \
  --ui "${MCL_UI:-tui}" --viz "${MCL_VIZ:-foxglove}" "$@"
