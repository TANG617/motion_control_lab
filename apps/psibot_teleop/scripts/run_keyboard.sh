#!/usr/bin/env bash
set -euo pipefail
install_prefix=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}
binary=${MCL_BINARY:-"${install_prefix}/bin/mcl_psibot_teleop"}
exec "${binary}" --address "${MCL_PSIBOT_ADDRESS:-127.0.0.1:6060}" \
  --mode "${MCL_PSIBOT_MODE:-single}" --ui "${MCL_UI:-tui}" "$@"
