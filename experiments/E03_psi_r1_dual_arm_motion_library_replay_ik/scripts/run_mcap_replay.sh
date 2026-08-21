#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
binary=${MCL_BINARY:-"${lab_root}/build/mcl_e03_batch_replay_ik"}
urdf=${MCL_URDF:-/workspace/models/r1.cos.urdf}
library=${MCL_LIBRARY_DIR:-/workspace/fixtures/datasets/psi_r1_dual_arm_motion_library}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" \
  --urdf "${urdf}" --library-dir "${library}" \
  --output-root "${MCL_OUTPUT_ROOT:-${lab_root}/runs/e03_batch}" \
  --launcher "$0" "$@"
