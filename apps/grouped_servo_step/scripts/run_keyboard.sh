#!/usr/bin/env bash
set -euo pipefail
lab_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
binary=${MCL_BINARY:-"${lab_root}/build/mcl_grouped_servo_step"}
urdf=${MCL_URDF:-/workspace/models/r1.cos.urdf}
exec "${lab_root}/scripts/launch_app.sh" "${binary}" teleop --urdf "${urdf}" \
  --red-rate "${MCL_RED_RATE_HZ:-1000}" --yellow-rate "${MCL_YELLOW_RATE_HZ:-100}" \
  --deadline-policy "${MCL_DEADLINE_POLICY:-strict}" \
  --ui "${MCL_UI:-tui}" --viz "${MCL_VIZ:-foxglove}" "$@"
