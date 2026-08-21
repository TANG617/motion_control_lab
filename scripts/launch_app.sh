#!/usr/bin/env bash
set -euo pipefail

if (($# == 0)); then
  echo "usage: $0 <binary> [arguments...]" >&2
  exit 64
fi

command=("$@")
if [[ ! -x "${command[0]}" ]]; then
  launcher_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
  binary_name=$(basename "${command[0]}")
  workspace_install_prefix=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}
  candidates=(
    "${launcher_root}/bin/${binary_name}"
  )
  if path_binary=$(command -v "${binary_name}" 2>/dev/null); then
    candidates+=("${path_binary}")
  fi
  candidates+=("${workspace_install_prefix}/bin/${binary_name}")

  resolved_binary=""
  for candidate in "${candidates[@]}"; do
    if [[ -x "${candidate}" ]]; then
      resolved_binary="${candidate}"
      break
    fi
  done
  if [[ -z "${resolved_binary}" ]]; then
    echo "motion-control-lab: executable not found: ${command[0]}" >&2
    echo "build/install ${binary_name}, add it to PATH, or set MCL_BINARY explicitly" >&2
    exit 127
  fi
  command[0]="${resolved_binary}"
fi
if [[ -n "${MCL_CPU_SET:-}" ]]; then
  command=(taskset -c "${MCL_CPU_SET}" "${command[@]}")
fi
if [[ -n "${MCL_RT_PRIORITY:-}" ]]; then
  command=(chrt -f "${MCL_RT_PRIORITY}" "${command[@]}")
fi

default_library_path="/workspace/install/algorithm/lib:/opt/openrobots/lib"
exec env LD_LIBRARY_PATH="${MCL_LD_LIBRARY_PATH:-${default_library_path}}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
  "${command[@]}"
