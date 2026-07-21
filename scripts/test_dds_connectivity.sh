#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 <network-interface> [timeout-seconds]" >&2
    exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
test_binary="${project_root}/build/dds_connectivity_test"
unitree_sdk_lib="/opt/unitree_robotics/lib"

if [[ ! -x "${test_binary}" ]]; then
    echo "DDS test binary not found. Build the project first: cmake --build build -j\$(nproc)" >&2
    exit 2
fi

# Keep the runtime DDS libraries consistent with the SDK used by CMake. This
# machine may also have a different CycloneDDS installation in /usr/local/lib;
# mixing its libddsc/libddscxx with the /opt SDK headers corrupts heap state.
export LD_LIBRARY_PATH="${unitree_sdk_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

exec "${test_binary}" "$@"
