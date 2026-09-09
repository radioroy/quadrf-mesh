#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Roy C. Gross
# SPDX-License-Identifier: Apache-2.0
# FPGA digital loopback smoke test.
set -euo pipefail

source_dir="${1:-.}"
build_dir="${2:-build-hil}"
[[ -f "$source_dir/CMakeLists.txt" ]] || {
  echo "candidate source is missing CMakeLists.txt" >&2
  exit 1
}
cmake -S "$source_dir" -B "$build_dir" -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DPHY_JTAG_PATH=/usr/bin/quadrf-jtag \
  -DBUILD_TESTING=ON
cmake --build "$build_dir" --parallel --target quadrf-lora-phy

# FPGA digital loopback only: this must not enable an OTA RF path.
timeout 180 "$build_dir/apps/quadrf-lora-phy" \
  --digital --selftest --preset shortturbo
