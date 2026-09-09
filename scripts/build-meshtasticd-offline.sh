#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Roy C. Gross
# SPDX-License-Identifier: Apache-2.0
# Compile the Meshtastic daemon from the verified source vendor set. This script
# is called by debian/rules; it must not resolve or download any dependency.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_root="${1:-$repo_root/debian/build-meshtasticd}"
output="${2:-$repo_root/debian/meshtasticd-quadrf}"
case "$build_root" in
  "$repo_root"/debian/build-meshtasticd|"$repo_root"/debian/build-meshtasticd/*) ;;
  *) echo "refusing unexpected Meshtastic build directory: $build_root" >&2; exit 1 ;;
esac

"$repo_root/scripts/verify-vendor.sh"
# shellcheck disable=SC1091
source "$repo_root/integrations/meshtastic/dependencies.lock"
# shellcheck disable=SC1091
source "$repo_root/vendor/manifest.env"
export SOURCE_DATE_EPOCH
export BUILD_EPOCH="$SOURCE_DATE_EPOCH"
export TZ=UTC
export LC_ALL=C.UTF-8
export PYTHONHASHSEED=0
export PLATFORMIO_SETTING_ENABLE_TELEMETRY=no
export PLATFORMIO_SETTING_CHECK_PLATFORMIO_INTERVAL=0
export PLATFORMIO_SETTING_CHECK_PLATFORMS_INTERVAL=0
export PLATFORMIO_SETTING_CHECK_LIBRARIES_INTERVAL=0
# Nanopb extras may try to pip-install protobuf/grpcio-tools. Generation is
# already complete in the locked source tree; do not contact an index.
export PIP_NO_INDEX=1
export PIP_DISABLE_PIP_VERSION_CHECK=1

if [[ -e "$build_root" ]]; then
  find "$build_root" -mindepth 1 -delete
else
  install -d -m755 "$build_root"
fi
tar -xf "$repo_root/vendor/meshtastic-firmware-patched.tar.xz" -C "$build_root"
tar -xf "$repo_root/vendor/platformio-sources.tar.xz" -C "$build_root"
tar -xf "$repo_root/vendor/platformio-wheelhouse.tar.xz" -C "$build_root"

python3 -m venv "$build_root/venv"
"$build_root/venv/bin/python" -m pip install \
  --disable-pip-version-check --no-index --require-hashes \
  --find-links "$build_root/wheelhouse" \
  --requirement "$repo_root/integrations/meshtastic/platformio-requirements.lock"
actual_pio_version="$("$build_root/venv/bin/pio" --version | sed -n 's/^PlatformIO Core, version //p')"
[[ "$actual_pio_version" == "$PLATFORMIO_CORE_VERSION" ]] || {
  echo "offline PlatformIO version mismatch: $actual_pio_version" >&2
  exit 1
}

export PLATFORMIO_CORE_DIR="$build_root/platformio"
export PLATFORMIO_BUILD_DIR="$build_root/pio-build"
export PLATFORMIO_LIBDEPS_DIR="$build_root/platformio/libdeps"
"$build_root/venv/bin/pio" run --project-dir "$build_root/firmware" \
  --environment native

built_binary=""
for candidate in program meshtasticd firmware; do
  if [[ -x "$PLATFORMIO_BUILD_DIR/native/$candidate" ]]; then
    built_binary="$PLATFORMIO_BUILD_DIR/native/$candidate"
    break
  fi
done
[[ -n "$built_binary" ]] || {
  echo "source build completed but no native Meshtastic daemon was found" >&2
  exit 1
}
file "$built_binary" | grep -q 'ELF' || {
  echo "Meshtastic source build did not produce an ELF executable" >&2
  exit 1
}
install -Dm755 "$built_binary" "$output"
printf 'Built meshtasticd from verified source: %s\n' "$output"
