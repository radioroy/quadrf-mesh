#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Roy C. Gross
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vendor_dir="${QUADRF_VENDOR_DIR:-$repo_root/vendor}"
lock_file="$repo_root/integrations/meshtastic/dependencies.lock"
python_lock="$repo_root/integrations/meshtastic/platformio-requirements.lock"
manifest_file="$vendor_dir/manifest.env"
checksum_file="$vendor_dir/SHA256SUMS"

expected=(
  dependencies.lock
  manifest.env
  meshtastic-firmware-patched.tar.xz
  meshtastic-web.tar
  platformio-requirements.lock
  platformio-sources.tar.xz
  platformio-wheelhouse.tar.xz
)

for file in "$lock_file" "$python_lock" "$manifest_file" "$checksum_file"; do
  [[ -f "$file" ]] || {
    echo "missing prepared package input: $file" >&2
    echo "run ./scripts/prepare-dependencies.sh first" >&2
    exit 1
  }
done

for name in "${expected[@]}"; do
  [[ -f "$vendor_dir/$name" ]] || {
    echo "missing prepared package input: vendor/$name" >&2
    exit 1
  }
done

[[ ! -e "$vendor_dir/meshtasticd-quadrf" ]] || {
  echo "prebuilt vendor/meshtasticd-quadrf is forbidden" >&2
  exit 1
}

(
  cd "$vendor_dir"
  sha256sum --check --strict SHA256SUMS
)

cmp --silent "$vendor_dir/dependencies.lock" "$lock_file" || {
  echo "vendored dependency lock differs from repository lock" >&2
  exit 1
}

echo "vendor/ artifacts verified successfully"
