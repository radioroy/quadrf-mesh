#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Roy C. Gross
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
lock_file="$repo_root/integrations/meshtastic/dependencies.lock"
python_lock="$repo_root/integrations/meshtastic/platformio-requirements.lock"
vendor_dir="${QUADRF_VENDOR_DIR:-$repo_root/vendor}"

# shellcheck disable=SC1090
source "$lock_file"
required_vars=(
  MESHTASTIC_FIRMWARE_REPOSITORY MESHTASTIC_FIRMWARE_TAG
  MESHTASTIC_FIRMWARE_COMMIT MESHTASTIC_FIRMWARE_TREE
  MESHTASTIC_WEB_RELEASE MESHTASTIC_WEB_ASSET_URL
  MESHTASTIC_WEB_ASSET_SHA256 PLATFORMIO_CORE_VERSION PLATFORMIO_PYTHON_VERSION
)
for name in "${required_vars[@]}"; do
  if [[ -z "${!name:-}" ]]; then
    echo "missing $name in $lock_file" >&2
    exit 1
  fi
done

for command_name in cp curl find git install patch python3 sha256sum tar; do
  command -v "$command_name" >/dev/null || {
    echo "required command not found: $command_name" >&2
    exit 1
  }
done

canonicalize_tree_modes() {
  local root="$1"
  local path
  while IFS= read -r -d '' path; do
    chmod 0755 "$path"
  done < <(find "$root" -type d -print0)
  while IFS= read -r -d '' path; do
    if [[ -x "$path" ]]; then
      chmod 0755 "$path"
    else
      chmod 0644 "$path"
    fi
  done < <(find "$root" -type f -print0)
}

work_dir="$(mktemp -d "${TMPDIR:-/tmp}/quadrf-mesh-deps.XXXXXX")"
if [[ "${QUADRF_KEEP_PREP_WORK:-0}" == 1 ]]; then
  trap 'echo "preserved preparation work: $work_dir" >&2' EXIT
else
  trap 'rm -rf "$work_dir"' EXIT
fi

checkout_dir="$work_dir/firmware-git"
firmware_dir="$work_dir/firmware"
wheelhouse_dir="$work_dir/wheelhouse"
platformio_stage="$work_dir/platformio-stage/platformio"
platformio_core="$work_dir/platformio-core"
platformio_build="$work_dir/platformio-build"
platformio_libdeps="$work_dir/platformio-libdeps"
web_archive="$work_dir/meshtastic-web.tar"
source_archive="$work_dir/meshtastic-firmware-patched.tar.xz"
platformio_archive="$work_dir/platformio-sources.tar.xz"
wheelhouse_archive="$work_dir/platformio-wheelhouse.tar.xz"
rendered_backend_patch="$work_dir/0000-add-quadrf-backend-sources.patch"

printf 'Fetching Meshtastic firmware %s (%s)\n' \
  "$MESHTASTIC_FIRMWARE_TAG" "$MESHTASTIC_FIRMWARE_COMMIT"
git clone --quiet --filter=blob:none --no-checkout \
  "$MESHTASTIC_FIRMWARE_REPOSITORY" "$checkout_dir"
git -C "$checkout_dir" fetch --quiet --depth=1 origin "$MESHTASTIC_FIRMWARE_COMMIT"
actual_commit="$(git -C "$checkout_dir" rev-parse FETCH_HEAD)"
actual_tree="$(git -C "$checkout_dir" rev-parse 'FETCH_HEAD^{tree}')"
[[ "$actual_commit" == "$MESHTASTIC_FIRMWARE_COMMIT" ]] || {
  echo "firmware commit mismatch: $actual_commit" >&2
  exit 1
}
[[ "$actual_tree" == "$MESHTASTIC_FIRMWARE_TREE" ]] || {
  echo "firmware tree mismatch: $actual_tree" >&2
  exit 1
}
source_date_epoch="$(git -C "$checkout_dir" show -s --format=%ct "$actual_commit")"
export SOURCE_DATE_EPOCH="$source_date_epoch"
export BUILD_EPOCH="$source_date_epoch"

# Export the verified tree
install -d -m755 "$firmware_dir"
git -C "$checkout_dir" archive --format=tar "$actual_commit" | tar -xf - -C "$firmware_dir"

# Remove unused checked-in host utility
rm -f "$firmware_dir/${MESHTASTIC_FIRMWARE_EXCLUDED_ARTIFACT_PATH:-bin/mergehex}"

# Render backend sources patch
"$repo_root/scripts/render-meshtastic-backend-patch.sh" >"$rendered_backend_patch"
cmp --silent "$rendered_backend_patch" \
  "$repo_root/integrations/meshtastic/patches/backend/0000-add-quadrf-backend-sources.patch" || {
  echo "generated Meshtastic backend source patch is stale" >&2
  exit 1
}

apply_series() {
  local series_dir="$1"
  local patch_name
  while IFS= read -r patch_name; do
    [[ -z "$patch_name" || "$patch_name" == \#* ]] && continue
    echo "Applying ${series_dir#"$repo_root/"}/$patch_name"
    patch --batch --forward --strip=1 --directory="$firmware_dir" \
      --input="$series_dir/$patch_name"
  done <"$series_dir/series"
}

apply_series "$repo_root/integrations/meshtastic/patches/backend"
if [[ "${QUADRF_ENABLE_DOWNSTREAM_PATCHES:-0}" == "1" ]]; then
  apply_series "$repo_root/integrations/meshtastic/patches/downstream"
fi
canonicalize_tree_modes "$firmware_dir"

# Download PlatformIO python closure
install -d -m755 "$wheelhouse_dir"
python3 -m pip download --disable-pip-version-check --require-hashes \
  --only-binary=:all: --platform any \
  --python-version "${PLATFORMIO_PYTHON_VERSION/./}" \
  --implementation py --abi none --dest "$wheelhouse_dir" \
  --requirement "$python_lock"
canonicalize_tree_modes "$wheelhouse_dir"

python3 -m venv "$work_dir/platformio-venv"
"$work_dir/platformio-venv/bin/python" -m pip install \
  --disable-pip-version-check --no-index --require-hashes \
  --find-links "$wheelhouse_dir" --requirement "$python_lock"

export PLATFORMIO_CORE_DIR="$platformio_core"
export PLATFORMIO_BUILD_DIR="$platformio_build"
export PLATFORMIO_LIBDEPS_DIR="$platformio_libdeps"
export PLATFORMIO_SETTING_ENABLE_TELEMETRY=no
export PLATFORMIO_SETTING_CHECK_PLATFORMIO_INTERVAL=0
export PLATFORMIO_SETTING_CHECK_PLATFORMS_INTERVAL=0
export PLATFORMIO_SETTING_CHECK_LIBRARIES_INTERVAL=0
"$work_dir/platformio-venv/bin/pio" pkg install \
  --project-dir "$firmware_dir" --environment native

install -d -m755 "$platformio_stage/libdeps"
cp -a "$platformio_core/platforms" "$platformio_stage/platforms"
cp -a "$platformio_core/packages" "$platformio_stage/packages"
cp -a "$platformio_libdeps/native" "$platformio_stage/libdeps/native"

# PlatformIO stores VCS package registration records inside `.git/.piopm`.
# Preserve them in the package roots before removing .git metadata.
while IFS= read -r -d '' git_dir; do
  pkg_dir="$(dirname "$git_dir")"
  if [[ -f "$git_dir/.piopm" && ! -f "$pkg_dir/.piopm" ]]; then
    install -m644 "$git_dir/.piopm" "$pkg_dir/.piopm"
  fi
done < <(find "$platformio_stage" -type d -name .git -print0)

# Remove checked-in build output from the framework if present
if [[ -d "$platformio_stage/packages/framework-portduino/cmake-build-debug" ]]; then
  rm -rf "$platformio_stage/packages/framework-portduino/cmake-build-debug"
fi

# Remove build debris and git metadata
find "$platformio_stage" -name .git -prune -exec rm -rf {} +
canonicalize_tree_modes "$platformio_stage"

[[ -f "$platformio_stage/packages/framework-portduino/.piopm" ]] || {
  echo "missing framework-portduino package metadata" >&2
  exit 1
}

# Fetch Meshtastic Web UI assets
printf 'Fetching Meshtastic Web %s\n' "$MESHTASTIC_WEB_RELEASE"
curl --fail --location --silent --show-error \
  "$MESHTASTIC_WEB_ASSET_URL" --output "$web_archive"
printf '%s  %s\n' "$MESHTASTIC_WEB_ASSET_SHA256" "$web_archive" | \
  sha256sum --check --strict

# Compress archives
XZ_OPT='-9e -T1' tar --sort=name --mtime="@$source_date_epoch" \
  --mode='u+rwX,go+rX,go-w' \
  --owner=0 --group=0 --numeric-owner -C "$work_dir" \
  -cJf "$source_archive" firmware
XZ_OPT='-9e -T1' tar --sort=name --mtime="@$source_date_epoch" \
  --mode='u+rwX,go+rX,go-w' \
  --owner=0 --group=0 --numeric-owner -C "$work_dir/platformio-stage" \
  -cJf "$platformio_archive" platformio
XZ_OPT='-9e -T1' tar --sort=name --mtime="@$source_date_epoch" \
  --mode='u+rwX,go+rX,go-w' \
  --owner=0 --group=0 --numeric-owner -C "$work_dir" \
  -cJf "$wheelhouse_archive" wheelhouse

install -d -m755 "$vendor_dir"
install -m644 "$source_archive" "$vendor_dir/meshtastic-firmware-patched.tar.xz"
install -m644 "$platformio_archive" "$vendor_dir/platformio-sources.tar.xz"
install -m644 "$wheelhouse_archive" "$vendor_dir/platformio-wheelhouse.tar.xz"
install -m644 "$web_archive" "$vendor_dir/meshtastic-web.tar"
install -m644 "$lock_file" "$vendor_dir/dependencies.lock"
install -m644 "$python_lock" "$vendor_dir/platformio-requirements.lock"

source_sha="$(sha256sum "$vendor_dir/meshtastic-firmware-patched.tar.xz" | cut -d' ' -f1)"
platformio_sha="$(sha256sum "$vendor_dir/platformio-sources.tar.xz" | cut -d' ' -f1)"
wheelhouse_sha="$(sha256sum "$vendor_dir/platformio-wheelhouse.tar.xz" | cut -d' ' -f1)"

cat >"$vendor_dir/manifest.env" <<MANIFEST
MESHTASTIC_FIRMWARE_COMMIT=$MESHTASTIC_FIRMWARE_COMMIT
MESHTASTIC_FIRMWARE_TREE=$MESHTASTIC_FIRMWARE_TREE
PLATFORMIO_CORE_VERSION=$PLATFORMIO_CORE_VERSION
SOURCE_DATE_EPOCH=$source_date_epoch
MESHTASTIC_SOURCE_SHA256=$source_sha
PLATFORMIO_SOURCES_SHA256=$platformio_sha
PLATFORMIO_WHEELHOUSE_SHA256=$wheelhouse_sha
MESHTASTIC_WEB_SHA256=$MESHTASTIC_WEB_ASSET_SHA256
DOWNSTREAM_PATCHES=${QUADRF_ENABLE_DOWNSTREAM_PATCHES:-0}
MANIFEST

(
  cd "$vendor_dir"
  sha256sum dependencies.lock manifest.env \
    meshtastic-firmware-patched.tar.xz meshtastic-web.tar \
    platformio-requirements.lock platformio-sources.tar.xz \
    platformio-wheelhouse.tar.xz >SHA256SUMS
)

printf 'Prepared vendor inputs in %s\n' "$vendor_dir"
cat "$vendor_dir/SHA256SUMS"
