#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Roy C. Gross
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
./scripts/verify-vendor.sh
if [[ -n "$(git status --porcelain --untracked-files=no)" ]]; then
  echo "refusing to package tracked/staged workspace changes; commit the reviewed tree first" >&2
  exit 1
fi

source_name="$(dpkg-parsechangelog -S Source)"
source_version="$(dpkg-parsechangelog -S Version)"
upstream_version="${source_version%%-*}"
SOURCE_DATE_EPOCH="$(date --date="$(dpkg-parsechangelog -S Date)" +%s)"
export SOURCE_DATE_EPOCH
export BUILD_EPOCH="$SOURCE_DATE_EPOCH"
export TZ=UTC
parent_dir="$(dirname "$repo_root")"
orig_tar="$parent_dir/${source_name}_${upstream_version}.orig.tar.xz"
staging="$(mktemp -d "${TMPDIR:-/tmp}/quadrf-orig.XXXXXX")"
temporary_tar="$(mktemp "$parent_dir/.${source_name}-orig.XXXXXX.tar.xz")"
trap 'rm -rf "$staging"; rm -f "$temporary_tar"' EXIT
source_root="$staging/${source_name}-${upstream_version}"
install -d -m755 "$source_root/vendor"

# The upstream tar is the committed Git tree (excluding Debian metadata) plus
# exactly the source/data files named by the verified vendor manifest.
git archive --format=tar HEAD -- . ':(exclude)debian' ':(exclude)vendor' | \
  tar -xf - -C "$source_root"
while read -r _digest name; do
  [[ "$name" != */* && -f "$repo_root/vendor/$name" ]] || {
    echo "unsafe or missing vendor manifest entry: $name" >&2
    exit 1
  }
  install -m644 "$repo_root/vendor/$name" "$source_root/vendor/$name"
done <"$repo_root/vendor/SHA256SUMS"
install -m644 "$repo_root/vendor/SHA256SUMS" "$source_root/vendor/SHA256SUMS"

while IFS= read -r -d '' path; do
  description="$(file -b "$path")"
  case "$path:$description" in
    *.o:*|*.a:*|*.so:*|*.so.*:*|*.dylib:*|*.dll:*|*.exe:*|*:*ELF*|*:*Mach-O*|*:*PE32*|*:*"current ar archive"*)
      echo "architecture-specific object rejected from orig source: $path ($description)" >&2
      exit 1
      ;;
  esac
done < <(find "$source_root" -type f -print0)

XZ_OPT='-9e -T1' tar --sort=name --mtime="@$SOURCE_DATE_EPOCH" \
  --owner=0 --group=0 --numeric-owner -C "$staging" \
  -cJf "$temporary_tar" "${source_name}-${upstream_version}"
if [[ -e "$orig_tar" ]]; then
  if ! cmp --silent "$temporary_tar" "$orig_tar"; then
    echo "refusing to overwrite different upstream archive: $orig_tar" >&2
    exit 1
  fi
else
  install -m644 "$temporary_tar" "$orig_tar"
fi

exec dpkg-buildpackage --no-sign "$@"
