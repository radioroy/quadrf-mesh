#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Roy C. Gross
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

patterns=(
  '-----BEGIN (RSA |EC |OPENSSH )?PRIVATE KEY-----'
  'github_pat_[A-Za-z0-9_]+'
  'gh[pousr]_[A-Za-z0-9]{20,}'
  'sshpass[[:space:]]+-p'
)

failed=0
for pattern in "${patterns[@]}"; do
  if git grep -nIE -e "$pattern" -- ':!scripts/check-sensitive-content.sh'; then
    failed=1
  fi
done
if (( failed != 0 )); then
  echo "tracked content contains a forbidden credential" >&2
  exit 1
fi
echo "no forbidden credentials found"
