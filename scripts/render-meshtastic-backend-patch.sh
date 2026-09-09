#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Roy C. Gross
# SPDX-License-Identifier: Apache-2.0
# Render the canonical adapter and Air-IPC header as a complete downstream patch.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
printf '%s\n' \
  'Description: Add the complete QuadRF Air-IPC Portduino backend sources' \
  ' The generated patch is verified against integrations/meshtastic/backend' \
  ' and include/quadrf/air_ipc.hpp before dependency preparation. This keeps' \
  ' the patch series self-contained while retaining one canonical source copy.' \
  'Forwarded: not-needed' \
  'Last-Update: 2026-08-20'

emit_file() {
  local source="$1"
  local destination="$2"
  local lines
  lines="$(wc -l <"$source")"
  printf 'diff --git a/%s b/%s\n' "$destination" "$destination"
  printf 'new file mode 100644\n'
  printf '%s\n' '--- /dev/null'
  printf '+++ b/%s\n' "$destination"
  printf '@@ -0,0 +1,%s @@\n' "$lines"
  sed 's/^/+/' "$source"
}

emit_file "$repo_root/integrations/meshtastic/backend/QuadRFRadio.h" \
  src/platform/portduino/QuadRFRadio.h
emit_file "$repo_root/integrations/meshtastic/backend/QuadRFRadio.cpp" \
  src/platform/portduino/QuadRFRadio.cpp
emit_file "$repo_root/include/quadrf/air_ipc.hpp" \
  src/platform/portduino/quadrf/air_ipc.hpp
emit_file "$repo_root/integrations/meshtastic/backend/QuadRFParrotModule.h" \
  src/platform/portduino/QuadRFParrotModule.h
emit_file "$repo_root/integrations/meshtastic/backend/QuadRFParrotModule.cpp" \
  src/platform/portduino/QuadRFParrotModule.cpp
emit_file "$repo_root/integrations/meshtastic/backend/QuadRFPingModule.h" \
  src/platform/portduino/QuadRFPingModule.h
emit_file "$repo_root/integrations/meshtastic/backend/QuadRFPingModule.cpp" \
  src/platform/portduino/QuadRFPingModule.cpp
