#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Roy C. Gross
# SPDX-License-Identifier: Apache-2.0
# Run inside the Debian Trixie arm64 build image with networking disabled.
set -euo pipefail

package_dir="${1:?usage: test-debian-packages.sh PACKAGE_DIRECTORY}"
package_dir="$(realpath "$package_dir")"
mapfile -t project_packages < <(find "$package_dir" -maxdepth 1 -type f \
    \( -name 'quadrf-lora-phy_*.deb' -o -name 'quadrf-meshtasticd_*.deb' -o -name 'quadrf-mesh_*.deb' \) | sort)
[[ "${#project_packages[@]}" == 3 ]] || {
  echo "expected exactly three project binary packages in $package_dir" >&2
  exit 1
}
for package in "${project_packages[@]}"; do
  arch="$(dpkg-deb --field "$package" Architecture)"
  [[ "$arch" == arm64 || "$arch" == all ]] || {
    echo "package has unexpected architecture $arch: $package" >&2
    exit 1
  }
done

work_dir="$(mktemp -d "${TMPDIR:-/tmp}/quadrf-install-test.XXXXXX")"
trap 'rm -rf "$work_dir"' EXIT
for dependency in quadrf-fpga quadrf-soapy bluez-firmware; do
  printf 'Section: misc\nPriority: optional\nPackage: %s\nVersion: 999.0\nArchitecture: all\nDescription: CI-only dependency stub\n' \
    "$dependency" >"$work_dir/$dependency.control"
  (cd "$work_dir" && equivs-build "$dependency.control" >/dev/null)
done
mapfile -t dependency_packages < <(find "$work_dir" -maxdepth 1 -name '*.deb' -type f | sort)
[[ "${#dependency_packages[@]}" == 3 ]]

apt-get install --yes --no-install-recommends \
  "${dependency_packages[@]}" "${project_packages[@]}"
installed="$(dpkg-query --show --showformat='${Package} ${Status}\n' \
  quadrf-lora-phy quadrf-meshtasticd quadrf-mesh)"
printf '%s\n' "$installed"
count="$(printf '%s\n' "$installed" | grep -c 'install ok installed' || true)"
[[ "$count" == 3 ]] || {
  echo "expected all three project packages to be fully installed" >&2
  exit 1
}
for binary in /usr/bin/quadrf-lora-phy /usr/bin/meshtasticd-quadrf \
              /usr/bin/quadrf-mesh-monitor /usr/bin/quadrf-ble-bridge \
              /usr/libexec/quadrf-meshtasticd-launch; do
  [[ -x "$binary" ]] || {
    echo "missing executable: $binary" >&2
    exit 1
  }
done
[[ -f /etc/default/quadrf-lora-phy ]] || {
  echo "missing /etc/default/quadrf-lora-phy" >&2
  exit 1
}
grep -qx 'QUADRF_LORA_PHY_MODE=--ota' /etc/default/quadrf-lora-phy || {
  echo "expected QUADRF_LORA_PHY_MODE=--ota in /etc/default/quadrf-lora-phy" >&2
  exit 1
}
grep -qx 'QUADRF_LORA_PHY_PRESET=shortturbo' /etc/default/quadrf-lora-phy || {
  echo "expected QUADRF_LORA_PHY_PRESET=shortturbo in /etc/default/quadrf-lora-phy" >&2
  exit 1
}
[[ -f /etc/default/quadrf-meshtasticd ]] || {
  echo "missing /etc/default/quadrf-meshtasticd" >&2
  exit 1
}
[[ -f /etc/meshtasticd/config.yaml ]] || {
  echo "missing /etc/meshtasticd/config.yaml (must be a file, not a directory)" >&2
  ls -la /etc/meshtasticd 2>&1 || true
  exit 1
}
desktop=/usr/share/applications/io.github.radioroy.QuadRFMesh.desktop
[[ -f "$desktop" ]] || {
  echo "missing QuadRF desktop entry: $desktop" >&2
  exit 1
}
grep -qx 'X-QuadRF-Desktop=true' "$desktop" || {
  echo "desktop entry is missing X-QuadRF-Desktop=true" >&2
  exit 1
}
[[ -x /usr/libexec/quadrf-mesh-mdns ]] || {
  echo "missing /usr/libexec/quadrf-mesh-mdns" >&2
  exit 1
}
[[ -x /usr/libexec/quadrf-mesh-open ]] || {
  echo "missing /usr/libexec/quadrf-mesh-open" >&2
  exit 1
}
grep -q -- '--no-window' /usr/libexec/quadrf-mesh-open || {
  echo "quadrf-mesh-open is missing --no-window" >&2
  exit 1
}
[[ -x /usr/lib/quadrf/apply.d/47-meshtasticd ]] || {
  echo "missing /usr/lib/quadrf/apply.d/47-meshtasticd" >&2
  exit 1
}
if grep -Fq quadrf.local /usr/libexec/quadrf-mesh-open \
        /usr/libexec/quadrf-mesh-mdns \
        /usr/libexec/quadrf-meshtasticd-launch; then
  echo "mesh helpers must not hardcode quadrf.local" >&2
  exit 1
fi
[[ "$(/usr/libexec/quadrf-mesh-mdns)" == quadrf.local ]] || {
  echo "quadrf-mesh-mdns default is not quadrf.local" >&2
  exit 1
}
[[ -f /usr/share/quadrf/apps.d/quadrf-meshtasticd.json ]] || {
  echo "missing QuadRF apps.d descriptor" >&2
  exit 1
}
apps_json=/usr/share/quadrf/apps.d/quadrf-meshtasticd.json
grep -Eq '"open": "https://[a-z0-9.-]+\.local:9443/"' "$apps_json" || {
  echo "apps.d descriptor must open the Meshtastic web UI on this unit :9443" >&2
  exit 1
}
grep -Fq '"id": "quadrf-mesh"' "$apps_json" || {
  echo "apps.d descriptor must keep id quadrf-mesh" >&2
  exit 1
}
grep -Fq '"service": "quadrf-meshtasticd.service"' "$apps_json" || {
  echo "apps.d descriptor must keep service quadrf-meshtasticd.service" >&2
  exit 1
}
grep -Fq '"exclusive": true' "$apps_json" || {
  echo "apps.d descriptor must stay exclusive" >&2
  exit 1
}
grep -Fq '"ready_port": 9443' "$apps_json" || {
  echo "apps.d descriptor must keep ready_port 9443" >&2
  exit 1
}
if ! grep -Fq 'meshtasticd-quadrf' "$apps_json" || ! grep -Fq 'quadrf-lora-phy' "$apps_json"; then
  echo "apps.d descriptor must list meshtasticd-quadrf and quadrf-lora-phy" >&2
  exit 1
fi
launch=/usr/libexec/quadrf-meshtasticd-launch
grep -Fq /etc/quadrf/tls/fullchain.pem "$launch" || {
  echo "launch script must prefer the appliance TLS fullchain" >&2
  exit 1
}
grep -Fq /etc/quadrf/tls/privkey.pem "$launch" || {
  echo "launch script must prefer the appliance TLS privkey" >&2
  exit 1
}
grep -Fq 'openssl req -x509' "$launch" || {
  echo "launch script must keep a self-signed TLS fallback" >&2
  exit 1
}
grep -Fq 'x11_active' "$launch" || {
  echo "launch script must skip the monitor when X11 is absent" >&2
  exit 1
}
apply=/usr/lib/quadrf/apply.d/47-meshtasticd
grep -Fq /etc/quadrf/tls/fullchain.pem "$apply" || {
  echo "apply hook must refresh ssl/ from the appliance fullchain" >&2
  exit 1
}
grep -Fq 'try-restart quadrf-meshtasticd.service' "$apply" || {
  echo "apply hook must reload meshtasticd after a cert refresh" >&2
  exit 1
}
grep -qx 'TryExec=meshtasticd-quadrf' "$desktop" || {
  echo "desktop entry is missing TryExec=meshtasticd-quadrf" >&2
  exit 1
}
[[ -f /usr/share/meshtasticd/web/index.html ]] || {
  echo "missing uncompressed Meshtastic web UI index.html" >&2
  ls -la /usr/share/meshtasticd/web 2>&1 || true
  exit 1
}
[[ ! -e /usr/share/meshtasticd/web/index.html.gz ]] || {
  echo "web UI still has precompressed index.html.gz; ulfius cannot serve it" >&2
  exit 1
}
[[ -f /usr/share/icons/hicolor/scalable/apps/io.github.radioroy.QuadRFMesh.svg ]] || {
  echo "missing QuadRF application icon" >&2
  exit 1
}
[[ -f /usr/share/metainfo/io.github.radioroy.QuadRFMesh.metainfo.xml ]] || {
  echo "missing AppStream metainfo" >&2
  exit 1
}
test ! -e /etc/systemd/system/multi-user.target.wants/quadrf-meshtasticd.service
test ! -e /etc/systemd/system/multi-user.target.wants/quadrf-lora-phy.service
test ! -e /etc/systemd/system/multi-user.target.wants/quadrf-ble-bridge.service
test -f /usr/lib/systemd/system/quadrf-ble-bridge.service
test -f /usr/lib/systemd/system/quadrf-lora-phy.service
test -f /usr/lib/systemd/system/quadrf-meshtasticd.service

apt-get purge --yes quadrf-mesh quadrf-meshtasticd quadrf-lora-phy quadrf-fpga quadrf-soapy bluez-firmware
test ! -e /usr/bin/quadrf-lora-phy
test ! -e /usr/bin/meshtasticd-quadrf
test ! -e /usr/bin/quadrf-mesh-monitor
test ! -e /usr/bin/quadrf-ble-bridge
if dpkg-query --show quadrf-lora-phy quadrf-meshtasticd quadrf-mesh >/dev/null 2>&1; then
  echo "project packages remain registered after purge" >&2
  exit 1
fi
echo "Debian Trixie arm64 install and purge passed"
