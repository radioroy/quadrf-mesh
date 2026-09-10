# Meshtastic Integration

This folder contains the code and patches needed to integrate Meshtastic with QuadRF hardware.

## Structure

- **`backend/`**: Contains `QuadRFRadio` (Air-IPC radio interface), `QuadRFPingModule` (range pings), and `QuadRFParrotModule` (parrot repeater).
- **`patches/backend/`**: Patches applied to upstream Meshtastic firmware to wire up the QuadRF radio backend.
- **`patches/downstream/`**: Optional patches for device branding, web UI routing, and PhoneAPI client delivery.
- **`examples/config.yaml`**: Sample configuration file for running `meshtasticd`.
- **`dependencies.lock`**: Pinned versions for Meshtastic firmware and web interface assets.

## Preparing Dependencies

Before building Debian packages, run:

```bash
./scripts/prepare-dependencies.sh
```

This clones the pinned Meshtastic firmware release, applies the QuadRF backend patches, and downloads the web interface assets into `vendor/`.
