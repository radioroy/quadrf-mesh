# Meshtastic Integration

This folder contains the code and patches needed to integrate Meshtastic with QuadRF hardware.

## Structure

- **`backend/`**: Contains `QuadRFRadio`, our custom Meshtastic `RadioInterface` implementation that talks to `quadrf-lora-phy` over Air-IPC.
- **`patches/backend/`**: Patches applied to upstream Meshtastic firmware to register `QuadRFRadio`.
- **`patches/downstream/`**: Optional patches for device branding and default node name configuration.
- **`examples/config.yaml`**: A sample configuration file for running `meshtasticd`.
- **`dependencies.lock`**: Pinned versions for the Meshtastic firmware and web interface.

## Preparing Dependencies

Before building Debian packages, run:

```bash
./scripts/prepare-dependencies.sh
```

This clones the pinned Meshtastic firmware release, applies the QuadRF backend patches, and downloads the web interface assets into `vendor/`.
