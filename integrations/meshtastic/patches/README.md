# Meshtastic Firmware Patches

These patches are applied to the pinned Meshtastic firmware commit during dependency preparation.

## Patch Series

### `backend/`
Required patches that add QuadRF radio support:
- Adds `QuadRFRadio`, `QuadRFPingModule`, `QuadRFParrotModule`, and Air-IPC headers.
- Registers the `quadrf` soft radio module in Portduino.
- Configures Meshtastic to connect to the Air-IPC Unix domain socket instead of using a hardware SPI bus.
- Pins SCons build tooling and prunes unused sensor dependencies.

### `downstream/`
Optional convenience patches (disabled by default):
- Populates hardware model and MAC address in node info replies.
- Sets default node display name.
- Adds single-page application routing support for the built-in web server.
- Adds per-client queue fanout and delivery optimizations for PhoneAPI.

To enable the optional downstream patches when preparing dependencies, set:

```bash
QUADRF_ENABLE_DOWNSTREAM_PATCHES=1 ./scripts/prepare-dependencies.sh
```
