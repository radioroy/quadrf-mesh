# Mesh Monitor Architecture and Controls

`quadrf-mesh-monitor` is an SDL2 desktop GUI.

```text
                     ┌──────────────────────┐
                     │ quadrf-mesh-monitor  │
                     │  (SDL2, DISPLAY=:1)  │
                     └───┬──────────────┬───┘
   JSON packet tap       │              │  Commands: GET STATUS, SET RANGE, SET PARROT
   /run/quadrf/          │              │  /run/quadrf/mesh_control.sock
   phy_telemetry.sock    │              │
                         ▼              ▼
              ┌──────────────────┐   ┌────────────────────┐
              │ quadrf-lora-phy  │   │ quadrf-meshtasticd │
              │ (SDR DSP engine) │   │ (Meshtastic stack) │
              └────────┬─────────┘   └──────────┬─────────┘
                       │                        │
                       └───────► Air-IPC ◄──────┘
                            /run/quadrf/phy.sock
```

---

## Sockets and IPC

| Socket Path | Server | Client | Format | Purpose |
| --- | --- | --- | --- | --- |
| `/run/quadrf/phy_telemetry.sock` | `quadrf-lora-phy` | `quadrf-mesh-monitor` | Line-delimited JSON | Decoded packet stream (channel SNR, SIR, LVL, RSSI) and `{"type":"modem"}` preset updates. |
| `/run/quadrf/mesh_control.sock` | `quadrf-meshtasticd` | `quadrf-mesh-monitor` | Line-delimited ASCII | Control interface for range pings (`SET RANGE`), parrot repeater (`SET PARROT`), and callsign queries (`GET STATUS`). |
| `/run/quadrf/phy.sock` | `quadrf-lora-phy` | `quadrf-meshtasticd` | Air-IPC binary | Bidirectional air-frame exchange between PHY and Meshtastic daemon. |

---

## Controls and Buttons

### BLE Button
- **Display**: `BLE: ON` (green) or `BLE: OFF`.
- **Function**: Toggles Bluetooth on or off (`quadrf-ble-bridge` systemd service). Enable this to connect to the QuadRF node from the Meshtastic mobile app (iOS or Android) over Bluetooth LE.

### Range Button (Pings)
- **Display**: `RANGE: OFF`, `RANGE: 5s`, or `RANGE: 10s` (amber when active).
- **Function**: Runs ping tests. Clicking cycles between OFF, 5 s, and 10 s intervals. Sends periodic single-hop broadcast heartbeats (`seq <n>`) on port 34 (`RANGE_TEST_APP`) to measure link range and packet delivery.

### Parrot Repeater Button
- **Display**: Pixel-art parrot icon (green border when active).
- **Function**: Toggles parrot repeater mode on or off. When enabled, incoming text messages (port 1) and range test pings (port 34) are repeated back to broadcast with hop limit 0, appending station callsign and RF metrics (`[P: DE <call> snr=... sir=... lvl=... cfo=...]`). Includes loop protection against repeater feedback and 1000 ms per-sender rate limiting.

### Clear Log Button
- **Function**: Clears the packet history table and resets the last-decoded packet card.

---

## Header Badges

- **`CALL:`**: Station callsign loaded from `/etc/quadrf/quadrf.conf` (defaults to `NOCALL` if unset).
- **`FREQ: 5800 MHz`**: Displays nominal 5.8 GHz operating band.
- **`PRESET:`**: Displays current active PHY modem preset (`ShortTurbo` or `ShortFast`).

---

## Self-TX Echo vs. Peer Reception

Due to full-duplex SDR transceiver operation, the QuadRF receiver detects its own transmissions via antenna cross-coupling.

1. **Air-IPC Filtering**: `quadrf-lora-phy` checks decoded frames against a buffer of recent transmissions. Local echoes are dropped (`echo-drop`) and never forwarded to `quadrf-meshtasticd`.
2. **Monitor Display**: All valid decodes are sent to the monitor telemetry socket:
   - **Local Echo (`[SELF]`)**: Displayed with grey typography and marked `[SELF-TX ECHO]` in amber.
   - **Peer Reception (`[PEER RX]`)**: Displayed with standard colors and marked `[PEER RX]` in green.
3. **Connectivity Note**: Only packets marked `[PEER RX]` indicate over-the-air reception from a remote peer node.

---

## Configuration and Radio Ownership

- **PHY Owns the Radio**: `quadrf-lora-phy` initializes the MAX2850 transceiver and FPGA at 5800 MHz (`QUADRF_LORA_PHY_FREQ`). Live LO tuning is performed through the QuadRF appliance GUI slider. Meshtastic `lora.override_frequency` must remain `0` (it does not tune the radio).
- **Two Supported Modem Presets**:
  - `ShortTurbo` (`SHORT_TURBO`): 500 kHz bandwidth, SF7 (default).
  - `ShortFast` (`SHORT_FAST`): 250 kHz bandwidth, SF7.
  Setting any other preset in Meshtastic reverts to Short Turbo. Changes apply live over Air-IPC `SetModem`.
- **Service Management**: `quadrf-meshtasticd` binds to `quadrf-lora-phy`. Stopping the daemon stops PHY. Restart both together:
  ```bash
  sudo systemctl restart quadrf-lora-phy quadrf-meshtasticd
  ```
