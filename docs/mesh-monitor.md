# Mesh Monitor Architecture and Controls

`quadrf-mesh-monitor` is an SDL2 desktop GUI.

```text
                     ┌──────────────────────┐
                     │ quadrf-mesh-monitor  │
                     │  (SDL2, DISPLAY=:1)  │
                     └───┬──────────────┬───┘
   JSON packet tap       │              │  Commands: GET STATUS, SET RANGE, SET PARROT,
   /run/quadrf/          │              │  SET PRESET
   phy_telemetry.sock    │              │  /run/quadrf/mesh_control.sock
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
| `/run/quadrf/mesh_control.sock` | `quadrf-meshtasticd` | `quadrf-mesh-monitor` | Line-delimited ASCII | Control interface for range pings (`SET RANGE`), parrot repeater (`SET PARROT`), modem preset (`SET PRESET <id>`), and status queries (`GET STATUS`: range, parrot, preset id, callsign). |
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

### Preset Selector
- **Display**: The `PRESET:` header badge. Clicking it opens a list of the ten supported presets (fastest to slowest airtime) with bandwidth, spreading factor (SF) and Hamming coding rate (`CR4/5` … `CR4/8`); the one on air is blue. Escape or a click outside closes it. `CR4/n` is LoRa FEC: 4 information bits plus `n−4` parity bits per codeword. `4/5` is the lightest (one parity bit, highest throughput); `4/8` is the heaviest (four parity bits, more airtime, more coding gain). The long presets use `4/8`; the rest use `4/5`.
- **Function**: Sends `SET PRESET <id>` (Air-IPC preset id, 0 = ShortTurbo ... 9 = LongTurbo) to `quadrf-meshtasticd`. The daemon does what the Meshtastic on-device preset menu does: sets `lora.modem_preset` (`use_preset` on, `coding_rate`, `channel_num` and `override_frequency` cleared), saves the config, pushes `SetModem` to the PHY at once, and restarts itself 7 s later. The restart drops the PhoneAPI connection, so the web client and phone app reconnect and read the new preset; the PHY keeps running. The badge shows `> <preset>` in amber until the PHY reports the change, or a red note if the daemon is not reachable or the PHY never confirms it (30 s).
- **Scope**: Only the local node changes. Every node in the mesh has to be switched to the same preset (from its own monitor, the app, or `meshtastic --set lora.modem_preset`), or they stop hearing each other.

### Clear Log Button
- **Function**: Clears the packet history table and resets the last-decoded packet card.

---

## Header Badges

- **`CALL:`**: Station callsign loaded from `/etc/quadrf/quadrf.conf` (defaults to `NOCALL` if unset).
- **`FREQ: 5800 MHz`**: Displays nominal 5.8 GHz operating band.
- **`PRESET:`**: Modem preset the PHY is running, as announced on the telemetry socket. Click to choose another; see Preset Selector.

---

## Packet Metrics

SNR, SIR and LVL are measured by `quadrf-lora-phy` on frames that pass the payload CRC. For those frames the PHY re-encodes the payload and measures at the transmitted chips, so the values do not depend on demodulator decisions.

- **SNR (dB)**: Channel SNR, signal power over noise power in the LoRa bandwidth (Semtech convention; ST: 500 kHz, LF: 250 kHz, VLS: 62.5 kHz). The DDC noise bandwidth excess (0.7 to 2.4 dB depending on preset) is removed. Within ±0.5 dB of the true value from the demod cliff up to about +15 dB; at higher SNR it compresses toward +20 dB (SF7) to +27 dB (SF10 and up), the floor set by the band-limited chirp edges at each symbol boundary. On air it is the post-dechirp SNR the demodulator works with, so LO phase noise and residual carrier wander count as noise: two QuadRFs at TX gain 0 read 13 to 17 dB while the burst-versus-gap power ratio in the same band is 22 to 25 dB. A preamble-only reference measurement on the raw capture agrees with the reported value to about 1 dB per frame. Colors follow the SF: amber below cliff + 7.5 dB, red below the cliff (−7.5 dB at SF7, 2.5 dB lower per SF step, −20 dB at SF12).
- **SIR (dB)**: Ratio of the wanted tone to the two strongest other tones per symbol (an unsynchronised co-SF interferer splits across two partial tones in each window), after subtracting the expected noise maxima. Within about 0.5 dB of the true power ratio for a co-SF LoRa interferer at SF10 and up, about 1.5 dB high at SF7. `>30` means nothing above the noise (3σ test). Between two QuadRFs with no interferer it reads 20 to 25 dB on short presets and 16 to 21 dB on LT/LF/LM: that is the LO phase-noise spurs and residual wander leakage of the wanted signal itself, not another transmitter. Green ≥ 12 dB, amber ≥ 6 dB (co-SF capture threshold), red below.
- **LVL (dBFS)**: Mean receive power at the DDC output relative to 16-bit full scale, signal plus noise in the channel filter. Matches the in-band burst power of the raw capture to within 0.6 dB. It follows the RX gain and is not an RSSI in dBm. Near sensitivity it is set by the noise floor.
- **CFO (Hz)**: Carrier frequency offset of the decoded frame, from the joint up-chirp / SFD-down-chirp solve. This is the difference between the peer TX LO and the local RX LO, in Hertz at RF. Between two QuadRFs at 5.8 GHz it typically sits around 10 to 15 kHz (~2 ppm of the LO). It is a real measurement the demodulator uses, and a jump of many kHz is a useful flag that the synths have moved. Self-TX echoes read near 0 Hz (same board). It is not a sample-clock error and cannot be converted to RATE ppm, because the ADC clock is not locked to the LO.
- **RATE (ppm)**: Sample-clock offset the receiver used for timing (Air-IPC Rate PPM). It is a tracking-loop value, not a measurement, and should not be read as the peer's clock error. On ST/SF/SS/MF it comes from preamble peak drift, which carrier wander dominates, so it moves around and often sits at the ±15 ppm clamp. On MS and the long presets the wander tracker absorbs timing drift and the field is 0. Wrap-point timing on captures put the actual QuadRF-to-QuadRF offset at about −7 to −14 ppm, and the CFO cannot stand in for it because the sample clock is not locked to the LO reference (CFO/f_RF is about +3 ppm). Decoding does not depend on this value.

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

- **PHY Owns the Radio**: `quadrf-lora-phy` initializes the MAX2850 transceiver and FPGA from `/etc/default/quadrf-lora-phy` (packaged: 5800 MHz, `--tx-ant 1 --rx-ant 1`). Edit that file and restart PHY to change startup RF. Live tuning is done with the QuadRF appliance GUI Tx or Rx frequency slider; PHY follows it and keeps the RX IF offset. The GUI TX/RX Ch: checkboxes and 4-channel (interleaved) RX mode override the packaged antenna mask until PHY is restarted. Meshtastic `lora.override_frequency` must remain `0` (it does not tune the radio).
- **Modem Presets**: Short Turbo (default) through Very Long Slow and Long Turbo; see the preset table in the top-level `README.md`. LITE_*/NARROW_* revert to Short Turbo. Changes apply live over Air-IPC `SetModem`.
- **Service Management**: `quadrf-meshtasticd` binds to `quadrf-lora-phy`. Stopping the daemon stops PHY. Restart both together:
  ```bash
  sudo systemctl restart quadrf-lora-phy quadrf-meshtasticd
  ```
