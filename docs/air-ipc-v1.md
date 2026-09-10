# Air-IPC v1 Protocol

Air-IPC is a Unix domain socket protocol used to pass raw radio frames between `quadrf-lora-phy` and `quadrf-meshtasticd`.

The header definition is self-contained in `include/quadrf/air_ipc.hpp`.

## Socket Location

By default, the server listens on:

```text
/run/quadrf/phy.sock
```

## Frame Header

Every message begins with a 6-byte header:

| Offset | Size | Field | Value / Description |
| ---: | ---: | --- | --- |
| 0 | 2 | Magic bytes | `0x51 0x46` (ASCII `'QF'`) |
| 2 | 1 | Protocol version | `1` |
| 3 | 1 | Message type | `1` = TxEnqueue, `2` = RxIndicate, `3` = SetModem |
| 4 | 2 | Payload length | uint16 (big-endian) |

Payload integers are little-endian.

## Message Types

### Type 1: `TxEnqueue`

Sent from the mesh daemon to `quadrf-lora-phy` to transmit a frame.

| Offset | Size | Field | Description |
| ---: | ---: | --- | --- |
| 0 | 8 | Center frequency | uint64 in Hz. Informational; `QuadRFRadio` always sends `0`. |
| 8 | 16..255 | Payload | Meshtastic air-frame bytes (16-byte header + encrypted payload) |

`quadrf-lora-phy` does not retune from this field. Modulation uses whichever center the local LO is already on.

PHY programs TX/RX frequency once at startup from `--freq` (`QUADRF_LORA_PHY_FREQ`). After that, mute/unmute only gates PA_BIAS / FPGA `disable_tx` and leaves frequency, gain, and bandwidth alone. The GUI LO control on `quadrf.local` overrides the packaged PHY center until PHY is restarted. Meshtastic `lora.override_frequency` stays `0` and does not tune the radio.

### Type 2: `RxIndicate`

Sent from `quadrf-lora-phy` to the mesh daemon when a frame is received.

| Offset | Size | Field | Description |
| ---: | ---: | --- | --- |
| 0 | 2 | SNR | int16, signed SNR in hundredths of a dB (e.g. `1250` = +12.50 dB) |
| 2 | 2 | RSSI | int16, signed RSSI in dBm |
| 4 | 4 | CFO | int32, signed carrier frequency offset in Hz |
| 8 | 4 | Rate PPM | int32, signed sample-rate offset in hundredths of a ppm (e.g. `150` = +1.50 ppm) |
| 12 | 8 | Center frequency | uint64, PHY configured center in Hz (startup `--freq`, not a live GUI LO readout) |
| 20 | 2 | SIR | int16, worst-symbol second-tone ratio in hundredths of a dB |
| 22 | 2 | LVL | int16, uncalibrated signal level in hundredths of a dBFS |
| 24 | 16..255 | Payload | Meshtastic air-frame bytes |

### Type 3: `SetModem`

Sent from `quadrf-meshtasticd` when Meshtastic applies a modem preset (`QuadRFRadio::reconfigure()` and after the Air-IPC socket connects).

| Offset | Size | Field | Description |
| ---: | ---: | --- | --- |
| 0 | 1 | Preset | `0` = Short Turbo (500 kHz / SF7), `1` = Short Fast (250 kHz / SF7) |

Only presets `0` and `1` are supported. Any unsupported preset reverts to Short Turbo. PHY reconstructs the TX modulator and RX demodulator without altering the LO frequency.
