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

PHY programs TX/RX frequency, gain, bandwidth, and antenna mask once at startup from `/etc/default/quadrf-lora-phy` (`QUADRF_LORA_PHY_FREQ`, `_TX_GAIN`, `_RX_GAIN`, `_TX_BW`, `_RX_BW`, `_TX_ANT`, `_RX_ANT`). After that, mute/unmute only gates PA_BIAS / FPGA `disable_tx` and leaves those settings alone. The GUI Tx/Rx frequency sliders move the channel live: PHY polls both synths while idle and re-places them (TX = channel, RX = channel − IF). The GUI TX/RX Ch: checkboxes and 4-channel (interleaved) RX mode likewise override `--tx-ant 1 --rx-ant 1` until PHY restart. Meshtastic `lora.override_frequency` stays `0` and does not tune the radio.

### Type 2: `RxIndicate`

Sent from `quadrf-lora-phy` to the mesh daemon when a frame is received.

| Offset | Size | Field | Description |
| ---: | ---: | --- | --- |
| 0 | 2 | SNR | int16, channel SNR in the LoRa bandwidth, hundredths of a dB (e.g. `1250` = +12.50 dB) |
| 2 | 2 | RSSI | int16, signed RSSI in dBm |
| 4 | 4 | CFO | int32, signed carrier frequency offset in Hz |
| 8 | 4 | Rate PPM | int32, signed sample-rate offset in hundredths of a ppm (e.g. `150` = +1.50 ppm) |
| 12 | 8 | Center frequency | uint64, PHY channel in Hz (startup `--freq`, then the last GUI retune PHY followed) |
| 20 | 2 | SIR | int16, wanted tone over noise-corrected second tone in hundredths of a dB; `3000` (30 dB) = no interferer detected |
| 22 | 2 | LVL | int16, uncalibrated signal level in hundredths of a dBFS |
| 24 | 16..255 | Payload | Meshtastic air-frame bytes |

### Type 3: `SetModem`

Sent from `quadrf-meshtasticd` when Meshtastic applies a modem preset (`QuadRFRadio::reconfigure()` and after the Air-IPC socket connects).

| Offset | Size | Field | Description |
| ---: | ---: | --- | --- |
| 0 | 1 | Preset | `0` ST, `1` SF, `2` SS, `3` MF, `4` MS, `5` LF, `6` LM, `7` LS, `8` VLS, `9` LT |

| Id | Preset | BW (kHz) | SF | CR |
| ---: | --- | ---: | ---: | --- |
| 0 | Short Turbo | 500 | 7 | 4/5 |
| 1 | Short Fast | 250 | 7 | 4/5 |
| 2 | Short Slow | 250 | 8 | 4/5 |
| 3 | Medium Fast | 250 | 9 | 4/5 |
| 4 | Medium Slow | 250 | 10 | 4/5 |
| 5 | Long Fast | 250 | 11 | 4/5 |
| 6 | Long Moderate | 125 | 11 | 4/8 |
| 7 | Long Slow | 125 | 12 | 4/8 |
| 8 | Very Long Slow | 62.5 | 12 | 4/8 |
| 9 | Long Turbo | 500 | 11 | 4/8 |

Ids are append-only. PHY ignores unknown ids and keeps the current preset; the backend reverts unsupported Meshtastic presets to Short Turbo before sending. PHY reconstructs the TX modulator, the RX DDC (output rate 2·BW) and demodulator without altering the LO frequency.
