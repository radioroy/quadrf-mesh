# QuadRF Mesh

QuadRF Mesh is an experimental Meshtastic daemon and LoRa-compatible PHY for the QuadRF.

## Architecture

```text
  Phone / CLI                 Browser
  PhoneAPI (TCP 4403)         HTTPS :9443
           \                     /
            \                   /
             ▼                 ▼
          quadrf-meshtasticd
                    │ Air-IPC socket (/run/quadrf/phy.sock)
                    ▼
          quadrf-lora-phy (DSP, framing, radio control)
                    │ SoapySDR + quadrf-jtag
                    ▼
          QuadRF SDR Hardware
```

- `quadrf-lora-phy`: Controls the RF frontend and FPGA, performs modulation/demodulation, and serves the Air-IPC socket.
- `quadrf-meshtasticd`: Meshtastic portduino daemon running our out-of-tree radio interface (`QuadRFRadio`). Serves the web UI on **9443** and PhoneAPI on **4403**.
- **Air-IPC**: A simple Unix domain socket protocol that passes raw radio packets between the PHY and the mesh stack (see [docs/air-ipc-v1.md](docs/air-ipc-v1.md)).



## Building from source

### Prerequisites

```bash
sudo apt-get install cmake g++ libfftw3-dev libsdl2-dev libsoapysdr-dev libssl-dev ninja-build
```

### Build the PHY and offline tests

```bash
cmake -S . -B build -DPHY_USE_NEON=OFF
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Enable NEON acceleration for Pi 5 ARM hardware:

```bash
cmake -S . -B build -DPHY_USE_NEON=ON -DPHY_JTAG_PATH=/usr/bin/quadrf-jtag
cmake --build build --parallel
```



## Running on QuadRF

Start both services:

```bash
sudo systemctl start quadrf-lora-phy quadrf-meshtasticd
```



### Connecting

- **Meshtastic Mobile App (Bluetooth LE)**: Press the **BLE** button in Mesh Monitor (or run `sudo systemctl start quadrf-ble-bridge`) to toggle Bluetooth on or off. Connect directly from the iOS or Android Meshtastic app.
- **Web UI**: Open `https://<hostname>.local:9443` in a browser (or `https://quadrf.local:9443`).
- **PhoneAPI / CLI**: TCP 4403 (`meshtastic --host 127.0.0.1:4403 --info`).
- **Mesh Monitor**: Desktop UI on display `:1` or launched from the **Mesh** app icon (see [docs/mesh-monitor.md](docs/mesh-monitor.md)).
  - **BLE**: Toggles Bluetooth on or off for mobile app connections.
  - **RANGE**: Sends pings at 5s or 10s intervals.
  - **Parrot**: Toggles parrot repeater mode.
  - **Clear Log**: Clears the packet log.



### Modem Presets and Frequency

All Meshtastic modem presets from Short Turbo through Very Long Slow are supported (sync word 0x2B, 16-symbol preamble, explicit header, CRC, LDRO at symbol times of 16.4 ms or more):

| Meshtastic `modem_preset` | PHY key | BW (kHz) | SF | CR | Air-IPC id |
| --- | --- | ---: | ---: | --- | ---: |
| `SHORT_TURBO` (default) | `shortturbo` | 500 | 7 | 4/5 | 0 |
| `SHORT_FAST` | `shortfast` | 250 | 7 | 4/5 | 1 |
| `SHORT_SLOW` | `shortslow` | 250 | 8 | 4/5 | 2 |
| `MEDIUM_FAST` | `mediumfast` | 250 | 9 | 4/5 | 3 |
| `MEDIUM_SLOW` | `mediumslow` | 250 | 10 | 4/5 | 4 |
| `LONG_FAST` | `longfast` | 250 | 11 | 4/5 | 5 |
| `LONG_MODERATE` | `longmoderate` | 125 | 11 | 4/8 | 6 |
| `LONG_SLOW` | `longslow` | 125 | 12 | 4/8 | 7 |
| `VERY_LONG_SLOW` | `verylongslow` | 62.5 | 12 | 4/8 | 8 |
| `LONG_TURBO` | `longturbo` | 500 | 11 | 4/8 | 9 |

`VERY_LONG_SLOW` is deprecated in Meshtastic 2.5+ and the pinned firmware maps it to Long Fast airtime; the QuadRF backend restores its last definition (62.5 kHz / SF12 / CR 4/8) so both ends agree. Its ±20 kHz CFO acquisition range is about one third of the bandwidth. Expect several seconds of airtime per text. LITE_* and NARROW_* presets, and custom modem settings that do not match a row above, revert to Short Turbo. A `lora.coding_rate` override on top of a preset is cleared (clients write back the CR 4/5 of the previous preset when switching to a CR 4/8 one) and the preset's own CR is used. Presets change live over Air-IPC `SetModem`, for example:

```bash
meshtastic --host 127.0.0.1:4403 --set lora.modem_preset LONG_FAST
```

The `PRESET:` badge in Mesh Monitor does the same from the desktop: it saves the preset in meshtasticd, which applies it and restarts so the web and phone clients reconnect with the new setting. Each node is switched on its own; set every node in the mesh to the same preset.

Mesh Monitor does not show sample-clock rate. That timing-loop correction is not a clock-offset measurement: preamble drift on the short presets is dominated by LO wander, and the long presets do not estimate it (0). QuadRF-to-QuadRF offsets measured from captures are about −7 to −14 ppm.

Measured between two QuadRFs at 5.8 GHz, TX gain 0, both directions: Short Turbo through Long Fast and Long Turbo deliver 85 to 100 % of beacon frames and nearly every mesh text; Long Moderate about 60 to 100 %. Long Slow and Very Long Slow acquire and decode the header but the payload CRC fails. The two units' LOs wander against each other by about 800 Hz RMS (swings of ±1 kHz over 10 to 30 ms), which is tens of chips within one SF12 symbol. The receiver tracks it across symbol boundaries; with the transmitted symbols known, the boundary carrier step can only be estimated to about 0.85 chip RMS at 15 dB, roughly 3 % of symbols outside the ±2-chip LDRO decision window, and the tracker still has heavier tails than that. A shared reference or a quieter LO is the fix for SF12 at 125 kHz and below.

`quadrf-lora-phy` owns the radio hardware. Startup RF (frequency, gain, bandwidth, antenna mask, preset) is set in `/etc/default/quadrf-lora-phy`; restart `quadrf-lora-phy` (and `quadrf-meshtasticd`) after edits. Live retuning is done with the QuadRF appliance GUI Tx or Rx frequency slider: PHY follows the move within a second, keeping TX on the channel and the RX LO 500 kHz below it (low-IF receive, `QUADRF_LORA_PHY_RX_IF_KHZ`). The GUI TX/RX Ch: checkboxes and 4-channel (interleaved) RX mode also override the packaged `--tx-ant 1 --rx-ant 1` until PHY is restarted. Meshtastic `lora.override_frequency` must remain `0` (it does not tune the radio).

## Building Debian packages

To build `.deb` packages for QuadRF:

```bash
./scripts/prepare-dependencies.sh
./scripts/build-debian.sh
```

This builds `quadrf-lora-phy` and `quadrf-meshtasticd` packages.

## License & Upstream Acknowledgments

This project is licensed under the GNU General Public License v3.0 (GPL-3.0). See [LICENSE](LICENSE) for details. Component-level copyright and licensing details are cataloged in [debian/copyright](debian/copyright).

### Upstream DSP Acknowledgments

- **[gr-lora_sdr](https://github.com/tapparelj/gr-lora_sdr)** (GPL-3.0): Joachim Tapparel, EPFL Telecommunication Circuits Laboratory, and contributors. The bit-level LoRa coding chain (whitening sequence, header generation and checksum, Hamming FEC, diagonal interleaver) and analytic chirp modulation adapt conventions from `gr-lora_sdr`.
  - Reference: J. Tapparel, O. Afisiadis, P. Mayoraz, A. Balatsoukas-Stimming, and A. Burg, *"An Open-Source LoRa Physical Layer Prototype on GNU Radio,"* 2020 IEEE SPAWC.
  - Reference: J. Tapparel and A. Burg, *"Design and Implementation of LoRa Physical Layer in GNU Radio,"* Proceedings of the GNU Radio Conference, 2024.
- **[gr-lora](https://github.com/rpp0/gr-lora)** (GPL-3.0): Pieter Robyns, Peter Quax, Wim Lamotte, and William Thenaers (Hasselt University). Foundational open-source LoRa SDR reverse-engineering, oversampled dechirp folding, and synchronization concepts, which `gr-lora_sdr` and subsequent SDR implementations use.
  - Reference: P. Robyns, P. Quax, W. Lamotte, and W. Thenaers, *"gr-lora: An efficient LoRa decoder for GNU Radio,"* Zenodo, 2017. [doi:10.5281/zenodo.853201](https://doi.org/10.5281/zenodo.853201).
- **[Meshtastic](https://github.com/meshtastic)** (GPL-3.0-only): Mesh networking stack and web interface. `quadrf-meshtasticd` integrates the Meshtastic portduino daemon with an out-of-tree `RadioInterface` backend communicating via Air-IPC.



## Future improvements

- Soft-decision Hamming decoding
- Low-SNR fractional CFO/STO estimators (Bernier / Yang-Wei)
- Post-sync channel filter centred on the measured CFO for the 62.5/125 kHz presets



### Trademarks & Disclaimer

QuadRF is a trademark of Scale RF Inc. This repository is an independent open-source project and is not an official Scale RF Inc. or Meshtastic product or support channel.