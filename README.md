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

Only two modem presets are supported:

- **Short Turbo** (`SHORT_TURBO`): 500 kHz bandwidth, SF7 (default).
- **Short Fast** (`SHORT_FAST`): 250 kHz bandwidth, SF7.

Other presets selected in Meshtastic are unsupported and revert to Short Turbo. Presets can be set via CLI:

```bash
meshtastic --host 127.0.0.1:4403 --set lora.modem_preset SHORT_FAST
```

`quadrf-lora-phy` owns the radio hardware. Startup center frequency is set by `QUADRF_LORA_PHY_FREQ` (default 5800 MHz). Live retuning can be done through the QuadRF appliance GUI LO slider. Meshtastic `lora.override_frequency` must remain `0` (it does not tune the radio).

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
- Remaining Meshtastic modem presets (SF 8-12)



### Trademarks & Disclaimer

QuadRF is a trademark of Scale RF Inc. This repository is an independent open-source project and is not an official Scale RF Inc. or Meshtastic product or support channel.