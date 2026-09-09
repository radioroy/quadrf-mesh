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



## Future improvements

- Soft-decision Hamming decoding
- Low-SNR fractional CFO/STO estimators (Bernier / Yang–Wei)
- Remaining Meshtastic modem presets (SF 8–12)



## Building from source



### Prerequisites

On Debian/Ubuntu systems:

```bash
sudo apt-get install cmake g++ libfftw3-dev libsdl2-dev libsoapysdr-dev libssl-dev ninja-build
```



### Build the PHY and offline tests

```bash
cmake -S . -B build -DPHY_USE_NEON=OFF
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Enable NEON acceleration for the Pi 5 ARM hardware:

```bash
cmake -S . -B build -DPHY_USE_NEON=ON -DPHY_JTAG_PATH=/usr/bin/quadrf-jtag
cmake --build build --parallel
```



## Running on QuadRF

On a QuadRF, start both services:

```bash
sudo systemctl start quadrf-lora-phy quadrf-meshtasticd
```

- **Web UI**: `https://<hostname>.local:9443` (Meshtastic HTTPS UI; not PhoneAPI).
- **Mesh Monitor**: The desktop monitor window launches automatically on display `:1` (see [docs/mesh-monitor.md](docs/mesh-monitor.md)).
- **PhoneAPI / CLI**: TCP 4403 (phone apps and `meshtastic --host`):

```bash
meshtastic --host 127.0.0.1:4403 --info
```



### Frequency and modem preset

`quadrf-lora-phy` owns the radio. Center frequency is `QUADRF_LORA_PHY_FREQ` (default 5800 MHz) at service start. The appliance GUI LO slider can retune that live; a PHY restart writes the packaged `--freq` again.

Meshtastic `lora.override_frequency` stays `0` (the web UI rejects 5800). It does not tune the LO. `QuadRFRadio` clears a nonzero override on startup.

`lora.modem_preset` is pushed to PHY over Air-IPC `SetModem` (Short Turbo or Short Fast). Frequency stays on PHY `--freq` / the appliance GUI. Other Meshtastic presets are not implemented in the demodulator and stay on Short Turbo.

```bash
meshtastic --host 127.0.0.1:4403 --set lora.modem_preset SHORT_FAST
```



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



### Trademarks & Disclaimer

QuadRF is a trademark of Scale RF Inc. This repository is an independent open-source project and is not an official Scale RF Inc. or Meshtastic product or support channel.