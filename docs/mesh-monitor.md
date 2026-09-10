# Mesh Monitor Architecture and Controls

`quadrf-mesh-monitor` is an SDL2 GUI for the phy.

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



## Sockets and IPC Topology


| Socket Path                      | Owner (Server)                       | Client                | Protocol               | Function                                                                            |
| -------------------------------- | ------------------------------------ | --------------------- | ---------------------- | ----------------------------------------------------------------------------------- |
| `/run/quadrf/phy_telemetry.sock` | `quadrf-lora-phy`                    | `quadrf-mesh-monitor` | Line-delimited JSON    | CRC-pass decode tap (including self-TX RF leak) plus a `{"type":"modem"}` status line on client connect and `SetModem`. |
| `/run/quadrf/mesh_control.sock`  | `quadrf-meshtasticd` (`QuadRFRadio`) | `quadrf-mesh-monitor` | Line-delimited ASCII   | Runtime control of daemon test modes (`RANGE`, `PARROT`) and callsign (`CALLSIGN`). Permissions: `0666`. |
| `/run/quadrf/phy.sock`           | `quadrf-lora-phy`                    | `quadrf-meshtasticd`  | Air-IPC binary framing | Framing protocol passing air packets between PHY and Meshtastic daemon.             |


---



## Controls and Buttons

### 1. BLE Bridge

- **Display**: `BLE: ON` (green) or `BLE: OFF`.
- **Operation**: Toggles systemd service `quadrf-ble-bridge` via `sudo systemctl start/stop quadrf-ble-bridge`.
- **Function**: Manages the BlueZ GATT bridge script (`/usr/bin/quadrf-ble-bridge`), which exposes the Meshtastic PhoneAPI over Bluetooth LE for mobile clients. Polled every $2\text{ s}$.



### 2. RX Mode Indicator

- **Display**: `RX: Beamforming` (mode 0) or `RX: 4-Ch Sum` (mode 1, blue border).
- **Note**: **This indicator is display-only.** Mouse hover is disabled, and clicks are ignored.
- **Hardware Register**: Polled every $2\text{ s}$ via `sudo quadrf-jtag --no-setup read 0x25`. Bit 0 indicates FPGA receiver configuration:
  - `0`: Single channel or steered beamforming weights.
  - `1`: Unsteered 4-channel sum.
- **Safety**: Uses `--no-setup` on the JTAG read to prevent disturbing the active PLL lock or RX chain.



### 3. Range Test Button (Ping)

- **Display**: `RANGE: OFF`, `RANGE: 5s`, or `RANGE: 10s` (amber when active).
- **Operation**: Clicking cycles state: $\text{OFF} \to 5\text{ s} \to 10\text{ s} \to \text{OFF}$.
- **Protocol**: Writes `SET RANGE <sec>\n` to `/run/quadrf/mesh_control.sock`.
- **Daemon Module**: Managed by `QuadRFPingModule` (`meshtastic_PortNum_RANGE_TEST_APP` / port `34`):
  - Wakes a dedicated thread that allocates data packets via `service->sendToMesh(p)`.
  - Target: Broadcast (`NODENUM_BROADCAST` = `0xFFFFFFFF`).
  - Hop Limit: $0$ (single-hop / line-of-sight test only; never relayed by mesh peers).
  - Flags: `want_response = false`, `want_ack = false`.
  - Payload: ASCII heartbeat string `seq <seq_num>`.
  - Protobuf sync: Sets `moduleConfig.has_range_test = true`, `moduleConfig.range_test.enabled = (sec > 0)`, `moduleConfig.range_test.sender = sec`.



### 4. Parrot Repeater Button

- **Display**: $16 \times 16$ pixel-art. Active state highlighted with a green border.
- **Operation**: Clicking toggles Parrot mode on or off via `SET PARROT <0|1>\n` over `/run/quadrf/mesh_control.sock`.
- **Daemon Module**: Managed by `QuadRFParrotModule`:
  - Operates promiscuously (`isPromiscuous = true`) filtering for `meshtastic_PortNum_TEXT_MESSAGE_APP` (port `1`) and `meshtastic_PortNum_RANGE_TEST_APP` (port `34`).
  - **Loop Guard**: Inspects payload prefix. Silently drops packets starting with `[P:` or `[PARROT` to prevent broadcast feedback storms across multiple repeaters.
  - **Self-Packet Filter**: Drops frames matching local node ID (`isFromUs(&mp)`).
  - **Per-Node Rate Limiter**: 16-slot circular cache enforcing a $1000\text{ ms}$ minimum cooldown per sender node ID (`mp.from`). Packets received inside the cooldown window are dropped.
  - **RF Metrics Echo**: Retrieves physical layer metrics stored by `QuadRFRadio` for the specific `mp.id`:
    - Format: `[P: snr=%+.1fdB cfo=%+.1fkHz ppm=%+.1f] <original payload>`
    - Falls back to `[P: snr=%+.1fdB]`  if cached metrics are missing.
  - **Reply Dispatch**: Sends response to `NODENUM_BROADCAST` (`0xFFFFFFFF`) on the same mesh channel with `hop_limit = 0` and `want_ack = false`.



### 5. Clear Log Button

- **Operation**: Clears the in-memory packet deque (`state.packets`) and resets the "LAST DECODED PACKET" card.
- **Scope**: Does not clear PHY statistics or systemd journals.

---



## Telemetry and Self-TX Echo Handling

Because QuadRF uses a full-duplex SDR transceiver (MAX2850 + LMS7002), the receiver chain detects the node's own transmissions through coupling.

```text
TxEnqueue ---> PHY TX chain ---> RF Antennas
                     │               │
                     │ (loopback)    │ (OTA cross-coupling)
                     ▼               ▼
                 PHY RX chain <──────┘
                     │
               Decoded Frame
                     │
           isRecentTxEcho() ?
          ┌──────────┴──────────┐
      YES │                     │ NO
          ▼                     ▼
   Tag: echo = 1         Tag: echo = 0
   Emit to Telemetry     Emit to Telemetry
   DROP from Air-IPC     FORWARD to Air-IPC
   (Daemon never sees)   (Daemon processes)
```

1. **PHY Recent TX Buffer**: `quadrf-lora-phy` maintains an 8-entry circular buffer of recent transmitted air frames (`recent_tx`).
2. **Echo Detection**: When an inbound frame decodes with valid CRC, PHY checks if the raw payload matches `recent_tx`.
3. **Air-IPC Filter**: If `isRecentTxEcho` matches, the packet is logged as `echo-drop` and discarded from Air-IPC. `quadrf-meshtasticd` never receives its own echoed packets.
4. **Monitor Display**: `broadcastTelemetry()` forwards **all** CRC-valid decodes to `/run/quadrf/phy_telemetry.sock` with the field `"echo": 1` or `"echo": 0`.
  - **Echo Frame (**`echo: 1`**)**: Displayed in the table with `[SELF]` next to the node ID, muted grey typography, and flagged in the summary card as `[SELF-TX ECHO]` in amber.
  - **Peer Frame (**`echo: 0`**)**: Displayed with standard colored typography and flagged in the summary card as `[PEER RX]` in green.
5. **Operational Consequence**: **A packet appearing in Mesh Monitor does not confirm peer connectivity.** Only rows marked `[PEER RX]` indicate true over-the-air reception from a remote node.

---



## Parameter Authority and Configuration Synchronization

Configuration of the RF and protocol parameters involves three separate software layers with differing levels of authority:

```text
Layer                  Configuration Surface              Parameters Governed
────────────────────────────────────────────────────────────────────────────────────────
Hardware / PHY         /etc/default/quadrf-lora-phy       RF Center Freq (5800 MHz)
                       systemd quadrf-lora-phy            Modem Preset (ShortTurbo)
                                                          TX Gain (25 dB), RX Gain (45 dB)
                                                          Antenna paths (TX 1, RX 1)

Meshtastic Daemon      meshtastic --host 127.0.0.1:4403   lora.override_frequency (0; unused)
                       /etc/meshtasticd/config.yaml       lora.modem_preset → Air-IPC SetModem
                                                          Channels, encryption, node name

Monitor GUI            quadrf-mesh-monitor                FREQ badge is static ("5800 MHz")
                                                          PRESET badge follows PHY telemetry
                                                          Runtime triggers: RANGE, PARROT, BLE
```



### 1. PHY Owns the RF Hardware

- `quadrf-lora-phy` holds authority over the MAX2850 transceiver and FPGA registers. Center frequency ($5800\text{ MHz}$), bandwidth ($500\text{ kHz}$), spreading factor ($7$), coding rate ($4/5$), and gains are locked at startup.
- In Air-IPC v1, the `TxEnqueue` message includes an 8-byte `freq_hz` field. **PHY does not retune hardware on** `TxEnqueue`**.** The field is informational; PHY modulates on whatever center frequency the local LO is already parked at.



### 2. Meshtastic Parameter Storage vs. Hardware Retuning

- `meshtasticd` accepts and stores settings such as `lora.override_frequency`, `lora.modem_preset`, and `tx_power` via PhoneAPI, CLI, or configuration files.
- `QuadRFRadio` forces `lora.override_frequency` to $0$ on init and reconfigure. Frequency is not a Meshtastic control: PHY `--freq` sets the LO at start, and the appliance GUI slider can retune it until PHY restarts. Air-IPC `TxEnqueue` always carries `freq_hz=0`.
- Changing `lora.modem_preset` in Meshtastic now sends Air-IPC `SetModem`. PHY rebuilds the TX/RX DSP (Short Turbo $500\text{ kHz}$ or Short Fast $250\text{ kHz}$) without retuning the LO. Unsupported presets stay on Short Turbo. `QUADRF_LORA_PHY_PRESET` is only the boot default until `meshtasticd` connects:
  ```bash
  meshtastic --host 127.0.0.1:4403 --set lora.modem_preset SHORT_TURBO
  ```
- **Region** `UNSET` **Default**: When the Meshtastic region is unconfigured (`UNSET`), the daemon's *software* frequency falls back to a sub-GHz slot (around $906.875\text{ MHz}$ or a hashed US slot). That value is not sent to PHY and does not move the LO.



### 3. Appliance Web UI and Out-of-Band Hardware Overrides

- The QuadRF appliance web interface (`quadrf.local:80/443`) and direct `quadrf-jtag` commands write directly to transceiver registers via SPI.
- Changing LO frequencies or gains in the appliance GUI overrides hardware registers out-of-band without notifying `quadrf-lora-phy` or `quadrf-meshtasticd`.
- This is the supported live frequency control. Meshtastic does not store or display the live LO.
- Additionally, restarting `quadrf-lora-phy` reprograms the MAX2850 using startup arguments (`/etc/default/quadrf-lora-phy`), reverting manual GUI adjustments.
- During idle states, PHY maintains the MAX2850 PLL locked in TX mode, gating transmission using only `PA_BIAS` and FPGA `disable_tx` to eliminate PLL settle delays. Invoking `quadrf-jtag --status` (without `--no-setup`) disrupts the synthesizer and desenses the RX chain.
- If hardware registers are disturbed, restart the stack to restore calibrated RF state:
  ```bash
  sudo systemctl restart quadrf-lora-phy quadrf-meshtasticd
  ```



### 4. Monitor Header Labels

- The `CALL:` badge displays the active station callsign in uppercase, synchronized with `/etc/quadrf/quadrf.conf` and `quadrf-meshtasticd` via `mesh_control.sock`.
- `FREQ: 5800 MHz` is a hardcoded UI constant. It does not follow the appliance GUI LO slider.
- `PRESET:` follows PHY. On telemetry connect, and after Air-IPC `SetModem`, PHY writes `{"type":"modem","preset":"shortturbo"|"shortfast"}`. The badge maps those keys to `ShortTurbo` / `ShortFast`. It does not read Meshtastic config (unsupported presets stay Short Turbo on PHY).



### 5. Service Coupling

- `quadrf-meshtasticd.service` declares `BindsTo=quadrf-lora-phy.service` and `PropagatesStopTo=quadrf-lora-phy.service`. Stopping `quadrf-meshtasticd` terminates `quadrf-lora-phy`. Both services must be restarted together when applying configuration changes.

