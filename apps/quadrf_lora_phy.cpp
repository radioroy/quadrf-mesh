// quadrf-lora-phy: QuadRF LoRa PHY process.
//
// Product path — air-frame IPC only (see quadrf/air_ipc.hpp):
//   meshtasticd RadioInterface  ──unix──►  quadrf-lora-phy  ──► QuadRF
//
//   ./quadrf-lora-phy [--digital|--ota] [--socket /run/quadrf/phy.sock] ...
//
// Optional PTY support: --legacy-pty exposes StreamAPI for
// serial clients against a PTY. Mesh identity and routing remain with the host.
//
// Logging → stderr. Socket path / PTY= printed on stdout for scripts.

#include <phy/bridge/session.hpp>
#include <quadrf/air_ipc.hpp>
#include <phy/lora/presets.hpp>
#include <phy/lora/receiver.hpp>
#include <phy/lora/transmitter.hpp>
#include <phy/mesh/packet.hpp>
#include <phy/radio/radio_session.hpp>
#include <phy/radio/rf_frontend.hpp>
#include <phy/radio/stream_pair.hpp>
#include <phy/stream/framer.hpp>

#include <SoapySDR/Constants.h>
#include <SoapySDR/Errors.hpp>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace phy;
using namespace phy::lora;
using namespace quadrf::air_ipc;

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) {
    g_running = false;
}

void usage(const char* prog) {
    std::cerr
        << "Usage: " << prog << " [options]\n"
        << "  --digital              FPGA digital loopback (default)\n"
        << "  --ota                  RF over the air\n"
        << "  --socket <path>        Air-IPC unix socket (default: /run/quadrf/phy.sock)\n"
        << "  --telemetry-socket <p> Telemetry broadcast unix socket (default: /run/quadrf/phy_telemetry.sock)\n"
        << "  --freq <mhz>           OTA center frequency (default: 5800)\n"
        << "  --tx-gain <db>         TX gain (default: 1 digital / 25 ota)\n"
        << "  --rx-gain <db>         RX gain (default: 10 digital / 45 ota)\n"
        << "  --tx-bw <mhz>          TX baseband filter bandwidth (default: 40 digital / 20 ota)\n"
        << "  --rx-bw <mhz>          RX baseband filter bandwidth (default: 40 digital / 4 ota)\n"
        << "  --bw <mhz>             Set both TX and RX filter bandwidths\n"
        << "  --tx-ant <mask>        TX antenna bitmask (default: 1)\n"
        << "  --rx-ant <mask>        RX antenna bitmask (default: 1)\n"
        << "  --amplitude <a>        TX amplitude 0..1 (default: 0.8 digital, 0.7 ota)\n"
        << "  --warmup <syms>        Carrier before each frame (default: 0 / 2 ota)\n"
        << "  --gap <syms>           Silence between frames (default: 64)\n"
        << "  --preset <name>        Meshtastic preset: shortturbo (default), shortfast\n"
        << "  --rx-only              OTA receive only (tx off; for uni peer RX)\n"
        << "  --pa-drain-ms <ms>     Extra idle after post-idle TX write (default: 0; env QUADRF_LORA_PHY_PA_DRAIN_MS)\n"
        << "  --selftest             Air-frame digital/OTA loopback, no IPC client\n"
        << "  --legacy-pty           Also speak StreamAPI on a PTY (bring-up only)\n"
        << "  --node <hex>           Node num for --legacy-pty (default: e58f0001)\n"
        << "  --symlink <path>       Symlink the legacy PTY slave here\n"
        << "  -h, --help\n";
}

bool parsePreset(const std::string& name, LoraParams& out) {
    return parseMeshtasticPreset(name, out);
}

const char* presetKeyFromId(uint8_t id) {
    switch (id) {
        case kPresetShortTurbo:
            return "shortturbo";
        case kPresetShortFast:
            return "shortfast";
        default:
            return nullptr;
    }
}

template <typename T, typename... Args>
void reconstruct(T& obj, Args&&... args) {
    obj.~T();
    ::new (static_cast<void*>(&obj)) T(std::forward<Args>(args)...);
}

struct ChunkQueue {
    std::mutex m;
    std::condition_variable cv;
    std::deque<std::vector<Sample>> q;
    static constexpr size_t kMaxChunks = 32;

    void push(std::vector<Sample>&& c) {
        {
            std::lock_guard<std::mutex> lk(m);
            if (q.size() >= kMaxChunks) {
                q.pop_front();
            }
            q.push_back(std::move(c));
        }
        cv.notify_one();
    }
    bool pop(std::vector<Sample>& out, int timeout_ms) {
        std::unique_lock<std::mutex> lk(m);
        if (!cv.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                         [&] { return !q.empty() || !g_running; })) {
            return false;
        }
        if (q.empty()) {
            return false;
        }
        out = std::move(q.front());
        q.pop_front();
        return true;
    }
    void flush() {
        std::lock_guard<std::mutex> lk(m);
        q.clear();
    }
};

bool writeAll(int fd, const uint8_t* data, size_t n) {
    size_t off = 0;
    while (off < n && g_running) {
        const ssize_t w = ::write(fd, data + off, n - off);
        if (w < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                pollfd pfd{fd, POLLOUT, 0};
                poll(&pfd, 1, 50);
                continue;
            }
            return false;
        }
        off += static_cast<size_t>(w);
    }
    return off == n;
}

int openPty(std::string& slave_path, const std::string& symlink_path) {
    const int master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0) {
        perror("posix_openpt");
        return -1;
    }
    if (grantpt(master) != 0 || unlockpt(master) != 0) {
        perror("grantpt/unlockpt");
        close(master);
        return -1;
    }
    char* name = ptsname(master);
    if (name == nullptr) {
        perror("ptsname");
        close(master);
        return -1;
    }
    slave_path = name;

    termios tio{};
    if (tcgetattr(master, &tio) == 0) {
        cfmakeraw(&tio);
        tio.c_cflag |= CLOCAL;
        tio.c_cflag &= static_cast<tcflag_t>(~HUPCL);
        tcsetattr(master, TCSANOW, &tio);
    }
    const int flags = fcntl(master, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(master, F_SETFL, flags | O_NONBLOCK);
    }

    if (!symlink_path.empty()) {
        unlink(symlink_path.c_str());
        if (symlink(slave_path.c_str(), symlink_path.c_str()) != 0) {
            perror("symlink");
        } else {
            std::cerr << "SYMLINK=" << symlink_path << " -> " << slave_path << "\n";
        }
    }
    return master;
}

int listenUnix(const std::string& path) {
    unlink(path.c_str());
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("socket");
        return -1;
    }
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (path.size() >= sizeof(addr.sun_path)) {
        std::cerr << "socket path too long\n";
        close(fd);
        return -1;
    }
    std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        perror("bind");
        close(fd);
        return -1;
    }
    // Node process on same box; keep the socket user-private.
    chmod(path.c_str(), 0600);
    if (listen(fd, 1) != 0) {
        perror("listen");
        close(fd);
        unlink(path.c_str());
        return -1;
    }
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
    return fd;
}

int listenUnixTelemetry(const std::string& path) {
    unlink(path.c_str());
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("socket (telemetry)");
        return -1;
    }
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (path.size() >= sizeof(addr.sun_path)) {
        std::cerr << "telemetry socket path too long\n";
        close(fd);
        return -1;
    }
    std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        perror("bind (telemetry)");
        close(fd);
        return -1;
    }
    // Allow local desktop GUI and monitoring tools to connect
    chmod(path.c_str(), 0666);
    if (listen(fd, 8) != 0) {
        perror("listen (telemetry)");
        close(fd);
        unlink(path.c_str());
        return -1;
    }
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
    return fd;
}

std::vector<uint8_t> buildSelftestAir(uint32_t from) {
    phy::mesh::Packet pkt;
    pkt.header.to = phy::mesh::kBroadcastNode;
    pkt.header.from = from;
    pkt.header.id = 0xA11CE;
    pkt.header.hop_limit = 3;
    pkt.header.hop_start = 3;
    pkt.header.channel = phy::mesh::kDefaultChannelHash;
    const char* text = "quadrf-air";
    pkt.encrypted.assign(text, text + std::strlen(text));
    std::vector<uint8_t> air;
    if (!phy::mesh::packPacket(pkt, air)) {
        return {};
    }
    return air;
}

}  // namespace

int main(int argc, char** argv) {
    bool ota = false;
    bool selftest = false;
    bool legacy_pty = false;
    bool rx_only = false;
    double center_mhz = 5800.0;
    int tx_gain = -1;
    int rx_gain = -1;
    int tx_bw = -1;
    int rx_bw = -1;
    int tx_ant = 1;
    int rx_ant = 1;
    float amplitude = -1.0f;
    int warmup_syms = -1;
    int gap_syms = 64;
    int pa_drain_ms = -1;
    std::string preset_name = "shortturbo";
    std::string socket_path = "/run/quadrf/phy.sock";
    std::string telemetry_socket_path = "/run/quadrf/phy_telemetry.sock";
    std::string symlink_path;
    uint32_t node_num = 0xE58F0001;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--ota") {
            ota = true;
        } else if (arg == "--digital") {
            ota = false;
        } else if (arg == "--rx-only") {
            rx_only = true;
        } else if (arg == "--selftest") {
            selftest = true;
        } else if (arg == "--legacy-pty") {
            legacy_pty = true;
        } else if (arg == "--freq" && i + 1 < argc) {
            center_mhz = std::stod(argv[++i]);
        } else if (arg == "--tx-gain" && i + 1 < argc) {
            tx_gain = std::stoi(argv[++i]);
        } else if (arg == "--rx-gain" && i + 1 < argc) {
            rx_gain = std::stoi(argv[++i]);
        } else if (arg == "--tx-bw" && i + 1 < argc) {
            tx_bw = std::stoi(argv[++i]);
        } else if (arg == "--rx-bw" && i + 1 < argc) {
            rx_bw = std::stoi(argv[++i]);
        } else if (arg == "--bw" && i + 1 < argc) {
            rx_bw = std::stoi(argv[++i]);
            tx_bw = rx_bw;
        } else if (arg == "--tx-ant" && i + 1 < argc) {
            tx_ant = std::stoi(argv[++i]);
        } else if (arg == "--rx-ant" && i + 1 < argc) {
            rx_ant = std::stoi(argv[++i]);
        } else if (arg == "--amplitude" && i + 1 < argc) {
            amplitude = std::stof(argv[++i]);
        } else if (arg == "--warmup" && i + 1 < argc) {
            warmup_syms = std::stoi(argv[++i]);
        } else if (arg == "--gap" && i + 1 < argc) {
            gap_syms = std::stoi(argv[++i]);
        } else if (arg == "--pa-drain-ms" && i + 1 < argc) {
            pa_drain_ms = std::stoi(argv[++i]);
        } else if (arg == "--preset" && i + 1 < argc) {
            preset_name = argv[++i];
        } else if (arg == "--socket" && i + 1 < argc) {
            socket_path = argv[++i];
        } else if (arg == "--telemetry-socket" && i + 1 < argc) {
            telemetry_socket_path = argv[++i];
        } else if (arg == "--node" && i + 1 < argc) {
            node_num = static_cast<uint32_t>(std::stoul(argv[++i], nullptr, 16));
        } else if (arg == "--symlink" && i + 1 < argc) {
            symlink_path = argv[++i];
            legacy_pty = true;  // symlink enables PTY path
        } else if (arg == "-h" || arg == "--help") {
            usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            usage(argv[0]);
            return 1;
        }
    }

    if (tx_gain < 0) {
        tx_gain = ota ? 25 : 1;
    }
    if (rx_gain < 0) {
        rx_gain = ota ? 45 : 10;
    }
    if (rx_bw < 0) {
        rx_bw = ota ? 4 : 40;
    }
    if (tx_bw < 0) {
        tx_bw = ota ? 20 : 40;
    }

    LoraParams params;
    if (!parsePreset(preset_name, params)) {
        std::cerr << "Unknown preset: " << preset_name << "\n";
        return 1;
    }
    if (amplitude < 0.0f) {
        amplitude = ota ? 0.7f : 0.8f;
    }
    if (warmup_syms < 0) {
        warmup_syms = ota ? 2 : 0;
    }
    if (pa_drain_ms < 0) {
        if (const char* env = std::getenv("QUADRF_LORA_PHY_PA_DRAIN_MS")) {
            pa_drain_ms = std::stoi(env);
        } else {
            // scalerf f7dba9d: TX ring 0.02 s (floor 2 DSI frames, ~38 ms)
            // and 3 DSI frames. write() of one PHY chunk (65536 host samples
            // @ 1 Msps, TX resampler 1→86.08 Msps) backpressures at airtime,
            // so the post-idle silence write already drains the pipeline.
            pa_drain_ms = 0;
        }
    }
    if (pa_drain_ms < 0) {
        pa_drain_ms = 0;
    }
    if (pa_drain_ms > 2000) {
        pa_drain_ms = 2000;
    }

    const uint64_t center_hz = frequencyMHzToQuantizedHz(center_mhz);

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // Legacy PhoneAPI session — only constructed when --legacy-pty is on.
    std::unique_ptr<phy::bridge::Session> session;
    if (legacy_pty) {
        phy::bridge::SessionConfig scfg;
        scfg.node_num = node_num;
        scfg.lora = params;
        session = std::make_unique<phy::bridge::Session>(scfg);
    }

    std::string slave_path;
    int pty_master = -1;
    if (legacy_pty && !selftest) {
        pty_master = openPty(slave_path, symlink_path);
        if (pty_master < 0) {
            return 1;
        }
        std::cout << "PTY=" << slave_path << std::endl;
    }

    int listen_fd = -1;
    int client_fd = -1;
    int telemetry_listen_fd = -1;
    std::vector<int> telemetry_clients;
    if (!selftest) {
        listen_fd = listenUnix(socket_path);
        if (listen_fd < 0) {
            if (pty_master >= 0) {
                close(pty_master);
            }
            return 1;
        }
        telemetry_listen_fd = listenUnixTelemetry(telemetry_socket_path);
        std::cout << "SOCKET=" << socket_path << std::endl;
        std::cout << "TELEM_SOCKET=" << telemetry_socket_path << std::endl;
        std::cerr << "quadrf-lora-phy ready: preset=" << preset_name
                  << (ota ? " mode=ota" : " mode=digital") << " ipc=" << socket_path
                  << " telem=" << telemetry_socket_path
                  << (legacy_pty ? " legacy-pty=on" : "") << "\n";
    } else {
        std::cerr << "quadrf-lora-phy selftest (" << (ota ? "ota" : "digital") << ", " << preset_name
                  << ")\n";
    }

    RfFrontend rf;
    uint16_t saved_reg = 0;
    try {
        saved_reg = rf.readRegister(0x2E);
    } catch (...) {
        saved_reg = 0;
    }

    size_t tx_enqueued = 0;
    size_t rx_delivered = 0;

    auto cleanup_socks = [&]() {
        if (client_fd >= 0) {
            close(client_fd);
            client_fd = -1;
        }
        if (listen_fd >= 0) {
            close(listen_fd);
            listen_fd = -1;
            unlink(socket_path.c_str());
        }
        for (int cfd : telemetry_clients) {
            if (cfd >= 0) {
                close(cfd);
            }
        }
        telemetry_clients.clear();
        if (telemetry_listen_fd >= 0) {
            close(telemetry_listen_fd);
            telemetry_listen_fd = -1;
            unlink(telemetry_socket_path.c_str());
        }
        if (pty_master >= 0) {
            close(pty_master);
            pty_master = -1;
        }
        if (!symlink_path.empty()) {
            unlink(symlink_path.c_str());
        }
    };

    try {
        RadioSession radio;
        radio.open();
        if (!rx_only) {
            radio.setSampleRate(SOAPY_SDR_TX, 0, params.sample_rate_hz);
        }
        radio.setSampleRate(SOAPY_SDR_RX, 0, params.sample_rate_hz);

        if (ota) {
            rf.setDigitalLoopback(false);
            if (rx_only) {
                rf.txOff();
                rf.configureRx(center_mhz, rx_gain, rx_bw, rx_ant);
            } else {
                // Arm TX once (PLL stays locked). Idle PA mute, not --tx off.
                rf.configureTx(center_mhz, tx_gain, tx_bw, tx_ant);
                rf.configureRx(center_mhz, rx_gain, rx_bw, rx_ant);
                rf.paMute();
            }
            std::cerr << "quadrf-lora-phy: OTA center=" << center_mhz << " MHz, tx_gain=" << tx_gain
                      << " dB (bw=" << tx_bw << " MHz), rx_gain=" << rx_gain
                      << " dB (bw=" << rx_bw << " MHz), tx_ant=" << tx_ant
                      << ", rx_ant=" << rx_ant << ", pa_drain=" << pa_drain_ms
                      << " ms, rx duck=0 dB during TX\n";
        } else {
            if (rx_only) {
                std::cerr << "--rx-only requires --ota\n";
                return 1;
            }
            rf.setDigitalLoopback(true);
        }

        StreamPair streams;
        if (!rx_only) {
            streams.setupTx(radio.device());
        }
        streams.setupRx(radio.device());
        const size_t mtu = streams.mtu();

        Transmitter tx(params, amplitude);
        tx.setWarmupSymbols(static_cast<uint32_t>(warmup_syms));
        tx.setGapSymbols(static_cast<uint32_t>(gap_syms));
        Receiver receiver(params);

        std::mutex modem_mu;
        std::mutex tx_mu;
        // Full-duplex SDR hears its own TX. Remember recent air frames and
        // suppress matching RX so the RadioInterface never sees self-echoes
        // (same contract as half-duplex LoRa silicon).
        std::mutex recent_tx_mu;
        std::deque<std::vector<uint8_t>> recent_tx;
        constexpr size_t kMaxRecentTx = 8;

        auto rememberTx = [&](const std::vector<uint8_t>& air) {
            std::lock_guard<std::mutex> lk(recent_tx_mu);
            recent_tx.push_back(air);
            while (recent_tx.size() > kMaxRecentTx) {
                recent_tx.pop_front();
            }
        };

        auto isRecentTxEcho = [&](const std::vector<uint8_t>& air) {
            std::lock_guard<std::mutex> lk(recent_tx_mu);
            for (const auto& tx_air : recent_tx) {
                if (tx_air == air) {
                    return true;
                }
            }
            return false;
        };

        auto enqueueAir = [&](const std::vector<uint8_t>& air) {
            if (rx_only) {
                std::cerr << "TX enqueue ignored (--rx-only)\n";
                return false;
            }
            std::lock_guard<std::mutex> modem_lk(modem_mu);
            std::lock_guard<std::mutex> lk(tx_mu);
            if (!tx.enqueue(air)) {
                std::cerr << "TX enqueue rejected (payload too long?)\n";
                return false;
            }
            rememberTx(air);
            ++tx_enqueued;
            return true;
        };

        // OTA half-duplex: MAX2850 stays in TX mode (PLL warm). Idle listen
        // gates PA_BIAS + FPGA disable_tx so PA noise does not desense RX.
        // During TX, drop RX gain via FPGA 0x6A so local coupling can't hurt
        // the LNA (in case of antenna disconnect / near-field).
        std::mutex rf_tx_mu;
        bool tx_unmuted = false;
        int rx_gain_restore = rx_gain;
        constexpr auto kPaSettle = std::chrono::milliseconds(2);
        constexpr int kRxGainDucked = 0;
        const auto pa_drain = std::chrono::milliseconds(pa_drain_ms);
        auto unmuteTxRf = [&]() -> bool {
            if (!ota || rx_only) {
                return false;
            }
            {
                std::lock_guard<std::mutex> lk(rf_tx_mu);
                if (tx_unmuted) {
                    return false;
                }
                // GUI / jtag can change 0x6A between hops; snapshot before duck.
                try {
                    rx_gain_restore = rf.readRxGainNoSetup();
                } catch (...) {
                    rx_gain_restore = rx_gain;
                }
                rf.setRxGainNoSetup(kRxGainDucked);
                rf.paUnmute();
                tx_unmuted = true;
            }
            std::this_thread::sleep_for(kPaSettle);
            std::cerr << "tx RF unmute (pa, rx gain ducked to " << kRxGainDucked
                      << " dB, restore=" << rx_gain_restore << " dB)\n";
            return true;
        };
        auto muteTxRf = [&]() {
            if (!ota || rx_only) {
                return;
            }
            std::lock_guard<std::mutex> lk(rf_tx_mu);
            if (tx_unmuted) {
                rf.paMute();
                tx_unmuted = false;
                rf.setRxGainNoSetup(rx_gain_restore);
                std::cerr << "tx RF mute (pa, rx gain restored to " << rx_gain_restore << " dB)\n";
            }
        };

        auto enqueueAirTx = enqueueAir;

        g_running = true;
        std::thread tx_thread;
        if (!rx_only) {
            tx_thread = std::thread([&]() {
                const size_t chunk_len = std::min<size_t>(mtu, 65536);
                std::vector<Sample> chunk(chunk_len);
                // Do not mute on the first idle pull: that chunk still holds
                // the burst tail (64-symbol gap ≈ 16.4 ms @ Short Turbo) plus
                // zeros to fill 65536 host samples. Mute after the next write
                // of pure silence. The TX resampler + 0.02 s ring make that
                // write() block ~one chunk of airtime (~65 ms), which covers
                // 3 DSI frames (~19 ms each). pa_drain is extra wall time
                // after that write; default 0.
                std::chrono::steady_clock::time_point idle_since;
                bool idle_timing = false;
                while (g_running) {
                    bool want_unmute = false;
                    bool want_mute_check = false;
                    {
                        std::lock_guard<std::mutex> lk(modem_mu);
                        want_unmute = ota && !tx.isIdle();
                        tx.pull(chunk.data(), chunk_len);
                        want_mute_check = ota && tx.isIdle() && tx_unmuted;
                    }
                    if (want_unmute) {
                        unmuteTxRf();
                        idle_timing = false;
                    }
                    size_t sent = 0;
                    while (g_running && sent < chunk_len) {
                        const int ret = streams.write(chunk.data() + sent, chunk_len - sent);
                        if (ret > 0) {
                            sent += static_cast<size_t>(ret);
                        } else if (ret != SOAPY_SDR_TIMEOUT) {
                            std::cerr << "TX stream error: " << SoapySDR::errToStr(ret) << "\n";
                            g_running = false;
                            break;
                        }
                    }
                    if (want_mute_check) {
                        if (!idle_timing) {
                            idle_since = std::chrono::steady_clock::now();
                            idle_timing = true;
                        } else if (std::chrono::steady_clock::now() - idle_since >= pa_drain) {
                            const auto idle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                      std::chrono::steady_clock::now() - idle_since)
                                                        .count();
                            muteTxRf();
                            std::cerr << "tx RF mute idle_wait=" << idle_ms << " ms\n";
                        }
                    }
                }
            });
        }

        ChunkQueue queue;
        std::thread rx_thread([&]() {
            while (g_running) {
                std::vector<Sample> chunk(mtu);
                const int ret = streams.read(chunk.data(), mtu);
                if (ret > 0) {
                    chunk.resize(static_cast<size_t>(ret));
                    queue.push(std::move(chunk));
                } else if (ret != SOAPY_SDR_TIMEOUT) {
                    std::cerr << "RX stream error: " << SoapySDR::errToStr(ret) << "\n";
                    g_running = false;
                    break;
                }
            }
        });

        streams.activate();
        const size_t skip_startup = static_cast<size_t>(0.5 * params.sample_rate_hz);
        size_t stream_pos = 0;

        Deframer ipc_deframer;
        phy::stream::Deframer pty_deframer;
        std::vector<uint8_t> sock_buf(4096);
        std::vector<uint8_t> pty_buf(4096);

        auto acceptClient = [&]() {
            if (listen_fd < 0) {
                return;
            }
            const int fd = ::accept(listen_fd, nullptr, nullptr);
            if (fd < 0) {
                return;
            }
            const int flags = fcntl(fd, F_GETFL, 0);
            if (flags >= 0) {
                fcntl(fd, F_SETFL, flags | O_NONBLOCK);
            }
            if (client_fd >= 0) {
                // One node per PHY; newest wins.
                close(client_fd);
            }
            client_fd = fd;
            ipc_deframer.reset();
            std::cerr << "ipc client connected\n";
        };

        auto sendRxIndicate = [&](const std::vector<uint8_t>& air, float snr_db,
                                  int32_t cfo_hz, float rate_ppm, float sir_db,
                                  float lvl_dbfs) {
            ++rx_delivered;
            if (client_fd < 0) {
                return;
            }
            RxIndicate rx;
            rx.snr_db = snr_db;
            rx.rssi_dbm = 0;  // no calibrated RSSI yet
            rx.cfo_hz = cfo_hz;
            rx.rate_ppm = rate_ppm;
            rx.freq_hz = center_hz;
            rx.sir_db = sir_db;
            rx.lvl_dbfs = lvl_dbfs;
            rx.air = air;
            std::vector<uint8_t> framed;
            if (!encodeRx(rx, framed)) {
                return;
            }
            if (!writeAll(client_fd, framed.data(), framed.size())) {
                std::cerr << "ipc write failed; dropping client\n";
                close(client_fd);
                client_fd = -1;
            }
        };

        auto acceptTelemetry = [&]() {
            if (telemetry_listen_fd < 0) {
                return;
            }
            while (true) {
                const int client = ::accept(telemetry_listen_fd, nullptr, nullptr);
                if (client < 0) {
                    break;
                }
                const int flags = fcntl(client, F_GETFL, 0);
                if (flags >= 0) {
                    fcntl(client, F_SETFL, flags | O_NONBLOCK);
                }
                telemetry_clients.push_back(client);
                char hello[80];
                const int hlen = snprintf(hello, sizeof(hello),
                    "{\"type\":\"modem\",\"preset\":\"%s\"}\n", preset_name.c_str());
                if (hlen > 0) {
                    const ssize_t written = ::write(client, hello, static_cast<size_t>(hlen));
                    (void)written;
                }
            }
        };

        auto writeTelemetryLine = [&](const char* line, size_t len) {
            for (auto it = telemetry_clients.begin(); it != telemetry_clients.end();) {
                ssize_t written = ::write(*it, line, len);
                if (written < 0 && (errno == EPIPE || errno == ECONNRESET || errno == EBADF)) {
                    close(*it);
                    it = telemetry_clients.erase(it);
                } else {
                    ++it;
                }
            }
        };

        auto broadcastModemStatus = [&]() {
            if (telemetry_clients.empty()) {
                return;
            }
            char line[80];
            const int len = snprintf(line, sizeof(line),
                "{\"type\":\"modem\",\"preset\":\"%s\"}\n", preset_name.c_str());
            if (len > 0) {
                writeTelemetryLine(line, static_cast<size_t>(len));
            }
        };

        auto broadcastTelemetry = [&](const ReceivedFrame& r, float snr, bool echo) {
            if (telemetry_clients.empty()) {
                return;
            }
            uint32_t to_node = 0, from_node = 0, packet_id = 0;
            if (r.decode.payload.size() >= 12) {
                const uint8_t* p = r.decode.payload.data();
                to_node = detail::readLe32(p);
                from_node = detail::readLe32(p + 4);
                packet_id = detail::readLe32(p + 8);
            }
            const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            char line[448];
            int len = snprintf(line, sizeof(line),
                "{\"ts\":%lld,\"from\":\"0x%08x\",\"to\":\"0x%08x\",\"id\":\"0x%08x\","
                "\"snr\":%.2f,\"sir\":%.2f,\"lvl\":%.1f,\"rssi\":%d,\"cfo\":%d,\"ppm\":%.2f,"
                "\"pre\":%u,\"len\":%zu,\"echo\":%d}\n",
                (long long)now_ms, from_node, to_node, packet_id,
                snr, (float)r.sir_db, (float)r.lvl_dbfs, (int)std::lround(r.lvl_dbfs),
                (int)r.cfo_hz, (float)r.rate_ppm,
                (unsigned)r.preamble_symbols, r.decode.payload.size(), echo ? 1 : 0);
            if (len <= 0) {
                return;
            }
            writeTelemetryLine(line, static_cast<size_t>(len));
        };

        auto flushLegacyFromRadio = [&]() {
            if (pty_master < 0 || !session) {
                return;
            }
            std::vector<uint8_t> payload;
            while (session->popFromRadio(payload)) {
                std::vector<uint8_t> framed;
                if (!phy::stream::frame(payload, framed)) {
                    continue;
                }
                if (!writeAll(pty_master, framed.data(), framed.size())) {
                    std::cerr << "PTY write failed\n";
                    g_running = false;
                    return;
                }
            }
        };

        auto drainRx = [&]() {
            ReceivedFrame r;
            while (receiver.pop(r)) {
                if (!r.synced || !r.decode.header.valid || !r.decode.crc_ok) {
                    std::cerr << "phy RX drop: synced=" << r.synced << " hdr="
                              << r.decode.header.valid << " crc=" << r.decode.crc_ok
                              << " pre=" << r.preamble_symbols << " cfo=" << r.cfo_hz
                              << " ppm=" << r.rate_ppm << " snr=" << r.snr_db
                              << " sir=" << r.sir_db << " lvl=" << r.lvl_dbfs
                              << " plen=" << r.decode.payload.size() << "\n";
                    continue;
                }
                const float snr = static_cast<float>(r.snr_db);
                // Drop OTA self-echoes (full-duplex). Keep them in --selftest.
                const bool echo = !selftest && isRecentTxEcho(r.decode.payload);
                broadcastTelemetry(r, snr, echo);
                std::cerr << "phy RX air=" << r.decode.payload.size() << " snr=" << snr
                          << " cfo=" << r.cfo_hz << " ppm=" << r.rate_ppm
                          << " sir=" << r.sir_db << " lvl=" << r.lvl_dbfs
                          << (echo ? " echo-drop" : "") << "\n";
                if (echo) {
                    continue;
                }
                sendRxIndicate(r.decode.payload, snr, static_cast<int32_t>(r.cfo_hz),
                               static_cast<float>(r.rate_ppm), static_cast<float>(r.sir_db),
                               static_cast<float>(r.lvl_dbfs));
                if (session) {
                    session->handleAirFrame(r.decode.payload.data(), r.decode.payload.size(), snr);
                }
            }
            flushLegacyFromRadio();
        };

        auto applyModem = [&](uint8_t preset_id) {
            const char* name = presetKeyFromId(preset_id);
            if (name == nullptr) {
                std::cerr << "ipc: unknown SetModem preset " << static_cast<int>(preset_id) << "\n";
                return;
            }
            LoraParams next;
            if (!parsePreset(name, next)) {
                std::cerr << "ipc: SetModem parse failed for " << name << "\n";
                return;
            }
            next.sample_rate_hz = params.sample_rate_hz;
            if (next.bandwidth_hz == params.bandwidth_hz &&
                next.spreading_factor == params.spreading_factor && next.cr == params.cr) {
                broadcastModemStatus();
                return;
            }
            std::lock_guard<std::mutex> modem_lk(modem_mu);
            std::lock_guard<std::mutex> tx_lk(tx_mu);
            tx.clearQueue();
            reconstruct(tx, next, amplitude);
            tx.setWarmupSymbols(static_cast<uint32_t>(warmup_syms));
            tx.setGapSymbols(static_cast<uint32_t>(gap_syms));
            queue.flush();
            reconstruct(receiver, next);
            params = next;
            preset_name = name;
            std::cerr << "ipc: modem preset=" << preset_name
                      << " bw=" << params.bandwidth_hz / 1e3 << " kHz sf="
                      << static_cast<int>(params.spreading_factor) << "\n";
            broadcastModemStatus();
        };

        auto handleIpcFrames = [&](const std::vector<Deframer::Frame>& frames) {
            for (const auto& fr : frames) {
                if (fr.type == MsgType::kSetModem) {
                    SetModem sm;
                    if (!decodeSetModem(fr.body, sm)) {
                        std::cerr << "ipc: bad SetModem\n";
                        continue;
                    }
                    applyModem(sm.preset);
                    continue;
                }
                if (fr.type != MsgType::kTxEnqueue) {
                    continue;
                }
                TxEnqueue txm;
                if (!decodeTx(fr.body, txm)) {
                    std::cerr << "ipc: bad TxEnqueue\n";
                    continue;
                }
                if (!frequencyRequestAccepted(txm.freq_hz, center_hz)) {
                    std::cerr << "ipc: rejecting TxEnqueue frequency " << txm.freq_hz
                              << " Hz; PHY is configured for " << center_hz << " Hz\n";
                    continue;
                }
                enqueueAirTx(txm.air);
            }
        };

        std::vector<uint8_t> selftest_air;
        if (selftest) {
            selftest_air = buildSelftestAir(node_num);
            if (selftest_air.empty()) {
                std::cerr << "selftest: failed to build air frame\n";
                g_running = false;
            }
        }

        const auto t0 = std::chrono::steady_clock::now();
        const double selftest_limit_sec = 12.0;

        while (g_running) {
            if (selftest) {
                const double elapsed =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                if (rx_delivered >= 1) {
                    break;
                }
                if (elapsed > selftest_limit_sec) {
                    std::cerr << "selftest timeout\n";
                    break;
                }
                if (stream_pos >= skip_startup && !selftest_air.empty()) {
                    for (;;) {
                        size_t queued = 0;
                        {
                            std::lock_guard<std::mutex> lk(modem_mu);
                            queued = tx.queued();
                        }
                        if (queued >= 2) {
                            break;
                        }
                        if (!enqueueAirTx(selftest_air)) {
                            break;
                        }
                    }
                }
            }

            acceptClient();
            acceptTelemetry();

            // Delayed PA mute watchdog: wait ~100 ms to settle when external enable is detected before muting
            bool tx_idle = false;
            {
                std::lock_guard<std::mutex> lk(modem_mu);
                tx_idle = tx.isIdle();
            }
            if (ota && tx_idle && !tx_unmuted) {
                static auto last_pa_poll = std::chrono::steady_clock::now();
                static auto pa_enable_detected = std::chrono::steady_clock::time_point{};
                static bool pa_waiting_to_mute = false;

                const auto now = std::chrono::steady_clock::now();
                if (!pa_waiting_to_mute) {
                    if (now - last_pa_poll >= std::chrono::milliseconds(200)) {
                        last_pa_poll = now;
                        uint16_t reg23 = 1;
                        try {
                            reg23 = rf.readRegisterNoSetup(0x23);
                        } catch (...) {}
                        if ((reg23 & 0x0001) == 0) {
                            pa_enable_detected = now;
                            pa_waiting_to_mute = true;
                        }
                    }
                } else {
                    if (now - pa_enable_detected >= std::chrono::milliseconds(100)) {
                        pa_waiting_to_mute = false;
                        bool still_idle = false;
                        {
                            std::lock_guard<std::mutex> lk(modem_mu);
                            still_idle = tx.isIdle();
                        }
                        if (still_idle && !tx_unmuted) {
                            rf.paMute();
                            std::cerr << "phy: external TX enable detected; muted PA after 100 ms settle delay\n";
                        }
                    }
                }
            }

            // Air-IPC: node → PHY
            if (client_fd >= 0) {
                pollfd pfd{client_fd, POLLIN, 0};
                if (poll(&pfd, 1, 0) > 0) {
                    if (pfd.revents & POLLIN) {
                        const ssize_t n = ::read(client_fd, sock_buf.data(), sock_buf.size());
                        if (n > 0) {
                            std::vector<Deframer::Frame> frames;
                            ipc_deframer.feed(sock_buf.data(), static_cast<size_t>(n), frames);
                            handleIpcFrames(frames);
                        } else {
                            close(client_fd);
                            client_fd = -1;
                            std::cerr << "ipc client disconnected\n";
                        }
                    } else if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                        close(client_fd);
                        client_fd = -1;
                        std::cerr << "ipc client disconnected\n";
                    }
                }
            }

            // Legacy PTY StreamAPI
            if (pty_master >= 0 && session) {
                pollfd pfd{pty_master, POLLIN, 0};
                if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
                    const ssize_t n = ::read(pty_master, pty_buf.data(), pty_buf.size());
                    if (n > 0) {
                        std::vector<std::vector<uint8_t>> payloads;
                        pty_deframer.feed(pty_buf.data(), static_cast<size_t>(n), payloads);
                        for (const auto& p : payloads) {
                            std::vector<std::vector<uint8_t>> air_out;
                            session->handleToRadio(p.data(), p.size(), air_out);
                            for (const auto& air : air_out) {
                                enqueueAirTx(air);
                            }
                        }
                        flushLegacyFromRadio();
                    }
                }
            }

            std::vector<Sample> chunk;
            bool had_samples = false;
            while (queue.pop(chunk, had_samples ? 0 : 20)) {
                had_samples = true;
                size_t off = 0;
                if (stream_pos < skip_startup) {
                    off = std::min(chunk.size(), skip_startup - stream_pos);
                }
                stream_pos += chunk.size();
                if (off < chunk.size()) {
                    receiver.feed(chunk.data() + off, chunk.size() - off);
                }
                drainRx();
            }
            if (!had_samples) {
                flushLegacyFromRadio();
            }
        }

        receiver.flush();
        drainRx();

        g_running = false;
        if (tx_thread.joinable()) {
            tx_thread.join();
        }
        rx_thread.join();
        streams.deactivate();

        if (ota) {
            muteTxRf();
            rf.txOff();
        }
        try {
            rf.writeRegister(0x2E, saved_reg);
        } catch (...) {
        }

        cleanup_socks();

        std::cerr << "stats: tx_enqueued=" << tx_enqueued << " rx_delivered=" << rx_delivered
                  << " tx_started=" << tx.framesStarted() << "\n";

        if (selftest) {
            const bool pass = rx_delivered >= 1;
            std::cerr << (pass ? "PASS" : "FAIL") << ": selftest air-frame loopback\n";
            return pass ? 0 : 1;
        }
        return 0;
    } catch (const std::exception& ex) {
        try {
            rf.writeRegister(0x2E, saved_reg);
        } catch (...) {
        }
        cleanup_socks();
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}
