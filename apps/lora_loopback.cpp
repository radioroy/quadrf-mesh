// Full LoRa frame loopback / soak over the QuadRF.
//
// Repeats a complete frame (preamble / sync / SFD / header / payload / CRC)
// with silence gaps and decodes the RX stream live with the streaming
// Receiver (chunk-fed state machine, no whole-capture pass). Reports frame
// decode rate, chip errors, rate/CFO, and dechirp SNR.
//
// Default path is FPGA digital loopback (reg 0x2E bit 2, RF untouched).
// --ota configures the RF chips instead and leaves loopback off.
// Long/low-SNR soak: --ota --frames 0 --no-dump --amplitude <a> [--max-per].

#include <phy/lora/coding.hpp>
#include <phy/lora/modulator.hpp>
#include <phy/lora/presets.hpp>
#include <phy/lora/receiver.hpp>
#include <phy/lora/transmitter.hpp>
#include <phy/radio/radio_session.hpp>
#include <phy/radio/rf_frontend.hpp>
#include <phy/radio/stream_pair.hpp>
#include <phy/util/debug_dumper.hpp>
#include <phy/util/stats.hpp>

#include <SoapySDR/Constants.h>
#include <SoapySDR/Errors.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace phy;
using namespace phy::lora;

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) {
    g_running = false;
}

void usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --ota              RF loopback instead of digital loopback\n"
              << "  --freq <mhz>       OTA center frequency (default: 5800)\n"
              << "  --tx-gain <db>     OTA TX gain (default: 1)\n"
              << "  --rx-gain <db>     OTA RX gain (default: 10)\n"
              << "  --tx-ant <mask>    TX antenna bitmask (default: 1;\n"
              << "                     1/2/4/8 = ant1..4, 15 = all)\n"
              << "  --rx-ant <mask>    RX antenna bitmask (default: 1)\n"
              << "  --duration <sec>   Run length (default: 6; 0 = frames only)\n"
              << "  --frames <n>       Stop after N found frames (default: 40;\n"
              << "                     0 = duration only — soak mode)\n"
              << "  --payload <len>    Payload bytes (default: 16)\n"
              << "  --amplitude <a>    TX amplitude 0..1 (default: 0.8;\n"
              << "                     lower ≈ weaker OTA / low SNR)\n"
              << "  --warmup <syms>    Carrier symbols before each frame\n"
              << "                     (default: 2 for --ota, 0 otherwise)\n"
              << "  --gap <syms>       Silence between frames (default: 12)\n"
              << "  --preset <name>    Meshtastic preset: shortturbo, shortfast\n"
              << "                     (default: SF11/BW500 test mode)\n"
              << "  --max-per <frac>   Pass if packet-error rate <= frac\n"
              << "                     (default: 0 = require all payloads OK)\n"
              << "  --no-dump          Skip raw IQ capture (needed for long soaks)\n"
              << "  --dump-dir <path>  Output directory (default: phy_dumps/lora)\n"
              << "  --rx-only          Listen only (no TX stream; OTA peer RX)\n"
              << "  -h, --help         Show help\n";
}

struct RunningStats {
    size_t n = 0;
    double sum = 0.0;
    double mn = 1e300;
    double mx = -1e300;

    void add(double v) {
        ++n;
        sum += v;
        mn = std::min(mn, v);
        mx = std::max(mx, v);
    }
    double mean() const { return n ? sum / static_cast<double>(n) : 0.0; }
};

bool parsePreset(const std::string& name, LoraParams& out) {
    return parseMeshtasticPreset(name, out);
}

// Chunks from the RX thread to the decode loop.
struct ChunkQueue {
    std::mutex m;
    std::condition_variable cv;
    std::deque<std::vector<Sample>> q;

    void push(std::vector<Sample>&& c) {
        {
            std::lock_guard<std::mutex> lk(m);
            q.push_back(std::move(c));
        }
        cv.notify_one();
    }
    bool pop(std::vector<Sample>& out, int timeout_ms) {
        std::unique_lock<std::mutex> lk(m);
        if (!cv.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&] { return !q.empty(); })) {
            return false;
        }
        out = std::move(q.front());
        q.pop_front();
        return true;
    }
};

}  // namespace

int main(int argc, char** argv) {
    bool ota = false;
    double center_mhz = 5800.0;
    int tx_gain = 1;
    int rx_gain = 10;
    int tx_ant = 1;
    int rx_ant = 1;
    double duration_sec = 6.0;
    size_t max_frames = 40;  // 0 = run until duration
    size_t payload_len = 16;
    float amplitude = 0.8f;
    int warmup_syms = -1;
    int gap_syms = 12;
    double max_per = 0.0;  // 0 => require every found frame to decode
    bool dump_iq = true;
    bool rx_only = false;
    std::string preset_name;
    std::string dump_dir = "phy_dumps/lora";

    LoraParams params;  // SF11, BW 500 kHz, 1 MSps, cr=1, crc on

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--ota") {
            ota = true;
        } else if (arg == "--rx-only") {
            rx_only = true;
        } else if (arg == "--freq" && i + 1 < argc) {
            center_mhz = std::stod(argv[++i]);
        } else if (arg == "--tx-gain" && i + 1 < argc) {
            tx_gain = std::stoi(argv[++i]);
        } else if (arg == "--rx-gain" && i + 1 < argc) {
            rx_gain = std::stoi(argv[++i]);
        } else if (arg == "--tx-ant" && i + 1 < argc) {
            tx_ant = std::stoi(argv[++i]);
        } else if (arg == "--rx-ant" && i + 1 < argc) {
            rx_ant = std::stoi(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            duration_sec = std::stod(argv[++i]);
        } else if (arg == "--frames" && i + 1 < argc) {
            max_frames = std::stoul(argv[++i]);
        } else if (arg == "--payload" && i + 1 < argc) {
            payload_len = std::stoul(argv[++i]);
        } else if (arg == "--amplitude" && i + 1 < argc) {
            amplitude = std::stof(argv[++i]);
        } else if (arg == "--warmup" && i + 1 < argc) {
            warmup_syms = std::stoi(argv[++i]);
        } else if (arg == "--gap" && i + 1 < argc) {
            gap_syms = std::stoi(argv[++i]);
        } else if (arg == "--preset" && i + 1 < argc) {
            preset_name = argv[++i];
            if (!parsePreset(preset_name, params)) {
                std::cerr << "Unknown preset: " << preset_name << "\n";
                return 1;
            }
        } else if (arg == "--max-per" && i + 1 < argc) {
            max_per = std::stod(argv[++i]);
        } else if (arg == "--no-dump") {
            dump_iq = false;
        } else if (arg == "--dump-dir" && i + 1 < argc) {
            dump_dir = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            usage(argv[0]);
            return 1;
        }
    }
    if (warmup_syms < 0) {
        warmup_syms = ota ? 2 : 0;
    }
    if (duration_sec <= 0.0 && max_frames == 0) {
        std::cerr << "Need --duration > 0 and/or --frames > 0\n";
        return 1;
    }

    std::signal(SIGINT, onSignal);

    std::vector<uint8_t> payload(payload_len);
    for (size_t i = 0; i < payload_len; ++i) {
        payload[i] = static_cast<uint8_t>(i * 37 + 11);
    }
    const std::vector<uint16_t> chips = encodeFrame(params, payload);
    const Modulator mod(params, amplitude);
    const IQBuffer frame = mod.frame(chips);  // dump / chip-error reference
    const uint32_t sps = mod.samplesPerSymbol();

    Transmitter tx(params, amplitude);
    tx.setWarmupSymbols(static_cast<uint32_t>(warmup_syms));
    tx.setGapSymbols(static_cast<uint32_t>(gap_syms));

    std::cout << "LoRa SF" << int(params.spreading_factor) << " BW"
              << params.bandwidth_hz / 1e3 << "k cr=" << int(params.cr)
              << (params.ldro ? " ldro" : "")
              << (preset_name.empty() ? "" : (" preset=" + preset_name)) << "\n";
    if (rx_only) {
        std::cout << "RX-only listen (no local TX); CRC-OK counts as success\n";
    } else {
        std::cout << "Frame: " << chips.size() << " data symbols, " << frame.size()
                  << " samples; streaming TX (amp " << amplitude << ", warmup "
                  << warmup_syms << " syms, gap " << gap_syms << ")\n";
    }
    if (max_frames == 0) {
        std::cout << "Soak: duration " << duration_sec << " s, no frame cap"
                  << (dump_iq ? "" : ", no IQ dump") << "\n";
    } else {
        std::cout << "Stop after " << max_frames << " frames or " << duration_sec << " s\n";
    }

    RfFrontend rf;
    const uint16_t saved_reg = rf.readRegister(0x2E);

    try {
        RadioSession session;
        session.open();
        if (!rx_only) {
            session.setSampleRate(SOAPY_SDR_TX, 0, params.sample_rate_hz);
        }
        session.setSampleRate(SOAPY_SDR_RX, 0, params.sample_rate_hz);
        session.printInfo();

        if (ota) {
            rf.setDigitalLoopback(false);
            if (rx_only) {
                rf.txOff();
                rf.configureRx(center_mhz, rx_gain, 40, rx_ant);
                std::cout << "OTA RX-only at " << center_mhz << " MHz, rx_gain=" << rx_gain
                          << " rx_ant=" << rx_ant << "\n";
            } else {
                rf.configureTx(center_mhz, tx_gain, 40, tx_ant);
                rf.configureRx(center_mhz, rx_gain, 40, rx_ant);
                std::cout << "OTA at " << center_mhz << " MHz, tx_gain=" << tx_gain
                          << " rx_gain=" << rx_gain << " amp=" << amplitude
                          << " tx_ant=" << tx_ant << " rx_ant=" << rx_ant << "\n";
            }
        } else {
            if (rx_only) {
                std::cerr << "--rx-only requires --ota\n";
                return 1;
            }
            rf.setDigitalLoopback(true);
            std::cout << "Digital loopback (reg 0x2E=0x04)\n";
        }

        StreamPair streams;
        if (!rx_only) {
            streams.setupTx(session.device());
        }
        streams.setupRx(session.device());
        const size_t mtu = streams.mtu();
        g_running = true;

        // Initialize receiver and FFTW planning before starting TX to prevent
        // queue underrun during initial planning latency.
        Receiver receiver(params);
        const size_t skip_startup = static_cast<size_t>(0.5 * params.sample_rate_hz);
        size_t stream_pos = 0;

        // Cap IQ dump so multi-minute soaks stay memory-bounded; stats still
        // run on whatever was kept when --no-dump is off.
        const size_t dump_cap = dump_iq ? static_cast<size_t>(params.sample_rate_hz *
                                                             std::min(duration_sec + 1.0, 30.0))
                                        : 0;
        std::vector<Sample> raw_dump;
        if (dump_cap) {
            raw_dump.reserve(dump_cap);
        }

        size_t n_found = 0, n_synced = 0, n_hdr = 0, n_crc = 0, n_payload = 0;
        size_t chips_total = 0, chips_wrong = 0;
        double err_max = 0.0;
        RunningStats ppm_stats, cfo_stats, snr_stats;

        std::ofstream frames_json;
        DebugDumper dumper(dump_iq, dump_dir);
        bool first_json = true;
        if (dump_iq) {
            frames_json.open(dump_dir + "/lora_frames.json", std::ios::trunc);
            frames_json << "[\n";
        }

        auto drainFrames = [&]() {
            ReceivedFrame r;
            while (receiver.pop(r)) {
                if (r.preamble_symbols == 0 && !r.synced) {
                    continue;  // aborted detection (noise trigger)
                }
                // Ignore a flush of a still-incomplete DATA frame at stop.
                if (r.synced && r.raw_values.size() < chips.size()) {
                    continue;
                }
                ++n_found;
                if (r.synced) {
                    ++n_synced;
                }
                if (r.decode.header.valid) {
                    ++n_hdr;
                }
                if (r.decode.crc_ok && r.decode.header.valid) {
                    ++n_crc;
                }
                const bool payload_ok =
                    r.decode.header.valid && r.decode.crc_ok &&
                    (rx_only || r.decode.payload == payload);
                if (payload_ok) {
                    ++n_payload;
                }
                ppm_stats.add(r.rate_ppm);
                cfo_stats.add(r.cfo_hz);
                snr_stats.add(r.snr_db);

                size_t wrong = 0;
                double frame_err_max = 0.0;
                const size_t n_cmp = std::min(r.raw_values.size(), chips.size());
                for (size_t s = 0; s < n_cmp; ++s) {
                    int d = std::abs(static_cast<int>(r.raw_values[s]) -
                                     static_cast<int>(chips[s]));
                    d = std::min<int>(d, static_cast<int>(chipCount(params)) - d);
                    if (d != 0) {
                        ++wrong;
                    }
                    frame_err_max = std::max(frame_err_max, static_cast<double>(d));
                }
                chips_total += n_cmp;
                chips_wrong += wrong;
                err_max = std::max(err_max, frame_err_max);

                std::printf(
                    "frame %4zu @%9zu: sync=%d pre=%zu rate=%+8.1f ppm cfo=%+6.0f Hz snr=%4.1f "
                    "hdr=%d crc=%d payload=%s chips %zu/%zu wrong (max err %.0f)\n",
                    n_found, r.start_sample, r.synced, r.preamble_symbols, r.rate_ppm, r.cfo_hz,
                    r.snr_db, r.decode.header.valid, r.decode.crc_ok, payload_ok ? "OK" : "BAD",
                    wrong, n_cmp, frame_err_max);

                if (dump_iq) {
                    if (!first_json) {
                        frames_json << ",\n";
                    }
                    first_json = false;
                    frames_json << "  {\"start\": " << r.start_sample
                                << ", \"synced\": " << r.synced << ", \"ppm\": " << r.rate_ppm
                                << ", \"cfo_hz\": " << r.cfo_hz << ", \"snr_db\": " << r.snr_db
                                << ", \"hdr\": " << r.decode.header.valid
                                << ", \"crc\": " << r.decode.crc_ok
                                << ", \"payload_ok\": " << payload_ok
                                << ", \"chips_wrong\": " << wrong << ", \"chips\": " << n_cmp
                                << "}";
                }
            }
        };

        // Pre-fill the TX queue; the main loop tops it up so the radio
        // thread only pulls samples (encode stays off the Soapy write path).
        auto topUpTx = [&]() {
            if (rx_only) {
                return;
            }
            while (tx.queued() < 8) {
                tx.enqueue(payload);
            }
        };
        topUpTx();

        std::thread tx_thread;
        if (!rx_only) {
            tx_thread = std::thread([&]() {
                const size_t chunk_len = std::min<size_t>(mtu, 65536);
                std::vector<Sample> chunk(chunk_len);
                while (g_running) {
                    tx.pull(chunk.data(), chunk_len);
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
                }
            });
        }

        ChunkQueue queue;
        std::atomic<size_t> rx_total{0};
        std::thread rx_thread([&]() {
            while (g_running) {
                std::vector<Sample> chunk(mtu);
                const int ret = streams.read(chunk.data(), mtu);
                if (ret > 0) {
                    chunk.resize(static_cast<size_t>(ret));
                    rx_total += chunk.size();
                    queue.push(std::move(chunk));
                } else if (ret != SOAPY_SDR_TIMEOUT) {
                    std::cerr << "RX stream error: " << SoapySDR::errToStr(ret) << "\n";
                    g_running = false;
                    break;
                }
            }
        });

        streams.activate();
        const auto t0 = std::chrono::steady_clock::now();
        const bool timed = duration_sec > 0.0;
        const auto t_end = t0 + std::chrono::duration<double>(timed ? duration_sec : 1e9);

        while (g_running) {
            if (timed && std::chrono::steady_clock::now() >= t_end) {
                break;
            }
            if (max_frames > 0 && n_found >= max_frames) {
                break;
            }
            topUpTx();
            std::vector<Sample> chunk;
            if (!queue.pop(chunk, 100)) {
                continue;
            }
            if (dump_cap && raw_dump.size() + chunk.size() <= raw_dump.capacity()) {
                raw_dump.insert(raw_dump.end(), chunk.begin(), chunk.end());
            }
            // skip the startup transient, then feed contiguously
            size_t off = 0;
            if (stream_pos < skip_startup) {
                off = std::min(chunk.size(), skip_startup - stream_pos);
            }
            stream_pos += chunk.size();
            if (off < chunk.size()) {
                receiver.feed(chunk.data() + off, chunk.size() - off);
            }
            drainFrames();
        }
        receiver.flush();
        drainFrames();

        g_running = false;
        if (tx_thread.joinable()) {
            tx_thread.join();
        }
        rx_thread.join();
        streams.deactivate();
        if (dump_iq) {
            frames_json << "\n]\n";
        }

        if (ota) {
            rf.txOff();
            rf.rxOff();
        }
        rf.writeRegister(0x2E, saved_reg);

        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const RxStats stats = computeStats(raw_dump.data(), raw_dump.size());
        std::cout << "\nStreamed " << stream_pos << " samples in " << elapsed << " s";
        if (raw_dump.empty()) {
            std::cout << " (no IQ dump)\n";
        } else {
            std::cout << ", RMS " << stats.rms_dbfs << " dBFS, clip "
                      << stats.clip_fraction * 100.0 << "%\n";
            dumper.write("lora_rx_raw", raw_dump.data(), raw_dump.size());
            dumper.write("lora_tx_frame", frame.data(), frame.size());
        }

        const double per =
            (n_found > 0)
                ? 1.0 - static_cast<double>(rx_only ? n_crc : n_payload) /
                            static_cast<double>(n_found)
                : 1.0;
        const double fps = (elapsed > 0.0) ? static_cast<double>(n_found) / elapsed : 0.0;
        const size_t n_ok = rx_only ? n_crc : n_payload;

        std::cout << "\n=== summary (" << (ota ? "OTA" : "digital")
                  << (rx_only ? ", rx-only" : ", streaming") << ") ===\n";
        std::cout << "frames found:    " << n_found << " (" << fps << " /s)\n";
        std::cout << "synced:          " << n_synced << "\n";
        std::cout << "header valid:    " << n_hdr << "\n";
        std::cout << "crc ok:          " << n_crc << "\n";
        std::cout << "payload correct: " << n_payload << "\n";
        std::printf("PER:             %.3f%% (%zu / %zu lost)\n", per * 100.0, n_found - n_ok,
                    n_found);
        if (chips_total) {
            std::printf("chip errors:     %zu / %zu (%.3f%%), max magnitude %.0f chips\n",
                        chips_wrong, chips_total, 100.0 * chips_wrong / chips_total, err_max);
        }
        if (ppm_stats.n) {
            std::printf("rate estimates:  %+.1f .. %+.1f ppm (mean %+.1f)\n", ppm_stats.mn,
                        ppm_stats.mx, ppm_stats.mean());
        }
        if (cfo_stats.n) {
            std::printf("cfo estimates:   %+.1f .. %+.1f Hz (mean %+.1f)\n", cfo_stats.mn,
                        cfo_stats.mx, cfo_stats.mean());
        }
        if (snr_stats.n) {
            std::printf("SNR (dechirp):   %.1f .. %.1f dB (mean %.1f)\n", snr_stats.mn,
                        snr_stats.mx, snr_stats.mean());
        }

        if (dump_iq) {
            std::ofstream manifest(dump_dir + "/lora_manifest.json", std::ios::trunc);
            manifest << "{\n"
                     << "  \"sample_rate_hz\": " << params.sample_rate_hz << ",\n"
                     << "  \"bandwidth_hz\": " << params.bandwidth_hz << ",\n"
                     << "  \"spreading_factor\": " << int(params.spreading_factor) << ",\n"
                     << "  \"sps\": " << sps << ",\n"
                     << "  \"preamble_len\": " << params.preamble_len << ",\n"
                     << "  \"data_symbols\": " << chips.size() << ",\n"
                     << "  \"frame_samples\": " << frame.size() << ",\n"
                     << "  \"payload_len\": " << payload_len << ",\n"
                     << "  \"amplitude\": " << amplitude << ",\n"
                     << "  \"warmup_syms\": " << warmup_syms << ",\n"
                     << "  \"gap_syms\": " << gap_syms << ",\n"
                     << "  \"tx_gain\": " << tx_gain << ",\n"
                     << "  \"rx_gain\": " << rx_gain << ",\n"
                     << "  \"tx_frames_started\": " << tx.framesStarted() << ",\n"
                     << "  \"ota\": " << (ota ? 1 : 0) << ",\n"
                     << "  \"elapsed_sec\": " << elapsed << ",\n"
                     << "  \"frames_found\": " << n_found << ",\n"
                     << "  \"frames_payload_ok\": " << n_payload << ",\n"
                     << "  \"per\": " << per << ",\n"
                     << "  \"snr_mean_db\": " << snr_stats.mean() << ",\n"
                     << "  \"snr_min_db\": " << (snr_stats.n ? snr_stats.mn : 0.0) << ",\n"
                     << "  \"ppm_mean\": " << ppm_stats.mean() << "\n"
                     << "}\n";
        }

        const bool pass = n_found > 0 && per <= max_per + 1e-12;
        std::cout << (pass ? "\nPASS" : "\nPARTIAL/FAIL") << ": " << n_ok << "/" << n_found
                  << (rx_only ? " frames CRC-OK" : " frames fully decoded") << " (PER "
                  << per * 100.0 << "% <= " << max_per * 100.0 << "%)\n";
        return pass ? 0 : 1;
    } catch (const std::exception& ex) {
        try {
            rf.writeRegister(0x2E, saved_reg);
        } catch (...) {
        }
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}
