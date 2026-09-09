// Characterizes the FPGA digital loopback path (reg 0x2E bit 2).
//
// Two stimulus phases, each dumped as raw IQ for offline analysis:
//   1. CW tone   -> measures effective TX->RX rate ratio and phase jumps
//   2. Up-chirps -> dechirped bin drift exposes ppm error and sample loss
//
// RF chips are never configured; this isolates the digital path.

#include <phy/config.hpp>
#include <phy/dsp/chirp_generator.hpp>
#include <phy/dsp/tone_generator.hpp>
#include <phy/radio/radio_session.hpp>
#include <phy/radio/rf_frontend.hpp>
#include <phy/radio/stream_pair.hpp>
#include <phy/util/debug_dumper.hpp>
#include <phy/util/stats.hpp>

#include <SoapySDR/Constants.h>
#include <SoapySDR/Errors.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) {
    g_running = false;
}

void usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --rate <hz>        Host sample rate (default: 1e6)\n"
              << "  --tone <hz>        Probe tone frequency (default: 100e3)\n"
              << "  --duration <sec>   Capture per phase (default: 4)\n"
              << "  --dump-dir <path>  Output directory (default: phy_dumps)\n"
              << "  -h, --help         Show help\n";
}

// TX thread keeps the ring saturated with a repeating pattern; RX thread
// captures into a pre-allocated buffer. Streams are activated only after
// both threads are staged so the first RX period is not empty.
std::vector<phy::Sample> capturePhase(phy::RadioSession& session, const phy::IQBuffer& pattern,
                                      double sample_rate_hz, double duration_sec) {
    phy::StreamPair streams;
    streams.setupTx(session.device());
    streams.setupRx(session.device());
    const size_t mtu = streams.mtu();

    const size_t max_rx = static_cast<size_t>(sample_rate_hz * (duration_sec + 1.0));
    std::vector<phy::Sample> captured(max_rx);
    std::atomic<size_t> rx_pos{0};

    g_running = true;

    std::thread tx_thread([&]() {
        size_t offset = 0;
        std::vector<phy::Sample> chunk(mtu);
        while (g_running) {
            for (size_t i = 0; i < mtu; ++i) {
                chunk[i] = pattern[offset];
                offset = (offset + 1) % pattern.size();
            }
            const int ret = streams.write(chunk.data(), mtu);
            if (ret < 0) {
                g_running = false;
                break;
            }
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::thread rx_thread([&]() {
        while (g_running) {
            if (rx_pos + mtu > captured.size()) {
                break;
            }
            const int ret = streams.read(captured.data() + rx_pos, mtu);
            if (ret > 0) {
                rx_pos += static_cast<size_t>(ret);
            } else if (ret != SOAPY_SDR_TIMEOUT) {
                g_running = false;
                break;
            }
        }
    });

    streams.activate();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(duration_sec * 1000.0)));
    g_running = false;

    tx_thread.join();
    rx_thread.join();
    streams.deactivate();

    captured.resize(rx_pos);
    return captured;
}

}  // namespace

int main(int argc, char** argv) {
    double sample_rate = 1e6;
    double tone_hz = 100e3;
    double duration_sec = 4.0;
    std::string dump_dir = "phy_dumps";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--rate" && i + 1 < argc) {
            sample_rate = std::stod(argv[++i]);
        } else if (arg == "--tone" && i + 1 < argc) {
            tone_hz = std::stod(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            duration_sec = std::stod(argv[++i]);
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

    std::signal(SIGINT, onSignal);

    phy::RfFrontend rf;
    const uint16_t saved_reg = rf.readRegister(0x2E);
    std::cout << "Saved FPGA reg 0x2E = 0x" << std::hex << saved_reg << std::dec << "\n";

    try {
        phy::RadioSession session;
        session.open();
        session.setSampleRate(SOAPY_SDR_TX, 0, sample_rate);
        session.setSampleRate(SOAPY_SDR_RX, 0, sample_rate);
        session.printInfo();

        rf.setDigitalLoopback(true);

        phy::DebugDumper dumper(true, dump_dir);

        // Phase 1: CW tone
        std::cout << "\n[Phase 1] tone " << tone_hz / 1e3 << " kHz, " << duration_sec << " s\n";
        phy::ToneGenerator tone(sample_rate, tone_hz);
        const phy::IQBuffer tone_pattern =
            tone.buffer(static_cast<size_t>(sample_rate));  // 1 s, integer-ish cycle count
        const std::vector<phy::Sample> rx_tone =
            capturePhase(session, tone_pattern, sample_rate, duration_sec);
        const phy::RxStats tone_stats = phy::computeStats(rx_tone.data(), rx_tone.size());
        std::cout << "  captured " << rx_tone.size() << " samples, RMS " << tone_stats.rms_dbfs
                  << " dBFS, clip " << tone_stats.clip_fraction * 100.0 << "%\n";
        dumper.write("probe_rx_tone", rx_tone.data(), rx_tone.size());

        // Phase 2: continuous up-chirps
        phy::PhyConfig cfg;
        cfg.sample_rate_hz = sample_rate;
        phy::ChirpGenerator chirp_gen(cfg);
        const uint32_t n_fft = chirp_gen.fftSize();
        std::cout << "\n[Phase 2] up-chirps SF" << int(cfg.spreading_factor) << " BW "
                  << cfg.bandwidth_hz / 1e3 << " kHz (" << n_fft << " samp/sym), "
                  << duration_sec << " s\n";
        const phy::IQBuffer chirp_pattern = chirp_gen.preamble(8);
        const std::vector<phy::Sample> rx_chirp =
            capturePhase(session, chirp_pattern, sample_rate, duration_sec);
        const phy::RxStats chirp_stats = phy::computeStats(rx_chirp.data(), rx_chirp.size());
        std::cout << "  captured " << rx_chirp.size() << " samples, RMS " << chirp_stats.rms_dbfs
                  << " dBFS, clip " << chirp_stats.clip_fraction * 100.0 << "%\n";
        dumper.write("probe_rx_chirp", rx_chirp.data(), rx_chirp.size());
        dumper.write("probe_tx_chirp_symbol", chirp_gen.upChirp().data(), n_fft);

        rf.writeRegister(0x2E, saved_reg);
        std::cout << "\nRestored FPGA reg 0x2E = 0x" << std::hex << saved_reg << std::dec << "\n";

        std::ofstream manifest(dump_dir + "/probe_manifest.json", std::ios::trunc);
        manifest << std::setprecision(12);
        manifest << "{\n"
                 << "  \"sample_rate_hz\": " << sample_rate << ",\n"
                 << "  \"tone_hz\": " << tone_hz << ",\n"
                 << "  \"bandwidth_hz\": " << cfg.bandwidth_hz << ",\n"
                 << "  \"spreading_factor\": " << int(cfg.spreading_factor) << ",\n"
                 << "  \"fft_size\": " << n_fft << ",\n"
                 << "  \"tone_samples\": " << rx_tone.size() << ",\n"
                 << "  \"chirp_samples\": " << rx_chirp.size() << "\n"
                 << "}\n";
        std::cout << "Dumps written to " << dump_dir << "/\n";
        return 0;
    } catch (const std::exception& ex) {
        try {
            rf.writeRegister(0x2E, saved_reg);
        } catch (...) {
        }
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}
