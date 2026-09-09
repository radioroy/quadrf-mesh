#include <phy/dsp/fft_engine.hpp>
#include <phy/dsp/tone_generator.hpp>
#include <phy/radio/radio_session.hpp>
#include <phy/radio/rf_frontend.hpp>
#include <phy/radio/stream_pair.hpp>
#include <phy/util/stats.hpp>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) {
    g_running = false;
}

struct CaptureResult {
    phy::RxStats stats{};
    double peak_hz = 0.0;
    size_t samples = 0;
};

void usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --rate <hz>        Sample rate (default: 1e6)\n"
              << "  --tone <hz>        TX tone frequency (default: 100e3)\n"
              << "  --duration <sec>   Capture time per phase (default: 2)\n"
              << "  -h, --help         Show help\n";
}

CaptureResult captureTone(phy::StreamPair& streams, phy::ToneGenerator& tone, double actual_rate,
                          double duration_sec) {
    const size_t mtu = streams.mtu();
    std::vector<phy::Sample> tx_buf(mtu);
    std::vector<phy::Sample> rx_buf(mtu);
    std::vector<phy::Sample> captured;
    captured.reserve(static_cast<size_t>(duration_sec * actual_rate));

    g_running = true;
    std::thread tx_thread([&]() {
        while (g_running) {
            tone.fill(tx_buf.data(), mtu);
            const int ret = streams.write(tx_buf.data(), mtu);
            if (ret < 0) {
                g_running = false;
                break;
            }
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(duration_sec * 1000.0));
    while (g_running && std::chrono::steady_clock::now() < deadline) {
        const int ret = streams.read(rx_buf.data(), mtu);
        if (ret > 0) {
            captured.insert(captured.end(), rx_buf.begin(), rx_buf.begin() + ret);
        } else if (ret != SOAPY_SDR_TIMEOUT) {
            g_running = false;
            break;
        }
    }

    g_running = false;
    tx_thread.join();

    CaptureResult result;
    result.samples = captured.size();
    if (captured.empty()) {
        return result;
    }

    result.stats = phy::computeStats(captured.data(), captured.size());

    constexpr size_t fft_size = 4096;
    if (captured.size() >= fft_size) {
        phy::FftEngine fft(fft_size);
        fft.forward(captured.data());
        const size_t peak_bin = fft.peakBin();
        result.peak_hz = peak_bin * actual_rate / static_cast<double>(fft_size);
    }

    return result;
}

}  // namespace

int main(int argc, char** argv) {
    double sample_rate = 1e6;
    double tone_hz = 100e3;
    double duration_sec = 2.0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--rate" && i + 1 < argc) {
            sample_rate = std::stod(argv[++i]);
        } else if (arg == "--tone" && i + 1 < argc) {
            tone_hz = std::stod(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            duration_sec = std::stod(argv[++i]);
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
    const uint16_t saved_reg_2e = rf.readRegister(0x2E);
    std::cout << "Saved FPGA reg 0x2E = 0x" << std::hex << saved_reg_2e << std::dec << "\n";

    try {
        phy::RadioSession session;
        session.open();
        session.setSampleRate(SOAPY_SDR_TX, 0, sample_rate);
        session.setSampleRate(SOAPY_SDR_RX, 0, sample_rate);
        const double actual_rate = session.sampleRate(SOAPY_SDR_TX, 0);
        session.printInfo();

        phy::StreamPair streams;
        streams.setupTx(session.device());
        streams.setupRx(session.device());
        streams.activate();

        phy::ToneGenerator tone(actual_rate, tone_hz);

        std::cout << "\nPhase 1: digital loopback OFF (0x2E=0x00)\n";
        rf.setDigitalLoopback(false);
        tone.reset();
        const CaptureResult off = captureTone(streams, tone, actual_rate, duration_sec);
        std::cout << "  samples=" << off.samples << " RMS=" << off.stats.rms_dbfs
                  << " dBFS peak_tone=" << off.peak_hz / 1e3 << " kHz\n";

        std::cout << "\nPhase 2: digital loopback ON (0x2E=0x04)\n";
        rf.setDigitalLoopback(true);
        tone.reset();
        const CaptureResult on = captureTone(streams, tone, actual_rate, duration_sec);
        std::cout << "  samples=" << on.samples << " RMS=" << on.stats.rms_dbfs
                  << " dBFS peak_tone=" << on.peak_hz / 1e3 << " kHz\n";

        streams.deactivate();

        rf.writeRegister(0x2E, saved_reg_2e);
        std::cout << "Restored FPGA reg 0x2E = 0x" << std::hex << saved_reg_2e << std::dec << "\n";

        if (off.samples < 1024 || on.samples < 1024) {
            std::cerr << "FAIL: insufficient RX samples\n";
            return 1;
        }

        const double rms_delta_db = on.stats.rms_dbfs - off.stats.rms_dbfs;
        const double tone_tol = tone_hz * 0.15;
        const bool tone_ok = std::abs(on.peak_hz - tone_hz) < tone_tol;
        const bool level_ok = rms_delta_db >= 8.0 && on.stats.rms_dbfs > -55.0;
        const bool clip_ok = on.stats.clip_fraction < 0.05;

        std::cout << "\nLoopback lift: " << rms_delta_db << " dB\n";

        if (tone_ok && level_ok && clip_ok) {
            std::cout << "PASS: digital loopback routes TX baseband to RX\n";
            return 0;
        }

        std::cout << "FAIL: digital loopback check (tone_ok=" << tone_ok
                  << " level_ok=" << level_ok << " clip_ok=" << clip_ok << ")\n";
        return 1;
    } catch (const std::exception& ex) {
        try {
            rf.writeRegister(0x2E, saved_reg_2e);
        } catch (...) {
        }
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}
