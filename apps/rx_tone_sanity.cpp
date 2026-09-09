#include <phy/config.hpp>
#include <phy/dsp/fft_engine.hpp>
#include <phy/radio/radio_session.hpp>
#include <phy/radio/rf_frontend.hpp>
#include <phy/radio/stream_pair.hpp>
#include <phy/util/stats.hpp>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

struct ToneGuard {
    phy::RfFrontend& rf;
    bool active = false;
    ~ToneGuard() {
        if (active) {
            try {
                rf.setRxTone(false);
            } catch (...) {
            }
        }
    }
};

void usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --rate <hz>        Sample rate (default: 1e6)\n"
              << "  --gain <db>        RX gain via jtag (default: 10)\n"
              << "  --freq <mhz>       Center frequency MHz (default: 5800)\n"
              << "  --tone-freq <mhz>  FPGA internal tone MHz (default: 0.1)\n"
              << "  --fft-size <n>     FFT size (default: 4096)\n"
              << "  --buffers <n>      RX buffers to capture (default: 8)\n"
              << "  -h, --help         Show help\n";
}

}  // namespace

int main(int argc, char** argv) {
    double sample_rate = 1e6;
    int rx_gain = 10;
    double center_mhz = 5800.0;
    double tone_freq_mhz = 0.1;
    size_t fft_size = 4096;
    int num_buffers = 8;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--rate" || arg == "-r") && i + 1 < argc) {
            sample_rate = std::stod(argv[++i]);
        } else if (arg == "--gain" && i + 1 < argc) {
            rx_gain = std::stoi(argv[++i]);
        } else if (arg == "--freq" && i + 1 < argc) {
            center_mhz = std::stod(argv[++i]);
        } else if (arg == "--tone-freq" && i + 1 < argc) {
            tone_freq_mhz = std::stod(argv[++i]);
        } else if (arg == "--fft-size" && i + 1 < argc) {
            fft_size = static_cast<size_t>(std::stoul(argv[++i]));
        } else if (arg == "--buffers" && i + 1 < argc) {
            num_buffers = std::stoi(argv[++i]);
        } else if (arg == "-h" || arg == "--help") {
            usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            usage(argv[0]);
            return 1;
        }
    }

    phy::RfFrontend rf;
    ToneGuard guard{rf};

    try {
        rf.configureRx(center_mhz, rx_gain);
        rf.setRxTone(true, tone_freq_mhz);
        guard.active = true;

        phy::RadioSession session;
        session.open();
        session.setSampleRate(SOAPY_SDR_RX, 0, sample_rate);
        const double actual_rate = session.sampleRate(SOAPY_SDR_RX, 0);
        session.printInfo();

        phy::StreamPair streams;
        streams.setupRx(session.device());
        streams.activate();

        const size_t mtu = streams.mtu();
        std::vector<phy::Sample> capture;
        capture.reserve(static_cast<size_t>(num_buffers) * mtu);

        std::vector<phy::Sample> chunk(mtu);
        int captured_buffers = 0;
        while (captured_buffers < num_buffers) {
            const int ret = streams.read(chunk.data(), mtu);
            if (ret > 0) {
                capture.insert(capture.end(), chunk.begin(), chunk.begin() + ret);
                ++captured_buffers;
            } else if (ret == SOAPY_SDR_TIMEOUT) {
                continue;
            } else {
                std::cerr << "readStream error: " << ret << "\n";
                break;
            }
        }

        streams.deactivate();

        const phy::RxStats stats = phy::computeStats(capture.data(), capture.size());
        std::cout << "Captured " << capture.size() << " samples\n";
        std::cout << "RMS: " << stats.rms_dbfs << " dBFS, peak: " << stats.peak_dbfs
                  << " dBFS, clip: " << (stats.clip_fraction * 100.0) << "%\n";

        if (capture.size() < fft_size) {
            std::cerr << "Not enough samples for FFT size " << fft_size << "\n";
            return 1;
        }

        phy::FftEngine fft(fft_size);
        fft.forward(capture.data());
        const size_t peak_bin = fft.peakBin();
        const double bin_hz = actual_rate / static_cast<double>(fft_size);
        const double peak_hz = peak_bin * bin_hz;

        std::cout << "FFT peak bin: " << peak_bin << " (" << peak_hz / 1e3 << " kHz)\n";
        std::cout << "Expected tone ~" << tone_freq_mhz * 1e6 / 1e3 << " kHz\n";

        const bool clip_ok = stats.clip_fraction < 0.05;
        const bool peak_ok = stats.peak_dbfs > -60.0;
        if (clip_ok && peak_ok) {
            std::cout << "PASS: signal present, clipping acceptable\n";
            return 0;
        }

        std::cout << "WARN: check gain — clip_ok=" << clip_ok << " peak_ok=" << peak_ok << "\n";
        return 1;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}
