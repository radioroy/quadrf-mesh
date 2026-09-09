#include <phy/dsp/fft_engine.hpp>
#include <phy/dsp/tone_generator.hpp>
#include <phy/radio/radio_session.hpp>
#include <phy/radio/rf_frontend.hpp>
#include <phy/radio/stream_pair.hpp>
#include <phy/util/ring_buffer.hpp>
#include <phy/util/stats.hpp>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
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

void usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --rate <hz>        Sample rate (default: 1e6)\n"
              << "  --tone <hz>        Software tone frequency (default: 100e3)\n"
              << "  --tx-gain <db>     TX gain via jtag (default: 1)\n"
              << "  --rx-gain <db>     RX gain via jtag (default: 10)\n"
              << "  --tx-ant <mask>    TX antenna bitmask (default: 1)\n"
              << "  --rx-ant <mask>    RX antenna bitmask (default: 1)\n"
              << "  --freq <mhz>       Center frequency MHz (default: 5800)\n"
              << "  --duration <sec>   Run time (default: 3)\n"
              << "  --tx-only          Transmit only, no RX\n"
              << "  --rx-only          RX only (TX off) — noise floor\n"
              << "  -h, --help         Show help\n";
}

}  // namespace

int main(int argc, char** argv) {
    double sample_rate = 1e6;
    double tone_hz = 100e3;
    int tx_gain = 1;
    int rx_gain = 10;
    int tx_ant = 1;
    int rx_ant = 1;
    double center_mhz = 5800.0;
    double duration_sec = 3.0;
    bool tx_only = false;
    bool rx_only = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--rate" && i + 1 < argc) {
            sample_rate = std::stod(argv[++i]);
        } else if (arg == "--tone" && i + 1 < argc) {
            tone_hz = std::stod(argv[++i]);
        } else if (arg == "--tx-gain" && i + 1 < argc) {
            tx_gain = std::stoi(argv[++i]);
        } else if (arg == "--rx-gain" && i + 1 < argc) {
            rx_gain = std::stoi(argv[++i]);
        } else if (arg == "--tx-ant" && i + 1 < argc) {
            tx_ant = std::stoi(argv[++i]);
        } else if (arg == "--rx-ant" && i + 1 < argc) {
            rx_ant = std::stoi(argv[++i]);
        } else if (arg == "--freq" && i + 1 < argc) {
            center_mhz = std::stod(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            duration_sec = std::stod(argv[++i]);
        } else if (arg == "--tx-only") {
            tx_only = true;
        } else if (arg == "--rx-only") {
            rx_only = true;
        } else if (arg == "-h" || arg == "--help") {
            usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            usage(argv[0]);
            return 1;
        }
    }
    if (tx_only && rx_only) {
        std::cerr << "Cannot combine --tx-only and --rx-only\n";
        return 1;
    }

    std::signal(SIGINT, onSignal);

    phy::RfFrontend rf;
    try {
        rf.setDigitalLoopback(false);
        if (!tx_only) {
            rf.configureRx(center_mhz, rx_gain, 40, rx_ant);
        }
        if (rx_only) {
            rf.txOff();
        } else {
            rf.configureTx(center_mhz, tx_gain, 40, tx_ant);
        }

        phy::RadioSession session;
        session.open();
        if (!rx_only) {
            session.setSampleRate(SOAPY_SDR_TX, 0, sample_rate);
        }
        if (!tx_only) {
            session.setSampleRate(SOAPY_SDR_RX, 0, sample_rate);
        }
        const double actual_rate =
            rx_only ? session.sampleRate(SOAPY_SDR_RX, 0) : session.sampleRate(SOAPY_SDR_TX, 0);
        session.printInfo();
        std::cout << "cfg: tx_gain=" << tx_gain << " rx_gain=" << rx_gain
                  << " tx_ant=" << tx_ant << " rx_ant=" << rx_ant
                  << (rx_only ? " (rx-only/noise)" : "") << "\n";

        phy::StreamPair streams;
        if (!rx_only) {
            streams.setupTx(session.device());
        }
        if (!tx_only) {
            streams.setupRx(session.device());
        }

        const size_t mtu = streams.mtu();
        std::vector<phy::Sample> tx_buf(mtu);
        phy::ToneGenerator tone(actual_rate, tone_hz);

        phy::RingBuffer rx_ring(mtu * 64);
        std::vector<phy::Sample> rx_chunk(mtu);

        streams.activate();

        std::thread tx_thread;
        if (!rx_only) {
            // Prime TX before RX starts.
            for (int i = 0; i < 3; ++i) {
                tone.fill(tx_buf.data(), mtu);
                streams.write(tx_buf.data(), mtu);
            }

            tx_thread = std::thread([&]() {
                while (g_running) {
                    tone.fill(tx_buf.data(), mtu);
                    const int ret = streams.write(tx_buf.data(), mtu);
                    if (ret < 0) {
                        std::cerr << "TX write error: " << ret << "\n";
                        g_running = false;
                        break;
                    }
                }
            });
        }

        std::thread rx_thread;
        if (!tx_only) {
            rx_thread = std::thread([&]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                while (g_running) {
                    const int ret = streams.read(rx_chunk.data(), mtu);
                    if (ret > 0) {
                        if (!rx_ring.push(rx_chunk.data(), static_cast<size_t>(ret))) {
                            std::cerr << "RX ring overflow\n";
                        }
                    } else if (ret != SOAPY_SDR_TIMEOUT) {
                        std::cerr << "RX read error: " << ret << "\n";
                        g_running = false;
                        break;
                    }
                }
            });
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(duration_sec * 1000.0)));
        g_running = false;

        if (tx_thread.joinable()) {
            tx_thread.join();
        }
        if (rx_thread.joinable()) {
            rx_thread.join();
        }

        streams.deactivate();

        if (tx_only) {
            std::cout << "TX-only run complete\n";
            return 0;
        }

        std::vector<phy::Sample> captured;
        captured.resize(mtu * 32);
        const size_t got = rx_ring.pop(captured.data(), captured.size(), 500);
        captured.resize(got);

        if (got < 1024) {
            std::cerr << "Insufficient RX samples captured: " << got << "\n";
            return 1;
        }

        const phy::RxStats stats = phy::computeStats(captured.data(), captured.size());
        std::cout << "RX RMS: " << stats.rms_dbfs << " dBFS, peak: " << stats.peak_dbfs
                  << " dBFS, clip: " << (stats.clip_fraction * 100.0) << "%\n";

        if (rx_only) {
            // Averaged power spectrum so a remote CW tone can be measured
            // against the median bin floor (per-antenna path health check).
            const size_t fft_size = 4096;
            phy::FftEngine fft(fft_size);
            std::vector<double> psd(fft_size, 0.0);
            size_t frames = 0;
            for (size_t off = 0; off + fft_size <= captured.size() && frames < 16;
                 off += fft_size, ++frames) {
                const auto& spec = fft.forward(captured.data() + off);
                for (size_t b = 0; b < fft_size; ++b) {
                    psd[b] += std::norm(spec[b]);
                }
            }
            for (auto& v : psd) {
                v /= static_cast<double>(frames > 0 ? frames : 1);
            }
            std::vector<double> sorted = psd;
            std::nth_element(sorted.begin(), sorted.begin() + fft_size / 2, sorted.end());
            const double median = std::max(sorted[fft_size / 2], 1e-30);
            size_t peak_bin = 0;
            for (size_t b = 1; b < fft_size; ++b) {
                if (psd[b] > psd[peak_bin]) {
                    peak_bin = b;
                }
            }
            const double bin_hz = actual_rate / static_cast<double>(fft_size);
            // Signed baseband frequency (bins above N/2 alias to negative).
            const double signed_hz = (peak_bin <= fft_size / 2)
                                         ? peak_bin * bin_hz
                                         : (static_cast<double>(peak_bin) - fft_size) * bin_hz;
            const double peak_db = 10.0 * std::log10(std::max(psd[peak_bin], 1e-30) / median);
            // Windowed search around the expected tone (+/- CFO headroom) so
            // local fixed spurs (e.g. +/-417 kHz) cannot mask a weak remote CW.
            const double win_lo = tone_hz - 50e3;
            const double win_hi = tone_hz + 50e3;
            size_t tone_bin = 0;
            double tone_pwr = 0.0;
            for (size_t b = 0; b < fft_size; ++b) {
                const double f = (b <= fft_size / 2)
                                     ? b * bin_hz
                                     : (static_cast<double>(b) - fft_size) * bin_hz;
                if (f >= win_lo && f <= win_hi && psd[b] > tone_pwr) {
                    tone_pwr = psd[b];
                    tone_bin = b;
                }
            }
            const double tone_hz_meas = (tone_bin <= fft_size / 2)
                                            ? tone_bin * bin_hz
                                            : (static_cast<double>(tone_bin) - fft_size) * bin_hz;
            const double tone_db = 10.0 * std::log10(std::max(tone_pwr, 1e-30) / median);
            std::cout << "RXMEAS: rx_ant=" << rx_ant << " rms_dbfs=" << stats.rms_dbfs
                      << " peak_khz=" << signed_hz / 1e3 << " peak_over_median_db=" << peak_db
                      << " tone_khz=" << tone_hz_meas / 1e3 << " tone_over_median_db=" << tone_db
                      << " frames=" << frames << "\n";
            return 0;
        }

        const size_t fft_size = 4096;
        phy::FftEngine fft(fft_size);
        fft.forward(captured.data());
        const size_t peak_bin = fft.peakBin();
        const double bin_hz = actual_rate / static_cast<double>(fft_size);
        const double peak_hz = peak_bin * bin_hz;

        std::cout << "Detected tone near " << peak_hz / 1e3 << " kHz (expected "
                  << tone_hz / 1e3 << " kHz)\n";

        const double tone_tol = tone_hz * 0.15;
        const bool tone_ok = std::abs(peak_hz - tone_hz) < tone_tol;
        const bool clip_ok = stats.clip_fraction < 0.05;

        if (tone_ok && clip_ok && stats.peak_dbfs > -50.0) {
            std::cout << "PASS: OTA tone loopback\n";
            return 0;
        }

        std::cout << "WARN: loopback check failed (tone_ok=" << tone_ok << " clip_ok=" << clip_ok
                  << ")\n";
        return 1;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}
