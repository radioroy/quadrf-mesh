#include <phy/config.hpp>
#include <phy/dsp/chirp_generator.hpp>
#include <phy/dsp/dechirper.hpp>
#include <phy/dsp/fft_engine.hpp>
#include <phy/dsp/peak_detector.hpp>
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
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) {
    g_running = false;
}

struct PhaseSkip {
    bool phase1 = false;
    bool phase2 = false;
    bool phase3 = false;
    bool phase4 = false;
};

struct CaptureResult {
    std::vector<phy::Sample> samples;
    double elapsed_sec = 0.0;
};

struct LinearFit {
    double slope = 0.0;
    double intercept = 0.0;
};

struct TestReport {
    double fs_nominal = 0.0;
    double fs_tx_reported = 0.0;
    double fs_rx_reported = 0.0;

    bool phase1_ran = false;
    bool phase1_ok = false;
    double f_internal_hz = 0.0;
    double tone_fpga_hz = 0.0;

    bool phase2_ran = false;
    double fs_wall = 0.0;
    double ppm_wall = 0.0;
    size_t wall_samples = 0;

    bool phase3_ran = false;
    double f_ota_measured_hz = 0.0;
    double fs_ota = 0.0;
    double ppm_ota = 0.0;

    bool phase4_ran = false;
    double fs_css = 0.0;
    double ppm_css = 0.0;
    double preamble_slope = 0.0;
    double preamble_intercept = 0.0;
    size_t preamble_hits = 0;
    size_t align_offset = 0;
    std::vector<size_t> preamble_bins;

    std::string verdict;
    int tx_gain = 0;
    int rx_gain = 0;
    double center_mhz = 0.0;
    double tone_hz = 0.0;
    double ppm_threshold = 100.0;
};

void usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --rate <hz>           Nominal sample rate (default: 1e6)\n"
              << "  --tone <hz>           OTA test tone frequency (default: 100e3)\n"
              << "  --tone-fpga-mhz <f>   Internal FPGA tone MHz for Phase 1 (default: 0.1)\n"
              << "  --wall-duration <s>   Phase 2 capture (default: 10)\n"
              << "  --ota-duration <s>    Phase 3/4 capture (default: 8)\n"
              << "  --tx-gain <db>        TX gain (default: 1)\n"
              << "  --rx-gain <db>        RX gain (default: 10)\n"
              << "  --freq <mhz>          Center frequency MHz (default: 5800)\n"
              << "  --ppm-threshold <n>   Error threshold in ppm (default: 100)\n"
              << "  --output <path>       Write JSON report\n"
              << "  --debug-dump-dir <p>  Write preamble bins JSON alongside report\n"
              << "  --skip-phase <1-4>    Skip individual phase (repeatable)\n"
              << "  -h, --help            Show help\n";
}

double ppmError(double measured, double nominal) {
    if (nominal == 0.0) {
        return 0.0;
    }
    return (measured - nominal) / nominal * 1e6;
}

LinearFit linearFit(const std::vector<double>& x, const std::vector<double>& y) {
    LinearFit fit;
    if (x.size() != y.size() || x.empty()) {
        return fit;
    }
    const size_t n = x.size();
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        sum_x += x[i];
        sum_y += y[i];
        sum_xx += x[i] * x[i];
        sum_xy += x[i] * y[i];
    }
    const double denom = static_cast<double>(n) * sum_xx - sum_x * sum_x;
    if (std::abs(denom) < 1e-12) {
        fit.intercept = sum_y / static_cast<double>(n);
        return fit;
    }
    fit.slope = (static_cast<double>(n) * sum_xy - sum_x * sum_y) / denom;
    fit.intercept = (sum_y - fit.slope * sum_x) / static_cast<double>(n);
    return fit;
}

double measureToneHz(const std::vector<phy::Sample>& samples, size_t fft_size, double fs_hz,
                     double expected_hz = 0.0) {
    if (samples.size() < fft_size) {
        return 0.0;
    }
    phy::FftEngine fft(fft_size);
    fft.forward(samples.data());
    const auto& spectrum = fft.spectrum();

    size_t peak_bin = 0;
    double peak_mag = 0.0;

    if (expected_hz > 0.0) {
        const size_t center = static_cast<size_t>(std::lround(expected_hz * static_cast<double>(fft_size) / fs_hz));
        const size_t half_width = std::max<size_t>(8, static_cast<size_t>(expected_hz * static_cast<double>(fft_size) / fs_hz * 0.5));
        const size_t lo = (center > half_width) ? (center - half_width) : 0;
        const size_t hi = std::min(fft_size / 2, center + half_width);
        for (size_t b = lo; b <= hi && b < fft_size; ++b) {
            const double re = spectrum[b].real();
            const double im = spectrum[b].imag();
            const double mag = re * re + im * im;
            if (mag > peak_mag) {
                peak_mag = mag;
                peak_bin = b;
            }
        }
    } else {
        peak_bin = fft.peakBin();
    }

    return static_cast<double>(peak_bin) * fs_hz / static_cast<double>(fft_size);
}

phy::PeakResult demodSymbol(const phy::Sample* rx, const phy::Sample* ref_down, uint32_t n_fft,
                            phy::FftEngine& fft, std::vector<phy::Sample>& work) {
    phy::dechirp(rx, ref_down, work.data(), n_fft);
    fft.forward(work.data());
    return phy::findPeakBin(fft.spectrum().data(), n_fft);
}

size_t findBestAlignment(const std::vector<phy::Sample>& captured, const phy::Sample* ref_down,
                         uint32_t n_fft, size_t preamble_count) {
    phy::FftEngine fft(n_fft);
    std::vector<phy::Sample> work(n_fft);

    size_t best_offset = 0;
    size_t best_hits = 0;
    double best_mag_sum = 0.0;

    if (captured.size() < preamble_count * n_fft) {
        return 0;
    }

    const size_t max_offset =
        std::min(static_cast<size_t>(n_fft), captured.size() - preamble_count * n_fft);
    for (size_t offset = 0; offset < max_offset; offset += 8) {
        size_t hits = 0;
        double mag_sum = 0.0;
        for (size_t sym = 0; sym < preamble_count; ++sym) {
            const size_t pos = offset + sym * n_fft;
            if (pos + n_fft > captured.size()) {
                hits = 0;
                break;
            }
            const phy::PeakResult peak =
                demodSymbol(captured.data() + pos, ref_down, n_fft, fft, work);
            const size_t wrapped = peak.bin % n_fft;
            const size_t dist = std::min(wrapped, n_fft - wrapped);
            if (dist <= 16) {
                ++hits;
            }
            mag_sum += peak.magnitude;
        }
        if (hits > best_hits || (hits == best_hits && mag_sum > best_mag_sum)) {
            best_hits = hits;
            best_mag_sum = mag_sum;
            best_offset = offset;
        }
    }
    return best_offset;
}

LinearFit preambleSlopeAtRate(const std::vector<phy::Sample>& captured, size_t align_offset,
                              const phy::PhyConfig& base_cfg, uint32_t n_fft,
                              size_t preamble_count, double fs_hz) {
    phy::PhyConfig cfg = base_cfg;
    cfg.sample_rate_hz = fs_hz;
    phy::ChirpGenerator ref_gen(cfg);

    phy::FftEngine fft(n_fft);
    std::vector<phy::Sample> work(n_fft);
    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(preamble_count);
    ys.reserve(preamble_count);

    for (size_t sym = 0; sym < preamble_count; ++sym) {
        const size_t pos = align_offset + sym * n_fft;
        if (pos + n_fft > captured.size()) {
            break;
        }
        const phy::PeakResult peak =
            demodSymbol(captured.data() + pos, ref_gen.downChirp().data(), n_fft, fft, work);
        xs.push_back(static_cast<double>(sym));
        ys.push_back(static_cast<double>(peak.bin % n_fft));
    }
    return linearFit(xs, ys);
}

double estimateFsFromPreambleDrift(const std::vector<phy::Sample>& captured, size_t align_offset,
                                   const phy::PhyConfig& base_cfg, uint32_t n_fft,
                                   size_t preamble_count, double fs_nominal, double* out_slope) {
    double best_fs = fs_nominal;
    double best_abs_slope = 1e9;

    // Scan +/- 3000 ppm in 1.0 ppm steps
    constexpr double ppm_min = -3000.0;
    constexpr double ppm_max = 3000.0;
    constexpr double ppm_step = 1.0;

    for (double ppm = ppm_min; ppm <= ppm_max; ppm += ppm_step) {
        const double fs_candidate = fs_nominal * (1.0 + ppm / 1e6);
        const LinearFit fit = preambleSlopeAtRate(captured, align_offset, base_cfg, n_fft,
                                                  preamble_count, fs_candidate);
        const double abs_slope = std::abs(fit.slope);
        if (abs_slope < best_abs_slope) {
            best_abs_slope = abs_slope;
            best_fs = fs_candidate;
            if (out_slope) {
                *out_slope = fit.slope;
            }
        }
    }
    return best_fs;
}

CaptureResult captureRxOnly(phy::StreamPair& streams, double duration_sec, double sample_rate_hz) {
    CaptureResult result;
    const size_t max_samples = static_cast<size_t>(sample_rate_hz * (duration_sec + 1.0));
    result.samples.resize(max_samples);

    const size_t mtu = streams.mtu();
    std::atomic<size_t> write_pos{0};
    g_running = true;

    const auto t0 = std::chrono::steady_clock::now();
    streams.activate();

    std::thread rx_thread([&]() {
        while (g_running) {
            if (write_pos + mtu > result.samples.size()) {
                break;
            }
            const int ret = streams.read(result.samples.data() + write_pos, mtu);
            if (ret > 0) {
                write_pos += static_cast<size_t>(ret);
            } else if (ret != SOAPY_SDR_TIMEOUT) {
                g_running = false;
                break;
            }
        }
    });

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(duration_sec * 1000.0)));
    const auto t1 = std::chrono::steady_clock::now();
    g_running = false;

    rx_thread.join();
    streams.deactivate();

    result.elapsed_sec =
        std::chrono::duration<double>(t1 - t0).count();
    result.samples.resize(write_pos);
    return result;
}

CaptureResult captureConcurrentTxRx(phy::StreamPair& streams, double duration_sec,
                                  double sample_rate_hz,
                                  const std::vector<phy::Sample>& continuous_tx) {
    CaptureResult result;
    const size_t max_samples = static_cast<size_t>(sample_rate_hz * (duration_sec + 1.0));
    result.samples.resize(max_samples);

    const size_t mtu = streams.mtu();
    std::atomic<size_t> rx_write_pos{0};
    g_running = true;

    size_t tx_offset = 0;
    std::thread tx_thread([&]() {
        while (g_running && tx_offset < continuous_tx.size()) {
            const size_t n = std::min(mtu, continuous_tx.size() - tx_offset);
            const int ret = streams.write(continuous_tx.data() + tx_offset, n);
            if (ret > 0) {
                tx_offset += static_cast<size_t>(ret);
            } else if (ret < 0) {
                g_running = false;
                break;
            }
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::thread rx_thread([&]() {
        while (g_running) {
            if (rx_write_pos + mtu > result.samples.size()) {
                break;
            }
            const int ret = streams.read(result.samples.data() + rx_write_pos, mtu);
            if (ret > 0) {
                rx_write_pos += static_cast<size_t>(ret);
            } else if (ret != SOAPY_SDR_TIMEOUT) {
                g_running = false;
                break;
            }
        }
    });

    const auto t0 = std::chrono::steady_clock::now();
    streams.activate();

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(duration_sec * 1000.0)));
    const auto t1 = std::chrono::steady_clock::now();
    g_running = false;

    tx_thread.join();
    rx_thread.join();
    streams.deactivate();

    result.elapsed_sec =
        std::chrono::duration<double>(t1 - t0).count();
    result.samples.resize(rx_write_pos);
    return result;
}

CaptureResult captureConcurrentToneTxRx(phy::StreamPair& streams, double duration_sec,
                                        double sample_rate_hz, double tone_hz) {
    const size_t mtu = streams.mtu();
    const size_t tx_samples_needed = static_cast<size_t>(sample_rate_hz * (duration_sec + 2.0));
    std::vector<phy::Sample> continuous_tx;
    continuous_tx.reserve(tx_samples_needed);

    phy::ToneGenerator tone(sample_rate_hz, tone_hz);
    std::vector<phy::Sample> chunk(mtu);
    while (continuous_tx.size() < tx_samples_needed) {
        tone.fill(chunk.data(), mtu);
        continuous_tx.insert(continuous_tx.end(), chunk.begin(), chunk.end());
    }

    return captureConcurrentTxRx(streams, duration_sec, sample_rate_hz, continuous_tx);
}

void writeJsonReport(const TestReport& r, const std::string& path) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "Failed to write report: " << path << "\n";
        return;
    }
    out << std::setprecision(10);
    out << "{\n";
    out << "  \"verdict\": \"" << r.verdict << "\",\n";
    out << "  \"fs_nominal_hz\": " << r.fs_nominal << ",\n";
    out << "  \"fs_tx_reported_hz\": " << r.fs_tx_reported << ",\n";
    out << "  \"fs_rx_reported_hz\": " << r.fs_rx_reported << ",\n";
    out << "  \"ppm_threshold\": " << r.ppm_threshold << ",\n";
    out << "  \"center_mhz\": " << r.center_mhz << ",\n";
    out << "  \"tx_gain\": " << r.tx_gain << ",\n";
    out << "  \"rx_gain\": " << r.rx_gain << ",\n";
    out << "  \"tone_hz\": " << r.tone_hz << ",\n";
    out << "  \"phase1\": {\n";
    out << "    \"ran\": " << (r.phase1_ran ? "true" : "false") << ",\n";
    out << "    \"ok\": " << (r.phase1_ok ? "true" : "false") << ",\n";
    out << "    \"tone_fpga_hz\": " << r.tone_fpga_hz << ",\n";
    out << "    \"f_internal_hz\": " << r.f_internal_hz << "\n";
    out << "  },\n";
    out << "  \"phase2\": {\n";
    out << "    \"ran\": " << (r.phase2_ran ? "true" : "false") << ",\n";
    out << "    \"fs_wall_hz\": " << r.fs_wall << ",\n";
    out << "    \"ppm_wall\": " << r.ppm_wall << ",\n";
    out << "    \"samples\": " << r.wall_samples << "\n";
    out << "  },\n";
    out << "  \"phase3\": {\n";
    out << "    \"ran\": " << (r.phase3_ran ? "true" : "false") << ",\n";
    out << "    \"f_ota_measured_hz\": " << r.f_ota_measured_hz << ",\n";
    out << "    \"fs_ota_hz\": " << r.fs_ota << ",\n";
    out << "    \"ppm_ota\": " << r.ppm_ota << "\n";
    out << "  },\n";
    out << "  \"phase4\": {\n";
    out << "    \"ran\": " << (r.phase4_ran ? "true" : "false") << ",\n";
    out << "    \"fs_css_hz\": " << r.fs_css << ",\n";
    out << "    \"ppm_css\": " << r.ppm_css << ",\n";
    out << "    \"preamble_slope_bins_per_sym\": " << r.preamble_slope << ",\n";
    out << "    \"preamble_intercept\": " << r.preamble_intercept << ",\n";
    out << "    \"preamble_hits\": " << r.preamble_hits << ",\n";
    out << "    \"align_offset\": " << r.align_offset << ",\n";
    out << "    \"preamble_peak_bins\": [";
    for (size_t i = 0; i < r.preamble_bins.size(); ++i) {
        if (i > 0) {
            out << ", ";
        }
        out << r.preamble_bins[i];
    }
    out << "]\n  }\n";
    out << "}\n";
}

std::string synthesizeVerdict(const TestReport& r) {
    if (r.phase1_ran && !r.phase1_ok) {
        return "INVALID";
    }

    const bool ota_error =
        (r.phase3_ran && std::abs(r.ppm_ota) > r.ppm_threshold) ||
        (r.phase4_ran && std::abs(r.ppm_css) > r.ppm_threshold);
    const bool wall_error = r.phase2_ran && std::abs(r.ppm_wall) > r.ppm_threshold;

    if (!ota_error && !wall_error) {
        return "NOT_SUPPORTED";
    }

    if (r.phase3_ran && r.phase4_ran && ota_error) {
        const double ota_css_ppm_diff = std::abs(r.ppm_ota - r.ppm_css);
        if (ota_css_ppm_diff < 50.0) {
            return "SUPPORTED";
        }
        return "PARTIAL";
    }

    if (ota_error || wall_error) {
        if (r.phase3_ran && r.phase4_ran) {
            return "PARTIAL";
        }
        return "SUPPORTED";
    }

    return "NOT_SUPPORTED";
}

void printReport(const TestReport& r) {
    std::cout << "\n========== Clock Coherence Report ==========\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Nominal rate:   " << r.fs_nominal / 1e6 << " MSps\n";
    std::cout << "TX reported:    " << r.fs_tx_reported / 1e6 << " MSps\n";
    std::cout << "RX reported:    " << r.fs_rx_reported / 1e6 << " MSps\n";

    if (r.phase1_ran) {
        std::cout << "\n--- Phase 1: Internal RX tone (FPGA reference) ---\n";
        std::cout << "  Expected: " << r.tone_fpga_hz / 1e3 << " kHz\n";
        std::cout << "  Measured: " << r.f_internal_hz / 1e3 << " kHz\n";
        std::cout << "  Status:   " << (r.phase1_ok ? "OK" : "FAIL") << "\n";
    }

    if (r.phase2_ran) {
        std::cout << "\n--- Phase 2: Wall-clock RX sample rate ---\n";
        std::cout << "  Samples:  " << r.wall_samples << "\n";
        std::cout << "  fs_wall:  " << r.fs_wall / 1e6 << " MSps\n";
        std::cout << "  ppm:      " << r.ppm_wall << "\n";
    }

    if (r.phase3_ran) {
        std::cout << "\n--- Phase 3: OTA tone loopback ---\n";
        std::cout << "  TX tone:  " << r.tone_hz / 1e3 << " kHz\n";
        std::cout << "  Measured: " << r.f_ota_measured_hz / 1e3 << " kHz\n";
        std::cout << "  fs_ota:   " << r.fs_ota / 1e6 << " MSps  (ppm=" << r.ppm_ota << ")\n";
    }

    if (r.phase4_ran) {
        std::cout << "\n--- Phase 4: OTA CSS preamble bin drift ---\n";
        std::cout << "  Align:    " << r.align_offset << " samples\n";
        std::cout << "  Hits:     " << r.preamble_hits << "/16\n";
        std::cout << "  Slope:    " << r.preamble_slope << " bins/symbol\n";
        std::cout << "  fs_css:   " << r.fs_css / 1e6 << " MSps  (ppm=" << r.ppm_css << ")\n";
        std::cout << "  Bins:    ";
        for (size_t b : r.preamble_bins) {
            std::cout << " " << b;
        }
        std::cout << "\n";
    }

    std::cout << "\n--- Verdict ---\n";
    std::cout << "  " << r.verdict << "\n";
    std::cout << "============================================\n";
}

}  // namespace

int main(int argc, char** argv) {
    double fs_nominal = 1e6;
    double tone_hz = 100e3;
    double tone_fpga_mhz = 0.1;
    double wall_duration = 10.0;
    double ota_duration = 8.0;
    int tx_gain = 1;
    int rx_gain = 10;
    double center_mhz = 5800.0;
    double ppm_threshold = 100.0;
    std::string output_path;
    std::string debug_dump_dir;
    PhaseSkip skip;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--rate" && i + 1 < argc) {
            fs_nominal = std::stod(argv[++i]);
        } else if (arg == "--tone" && i + 1 < argc) {
            tone_hz = std::stod(argv[++i]);
        } else if (arg == "--tone-fpga-mhz" && i + 1 < argc) {
            tone_fpga_mhz = std::stod(argv[++i]);
        } else if (arg == "--wall-duration" && i + 1 < argc) {
            wall_duration = std::stod(argv[++i]);
        } else if (arg == "--ota-duration" && i + 1 < argc) {
            ota_duration = std::stod(argv[++i]);
        } else if (arg == "--tx-gain" && i + 1 < argc) {
            tx_gain = std::stoi(argv[++i]);
        } else if (arg == "--rx-gain" && i + 1 < argc) {
            rx_gain = std::stoi(argv[++i]);
        } else if (arg == "--freq" && i + 1 < argc) {
            center_mhz = std::stod(argv[++i]);
        } else if (arg == "--ppm-threshold" && i + 1 < argc) {
            ppm_threshold = std::stod(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            output_path = argv[++i];
        } else if (arg == "--debug-dump-dir" && i + 1 < argc) {
            debug_dump_dir = argv[++i];
        } else if (arg == "--skip-phase" && i + 1 < argc) {
            const int p = std::stoi(argv[++i]);
            if (p == 1) {
                skip.phase1 = true;
            } else if (p == 2) {
                skip.phase2 = true;
            } else if (p == 3) {
                skip.phase3 = true;
            } else if (p == 4) {
                skip.phase4 = true;
            }
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

    TestReport report;
    report.fs_nominal = fs_nominal;
    report.tone_hz = tone_hz;
    report.tone_fpga_hz = tone_fpga_mhz * 1e6;
    report.tx_gain = tx_gain;
    report.rx_gain = rx_gain;
    report.center_mhz = center_mhz;
    report.ppm_threshold = ppm_threshold;

    phy::RfFrontend rf;

    try {
        phy::RadioSession session;
        session.open();
        session.setSampleRate(SOAPY_SDR_TX, 0, fs_nominal);
        session.setSampleRate(SOAPY_SDR_RX, 0, fs_nominal);
        report.fs_tx_reported = session.sampleRate(SOAPY_SDR_TX, 0);
        report.fs_rx_reported = session.sampleRate(SOAPY_SDR_RX, 0);
        session.printInfo();

        if (std::abs(report.fs_tx_reported - report.fs_rx_reported) > 1.0) {
            std::cout << "Note: TX and RX reported rates differ by "
                      << (report.fs_tx_reported - report.fs_rx_reported) << " Hz\n";
        }

        constexpr size_t kFftSize = 4096;

        // --- Phase 1: Internal RX tone ---
        if (!skip.phase1) {
            report.phase1_ran = true;
            std::cout << "\n[Phase 1] Internal RX tone...\n";

            rf.configureRx(center_mhz, rx_gain);
            rf.setRxTone(true, tone_fpga_mhz);

            phy::StreamPair streams;
            streams.setupRx(session.device());

            const CaptureResult cap = captureRxOnly(streams, 2.0, fs_nominal);
            rf.setRxTone(false);

            if (cap.samples.size() < kFftSize) {
                std::cerr << "Phase 1: insufficient samples\n";
                report.phase1_ok = false;
            } else {
                report.f_internal_hz = measureToneHz(cap.samples, kFftSize, fs_nominal);
                const phy::RxStats stats = phy::computeStats(cap.samples.data(), cap.samples.size());
                // FPGA tone is generated in the tile sample domain and resampled to the host
                // rate; peak frequency at 1 MSps need not equal tone_fpga_hz.  Validate RX path
                // the same way as rx_tone_sanity: signal present, clipping acceptable.
                const bool peak_ok = stats.peak_dbfs > -60.0;
                const bool clip_ok = stats.clip_fraction < 0.05;
                report.phase1_ok = peak_ok && clip_ok;
                std::cout << "  Internal tone: FPGA target " << report.tone_fpga_hz / 1e3
                          << " kHz, host FFT peak " << report.f_internal_hz / 1e3 << " kHz\n";
                std::cout << "  RX RMS: " << stats.rms_dbfs << " dBFS, peak: " << stats.peak_dbfs
                          << " dBFS, clip: " << (stats.clip_fraction * 100.0) << "%\n";
                std::cout << "  Status:   " << (report.phase1_ok ? "OK" : "FAIL") << "\n";
            }

        } else {
            report.phase1_ok = true;
        }

        if (report.phase1_ran && !report.phase1_ok) {
            report.verdict = synthesizeVerdict(report);
            printReport(report);
            if (!output_path.empty()) {
                writeJsonReport(report, output_path);
            }
            std::cerr << "Phase 1 failed — OTA phases skipped (RX path invalid).\n";
            return 1;
        }

        // --- Phase 2: Wall-clock RX rate ---
        if (!skip.phase2) {
            report.phase2_ran = true;
            std::cout << "\n[Phase 2] Wall-clock RX sample counting...\n";

            rf.configureRx(center_mhz, rx_gain);

            phy::StreamPair streams;
            streams.setupRx(session.device());

            const CaptureResult cap = captureRxOnly(streams, wall_duration, fs_nominal);
            report.wall_samples = cap.samples.size();
            if (cap.elapsed_sec > 0.0) {
                report.fs_wall = static_cast<double>(cap.samples.size()) / cap.elapsed_sec;
                report.ppm_wall = ppmError(report.fs_wall, fs_nominal);
            }
            std::cout << "  " << report.wall_samples << " samples in " << cap.elapsed_sec
                      << " s => " << report.fs_wall / 1e6 << " MSps (ppm=" << report.ppm_wall
                      << ")\n";
        }

        // --- Phase 3: OTA tone loopback ---
        if (!skip.phase3) {
            report.phase3_ran = true;
            std::cout << "\n[Phase 3] OTA tone loopback...\n";

            rf.configureRx(center_mhz, rx_gain);
            rf.configureTx(center_mhz, tx_gain);

            phy::StreamPair streams;
            streams.setupTx(session.device());
            streams.setupRx(session.device());

            const CaptureResult cap =
                captureConcurrentToneTxRx(streams, ota_duration, fs_nominal, tone_hz);

            if (cap.samples.size() < kFftSize) {
                std::cerr << "Phase 3: insufficient RX samples\n";
            } else {
                report.f_ota_measured_hz = measureToneHz(cap.samples, kFftSize, fs_nominal);
                if (report.f_ota_measured_hz > 0.0) {
                    report.fs_ota = fs_nominal * (tone_hz / report.f_ota_measured_hz);
                    report.ppm_ota = ppmError(report.fs_ota, fs_nominal);
                }
                std::cout << "  OTA tone: expected " << tone_hz / 1e3 << " kHz, measured "
                          << report.f_ota_measured_hz / 1e3 << " kHz\n";
                std::cout << "  Implied fs_ota: " << report.fs_ota / 1e6
                          << " MSps (ppm=" << report.ppm_ota << ")\n";
            }
        }

        // --- Phase 4: OTA CSS preamble drift ---
        if (!skip.phase4) {
            report.phase4_ran = true;
            std::cout << "\n[Phase 4] OTA CSS preamble bin drift...\n";

            rf.configureRx(center_mhz, rx_gain);
            rf.configureTx(center_mhz, tx_gain);

            phy::PhyConfig cfg;
            cfg.center_freq_hz = center_mhz * 1e6;
            cfg.sample_rate_hz = fs_nominal;

            phy::ChirpGenerator chirp_gen(cfg);
            const uint32_t n_fft = chirp_gen.fftSize();

            phy::IQBuffer frame = chirp_gen.preamble(cfg.preamble_chirps);
            const phy::IQBuffer sync = chirp_gen.syncWord();
            const phy::IQBuffer sfd = chirp_gen.sfd();
            frame.insert(frame.end(), sync.begin(), sync.end());
            frame.insert(frame.end(), sfd.begin(), sfd.end());

            const size_t tx_needed = static_cast<size_t>(fs_nominal * (ota_duration + 1.0));
            std::vector<phy::Sample> continuous_tx;
            continuous_tx.reserve(tx_needed);
            while (continuous_tx.size() < tx_needed) {
                continuous_tx.insert(continuous_tx.end(), frame.begin(), frame.end());
            }

            phy::StreamPair streams;
            streams.setupTx(session.device());
            streams.setupRx(session.device());

            const CaptureResult cap =
                captureConcurrentTxRx(streams, ota_duration, fs_nominal, continuous_tx);

            if (cap.samples.size() < n_fft * (cfg.preamble_chirps + 2)) {
                std::cerr << "Phase 4: insufficient RX data\n";
            } else {
                report.align_offset = findBestAlignment(cap.samples, chirp_gen.downChirp().data(),
                                                        n_fft, cfg.preamble_chirps);

                phy::FftEngine fft(n_fft);
                std::vector<phy::Sample> work(n_fft);
                for (size_t sym = 0; sym < cfg.preamble_chirps; ++sym) {
                    const size_t pos = report.align_offset + sym * n_fft;
                    const phy::PeakResult peak = demodSymbol(
                        cap.samples.data() + pos, chirp_gen.downChirp().data(), n_fft, fft, work);
                    report.preamble_bins.push_back(peak.bin % n_fft);
                    const size_t wrapped = peak.bin % n_fft;
                    const size_t dist = std::min(wrapped, n_fft - wrapped);
                    if (dist <= 16) {
                        ++report.preamble_hits;
                    }
                }

                std::vector<double> xs(report.preamble_bins.size());
                for (size_t i = 0; i < xs.size(); ++i) {
                    xs[i] = static_cast<double>(i);
                }
                std::vector<double> ys(report.preamble_bins.begin(), report.preamble_bins.end());
                const LinearFit nominal_fit = linearFit(xs, ys);
                report.preamble_slope = nominal_fit.slope;
                report.preamble_intercept = nominal_fit.intercept;

                double best_slope = 0.0;
                report.fs_css = estimateFsFromPreambleDrift(
                    cap.samples, report.align_offset, cfg, n_fft, cfg.preamble_chirps, fs_nominal,
                    &best_slope);
                report.ppm_css = ppmError(report.fs_css, fs_nominal);

                const LinearFit corrected = preambleSlopeAtRate(
                    cap.samples, report.align_offset, cfg, n_fft, cfg.preamble_chirps, report.fs_css);
                report.preamble_slope = corrected.slope;
                report.preamble_intercept = corrected.intercept;

                std::cout << "  Align offset: " << report.align_offset << "\n";
                std::cout << "  Preamble hits: " << report.preamble_hits << "/"
                          << cfg.preamble_chirps << "\n";
                std::cout << "  Nominal slope: " << nominal_fit.slope << " bins/sym\n";
                std::cout << "  fs_css: " << report.fs_css / 1e6
                          << " MSps (ppm=" << report.ppm_css << ")\n";
                std::cout << "  Residual slope at fs_css: " << report.preamble_slope
                          << " bins/sym\n";
            }
        }

        report.verdict = synthesizeVerdict(report);
        printReport(report);

        if (!output_path.empty()) {
            writeJsonReport(report, output_path);
            std::cout << "Report written to " << output_path << "\n";
        }

        if (!debug_dump_dir.empty() && report.phase4_ran) {
            const std::string bins_path = debug_dump_dir + "/clock_preamble_bins.json";
            std::ofstream bout(bins_path);
            if (bout) {
                bout << std::setprecision(10);
                bout << "{\n";
                bout << "  \"fs_nominal_hz\": " << report.fs_nominal << ",\n";
                bout << "  \"fs_ota_hz\": " << report.fs_ota << ",\n";
                bout << "  \"fs_css_hz\": " << report.fs_css << ",\n";
                bout << "  \"fs_wall_hz\": " << report.fs_wall << ",\n";
                bout << "  \"preamble_slope\": " << report.preamble_slope << ",\n";
                bout << "  \"preamble_peak_bins\": [";
                for (size_t i = 0; i < report.preamble_bins.size(); ++i) {
                    if (i > 0) {
                        bout << ", ";
                    }
                    bout << report.preamble_bins[i];
                }
                bout << "]\n}\n";
                std::cout << "Debug bins written to " << bins_path << "\n";
            }
        }

        if (report.verdict == "INVALID") {
            return 1;
        }
        if (report.verdict == "PARTIAL") {
            return 1;
        }
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}
