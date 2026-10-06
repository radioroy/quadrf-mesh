// Raw RX IQ capture for front-end characterization.
//
// Writes host-rate CF32 straight from SoapySDR (driver Farrow output, no PHY
// DSP) so the spectrum around the RX LO can be inspected offline: DC offset,
// flicker skirt, TX LO leakage, analog LPF shape, and driver aliasing.
//
//   iq_capture --rate 8e6 --seconds 2 --out /tmp/idle_8M.cf32 --tx idle
//
// --lo-offset-khz K tunes the RX LO to (freq - K kHz) so a signal at --freq
// lands at +K kHz in baseband.
// --tx idle reproduces the PHY listen state: MAX2850 in TX mode with the PLL
// locked at --freq and PA_BIAS / FPGA disable_tx gated.

#include <phy/radio/radio_session.hpp>
#include <phy/radio/rf_frontend.hpp>
#include <phy/radio/stream_pair.hpp>

#include <SoapySDR/Constants.h>
#include <SoapySDR/Errors.hpp>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <thread>
#include <iostream>
#include <string>
#include <vector>

using namespace phy;

namespace {

void usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --rate <hz>          Host sample rate (default: 1e6)\n"
              << "  --freq <mhz>         Channel center (default: 5800)\n"
              << "  --lo-offset-khz <k>  RX LO = freq - k kHz (default: 0)\n"
              << "  --rx-gain <db>       RX gain (default: 45)\n"
              << "  --rx-bw <mhz>        MAX2851 baseband filter (default: 4)\n"
              << "  --rx-ant <mask>      RX antenna mask (default: 1)\n"
              << "  --seconds <s>        Capture length after 0.5 s settle (default: 2)\n"
              << "  --tx <off|idle>      Local TX state during capture (default: idle)\n"
              << "  --tx-gain <db>       TX gain for --tx idle (default: 25)\n"
              << "  --tx-lo-offset-khz   TX LO = freq - k kHz for --tx idle (default: 0)\n"
              << "  --pol <rhcp|lhcp>    RX polarization request (default: rhcp)\n"
              << "  --rx-after-mute      Program RX after PA mute (pol write takes effect)\n"
              << "  --tx-stream          Also run a 1 Msps TX stream of zeros (PHY idle)\n"
              << "  --format <cf32|cs16> Output format; cs16 = CF32 x 32512 (default: cf32)\n"
              << "  --out <path>         Output file (required)\n";
}

}  // namespace

int main(int argc, char** argv) {
    double rate = 1e6;
    double freq_mhz = 5800.0;
    double lo_off_khz = 0.0;
    double tx_lo_off_khz = 0.0;
    int rx_gain = 45;
    int rx_bw = 4;
    int rx_ant = 1;
    int tx_gain = 25;
    double seconds = 2.0;
    std::string tx_mode = "idle";
    std::string format = "cf32";
    bool tx_stream = false;
    bool rx_after_mute = false;
    std::string pol = "rhcp";
    std::string out_path;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : ""; };
        if (a == "--rate") rate = std::stod(next());
        else if (a == "--freq") freq_mhz = std::stod(next());
        else if (a == "--lo-offset-khz") lo_off_khz = std::stod(next());
        else if (a == "--tx-lo-offset-khz") tx_lo_off_khz = std::stod(next());
        else if (a == "--rx-gain") rx_gain = std::stoi(next());
        else if (a == "--rx-bw") rx_bw = std::stoi(next());
        else if (a == "--rx-ant") rx_ant = std::stoi(next());
        else if (a == "--tx-gain") tx_gain = std::stoi(next());
        else if (a == "--seconds") seconds = std::stod(next());
        else if (a == "--tx") tx_mode = next();
        else if (a == "--out") out_path = next();
        else if (a == "--format") format = next();
        else if (a == "--tx-stream") tx_stream = true;
        else if (a == "--pol") pol = next();
        else if (a == "--rx-after-mute") rx_after_mute = true;
        else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
        else { std::cerr << "Unknown argument: " << a << "\n"; usage(argv[0]); return 1; }
    }
    if (out_path.empty()) {
        usage(argv[0]);
        return 1;
    }

    RfFrontend rf;
    const uint16_t saved_reg = rf.readRegister(0x2E);
    try {
        RadioSession radio;
        radio.open();
        if (tx_stream) {
            radio.setSampleRate(SOAPY_SDR_TX, 0, 1e6);
        }
        radio.setSampleRate(SOAPY_SDR_RX, 0, rate);
        rf.setDigitalLoopback(false);
        const double rx_lo = freq_mhz - lo_off_khz * 1e-3;
        const RfPolarization rpol = (pol == "lhcp") ? RfPolarization::Lhcp : RfPolarization::Rhcp;
        if (tx_mode == "idle") {
            // PHY order: RX is programmed while MAX2850 TX is enabled (0x23=0),
            // then PA mute sets disable_tx.
            rf.configureTx(freq_mhz - tx_lo_off_khz * 1e-3, tx_gain, 20, 1);
            if (rx_after_mute) {
                rf.paMute();
                rf.configureRx(rx_lo, rx_gain, rx_bw, rx_ant, rpol);
            } else {
                rf.configureRx(rx_lo, rx_gain, rx_bw, rx_ant, rpol);
                rf.paMute();
            }
        } else {
            rf.txOff();
            rf.configureRx(rx_lo, rx_gain, rx_bw, rx_ant, rpol);
        }
        std::fprintf(stderr, "fpga 0x24 (pol) = 0x%04x\n", rf.readRegisterNoSetup(0x24));
        std::fprintf(stderr, "rx lo %.6f MHz rate %.3f Msps gain %d bw %d tx=%s\n", rx_lo,
                     rate / 1e6, rx_gain, rx_bw, tx_mode.c_str());

        StreamPair streams;
        if (tx_stream) {
            streams.setupTx(radio.device());
        }
        streams.setupRx(radio.device());
        const size_t mtu = streams.mtu();
        const size_t skip = static_cast<size_t>(0.5 * rate);
        const size_t want = static_cast<size_t>(seconds * rate);
        // CS16 at 256 counts per CS8 LSB keeps the driver's float Farrow
        // output without re-quantizing to 8 bits, at half the RAM of CF32.
        const bool cs16 = (format == "cs16");
        std::vector<Sample> cap(cs16 ? 0 : want);
        std::vector<int16_t> cap16(cs16 ? 2 * want : 0);
        constexpr float kCs16Scale = 127.0f * 256.0f;
        std::vector<Sample> chunk(mtu);
        const int g_before = rf.readRxGainNoSetup();
        streams.activate();
        std::atomic<bool> tx_run{tx_stream};
        std::thread tx_thread;
        if (tx_stream) {
            tx_thread = std::thread([&]() {
                std::vector<Sample> zeros(65536, Sample(0.0f, 0.0f));
                while (tx_run) {
                    streams.write(zeros.data(), zeros.size());
                }
            });
        }
        const int g_after = rf.readRxGainNoSetup();
        if (g_after != rx_gain) {
            rf.setRxGainNoSetup(rx_gain);
        }
        std::fprintf(stderr, "0x6A gain: after configureRx %d, after activate %d, now %d\n",
                     g_before, g_after, rf.readRxGainNoSetup());
        size_t seen = 0, got = 0;
        double p = 0.0;
        Sample dc(0.0f, 0.0f);
        bool announced = false;
        while (got < want) {
            const int ret = streams.read(chunk.data(), mtu);
            if (ret == SOAPY_SDR_TIMEOUT) continue;
            if (ret < 0) {
                std::cerr << "RX error " << SoapySDR::errToStr(ret) << "\n";
                break;
            }
            for (int k = 0; k < ret && got < want; ++k, ++seen) {
                if (seen < skip) continue;
                const Sample s = chunk[k];
                p += std::norm(s);
                dc += s;
                if (cs16) {
                    cap16[2 * got] = static_cast<int16_t>(std::lrint(s.real() * kCs16Scale));
                    cap16[2 * got + 1] = static_cast<int16_t>(std::lrint(s.imag() * kCs16Scale));
                } else {
                    cap[got] = s;
                }
                ++got;
            }
            if (!announced && got > 0) {
                std::fprintf(stderr, "CAPTURING\n");
                announced = true;
            }
        }
        tx_run = false;
        if (tx_thread.joinable()) tx_thread.join();
        streams.deactivate();
        if (tx_mode == "idle") rf.txOff();
        rf.writeRegister(0x2E, saved_reg);

        FILE* f = std::fopen(out_path.c_str(), "wb");
        if (!f) {
            std::perror("fopen");
            return 1;
        }
        if (cs16) {
            std::fwrite(cap16.data(), sizeof(int16_t), 2 * got, f);
        } else {
            std::fwrite(cap.data(), sizeof(Sample), got, f);
        }
        std::fclose(f);
        dc /= static_cast<float>(got ? got : 1);
        std::fprintf(stderr, "wrote %zu samples, rms %.2f dBFS, dc %.2f dBFS\n", got,
                     10.0 * std::log10(p / (got ? got : 1) + 1e-30),
                     20.0 * std::log10(std::abs(dc) + 1e-30));
    } catch (const std::exception& ex) {
        try { rf.writeRegister(0x2E, saved_reg); } catch (...) {}
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
    return 0;
}
