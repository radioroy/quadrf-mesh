// Offline validation of the RX DDC: filter response and an IF-offset frame
// through Ddc -> Receiver, including a DC term at the RX LO that would sit
// inside the channel at zero IF.

#include <phy/dsp/ddc.hpp>
#include <phy/lora/coding.hpp>
#include <phy/lora/modulator.hpp>
#include <phy/lora/presets.hpp>
#include <phy/lora/receiver.hpp>

#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace phy;
using namespace phy::lora;

namespace {

int g_failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
    if (!ok) {
        ++g_failures;
    }
}

// Steady-state gain (dB) of the DDC for a tone at f_in Hz in the input stream.
double toneGainDb(const Ddc::Config& cfg, double f_in) {
    Ddc ddc(cfg);
    const size_t n = static_cast<size_t>(cfg.fs_in * 4e-3);
    std::vector<Sample> x(n), y;
    for (size_t k = 0; k < n; ++k) {
        const double ph = 2.0 * M_PI * f_in * static_cast<double>(k) / cfg.fs_in;
        x[k] = Sample(static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
    }
    ddc.process(x.data(), n, y);
    double p = 0.0;
    const size_t skip = y.size() / 2;
    for (size_t k = skip; k < y.size(); ++k) {
        p += std::norm(y[k]);
    }
    return 10.0 * std::log10(p / static_cast<double>(y.size() - skip) + 1e-30);
}

}  // namespace

int main() {
    std::printf("DDC response (8 Msps in, IF +500 kHz, 1 Msps out)\n");
    Ddc::Config cfg;
    cfg.fs_in = 8e6;
    cfg.f_if = 500e3;
    {
        Ddc probe(cfg);
        std::printf("  taps: stage1 %zu, stage2 %zu\n", probe.stage1Taps(), probe.stage2Taps());
    }
    const double g_c = toneGainDb(cfg, 500e3);
    const double g_pe = std::max(std::abs(toneGainDb(cfg, 500e3 + 270e3)),
                                 std::abs(toneGainDb(cfg, 500e3 - 270e3)));
    const double g_dc = toneGainDb(cfg, 0.0);
    const double g_alias = toneGainDb(cfg, 500e3 + 1e6 + 100e3);
    const double g_st = toneGainDb(cfg, 500e3 + 400e3);
    std::printf("  center %.3f dB, |pass edge| %.3f dB, DC %.1f dB, +400k %.1f dB, "
                "alias(+1.1M) %.1f dB\n",
                g_c, g_pe, g_dc, g_st, g_alias);
    check(std::abs(g_c) < 0.05, "unity gain at channel center");
    check(g_pe < 0.1, "passband flat to +-270 kHz");
    check(g_dc < -65.0, "RX LO / DC term rejected");
    check(g_st < -65.0, "stopband beyond 330 kHz");
    check(g_alias < -65.0, "aliasing image rejected");

    std::printf("IF frame through Ddc -> Receiver\n");
    LoraParams rx_params;
    parseMeshtasticPreset("shortturbo", rx_params);
    LoraParams tx_params;
    parseMeshtasticPreset("shortturbo", tx_params, cfg.fs_in);
    std::vector<uint8_t> payload(32);
    for (size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>(i * 29 + 3);
    }
    const Modulator mod(tx_params, 0.1f);
    const IQBuffer frame = mod.frame(encodeFrame(tx_params, payload));
    const size_t lead = static_cast<size_t>(cfg.fs_in * 3e-3);
    IQBuffer x(lead + frame.size() + lead, Sample(0.0f, 0.0f));
    std::mt19937 rng(7);
    // per-sample noise at 8 Msps; ~-3 dB in-channel SNR after the channel filter
    std::normal_distribution<float> g(0.0f, 0.1f);
    const Sample dc(0.2f, -0.1f);
    const double w = 2.0 * M_PI * (cfg.f_if + 3.1e3) / cfg.fs_in;  // + small CFO
    for (size_t k = 0; k < x.size(); ++k) {
        Sample s(0.0f, 0.0f);
        if (k >= lead && k < lead + frame.size()) {
            const double ph = w * static_cast<double>(k);
            s = frame[k - lead] * Sample(static_cast<float>(std::cos(ph)),
                                          static_cast<float>(std::sin(ph)));
        }
        x[k] = s + dc + Sample(g(rng), g(rng));
    }
    for (const double if_hz : {cfg.f_if, 0.0}) {
        Ddc::Config c2 = cfg;
        c2.f_if = if_hz;
        Ddc ddc(c2);
        Receiver rx(rx_params);
        std::vector<Sample> bb;
        for (size_t off = 0; off < x.size(); off += 10007) {
            bb.clear();
            ddc.process(x.data() + off, std::min<size_t>(10007, x.size() - off), bb);
            rx.feed(bb.data(), bb.size());
        }
        rx.flush();
        ReceivedFrame r;
        bool ok = false;
        double snr = 0.0;
        while (rx.pop(r)) {
            if (r.decode.crc_ok && r.decode.payload == payload) {
                ok = true;
                snr = r.snr_db;
            }
        }
        if (if_hz != 0.0) {
            check(ok, "decode at +500 kHz IF with DC 13 dB above signal (snr " +
                          std::to_string(snr) + ")");
        } else {
            std::printf("  (DDC tuned to 0 Hz instead: decode=%d, expected 0)\n", int(ok));
            check(!ok, "mis-tuned DDC rejects the frame");
        }
    }

    std::printf("%s\n", g_failures ? "FAIL" : "PASS");
    return g_failures ? 1 : 0;
}
