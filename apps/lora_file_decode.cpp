// Offline decode of a CF32 capture through the streaming Receiver.
//
//   lora_file_decode cap.cf32 --rate 8e6 --if-khz 500
//
// Runs the PHY receive chain: Ddc (mix by -if, rxChannelFilter for the
// preset, decimate to 2 x BW) then the Receiver. A file already at 2 x BW
// with --if-khz 0 goes straight in. Beacon payloads (lora_beacon) are
// checked and counted by sequence number.

#include "beacon_payload.hpp"

#include <phy/dsp/ddc.hpp>
#include <phy/lora/presets.hpp>
#include <phy/lora/receiver.hpp>
#include <phy/lora/rx_chain.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <vector>

using namespace phy;
using namespace phy::lora;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: " << argv[0]
                  << " file.cf32 [--rate hz] [--if-khz k] [--pass-khz k] [--stop-khz k]"
                     " [--preset name] [--dump-bb out.cf32] [--power-fold] [--hard]"
                     " [--timing-kp g] [--timing-ki g] [--refine-gate dB]"
                     " [--awgn-n0 dBFS/Hz] [--seed n] [-v]\n";
        return 1;
    }
    std::string path = argv[1];
    double rate = 1e6;
    double if_khz = 0.0;
    double pass_khz = -1.0, stop_khz = -1.0;  // default: rxChannelFilter
    std::string preset = "shortturbo";
    std::string dump_path;
    bool verbose = false;
    bool power_fold = false;
    bool hard = false;
    double awgn_n0 = -999.0;
    unsigned seed = 1;
    double kp = -1.0, ki = -1.0, refine_gate = -999.0;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : ""; };
        if (a == "--rate") rate = std::stod(next());
        else if (a == "--if-khz") if_khz = std::stod(next());
        else if (a == "--pass-khz") pass_khz = std::stod(next());
        else if (a == "--stop-khz") stop_khz = std::stod(next());
        else if (a == "--preset") preset = next();
        else if (a == "--dump-bb") dump_path = next();
        else if (a == "--power-fold") power_fold = true;
        else if (a == "--hard") hard = true;
        else if (a == "--awgn-n0") awgn_n0 = std::stod(next());
        else if (a == "--seed") seed = static_cast<unsigned>(std::stoul(next()));
        else if (a == "--timing-kp") kp = std::stod(next());
        else if (a == "--timing-ki") ki = std::stod(next());
        else if (a == "--refine-gate") refine_gate = std::stod(next());
        else if (a == "-v") verbose = true;
        else { std::cerr << "unknown arg " << a << "\n"; return 1; }
    }
    LoraParams params;
    if (!parseMeshtasticPreset(preset, params)) {
        std::cerr << "bad preset\n";
        return 1;
    }
    params.coherent_fold = !power_fold;
    params.soft_decoding = !hard;
    if (kp >= 0.0) params.timing_kp = kp;
    if (ki >= 0.0) params.timing_ki = ki;
    if (refine_gate > -900.0) params.refine_min_snr_db = refine_gate;
    params.sample_rate_hz = meshtasticRxRate(params);
    Ddc::Config dc = rxChannelFilter(params, rate, if_khz * 1e3);
    if (pass_khz > 0.0) dc.pass_hz = pass_khz * 1e3;
    if (stop_khz > 0.0) dc.stop_hz = stop_khz * 1e3;
    Ddc ddc(dc);
    params.rx_noise_bw_hz = ddc.noiseBandwidthHz();
    Receiver rx(params);

    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        std::perror("fopen");
        return 1;
    }
    FILE* dump = dump_path.empty() ? nullptr : std::fopen(dump_path.c_str(), "wb");

    size_t n_det = 0, n_sync = 0, n_hdr = 0, n_crc = 0, n_ok = 0, n_bad_payload = 0;
    std::set<uint16_t> seqs;
    double snr_sum = 0.0, sir_sum = 0.0, lvl_sum = 0.0, cfo_sum = 0.0;
    double snr_min = 1e9, snr_max = -1e9;
    auto drain = [&]() {
        ReceivedFrame r;
        while (rx.pop(r)) {
            if (r.preamble_symbols == 0 && !r.synced) {
                continue;
            }
            ++n_det;
            n_sync += r.synced;
            n_hdr += r.decode.header.valid;
            const bool crc = r.decode.header.valid && r.decode.crc_ok;
            n_crc += crc;
            uint16_t seq = 0;
            const bool ok = crc && beaconSeq(r.decode.payload, seq);
            if (crc && !ok) ++n_bad_payload;
            if (ok) {
                ++n_ok;
                seqs.insert(seq);
                snr_sum += r.snr_db;
                sir_sum += r.sir_db;
                lvl_sum += r.lvl_dbfs;
                cfo_sum += r.cfo_hz;
                snr_min = std::min(snr_min, r.snr_db);
                snr_max = std::max(snr_max, r.snr_db);
            }
            if (verbose) {
                std::printf("frame start=%zu sync=%d hdr=%d crc=%d ok=%d seq=%u snr=%.2f sir=%.1f "
                            "lvl=%.1f cfo=%.0f ppm=%.2f soft=%d plen=%zu\n",
                            r.start_sample, int(r.synced), int(r.decode.header.valid), int(crc),
                            int(ok), ok ? seq : 0, r.snr_db, r.sir_db, r.lvl_dbfs, r.cfo_hz,
                            r.rate_ppm, int(r.soft_decoded), r.decode.payload.size());
            }
        }
    };

    // White complex noise at the input rate: N0 in dBFS/Hz -> per-component
    // sigma^2 = N0 * fs / 2.
    const float awgn_sigma =
        (awgn_n0 > -900.0) ? static_cast<float>(std::sqrt(std::pow(10.0, awgn_n0 / 10.0) * rate / 2.0))
                           : 0.0f;
    std::mt19937 rng(seed);
    std::normal_distribution<float> gauss(0.0f, 1.0f);

    const bool cs16 = path.size() > 5 && path.compare(path.size() - 5, 5, ".cs16") == 0;
    std::vector<Sample> in(1 << 16), bb;
    std::vector<int16_t> raw16(2 * in.size());
    size_t total = 0;
    for (;;) {
        size_t got = 0;
        if (cs16) {
            got = std::fread(raw16.data(), 2 * sizeof(int16_t), in.size(), f);
            constexpr float kInv = 1.0f / (127.0f * 256.0f);
            for (size_t k = 0; k < got; ++k) {
                in[k] = Sample(raw16[2 * k] * kInv, raw16[2 * k + 1] * kInv);
            }
        } else {
            got = std::fread(in.data(), sizeof(Sample), in.size(), f);
        }
        if (got == 0) break;
        total += got;
        if (awgn_sigma > 0.0f) {
            for (size_t k = 0; k < got; ++k) {
                in[k] += Sample(awgn_sigma * gauss(rng), awgn_sigma * gauss(rng));
            }
        }
        bb.clear();
        ddc.process(in.data(), got, bb);
        if (dump) std::fwrite(bb.data(), sizeof(Sample), bb.size(), dump);
        rx.feed(bb.data(), bb.size());
        drain();
    }
    rx.flush();
    drain();
    std::fclose(f);
    if (dump) std::fclose(dump);

    const unsigned span = seqs.empty() ? 0 : (*seqs.rbegin() - *seqs.begin() + 1);
    std::printf("%s: %.2f s, ddc=%s taps=%zu+%zu if=%.0fk | det=%zu sync=%zu hdr=%zu crc=%zu "
                "ok=%zu bad=%zu uniq=%zu span=%u",
                path.c_str(), total / rate, ddc.passthrough() ? "off" : "on", ddc.stage1Taps(),
                ddc.stage2Taps(), if_khz, n_det, n_sync, n_hdr, n_crc, n_ok, n_bad_payload,
                seqs.size(), span);
    if (n_ok) {
        std::printf(" | snr %.2f (%.2f..%.2f) sir %.1f lvl %.1f cfo %.0f", snr_sum / n_ok, snr_min,
                    snr_max, sir_sum / n_ok, lvl_sum / n_ok, cfo_sum / n_ok);
    }
    std::printf(" | coh %zu/%zu | soft hdr=%zu frames=%zu\n", rx.coherentSymbols(),
                rx.dataSymbols(), rx.softHeaders(), rx.softFrames());
    return 0;
}
