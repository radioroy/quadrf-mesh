// Synthetic PER / metric sweep for one preset through the PHY receive chain
// (Ddc -> Receiver), the same objects quadrf-lora-phy runs on air.
//
//   lora_awgn_sweep --preset longslow --snr -22:-14:1 --frames 60 --cfo-hz 14000 --ppm 2.4
//
// Each SNR point streams --frames beacon frames (32 B) back to back with
// random gaps and sub-sample start offsets. SNR is S / (N0 * BW). The
// transmit side is synthesized at --os-in x BW with carrier offset and a
// sample-clock offset, then fed to the DDC exactly like the host stream.
// --intf-db adds a continuous co-SF LoRa interferer (random symbols, own
// timing and CFO) at that power relative to the wanted signal.
//
// One line per point:
//   preset snr ok n wrong snr_rep sir_rep lvl_rep lvl_true
// lvl_true is the DDC-output S+N+I power in dBFS the LVL metric should read.

#include "beacon_payload.hpp"

#include <phy/dsp/ddc.hpp>
#include <phy/dsp/resampler.hpp>
#include <phy/lora/coding.hpp>
#include <phy/lora/modulator.hpp>
#include <phy/lora/presets.hpp>
#include <phy/lora/receiver.hpp>
#include <phy/lora/rx_chain.hpp>

#include <cmath>
#include <fstream>
#include <cstdio>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <vector>

using namespace phy;
using namespace phy::lora;

namespace {

std::vector<double> parseSnrs(const std::string& s) {
    std::vector<double> out;
    const size_t c1 = s.find(':');
    if (c1 == std::string::npos) {
        size_t pos = 0;
        while (pos < s.size()) {
            size_t e = s.find(',', pos);
            if (e == std::string::npos) e = s.size();
            out.push_back(std::stod(s.substr(pos, e - pos)));
            pos = e + 1;
        }
        return out;
    }
    const size_t c2 = s.find(':', c1 + 1);
    const double a = std::stod(s.substr(0, c1));
    const double b = std::stod(s.substr(c1 + 1, c2 - c1 - 1));
    const double st = (c2 == std::string::npos) ? 1.0 : std::stod(s.substr(c2 + 1));
    for (double v = a; v <= b + 1e-9; v += st) out.push_back(v);
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    std::string preset = "shortturbo";
    std::string snr_arg = "-10:0:1";
    size_t frames = 40;
    double cfo_hz = 0.0, ppm = 0.0, intf_db = -999.0, intf_off_hz = -2300.0;
    double os_in = 8.0;
    double pass_khz = -1.0, stop_khz = -1.0;
    double amp = 0.05;  // ~ -26 dBFS, typical OTA level
    unsigned seed = 1;
    bool hard = false, verbose = false;
    std::string fm_file;
    double fm_scale = 1.0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : ""; };
        if (a == "--preset") preset = next();
        else if (a == "--snr") snr_arg = next();
        else if (a == "--frames") frames = std::stoul(next());
        else if (a == "--cfo-hz") cfo_hz = std::stod(next());
        else if (a == "--ppm") ppm = std::stod(next());
        else if (a == "--intf-db") intf_db = std::stod(next());
        else if (a == "--intf-off-hz") intf_off_hz = std::stod(next());
        else if (a == "--os-in") os_in = std::stod(next());
        else if (a == "--pass-khz") pass_khz = std::stod(next());
        else if (a == "--stop-khz") stop_khz = std::stod(next());
        else if (a == "--amp") amp = std::stod(next());
        else if (a == "--seed") seed = static_cast<unsigned>(std::stoul(next()));
        else if (a == "--hard") hard = true;
        else if (a == "--fm-file") fm_file = next();
        else if (a == "--fm-scale") fm_scale = std::stod(next());
        else if (a == "-v") verbose = true;
        else {
            std::cerr << "usage: " << argv[0]
                      << " [--preset p] [--snr a:b:step|a,b,..] [--frames n] [--cfo-hz f]"
                         " [--ppm p] [--intf-db d] [--intf-off-hz f] [--os-in k] [--pass-khz k] [--stop-khz k]"
                         " [--amp a] [--seed n] [--hard] [--fm-file f32_hz_at_1khz] [--fm-scale k] [-v]\n";
            return 1;
        }
    }
    LoraParams rxp;
    if (!parseMeshtasticPreset(preset, rxp)) {
        std::cerr << "bad preset " << preset << "\n";
        return 1;
    }
    rxp.sample_rate_hz = meshtasticRxRate(rxp);
    rxp.soft_decoding = !hard;
    LoraParams txp = rxp;
    const double bw = rxp.bandwidth_hz;
    txp.sample_rate_hz = os_in * bw;
    const double fs_in = txp.sample_rate_hz;

    Ddc::Config dc = rxChannelFilter(rxp, fs_in, 0.0);
    if (pass_khz > 0.0) dc.pass_hz = pass_khz * 1e3;
    if (stop_khz > 0.0) dc.stop_hz = stop_khz * 1e3;
    rxp.rx_noise_bw_hz = Ddc(dc).noiseBandwidthHz();
    if (verbose) {
        std::printf("ddc pass %.1f stop %.1f kHz, noise bw %.1f kHz (%.2f dB over BW)\n",
                    dc.pass_hz / 1e3, dc.stop_hz / 1e3, rxp.rx_noise_bw_hz / 1e3,
                    10.0 * std::log10(rxp.rx_noise_bw_hz / bw));
    }

    const Modulator mod(txp, 1.0f);
    const size_t sps_in = mod.samplesPerSymbol();
    const double ratio = 1.0 / (1.0 + ppm * 1e-6);  // TX clock fast -> fewer RX samples per symbol

    // Carrier wander replay: float32 Hz deviations at 1 kHz (e.g. a CW
    // capture between two units), linearly interpolated and looped.
    std::vector<float> fm;
    if (!fm_file.empty()) {
        std::ifstream in(fm_file, std::ios::binary);
        in.seekg(0, std::ios::end);
        fm.resize(static_cast<size_t>(in.tellg()) / sizeof(float));
        in.seekg(0);
        in.read(reinterpret_cast<char*>(fm.data()), static_cast<std::streamsize>(fm.size() * sizeof(float)));
        if (fm.size() < 2) {
            std::cerr << "bad --fm-file " << fm_file << "\n";
            return 1;
        }
    }

    std::mt19937 rng(seed);
    std::normal_distribution<float> gauss(0.0f, 1.0f);
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    for (const double snr_db : parseSnrs(snr_arg)) {
        Ddc ddc(dc);
        Receiver rx(rxp);
        const double s_pow = amp * amp;
        const double n0 = s_pow / (bw * std::pow(10.0, snr_db / 10.0));
        const float sigma = static_cast<float>(std::sqrt(n0 * fs_in / 2.0));
        const double i_amp = (intf_db > -900.0) ? amp * std::pow(10.0, intf_db / 20.0) : 0.0;

        size_t ok = 0, wrong = 0;
        std::set<uint16_t> seen;
        double snr_sum = 0.0, sir_sum = 0.0, lvl_sum = 0.0;
        std::vector<Sample> bb;
        std::vector<double> tx_starts;  // receiver-rate stream index of each preamble
        std::vector<std::vector<uint16_t>> tx_chips;
        size_t in_pos = 0;
        auto drain = [&]() {
            ReceivedFrame r;
            while (rx.pop(r)) {
                const bool crc = r.synced && r.decode.header.valid && r.decode.crc_ok;
                if (!crc) {
                    if (verbose && r.preamble_symbols > 0) {
                        // Nearest emitted frame by start position; list the
                        // symbol indices that read wrong and by how much.
                        size_t best = 0;
                        double best_d = 1e300;
                        for (size_t k = 0; k < tx_starts.size(); ++k) {
                            const double d = std::abs(tx_starts[k] - static_cast<double>(r.start_sample));
                            if (d < best_d) {
                                best_d = d;
                                best = k;
                            }
                        }
                        std::printf("  miss sync=%d hdr=%d pre=%zu cfo=%.0f ppm=%.2f snr=%.2f dstart=%.0f errs:",
                                    int(r.sync_ok), int(r.decode.header.valid), r.preamble_symbols,
                                    r.cfo_hz, r.rate_ppm, r.snr_db, best_d);
                        if (best < tx_chips.size()) {
                            const auto& want = tx_chips[best];
                            const DecodeResult hd = decodeFrame(rxp, r.raw_values);
                            std::printf(" nsym=%zu/%zu hard_crc=%d soft=%d", r.raw_values.size(),
                                        want.size(), int(hd.crc_ok), int(r.soft_decoded));
                            for (size_t s = 0; s < want.size() && s < r.raw_values.size(); ++s) {
                                if (want[s] != r.raw_values[s]) {
                                    const int dv = static_cast<int>(r.raw_values[s]) - static_cast<int>(want[s]);
                                    std::printf(" %zu:%+d", s, dv);
                                }
                            }
                        }
                        std::printf("\n");
                    }
                    continue;
                }
                uint16_t seq = 0;
                if (!beaconSeq(r.decode.payload, seq) || seen.count(seq)) {
                    ++wrong;
                    if (verbose) {
                        const bool parsed = beaconSeq(r.decode.payload, seq);
                        std::printf("  WRONG %s seq=%u len=%zu start=%llu snr=%.2f soft=%d\n",
                                    parsed ? "dup" : "garbage", seq, r.decode.payload.size(),
                                    static_cast<unsigned long long>(r.start_sample), r.snr_db,
                                    int(r.soft_decoded));
                        if (r.decode.payload.size() == 32 && seq >= 1 && seq <= tx_chips.size()) {
                            const auto& want = tx_chips[seq - 1];
                            std::printf("    cfo=%.0f ppm=%.2f nsym=%zu/%zu errs:", r.cfo_hz, r.rate_ppm,
                                        r.raw_values.size(), want.size());
                            for (size_t s = 0; s < want.size() && s < r.raw_values.size(); ++s) {
                                if (want[s] != r.raw_values[s]) {
                                    std::printf(" %zu:%+d", s,
                                                static_cast<int>(r.raw_values[s]) - static_cast<int>(want[s]));
                                }
                            }
                            std::printf("\n");
                        }
                    }
                    continue;
                }
                seen.insert(seq);
                ++ok;
                snr_sum += r.snr_db;
                sir_sum += r.sir_db;
                lvl_sum += r.lvl_dbfs;
                if (verbose) {
                    std::printf("  ok seq=%u snr=%.2f sir=%.1f lvl=%.1f cfo=%.0f ppm=%.2f soft=%d\n",
                                seq, r.snr_db, r.sir_db, r.lvl_dbfs, r.cfo_hz, r.rate_ppm,
                                int(r.soft_decoded));
                }
            }
        };

        // Interferer: a free-running random-symbol chirp stream, offset in
        // time by a random number of samples and --intf-off-hz from the
        // wanted carrier (default -2.3 kHz: another unit's oscillator), so it
        // stays in-channel but off the wanted symbol grid and bins.
        std::vector<Sample> intf_sym;
        size_t intf_pos = sps_in;  // forces a fresh symbol on first use
        std::uniform_int_distribution<int> chip_dist(0, static_cast<int>(chipCount(txp)) - 1);
        double intf_ph = 0.0;
        const double intf_w = 2.0 * M_PI * (cfo_hz + intf_off_hz) / fs_in;

        double ph = 0.0;
        const double w = 2.0 * M_PI * cfo_hz / fs_in;
        double fm_ms = fm.empty() ? 0.0 : uni(rng) * static_cast<double>(fm.size());
        const double fm_step = 1e3 / fs_in;
        double lvl_true_acc = 0.0;
        size_t lvl_true_n = 0;
        auto emit = [&](const std::vector<Sample>& sig, bool has_sig) {
            std::vector<Sample> x(sig.size());
            for (size_t k = 0; k < sig.size(); ++k) {
                Sample v = sig[k] * static_cast<float>(amp);
                v *= Sample(static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
                ph += w;
                if (!fm.empty()) {
                    const size_t i0 = static_cast<size_t>(fm_ms);
                    const double a = fm_ms - static_cast<double>(i0);
                    const double df = (1.0 - a) * fm[i0] + a * fm[(i0 + 1) % fm.size()];
                    ph += 2.0 * M_PI * fm_scale * df / fs_in;
                    fm_ms += fm_step;
                    if (fm_ms >= static_cast<double>(fm.size())) fm_ms -= static_cast<double>(fm.size());
                }
                if (ph > M_PI) ph -= 2.0 * M_PI;
                if (i_amp > 0.0) {
                    if (intf_pos >= intf_sym.size()) {
                        intf_sym = buildUpchirp(static_cast<uint16_t>(chip_dist(rng)),
                                                txp.spreading_factor,
                                                static_cast<uint32_t>(os_in));
                        intf_pos = 0;
                    }
                    Sample iv = intf_sym[intf_pos++] * static_cast<float>(i_amp);
                    iv *= Sample(static_cast<float>(std::cos(intf_ph)),
                                 static_cast<float>(std::sin(intf_ph)));
                    intf_ph += intf_w;
                    if (intf_ph > M_PI) intf_ph -= 2.0 * M_PI;
                    v += iv;
                }
                x[k] = v + Sample(sigma * gauss(rng), sigma * gauss(rng));
            }
            in_pos += x.size();
            bb.clear();
            ddc.process(x.data(), x.size(), bb);
            if (has_sig) {
                for (const Sample& s : bb) lvl_true_acc += std::norm(s);
                lvl_true_n += bb.size();
            }
            rx.feed(bb.data(), bb.size());
            drain();
        };

        if (i_amp > 0.0) {
            intf_pos = static_cast<size_t>(uni(rng) * static_cast<double>(sps_in));
            intf_sym = buildUpchirp(0, txp.spreading_factor, static_cast<uint32_t>(os_in));
        }
        std::vector<Sample> gap;
        for (size_t f = 0; f < frames; ++f) {
            const uint16_t seq = static_cast<uint16_t>(f + 1);
            const std::vector<uint16_t> chips = encodeFrame(txp, beaconPayload(seq));
            const IQBuffer clean = mod.frame(chips);
            // Sample-clock offset and a random sub-sample start.
            const size_t n_out = static_cast<size_t>(static_cast<double>(clean.size()) * ratio) - 4;
            std::vector<Sample> rs(n_out);
            const size_t got = resampleCatmullRom(clean.data(), clean.size(), 1.0 + uni(rng),
                                                  1.0 / ratio, rs.data(), n_out);
            rs.resize(got);
            gap.assign(static_cast<size_t>((2.0 + 2.0 * uni(rng)) * static_cast<double>(sps_in)),
                       Sample(0.0f, 0.0f));
            emit(gap, false);
            tx_starts.push_back(static_cast<double>(in_pos) * rxp.sample_rate_hz / fs_in);
            tx_chips.push_back(chips);
            emit(rs, true);
        }
        gap.assign(30 * sps_in, Sample(0.0f, 0.0f));
        emit(gap, false);
        rx.flush();
        drain();

        const double lvl_true = 10.0 * std::log10(lvl_true_acc / std::max<size_t>(lvl_true_n, 1));
        std::printf("%s %.2f %zu %zu %zu %.2f %.2f %.2f %.2f\n", preset.c_str(), snr_db, ok, frames,
                    wrong, ok ? snr_sum / ok : NAN, ok ? sir_sum / ok : NAN,
                    ok ? lvl_sum / ok : NAN, lvl_true);
        std::fflush(stdout);
    }
    return 0;
}
