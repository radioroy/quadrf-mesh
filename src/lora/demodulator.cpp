#include <phy/lora/demodulator.hpp>

#include <phy/dsp/resampler.hpp>

#include <algorithm>
#include <cmath>

namespace phy::lora {

IQBuffer resampleCubic(const IQBuffer& in, double start, double ratio, size_t out_count) {
    IQBuffer out(out_count);
    const size_t n = resampleCatmullRom(in.data(), in.size(), start, ratio, out.data(), out_count);
    out.resize(n);
    return out;
}

Demodulator::Demodulator(const LoraParams& params) : params_(params), sym_(params) {
    sps_ = samplesPerSymbol(params_);
    n_chips_ = chipCount(params_);
}

ChipPeak Demodulator::demodWindow(const Sample* window, bool downchirp_ref) {
    return sym_.demod(window, downchirp_ref);
}

ChipPeak Demodulator::demodAt(const IQBuffer& rx, size_t pos, bool downchirp_ref) {
    return sym_.demod(rx.data() + pos, downchirp_ref);
}

FrameResult Demodulator::processFrame(const IQBuffer& rx, size_t from) {
    FrameResult result;
    if (rx.size() < from + 16 * sps_) {
        return result;
    }

    // --- coarse preamble search on the raw capture ---
    // Step one full symbol at a time: a window straddling two identical
    // up-chirps yields a stable peak across full-symbol steps, whereas
    // half-symbol steps alternate by n_chips/os and never look contiguous.
    constexpr double kSnrThresh = 50.0;     // peak power over median bin power
    constexpr double kMaxStepChips = 64.0;  // tolerated drift between windows
    constexpr int kRunNeeded = 6;

    size_t run_start = 0;
    int run_len = 0;
    double prev_chip = 0.0;
    size_t found = 0;
    for (size_t pos = from; pos + sps_ <= rx.size(); pos += sps_) {
        const ChipPeak pk = demodAt(rx, pos);
        const bool strong = pk.noise > 0.0 && pk.magnitude / pk.noise > kSnrThresh &&
                            pk.magnitude > 1e-9;
        const bool contiguous =
            run_len > 0 && std::abs(wrapSigned(pk.chip - prev_chip, n_chips_)) < kMaxStepChips;
        if (strong && (run_len == 0 || contiguous)) {
            if (run_len == 0) {
                run_start = pos;
            }
            ++run_len;
            prev_chip = pk.chip;
            if (run_len >= kRunNeeded) {
                found = run_start;
                break;
            }
        } else {
            run_len = strong ? 1 : 0;
            run_start = pos;
            prev_chip = pk.chip;
        }
    }
    if (run_len < kRunNeeded) {
        return result;
    }

    // --- estimate rate error from preamble chip drift, then resample ---
    // The dechirp peak smears when the rate is off, biasing the slope
    // estimate low; iterate measure -> resample until the residual slope is
    // negligible. Ratios compose, and each resample runs from the original
    // capture so interpolation error does not accumulate.
    const uint32_t os = sps_ / n_chips_;
    const size_t frame_syms_max =
        params_.preamble_len + 5 + frameSymbolCount(params_, 255) + 2;
    const size_t back = std::min(found, static_cast<size_t>(sps_ / 2));

    double total_ratio = 1.0;
    IQBuffer resampled;
    for (int iter = 0; iter < 3; ++iter) {
        const IQBuffer& buf = (iter == 0) ? rx : resampled;
        const size_t start = (iter == 0) ? found : back;
        const double slope = preambleSlope(sym_, buf.data(), buf.size(), start,
                                           params_.preamble_len, &result.preamble_chips,
                                           &result.preamble_mags);
        if (std::isnan(slope)) {
            if (iter == 0) {
                return result;
            }
            break;
        }
        // A stretched capture (more RX samples per nominal sample) makes
        // each dechirp window start progressively early; the peak climbs
        // 1/os chip per sample of early-ness, so slope maps to the ratio:
        const double ratio_i = 1.0 - slope * os / static_cast<double>(sps_);
        total_ratio *= ratio_i;

        const size_t nominal_len = std::min(
            frame_syms_max * static_cast<size_t>(sps_) + sps_,
            static_cast<size_t>(static_cast<double>(rx.size() - (found - back)) / total_ratio));
        resampled = resampleCubic(rx, static_cast<double>(found - back), total_ratio, nominal_len);
        if (resampled.size() < 20 * sps_) {
            return result;
        }
        if (std::abs(ratio_i - 1.0) < 30e-6) {
            break;  // converged
        }
    }
    result.rate_ratio = total_ratio;
    result.rate_ppm = (total_ratio - 1.0) * 1e6;

    // --- fine sync on the resampled stream ---
    // scan one symbol of offsets; maximize preamble energy
    size_t best_off = 0;
    double best_metric = 0.0;
    const size_t n_probe = 6;
    for (size_t off = 0; off + (n_probe + 1) * sps_ <= resampled.size() && off < sps_;
         off += os) {
        double metric = 0.0;
        for (size_t s = 0; s < n_probe; ++s) {
            metric += demodWindow(resampled.data() + off + s * sps_).magnitude;
        }
        if (metric > best_metric) {
            best_metric = metric;
            best_off = off;
        }
    }

    // Anchor on the strongest window near best_off. The energy metric is
    // nearly flat inside the preamble (straddling windows still peak), so
    // best_off itself can land on the RX AGC settling ramp at the very start
    // of the frame, where the peak is weak and its chip value unreliable.
    size_t pre_start = best_off;
    double first_mag = 0.0;
    for (size_t k = 0; k < n_probe; ++k) {
        const size_t cand = best_off + k * sps_;
        if (cand + sps_ > resampled.size()) {
            break;
        }
        const double m = demodWindow(resampled.data() + cand).magnitude;
        if (m > first_mag) {
            first_mag = m;
            pre_start = cand;
        }
    }
    while (pre_start >= sps_) {
        const ChipPeak pk = demodWindow(resampled.data() + pre_start - sps_);
        if (pk.magnitude < 0.25 * first_mag) {
            break;
        }
        const ChipPeak cur = demodWindow(resampled.data() + pre_start);
        if (std::abs(wrapSigned(pk.chip - cur.chip, n_chips_)) > 8.0) {
            break;
        }
        pre_start -= sps_;
    }

    result.start_sample = found - back + static_cast<size_t>(pre_start * result.rate_ratio);

    // count preamble symbols until the sync word breaks the pattern
    std::vector<double> pre_chips;
    size_t n_pre = 0;
    double ref_acc = 0.0;
    while (pre_start + (n_pre + 1) * sps_ <= resampled.size()) {
        const ChipPeak pk = demodWindow(resampled.data() + pre_start + n_pre * sps_);
        if (n_pre > 0) {
            const double d = wrapSigned(pk.chip - pre_chips.back(), n_chips_);
            if (std::abs(d) > 6.0) {
                break;
            }
        }
        pre_chips.push_back(pk.chip);
        ++n_pre;
        if (n_pre > static_cast<size_t>(params_.preamble_len) + 2) {
            break;  // runaway; sync detection below will fail loudly
        }
    }
    result.preamble_symbols = n_pre;
    if (n_pre < 4) {
        return result;
    }
    // reference chip: average of the last few preamble symbols (most stable)
    const size_t n_ref = std::min<size_t>(4, n_pre);
    for (size_t s = n_pre - n_ref; s < n_pre; ++s) {
        ref_acc += wrapSigned(pre_chips[s] - pre_chips[n_pre - 1], n_chips_);
    }
    double ref = pre_chips[n_pre - 1] + ref_acc / n_ref;
    result.ref_chip = ref;

    // --- sync word ---
    const size_t sync_pos = pre_start + n_pre * sps_;
    if (sync_pos + 2 * sps_ > resampled.size()) {
        return result;
    }
    const ChipPeak s0 = demodWindow(resampled.data() + sync_pos);
    const ChipPeak s1 = demodWindow(resampled.data() + sync_pos + sps_);
    result.sync_err0 = wrapSigned(s0.chip - ref - syncChip0(params_), n_chips_);
    result.sync_err1 = wrapSigned(s1.chip - ref - syncChip1(params_), n_chips_);
    result.sync_ok = std::abs(result.sync_err0) < 3.0 && std::abs(result.sync_err1) < 3.0;

    // --- SFD (2.25 downchirps) ---
    const size_t sfd_pos = sync_pos + 2 * sps_;
    if (sfd_pos + 2 * sps_ > resampled.size()) {
        return result;
    }
    const ChipPeak d0 = demodWindow(resampled.data() + sfd_pos, true);
    const ChipPeak d1 = demodWindow(resampled.data() + sfd_pos + sps_, true);
    result.sfd_ok = d0.magnitude > 0.25 * first_mag && d1.magnitude > 0.25 * first_mag;
    result.synced = result.sync_ok && result.sfd_ok;

    // --- data symbols: fractional window + same-symbol sub-chip refine ---
    // After rate correction the resampled stream is near ratio 1; a late
    // window still reads high by 1/os chip per sample. Track a floating
    // start so residual timing can be steered without fighting the lattice.
    double data_f = static_cast<double>(sfd_pos + 2 * sps_ + sps_ / 4);
    const size_t max_syms = frameSymbolCount(params_, 255);
    size_t needed = 8;  // until the header tells us more
    bool have_len = false;
    IQBuffer win(sps_);
    double timing_int = 0.0;

    for (size_t s = 0; s < max_syms && result.raw_values.size() < needed; ++s) {
        if (data_f < 1.0 ||
            data_f + static_cast<double>(sps_) + 3.0 >= static_cast<double>(resampled.size())) {
            break;
        }
        size_t n = resampleCatmullRom(resampled.data(), resampled.size(), data_f, 1.0, win.data(),
                                      sps_);
        if (n < sps_) {
            break;
        }
        ChipPeak pk = demodWindow(win.data());
        double rel = wrapSigned(pk.chip - ref, n_chips_);
        double rel_pos = (rel < 0.0) ? rel + n_chips_ : rel;
        double frac = chipFrac(rel_pos);

        refineSymbolWindow(pk, rel_pos, frac, data_f, ref, n_chips_, os, 1.0,
                           [&](double shift, ChipPeak& out) {
                               const double s2 = data_f + shift;
                               if (s2 < 1.0 ||
                                   s2 + static_cast<double>(sps_) + 3.0 >=
                                       static_cast<double>(resampled.size())) {
                                   return false;
                               }
                               const size_t n2 = resampleCatmullRom(
                                   resampled.data(), resampled.size(), s2, 1.0, win.data(), sps_);
                               if (n2 < sps_) {
                                   return false;
                               }
                               out = demodWindow(win.data());
                               return true;
                           });

        const auto value = static_cast<uint16_t>(
            static_cast<uint32_t>(std::lround(rel_pos)) % n_chips_);
        result.raw_values.push_back(value);
        result.raw_chip_pos.push_back(pk.chip);
        result.data_mags.push_back(pk.magnitude);

        timing_int += 0.15 * frac;
        data_f += static_cast<double>(sps_) - chipsToRawSamples(0.7 * frac + timing_int, os, 1.0);

        if (!have_len && result.raw_values.size() >= 8) {
            const DecodeResult head = decodeFrame(params_, result.raw_values);
            if (head.header.valid) {
                LoraParams pl = params_;
                pl.cr = head.header.cr;
                pl.has_crc = head.header.has_crc;
                needed = frameSymbolCount(pl, head.header.payload_len);
                have_len = true;
            } else {
                break;  // bad header; stop early
            }
        }
    }

    result.decode = decodeFrame(params_, result.raw_values);
    return result;
}

}  // namespace phy::lora
