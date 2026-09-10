#include <phy/lora/receiver.hpp>

#include <phy/dsp/resampler.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#ifdef RX_TRACE
#include <cstdio>
#endif

namespace phy::lora {

namespace {

// Coarse detection: peak/median power ratio in the dechirped FFT per symbol.
// Idle-noise ratios sit at 4.5-8 (max of 128 exponential bins), so 12 (10.8 dB)
// clears the floor while maintaining sensitivity for peer frames.
constexpr double kSnrThresh = 12.0;
// Preamble up-chirps repeat the same chip, so consecutive coarse windows drift
// sub-chip; random payload chips step uniformly over the full range. 8 chips
// retains valid preambles while rejecting false runs on frame tails or uncorrelated noise.
constexpr double kMaxStepChips = 8.0;
constexpr int kRunNeeded = 6;
constexpr double kSyncTolChips = 3.0;

std::vector<uint16_t> shiftValues(const std::vector<uint16_t>& v, int sh, uint32_t n_chips) {
    std::vector<uint16_t> out(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        out[i] = static_cast<uint16_t>((v[i] + sh + n_chips) % n_chips);
    }
    return out;
}

}  // namespace

Receiver::Receiver(const LoraParams& params) : params_(params), sym_(params) {
    sps_ = samplesPerSymbol(params_);
    n_chips_ = chipCount(params_);
    os_ = osFactor(params_);
    win_.resize(sps_);
}

void Receiver::feed(const Sample* chunk, size_t n) {
    buf_.insert(buf_.end(), chunk, chunk + n);
    next_abs_ += n;
    process();
}

void Receiver::reset() {
    buf_.clear();
    out_.clear();
    base_ = 0;
    next_abs_ = 0;
    resetToSearch(0);
}

void Receiver::flush() {
    if (state_ != State::kData) {
        return;
    }
    finalizeData();
}

bool Receiver::pop(ReceivedFrame& out) {
    if (out_.empty()) {
        return false;
    }
    out = std::move(out_.front());
    out_.pop_front();
    return true;
}

const Sample* Receiver::rawWindow(size_t abs_pos) const {
    return buf_.data() + (abs_pos - base_);
}

void Receiver::trimFront(size_t keep_from_abs) {
    if (keep_from_abs <= base_) {
        return;
    }
    const size_t drop = std::min(keep_from_abs - base_, buf_.size());
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<ptrdiff_t>(drop));
    base_ += drop;
}

void Receiver::resetToSearch(size_t scan_from_abs) {
    state_ = State::kSearch;
    run_len_ = 0;
    scan_abs_ = std::max(scan_from_abs, base_);
    resampled_.clear();
}

void Receiver::process() {
    for (;;) {
        bool progress = false;
        switch (state_) {
            case State::kSearch: progress = stepSearch(); break;
            case State::kSync: progress = stepSync(); break;
            case State::kData: progress = stepData(); break;
        }
        if (!progress) {
            break;
        }
    }
}

bool Receiver::stepSearch() {
    if (scan_abs_ < base_) {
        scan_abs_ = base_;
    }
    if (scan_abs_ + sps_ > next_abs_) {
        // bound memory while idle: keep half a symbol of pre-detection
        // history plus resampler margin behind the active run / scan point
        const size_t margin = sps_ / 2 + 8;
        const size_t keep = (run_len_ > 0) ? run_start_abs_ : scan_abs_;
        trimFront(keep > margin ? keep - margin : 0);
        return false;
    }

    const ChipPeak pk = sym_.demod(rawWindow(scan_abs_));
    const bool strong = pk.noise > 0.0 && pk.magnitude > kSnrThresh * pk.noise;
    const bool contiguous =
        run_len_ > 0 && std::abs(wrapSigned(pk.chip - prev_chip_, n_chips_)) < kMaxStepChips;

    if (strong && (run_len_ == 0 || contiguous)) {
        if (run_len_ == 0) {
            run_start_abs_ = scan_abs_;
        }
        ++run_len_;
        prev_chip_ = pk.chip;
        if (run_len_ >= kRunNeeded) {
            found_abs_ = run_start_abs_;
            state_ = State::kSync;
            scan_abs_ += sps_;
            return true;
        }
    } else {
        run_len_ = strong ? 1 : 0;
        run_start_abs_ = scan_abs_;
        prev_chip_ = pk.chip;
    }
    scan_abs_ += sps_;
    return true;
}

bool Receiver::stepSync() {
    // Sync needs the rest of the preamble, the sync word, and the SFD
    // buffered (plus rate-offset slack) before it can run in one shot.
    const size_t span_syms = static_cast<size_t>(params_.preamble_len) + 7;
    const size_t res_len = span_syms * sps_;
    if (next_abs_ < found_abs_ + res_len + sps_ / 2 + 16) {
        return false;
    }

    ReceivedFrame fr;
    ++frames_detected_;
    const size_t fail_resume = found_abs_ + (span_syms - 2) * sps_;

    // --- iterative rate estimate from the preamble chip drift ---
    const size_t back = std::min(found_abs_ - base_, static_cast<size_t>(sps_ / 2));
    origin_abs_ = found_abs_ - back;
    total_ratio_ = 1.0;
    for (int iter = 0; iter < 3; ++iter) {
        double slope;
        if (iter == 0) {
            slope = preambleSlope(sym_, buf_.data(), buf_.size(), found_abs_ - base_,
                                  params_.preamble_len);
        } else {
            slope = preambleSlope(sym_, resampled_.data(), resampled_.size(), back,
                                  params_.preamble_len);
        }
        if (std::isnan(slope)) {
            if (iter == 0) {
                // Late detections leave too few preamble windows to fit.
                // True inter-unit rate offsets are ~2 ppm (shared-band TCXOs),
                // so proceed at ratio 1 and let the data PI loop absorb it.
                slope = 0.0;
            } else {
                break;
            }
        }
        const double ratio_i = 1.0 - slope * os_ / static_cast<double>(sps_);
        total_ratio_ *= ratio_i;
        if (params_.max_rate_ppm > 0.0) {
            // Noise jitter on a short preamble fit can read hundreds of ppm;
            // beyond the oscillator bound it is unphysical, and resampling with
            // it drifts the payload grid and causes CRC failures.
            const double lim = params_.max_rate_ppm * 1e-6;
            total_ratio_ = std::clamp(total_ratio_, 1.0 - lim, 1.0 + lim);
        }

        resampled_.resize(res_len);
        const size_t n = resampleCatmullRom(buf_.data(), buf_.size(),
                                            static_cast<double>(origin_abs_ - base_),
                                            total_ratio_, resampled_.data(), res_len);
        resampled_.resize(n);
        if (n + 2 * sps_ < res_len) {
            out_.push_back(fr);
            resetToSearch(fail_resume);
            return true;
        }
        if (std::abs(ratio_i - 1.0) < 30e-6) {
            break;
        }
    }
    fr.rate_ratio = total_ratio_;
    fr.rate_ppm = (total_ratio_ - 1.0) * 1e6;

    // --- anchor on the strongest early window; walk back to the preamble
    // start (the coarse run can begin mid-preamble or on the AGC ramp) ---
    size_t pre_start = back;
    double first_mag = 0.0;
    for (size_t k = 0; k < 6; ++k) {
        const size_t cand = back + k * sps_;
        if (cand + sps_ > resampled_.size()) {
            break;
        }
        const double m = sym_.demod(resampled_.data() + cand).magnitude;
        if (m > first_mag) {
            first_mag = m;
            pre_start = cand;
        }
    }
    while (pre_start >= sps_) {
        const ChipPeak prev = sym_.demod(resampled_.data() + pre_start - sps_);
        if (prev.magnitude < 0.25 * first_mag) {
            break;
        }
        const ChipPeak cur = sym_.demod(resampled_.data() + pre_start);
        if (std::abs(wrapSigned(prev.chip - cur.chip, n_chips_)) > 8.0) {
            break;
        }
        pre_start -= sps_;
    }

    // count preamble symbols
    std::vector<double> pre_chips;
    while (pre_start + (pre_chips.size() + 1) * sps_ <= resampled_.size()) {
        const ChipPeak pk = sym_.demod(resampled_.data() + pre_start + pre_chips.size() * sps_);
        if (!pre_chips.empty() &&
            std::abs(wrapSigned(pk.chip - pre_chips.back(), n_chips_)) > 6.0) {
            // Check if this is the sync word or noise
            break;
        }
        pre_chips.push_back(pk.chip);
        if (pre_chips.size() > static_cast<size_t>(params_.preamble_len) + 2) {
            break;
        }
    }
    const size_t n_pre = pre_chips.size();
    fr.preamble_symbols = n_pre;
    if (n_pre < 4) {
        out_.push_back(fr);
        resetToSearch(fail_resume);
        return true;
    }

    // --- joint CFO / timing from the up (preamble) and down (SFD) peaks:
    // cfo = wrap(u + d)/2, tau = os * wrap(u - d)/2.
    const size_t n_u = std::min<size_t>(3, n_pre);
    double u_acc = 0.0;
    for (size_t s = n_pre - n_u; s < n_pre; ++s) {
        u_acc += wrapSigned(pre_chips[s] - pre_chips[n_pre - 1], n_chips_);
    }
    const double u = pre_chips[n_pre - 1] + u_acc / static_cast<double>(n_u);

    const size_t sfd_grid = pre_start + (n_pre + 2) * sps_;
    if (sfd_grid + 2 * sps_ > resampled_.size()) {
        out_.push_back(fr);
        resetToSearch(fail_resume);
        return true;
    }
    const ChipPeak dw0 = sym_.demod(resampled_.data() + sfd_grid, true);
    const ChipPeak dw1 = sym_.demod(resampled_.data() + sfd_grid + sps_, true);
    const ChipPeak& dw_a = (dw1.magnitude > dw0.magnitude) ? dw1 : dw0;
    const ChipPeak& dw_b = (dw1.magnitude > dw0.magnitude) ? dw0 : dw1;

    const double half_sym = static_cast<double>(sps_) / 2.0;

    // cfo = wrap(u + d)/2 is unambiguous given a clean SFD read; the timing
    // solve carries a half-symbol ambiguity the sync word check resolves.
    // When |tau| nears a quarter symbol one SFD window straddles the 0.25
    // down-chirp and the first data symbol, degrading peak quality. Try
    // both windows (stronger first) instead of trusting magnitude alone.
    double cfo = wrapSigned(u + dw_a.chip, n_chips_) / 2.0;
    double tau = os_ * wrapSigned(u - dw_a.chip, n_chips_) / 2.0;
    double aligned0 = static_cast<double>(pre_start) - tau;
    bool sync_found = false;
    size_t k_sync = n_pre;
    // SFD candidates: both windows' winners, then their runner-up peaks
    // (an LO-ripple sideband can exceed the true down-chirp tone).
    std::vector<double> dw_cands = {dw_a.chip, dw_b.chip};
    for (const ChipPeak* dwp : {&dw_a, &dw_b}) {
        if (dwp->mag2 > 0.30 * dwp->magnitude) {
            dw_cands.push_back(static_cast<double>(dwp->chip2_int));
        }
    }
    // The sync-word residual check is degenerate along the cfo/tau ambiguity
    // line: a candidate whose cfo and tau are jointly wrong (sideband-hit
    // SFD read) still zeroes e0/e1 because the mis-timed windows read high
    // by the same amount. The mis-timed grid straddles symbol boundaries
    // though, so its sync peaks are weak: score every passing candidate by
    // sync-symbol magnitude and keep the strongest.
    double best_score = 0.0;
    for (const double dw_chip : dw_cands) {
        const double cfo_c = wrapSigned(u + dw_chip, n_chips_) / 2.0;
        const double tau0 = os_ * wrapSigned(u - dw_chip, n_chips_) / 2.0;
        const double tau_alt = (tau0 > 0.0) ? tau0 - half_sym : tau0 + half_sym;
        for (const double tau_cand : {tau0, tau_alt}) {
            const double a0 = static_cast<double>(pre_start) - tau_cand;
            for (int dk = -1; dk <= 1; ++dk) {
                const double q = a0 + (static_cast<double>(n_pre) + dk) * sps_;
                if (q < 0.0) {
                    continue;
                }
                const size_t qi = static_cast<size_t>(std::llround(q));
                if (qi + 2 * sps_ > resampled_.size()) {
                    continue;
                }
                const ChipPeak s0 = sym_.demod(resampled_.data() + qi);
                const ChipPeak s1 = sym_.demod(resampled_.data() + qi + sps_);
                const double e0 = wrapSigned(s0.chip - cfo_c - syncChip0(params_), n_chips_);
                const double e1 = wrapSigned(s1.chip - cfo_c - syncChip1(params_), n_chips_);
                const double score = s0.magnitude + s1.magnitude;
                if (std::abs(e0) < kSyncTolChips && std::abs(e1) < kSyncTolChips &&
                    score > best_score) {
                    sync_found = true;
                    best_score = score;
                    k_sync = n_pre + dk;
                    cfo = cfo_c;
                    tau = tau_cand;
                    aligned0 = a0;
                    fr.sync_err0 = e0;
                    fr.sync_err1 = e1;
                }
            }
        }
    }
    fr.cfo_chips = cfo;
    fr.cfo_hz = cfo * params_.bandwidth_hz / static_cast<double>(n_chips_);

    fr.sync_ok = sync_found;
    fr.tau_samples = tau;

    // SFD magnitude check on the aligned grid
    {
        const double q = aligned0 + (static_cast<double>(k_sync) + 2.0) * sps_;
        const size_t qi = static_cast<size_t>(std::llround(std::max(q, 0.0)));
        if (qi + sps_ <= resampled_.size()) {
            const ChipPeak sfd = sym_.demod(resampled_.data() + qi, true);
            fr.sfd_ok = sfd.magnitude > 0.25 * first_mag;
        }
    }
    fr.synced = fr.sync_ok && fr.sfd_ok;

    // Calibrate through the same raw-domain resampling the data path uses
    // (the sync-span buffer sits on a slightly different grid; at os=2 a
    // one-sample difference is half a chip and flips the rounding).
    // Up-chirp windows read cfo_err + delta/os and the SFD down-chirp
    // windows read cfo_err - delta/os, where delta is the residual window
    // lateness, so the pair separates reference error from timing error.
    auto calDemod = [&](double sym_idx, bool down, double expect, double& err) -> bool {
        const double q = aligned0 + sym_idx * sps_;
        const double raw_f =
            static_cast<double>(origin_abs_) + q * total_ratio_ - static_cast<double>(base_);
        if (raw_f < 1.0) {
            return false;
        }
        const double raw_end = raw_f + static_cast<double>(sps_) * total_ratio_ + 3.0;
        if (raw_end >= static_cast<double>(buf_.size())) {
            return false;
        }
        const size_t n =
            resampleCatmullRom(buf_.data(), buf_.size(), raw_f, total_ratio_, win_.data(), sps_);
        if (n < sps_) {
            return false;
        }
        const ChipPeak pk = sym_.demod(win_.data(), down);
        err = wrapSigned(pk.chip - expect - cfo, n_chips_);
#ifdef RX_TRACE
        std::fprintf(stderr, "  cal sym_idx=%5.1f down=%d expect=%3.0f read=%8.3f err=%+7.3f\n",
                     sym_idx, int(down), expect, pk.chip, err);
#endif
        return true;
    };

    double up_acc = 0.0, dn_acc = 0.0;
    int up_cnt = 0, dn_cnt = 0;
    for (int j = 0; j < 4; ++j) {
        // j=0,1: sync word symbols (known chips); j=2,3: late preamble
        const double sym_idx = sync_found ? (j < 2 ? static_cast<double>(k_sync) + j
                                                   : static_cast<double>(k_sync) - (j - 1))
                                          : static_cast<double>(k_sync) - (j + 1);
        const double expect = (sync_found && j == 0)   ? syncChip0(params_)
                              : (sync_found && j == 1) ? syncChip1(params_)
                                                       : 0.0;
        double err;
        // Bound-check each calibration read: one edge-straddling symbol can
        // return a peak tens of chips off and drag ref_/timing_chips with it.
        if (calDemod(sym_idx, false, expect, err) && std::abs(err) <= kSyncTolChips) {
            up_acc += err;
            ++up_cnt;
        }
    }
    for (int j = 0; j < 2; ++j) {
        // SFD down-chirps span 2.25 symbols after the sync word
        double err;
        if (calDemod(static_cast<double>(k_sync) + 2.0 + j, true, 0.0, err) &&
            std::abs(err) <= kSyncTolChips) {
            dn_acc += err;
            ++dn_cnt;
        }
    }

    double ref = cfo;
    double timing_chips = 0.0;
    if (up_cnt > 0 && dn_cnt > 0) {
        const double m_up = up_acc / up_cnt;
        const double m_dn = dn_acc / dn_cnt;
        ref = cfo + (m_up + m_dn) / 2.0;
        timing_chips = (m_up - m_dn) / 2.0;
    } else if (up_cnt > 0) {
        ref = cfo + up_acc / up_cnt;
    }
    ref_ = ref;
#ifdef RX_TRACE
    std::fprintf(stderr,
                 "SYNC u=%.3f d0=%.3f d1=%.3f cfo=%+.3f tau=%+.1f k_sync=%zu n_pre=%zu "
                 "sync=%d ref=%.3f timing=%+.3f chips ratio=%+.1fppm\n",
                 u, dw0.chip, dw1.chip, cfo, tau, k_sync, n_pre, int(sync_found), ref,
                 timing_chips, (total_ratio_ - 1.0) * 1e6);
#endif

    // data begins 2.25 down-chirps after the sync word, on the aligned grid;
    // remove the measured residual window lateness
    const double data_q = aligned0 + (static_cast<double>(k_sync) + 2.0 + 2.25) * sps_;
    data_raw_f_ = static_cast<double>(origin_abs_) + data_q * total_ratio_ -
                  timing_chips * os_ * total_ratio_;
    fr.start_sample = origin_abs_ +
                      static_cast<size_t>(std::max(0.0, aligned0 * total_ratio_));

    cur_ = std::move(fr);
    data_sym_ = 0;
    timing_int_ = 0.0;
    sym_fracs_.clear();
    alt_syms_.clear();
    needed_syms_ = 8;
    have_len_ = false;
    snr_acc_ = 0.0;
    sir_min_ = std::numeric_limits<double>::infinity();
    pwr_acc_ = 0.0;
    metric_n_ = 0;
    state_ = State::kData;
    resampled_.clear();
    return true;
}

bool Receiver::stepData() {
    const size_t max_syms = frameSymbolCount(params_, 255) + 2;
    const double step = static_cast<double>(sps_) * total_ratio_;

    while (cur_.raw_values.size() < needed_syms_ && data_sym_ < max_syms) {
        const double start_rel = data_raw_f_ - static_cast<double>(base_);
        if (start_rel < 1.0) {
            break;  // history lost; finalize with what we have
        }
        const double end_rel = start_rel + step + 3.0;
        if (end_rel >= static_cast<double>(buf_.size())) {
            return false;  // wait for more input
        }
        const size_t n = resampleCatmullRom(buf_.data(), buf_.size(), start_rel, total_ratio_,
                                            win_.data(), sps_);
        if (n < sps_) {
            return false;
        }

        // Inter-unit CFO leaves ref_ fractional (~0.5 chip worst case), causing
        // scalloping loss (up to ~4 dB) and neighbor-bin leakage in the dechirped FFT.
        // Derotate the fractional part so peaks land on integer bins and reference
        // against the integer remainder.
        const double ref_int = std::round(ref_);
        const double cfo_frac = ref_ - ref_int;
        auto derotate = [&]() {
            if (std::abs(cfo_frac) < 1e-3) {
                return;
            }
            const float dphi =
                static_cast<float>(-2.0 * M_PI * cfo_frac / static_cast<double>(sps_));
            const Sample w = std::polar(1.0f, dphi);
            Sample acc(1.0f, 0.0f);
            for (size_t i = 0; i < sps_; ++i) {
                win_[i] *= acc;
                acc *= w;
            }
        };
        derotate();

        ChipPeak pk = sym_.demod(win_.data());
        double rel = wrapSigned(pk.chip - ref_int, n_chips_);
        double rel_pos = (rel < 0.0) ? rel + n_chips_ : rel;
        double frac = chipFrac(rel_pos);

        // Same-symbol sub-chip refine (see refineSymbolWindow). Fold any
        // accepted shift into data_raw_f_ so the rest of the frame stays
        // centered on the corrected grid.
        {
            double start = start_rel;
            if (refineSymbolWindow(pk, rel_pos, frac, start, ref_int, n_chips_, os_, total_ratio_,
                                   [&](double shift, ChipPeak& out) {
                                       const double s2 = start_rel + shift;
                                       if (s2 < 1.0 ||
                                           s2 + step + 3.0 >= static_cast<double>(buf_.size())) {
                                           return false;
                                       }
                                       const size_t n2 =
                                           resampleCatmullRom(buf_.data(), buf_.size(), s2,
                                                              total_ratio_, win_.data(), sps_);
                                       if (n2 < sps_) {
                                           return false;
                                       }
                                       derotate();
                                       out = sym_.demod(win_.data());
                                       return true;
                                   })) {
                data_raw_f_ += start - start_rel;
            }
        }

        sym_fracs_.push_back(chipFrac(rel_pos));

        const auto value = static_cast<uint16_t>(
            static_cast<uint32_t>(std::lround(rel_pos)) % n_chips_);
        // A strong runner-up peak means the winner may be an LO-ripple FM
        // sideband of the real tone (seen at -1..-3 dBc in bad symbols
        // versus -16 dBc quiescent). Keep the alternate for CRC arbitration.
        if (pk.mag2 > 0.30 * pk.magnitude) {
            const double rel2 = wrapSigned(static_cast<double>(pk.chip2_int) - ref_int, n_chips_);
            const double rel2_pos = (rel2 < 0.0) ? rel2 + n_chips_ : rel2;
            const auto alt = static_cast<uint16_t>(
                static_cast<uint32_t>(std::lround(rel2_pos)) % n_chips_);
            if (alt != value) {
                alt_syms_.push_back({cur_.raw_values.size(), alt, pk.mag2 / pk.magnitude});
            }
        }
        cur_.raw_values.push_back(value);
        if (pk.total_power > 0.0) {
            snr_acc_ += channelSnrDb(pk, n_chips_, params_.spreading_factor);
            const double sir = sirDb(pk);
            if (sir < sir_min_) {
                sir_min_ = sir;
            }
            pwr_acc_ += pk.total_power;
            ++metric_n_;
        }

        // Decision-directed PI timing loop. Residual rate error shows up as
        // a fractional chip offset; P absorbs leftover sub-chip error after
        // the same-symbol refine, I learns the preamble rate residual
        // (~+/-15 ppm). A late window reads high, so positive frac steers
        // the next start earlier.
        timing_int_ += 0.15 * frac;
        const double adj = -chipsToRawSamples(0.7 * frac + timing_int_, os_, total_ratio_);
#ifdef RX_TRACE
        std::fprintf(stderr, "  sym %2zu rel=%9.3f frac=%+6.3f int=%+6.3f\n", data_sym_, rel_pos,
                     frac, timing_int_);
#endif

        data_raw_f_ += step + adj;
        ++data_sym_;
        const double keep = data_raw_f_ - 8.0;
        if (keep > 0.0) {
            trimFront(static_cast<size_t>(keep));
        }

        if (!have_len_ && cur_.raw_values.size() >= 8) {
            DecodeResult head = decodeFrame(params_, cur_.raw_values);
            if (!head.header.valid) {
                // A stream step between the sync calibration and the data
                // start (nothing observes that span) offsets the whole grid
                // by a chip; the header checksum resolves the hypothesis.
                for (const int sh : {+1, -1}) {
                    const auto shifted = shiftValues(cur_.raw_values, sh, n_chips_);
                    DecodeResult h2 = decodeFrame(params_, shifted);
                    if (h2.header.valid) {
                        cur_.raw_values = shifted;
                        ref_ -= sh;  // future symbols read consistently
                        head = std::move(h2);
                        break;
                    }
                }
            }
            if (!head.header.valid) {
                // single sideband-hopped symbol inside the header block
                for (const AltSym& a : alt_syms_) {
                    if (a.idx >= 8) {
                        continue;
                    }
                    auto cand = cur_.raw_values;
                    cand[a.idx] = a.value;
                    DecodeResult h2 = decodeFrame(params_, cand);
                    if (h2.header.valid) {
                        cur_.raw_values = std::move(cand);
                        head = std::move(h2);
                        break;
                    }
                }
            }
            if (!head.header.valid) {
                break;  // bad header; stop early
            }
            LoraParams pl = params_;
            pl.cr = head.header.cr;
            pl.has_crc = head.header.has_crc;
            needed_syms_ = frameSymbolCount(pl, head.header.payload_len);
            have_len_ = true;
        }
    }

    finalizeData();
    return true;
}

void Receiver::finalizeData() {
    cur_.decode = decodeFrame(params_, cur_.raw_values);
    // Chip-lattice offsets can slip past the header (the reduced-rate
    // header drops the two LSBs, so +/-1 chip usually leaves it valid);
    // the payload CRC is the checksum that actually resolves them. Try the
    // whole frame shifted, then tail or single-symbol shifts anchored at
    // the symbols whose readings sat closest to the rounding boundary
    // (that is where a stream step or slow drift flipped the lattice).
    if (cur_.decode.header.valid && cur_.decode.header.has_crc && cur_.decode.crc_ok) {
        cur_.synced = true;
    }
    if (cur_.decode.header.valid && cur_.decode.header.has_crc && !cur_.decode.crc_ok) {
        auto tryDecode = [&](std::vector<uint16_t>&& cand) {
            DecodeResult d2 = decodeFrame(params_, cand);
            if (d2.header.valid && d2.crc_ok) {
                cur_.raw_values = std::move(cand);
                cur_.decode = std::move(d2);
                return true;
            }
            return false;
        };

        bool fixed = false;
        for (const int sh : {-1, +1}) {
            if ((fixed = tryDecode(shiftValues(cur_.raw_values, sh, n_chips_)))) {
                break;
            }
        }

        std::vector<size_t> anchors;
        for (size_t s = 0; s < sym_fracs_.size() && s < cur_.raw_values.size(); ++s) {
            if (std::abs(sym_fracs_[s]) > 0.25) {
                anchors.push_back(s);
            }
        }
        std::sort(anchors.begin(), anchors.end(), [&](size_t a, size_t b) {
            return std::abs(sym_fracs_[a]) > std::abs(sym_fracs_[b]);
        });
        if (anchors.size() > 6) {
            anchors.resize(6);
        }

        // lone flipped symbol
        for (size_t m = 0; !fixed && m < cur_.raw_values.size(); ++m) {
            for (const int sh : {-1, +1}) {
                auto one = cur_.raw_values;
                one[m] = static_cast<uint16_t>((one[m] + sh + n_chips_) % n_chips_);
                if ((fixed = tryDecode(std::move(one)))) {
                    break;
                }
            }
        }

        for (size_t ai = 0; !fixed && ai < anchors.size(); ++ai) {
            const size_t m = anchors[ai];
            for (const int sh : {-1, +1}) {
                // tail shift from the anchor onward
                auto tail = cur_.raw_values;
                for (size_t s = m; s < tail.size(); ++s) {
                    tail[s] = static_cast<uint16_t>((tail[s] + sh + n_chips_) % n_chips_);
                }
                if ((fixed = tryDecode(std::move(tail)))) {
                    break;
                }
            }
        }

        // Sideband arbitration: swap in runner-up peaks for the symbols
        // whose second peak rivaled the winner. Each ambiguous symbol is
        // independently either the winner or the runner-up, so search all
        // subsets of the strongest 6 (63 decodes) plus pairs across 8.
        if (!fixed && !alt_syms_.empty()) {
            auto alts = alt_syms_;
            std::sort(alts.begin(), alts.end(),
                      [](const AltSym& a, const AltSym& b) { return a.ratio > b.ratio; });
            if (alts.size() > 8) {
                alts.resize(8);
            }
            const size_t k = std::min<size_t>(alts.size(), 6);
            for (uint32_t mask = 1; !fixed && mask < (1u << k); ++mask) {
                auto cand = cur_.raw_values;
                bool ok = true;
                for (size_t i = 0; i < k; ++i) {
                    if (!(mask & (1u << i))) {
                        continue;
                    }
                    if (alts[i].idx >= cand.size()) {
                        ok = false;
                        break;
                    }
                    cand[alts[i].idx] = alts[i].value;
                }
                if (ok) {
                    fixed = tryDecode(std::move(cand));
                }
            }
            for (size_t i = 0; !fixed && i < alts.size(); ++i) {
                for (size_t j = i + 1; !fixed && j < alts.size(); ++j) {
                    if (alts[i].idx >= cur_.raw_values.size() ||
                        alts[j].idx >= cur_.raw_values.size()) {
                        continue;
                    }
                    auto two = cur_.raw_values;
                    two[alts[i].idx] = alts[i].value;
                    two[alts[j].idx] = alts[j].value;
                    fixed = tryDecode(std::move(two));
                }
            }
        }
    }
    if (metric_n_ > 0) {
        cur_.snr_db = snr_acc_ / static_cast<double>(metric_n_);
        cur_.sir_db = std::isfinite(sir_min_) ? sir_min_ : 0.0;
        cur_.lvl_dbfs = levelDbfs(pwr_acc_ / static_cast<double>(metric_n_), sps_);
    }
    out_.push_back(std::move(cur_));
    cur_ = ReceivedFrame{};
    resetToSearch(static_cast<size_t>(data_raw_f_) + sps_);
}

}  // namespace phy::lora
