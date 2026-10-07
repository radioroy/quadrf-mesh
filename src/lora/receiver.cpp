#include <phy/lora/receiver.hpp>

#include <phy/dsp/resampler.hpp>

#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <limits>
#ifdef RX_TRACE
#include <cstdio>
#endif

namespace phy::lora {

namespace {

// Coarse detection: peak/median power ratio in the dechirped FFT per symbol.
// Idle-noise ratios sit at 4.5-8 (max of 128 exponential bins), so a single
// window crosses 6 often; the run-of-6 requirement below is what rejects noise.
// 30 s of pure noise at 1 Msps gave 0 false syncs at 6 (3 at 5). Against 12,
// PER at -9 dB channel SNR drops from 0.65 to 0.17 because detection, not
// decoding, was the limit.
constexpr double kSnrThresh = 6.0;
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

// Soft analogue of shiftValues on one stored row: value u now reads what
// value u - sh held.
void rotateSoftRow(float* m, uint32_t n_chips, int sh) {
    const uint32_t s = static_cast<uint32_t>((sh % static_cast<int>(n_chips) + static_cast<int>(n_chips)) %
                                             static_cast<int>(n_chips));
    std::rotate(m, m + (n_chips - s) % n_chips, m + n_chips);
}

}  // namespace

// The explicit header carries only a 5-bit checksum, so ~1 in 32 noise
// headers parse as valid; one that also reads CRC-off would pass with no
// payload check at all. When the link runs with CRC, such headers are noise.
void Receiver::enforceCrcFlag(DecodeResult& d) const {
    if (params_.has_crc && d.header.valid && !d.header.has_crc) {
        d.header.valid = false;
        d.crc_ok = false;
    }
}

DecodeResult Receiver::decodeHard(const std::vector<uint16_t>& values) const {
    DecodeResult d = decodeFrame(params_, values);
    enforceCrcFlag(d);
    return d;
}

DecodeResult Receiver::decodeSoft(const SoftSymbols& soft, int shift) const {
    DecodeResult d = decodeFrameSoft(params_, soft, shift);
    enforceCrcFlag(d);
    if (d.crc_ok && d.tail_margin < params_.soft_tail_margin) {
        d.crc_ok = false;
    }
    return d;
}

SoftSymbols Receiver::softRows(size_t n_syms) const {
    SoftSymbols soft;
    soft.mags = soft_mags_.data();
    soft.n_syms = std::min(n_syms, soft_mags_.size() / n_chips_);
    estimateSoftScale(soft.mags, soft.n_syms, n_chips_, soft.tone_amp, soft.noise_pow);
    return soft;
}

Receiver::Receiver(const LoraParams& params) : params_(params), sym_(params) {
    sps_ = samplesPerSymbol(params_);
    n_chips_ = chipCount(params_);
    os_ = osFactor(params_);
    sf_gain_db_ = 10.0 * std::log10(static_cast<double>(n_chips_) / 128.0);
    win_.resize(sps_);

    const double t_sym = static_cast<double>(n_chips_) / params_.bandwidth_hz;
    sigma_w_ = wanderChipsPerSymbol(t_sym, params_.wander_rate_hz_per_s, params_.wander_cap_hz);
    wander_ = sigma_w_ >= 0.05 && os_ >= 2;
    step_tol_ = kMaxStepChips;
    sync_tol_ = kSyncTolChips;
    if (wander_) {
        // Phase steps between sub-blocks must stay well inside +-pi: the
        // residual after the provisional decision spans ~sigma_w, so aim
        // for ~1/8 chip per sub-block.
        n_sub_max_ = 4;
        while (n_sub_max_ < 8.0 * sigma_w_ + 4.0 && n_sub_max_ * 16 < sps_) {
            n_sub_max_ *= 2;
        }
        // The wander's correlation time is ~10 ms (spectrum flat to ~30 Hz):
        // below that a slope carries into the next symbol, above it the
        // carrier has already turned.
        trk_beta_ = std::exp(-t_sym / 10e-3);
        joint_frac_ = std::min(1.0, 1.5e-3 / t_sym);
        kf_q_ = params_.wander_irw_q * std::pow(t_sym, 5.0);
        step_tol_ = std::max(kMaxStepChips, 4.0 * sigma_w_);
        sync_tol_ = std::max(kSyncTolChips, 3.0 * sigma_w_);
        // Legendre order: a ramp is all a 1-2 ms symbol sees of the wander.
        // The 60-150 Hz part (~70 Hz RMS) is a full cycle inside an 8 ms
        // SF11 symbol, and VLS symbols (65 ms) hold several.
        const int order = sigma_w_ < 1.0 ? 1 : sigma_w_ < 2.0 ? 2 : sigma_w_ < 4.0 ? 3 : 5;
        wfit_.configure(sps_, n_chips_, sigma_w_, order);
        dech_.resize(sps_);
    }
}

uint32_t Receiver::subBlocks(const ChipPeak& pk) const {
    // Folded noise bins are sums of os exponentials; mean/median ~1.19 at os=2.
    const double noise_var = 1.19 * pk.noise / (static_cast<double>(os_) * sps_);
    const double sig = pk.total_power / (static_cast<double>(sps_) * sps_) - noise_var;
    if (noise_var <= 0.0 || sig <= 0.0) {
        return 4;
    }
    const double snr = sig / noise_var;  // per sample
    uint32_t n = n_sub_max_;
    while (n > 4 && (static_cast<double>(sps_) / n) * snr < 4.0) {
        n /= 2;
    }
    return n;
}

bool Receiver::fitWindow(const Sample* w, bool down, int32_t b1, int32_t wrap,
                         const ChipPeak& pk) {
    const IQBuffer& ref = down ? sym_.upchirp() : sym_.downchirp();
    for (uint32_t i = 0; i < sps_; ++i) {
        dech_[i] = w[i] * ref[i];
    }
    const double noise_var = 1.19 * pk.noise / (static_cast<double>(os_) * sps_);
    const int32_t sps = static_cast<int32_t>(sps_);
    return wfit_.fit(dech_.data(), ((b1 % sps) + sps) % sps, wrap, noise_var, subBlocks(pk));
}

void Receiver::derotateRamp(Sample* w, double f0, double slope) const {
    if (std::abs(f0) < 1e-4 && std::abs(slope) < 1e-4) {
        return;
    }
    const double inv = 1.0 / sps_;
    double phase = 0.0;
    for (uint32_t n = 0; n < sps_; ++n) {
        const double f = f0 + slope * (n + 0.5) * inv;
        w[n] *= Sample(static_cast<float>(std::cos(phase)), static_cast<float>(-std::sin(phase)));
        phase += 2.0 * M_PI * f * inv;
    }
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
        run_len_ > 0 && std::abs(wrapSigned(pk.chip - prev_chip_, n_chips_)) < step_tol_;

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
        if (wander_) {
            // Carrier wander over the preamble reads as tens of ppm of
            // rate; the true ~2 ppm is absorbed by the carrier tracker.
            slope = 0.0;
        } else if (iter == 0) {
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
        if (std::abs(wrapSigned(prev.chip - cur.chip, n_chips_)) > step_tol_) {
            break;
        }
        pre_start -= sps_;
    }

    // count preamble symbols
    std::vector<double> pre_chips;
    // When the wander bound is well past the sync word's 16-chip step the
    // chip test cannot end the preamble; the SFD can, since its windows
    // demodulate louder as down-chirps. Everything up to there is preamble
    // plus the two sync symbols.
    const bool count_by_sfd = wander_ && step_tol_ >= 2.0 * syncChip0(params_);
    while (pre_start + (pre_chips.size() + 1) * sps_ <= resampled_.size()) {
        const Sample* w = resampled_.data() + pre_start + pre_chips.size() * sps_;
        const ChipPeak pk = sym_.demod(w);
        if (count_by_sfd) {
            // In-symbol wander smears the up-chirp peak several-fold, but
            // a down-dechirped up-chirp spreads over 2 BW and stays far lower.
            if (pk.magnitude < 0.08 * first_mag ||
                sym_.demod(w, true).magnitude > 0.7 * pk.magnitude) {
                break;
            }
            pre_chips.push_back(pk.chip);
            if (pre_chips.size() > static_cast<size_t>(params_.preamble_len) + 4) {
                break;
            }
            continue;
        }
        if (!pre_chips.empty() &&
            std::abs(wrapSigned(pk.chip - pre_chips.back(), n_chips_)) >
                std::max(6.0, step_tol_)) {
            // Check if this is the sync word or noise
            break;
        }
        pre_chips.push_back(pk.chip);
        if (pre_chips.size() > static_cast<size_t>(params_.preamble_len) + 2) {
            break;
        }
    }
    if (count_by_sfd && pre_chips.size() >= 6) {
        pre_chips.resize(pre_chips.size() - 2);
    }
    const size_t n_pre = pre_chips.size();
    fr.preamble_symbols = n_pre;
#ifdef RX_TRACE
    std::fprintf(stderr, "PRE start=%zu:", pre_start);
    for (size_t s = 0; s < n_pre + 4 && pre_start + (s + 1) * sps_ <= resampled_.size(); ++s) {
        const ChipPeak pk = sym_.demod(resampled_.data() + pre_start + s * sps_);
        std::fprintf(stderr, " %.1f/%.2g", pk.chip, pk.magnitude);
    }
    std::fprintf(stderr, "\n");
#endif
    if (n_pre < 4) {
#ifdef RX_TRACE
        std::fprintf(stderr, "SYNCFAIL n_pre=%zu first_mag=%.3g found=%zu\n", n_pre, first_mag, found_abs_);
#endif
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
    double u = pre_chips[n_pre - 1] + u_acc / static_cast<double>(n_u);
    if (wander_ && std::max(6.0, step_tol_) >= syncChip0(params_)) {
        // The widened step bound lets the sync word into pre_chips; the
        // median stays on the preamble.
        std::vector<double> rel(n_pre);
        for (size_t s = 0; s < n_pre; ++s) {
            rel[s] = wrapSigned(pre_chips[s] - pre_chips[0], n_chips_);
        }
        std::nth_element(rel.begin(), rel.begin() + n_pre / 2, rel.end());
        u = pre_chips[0] + rel[n_pre / 2];
    }

    // Carrier trajectory of one sync-span window with a known tone:
    // start / end frequency (folded chips) and end slope.
    struct KnownFit {
        double f0 = 0.0, f1 = 0.0, slope1 = 0.0, var1 = 0.0, mag = 0.0;
    };
    // aligned: the window sits on the symbol grid, so an up-chirp of value
    // `expect` folds at chip N - expect and a down-chirp not at all.
    auto fitKnown = [&](const Sample* w, bool down, bool aligned, uint16_t expect,
                        KnownFit& kf) -> bool {
        const ChipPeak pk = sym_.demod(w, down);
        const double c = std::round(pk.chip);
        int32_t b1 = static_cast<int32_t>(c) % static_cast<int32_t>(n_chips_);
        int32_t wrap = -1;
        if (aligned) {
            const double off = wrapSigned(c - (down ? 0.0 : expect), n_chips_);
            b1 = static_cast<int32_t>((down ? 0.0 : expect) + off);
            wrap = static_cast<int32_t>(down ? sps_ : (n_chips_ - expect) * os_);
        }
        if (!fitWindow(w, down, b1, wrap, pk)) {
            return false;
        }
        kf.f0 = b1 + wfit_.eval(0.0);
        kf.f1 = b1 + wfit_.eval(1.0);
        kf.slope1 = wfit_.meanSlope();
        kf.var1 = wfit_.var(1.0);
        kf.mag = pk.magnitude;
        return true;
    };
    // At SF11/12 the per-symbol wander exceeds the 16-chip spacing of the
    // sync word values, so absolute reads cannot tell preamble from sync
    // word. The carrier is continuous across symbol boundaries though, so
    // the tone jumps there (0 -> 16 -> 88) survive.
    const bool sync_by_jumps = wander_ && sigma_w_ > 2.0;
    const int dk_lo = wander_ ? -3 : -1;
    // Under wander the carrier at the sync word has moved from the preamble /
    // SFD average by up to ~1.5 kHz, which offsets e0 and e1 alike. Test the
    // sync word on their difference and allow that much common offset; the
    // boundary calibration below re-solves CFO and timing.
    const double common_tol =
        wander_ ? std::max(sync_tol_, 1500.0 * n_chips_ / params_.bandwidth_hz) : sync_tol_;
    auto syncPass = [&](double e0, double e1) {
        if (!wander_) {
            return std::abs(e0) < kSyncTolChips && std::abs(e1) < kSyncTolChips;
        }
        return std::abs(e1 - e0) < sync_tol_ && std::abs(0.5 * (e0 + e1)) < common_tol;
    };

    const size_t sfd_grid = pre_start + (n_pre + 2) * sps_;
    if (sfd_grid + 2 * sps_ > resampled_.size()) {
#ifdef RX_TRACE
        std::fprintf(stderr, "SYNCFAIL sfd_grid n_pre=%zu\n", n_pre);
#endif
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
    //
    // (u + d)/2 also only fixes CFO modulo N/2 chips. When the configured
    // CFO bound reaches past BW/4 the alias cfo -+ N/2 is physical too; it
    // pairs with the other half-symbol timing, which the tau loop covers.
    const double half_chips = static_cast<double>(n_chips_) / 2.0;
    const double max_cfo_chips =
        params_.max_cfo_hz * static_cast<double>(n_chips_) / params_.bandwidth_hz;
    double best_score = 0.0;
    std::vector<std::pair<double, double>> hyps;  // (cfo chips, tau samples)
    for (const double dw_chip : dw_cands) {
        const double cfo_main = wrapSigned(u + dw_chip, n_chips_) / 2.0;
        const double cfo_alias = (cfo_main > 0.0) ? cfo_main - half_chips : cfo_main + half_chips;
        const double tau0 = os_ * wrapSigned(u - dw_chip, n_chips_) / 2.0;
        const double tau_alt = (tau0 > 0.0) ? tau0 - half_sym : tau0 + half_sym;
        hyps.clear();
        hyps.push_back({cfo_main, tau0});
        hyps.push_back({cfo_main, tau_alt});
        if (std::abs(cfo_alias) <= max_cfo_chips) {
            hyps.push_back({cfo_alias, tau0});
            hyps.push_back({cfo_alias, tau_alt});
        }
        for (const auto& [cfo_c, tau_cand] : hyps) {
            const double a0 = static_cast<double>(pre_start) - tau_cand;
            for (int dk = dk_lo; dk <= 1; ++dk) {
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
                double score = s0.magnitude + s1.magnitude;
#ifdef RX_TRACE
                std::fprintf(stderr, "  cand cfo=%+8.3f tau=%+7.1f dk=%+d e0=%+8.3f e1=%+8.3f m=%.3g\n",
                             cfo_c, tau_cand, dk, e0, e1, score);
#endif
                if (sync_by_jumps && syncPass(e0, e1)) {
                    KnownFit kp, k0, k1;
                    if (qi < sps_ ||
                        !fitKnown(resampled_.data() + qi - sps_, false, false, 0, kp) ||
                        !fitKnown(resampled_.data() + qi, false, false, 0, k0) ||
                        !fitKnown(resampled_.data() + qi + sps_, false, false, 0, k1)) {
                        continue;
                    }
                    const double j0 = wrapSigned(k0.f0 - kp.f1 - syncChip0(params_), n_chips_);
                    const double j1 = wrapSigned(k1.f0 - k0.f1 - (syncChip1(params_) -
                                                                  syncChip0(params_)),
                                                 n_chips_);
                    const double jerr = std::abs(j0) + std::abs(j1);
#ifdef RX_TRACE
                    std::fprintf(stderr, "    jumps dk=%+d j0=%+.2f j1=%+.2f (kp %.1f..%.1f k0 %.1f..%.1f k1 %.1f..%.1f)\n", dk, j0, j1,
                                 kp.f0, kp.f1, k0.f0, k0.f1, k1.f0, k1.f1);
#endif
                    // Edge extrapolations of the window fits scatter by
                    // ~0.5 sigma_w each; a wrong dk misses by the 16 / 72
                    // chip sync steps themselves.
                    if (jerr > std::max(6.0, 2.0 * sigma_w_)) {
                        continue;
                    }
                    // Prefer the tightest jumps; magnitude only breaks ties.
                    score = score / (1.0 + jerr);
                }
                if (syncPass(e0, e1) && score > best_score) {
                    sync_found = true;
                    best_score = score;
                    k_sync = n_pre + dk;
                    cfo = wander_ ? cfo_c + 0.5 * (e0 + e1) : cfo_c;
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
    // shift_out (wander mode): the window was taken at the nearest whole
    // sample instead, this many samples late.
    auto calWindow = [&](double sym_idx, double* shift_out = nullptr) -> bool {
        const double q = aligned0 + sym_idx * sps_;
        double raw_f =
            static_cast<double>(origin_abs_) + q * total_ratio_ - static_cast<double>(base_);
        if (shift_out != nullptr) {
            const double r = std::round(raw_f);
            *shift_out = r - raw_f;
            raw_f = r;
        }
        if (raw_f < 1.0) {
            return false;
        }
        const double raw_end = raw_f + static_cast<double>(sps_) * total_ratio_ + 3.0;
        if (raw_end >= static_cast<double>(buf_.size())) {
            return false;
        }
        const size_t n =
            resampleCatmullRom(buf_.data(), buf_.size(), raw_f, total_ratio_, win_.data(), sps_);
        return n >= sps_;
    };
    auto calDemod = [&](double sym_idx, bool down, double expect, double& err) -> bool {
        if (!calWindow(sym_idx)) {
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
        if (calDemod(sym_idx, false, expect, err) && std::abs(err) <= sync_tol_) {
            up_acc += err;
            ++up_cnt;
        }
    }
    for (int j = 0; j < 2; ++j) {
        // SFD down-chirps span 2.25 symbols after the sync word
        double err;
        if (calDemod(static_cast<double>(k_sync) + 2.0 + j, true, 0.0, err) &&
            std::abs(err) <= sync_tol_) {
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
    trk_slope_ = 0.0;
    trk_var_ = sigma_w_ * sigma_w_;
    if (wander_ && sync_found) {
        // The averages above mix reads taken up to 4 symbols apart. Instead
        // take the carrier at the sync-word / SFD boundary, where the up
        // window (cfo + t) and down window (cfo - t) see the same carrier,
        // then follow it through both down-chirps to the data start.
        // Whole-sample windows: the Catmull-Rom kernel at a fractional delay
        // has a phase error that grows toward the band edge, which the
        // dechirped tone turns into a phase step at the chirp wrap and slow
        // in-symbol frequency trends. A late window reads up-chirps high and
        // down-chirps low by shift/os chips; take that out instead.
        KnownFit su, d0, d1;
        double sh_u = 0.0, sh_d0 = 0.0, sh_d1 = 0.0;
        const double k = static_cast<double>(k_sync);
        if (calWindow(k + 1.0, &sh_u) &&
            fitKnown(win_.data(), false, true, syncChip1(params_), su) &&
            calWindow(k + 2.0, &sh_d0) && fitKnown(win_.data(), true, true, 0, d0) &&
            calWindow(k + 3.0, &sh_d1) && fitKnown(win_.data(), true, true, 0, d1)) {
            const double os = static_cast<double>(os_);
            su.f0 -= sh_u / os;
            su.f1 -= sh_u / os;
            d0.f0 += sh_d0 / os;
            d0.f1 += sh_d0 / os;
            d1.f0 += sh_d1 / os;
            d1.f1 += sh_d1 / os;
            const double m_up = wrapSigned(su.f1 - syncChip1(params_) - cfo, n_chips_);
            const double m_dn = wrapSigned(d0.f0 - cfo, n_chips_);
            if (std::abs(m_up) <= common_tol && std::abs(m_dn) <= common_tol) {
                timing_chips = (m_up - m_dn) / 2.0;
                const double drift = wrapSigned(d1.f1 - d0.f0, n_chips_);
                // the 0.25 down-chirp before the data is not observed
                ref = cfo + (m_up + m_dn) / 2.0 + drift + 0.25 * d1.slope1;
                trk_slope_ = trk_beta_ * d1.slope1;
                trk_var_ = 0.25 * su.var1 + d1.var1 + 0.0625 * sigma_w_ * sigma_w_;
            }
        }
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
    if (wander_) {
        // Ratio is 1 and the step a whole symbol, so rounding once keeps every
        // data window on whole samples (no Catmull-Rom phase error, see the
        // calibration above); the sub-sample lateness becomes a carrier offset.
        const double r = std::round(data_raw_f_);
        ref_ += (r - data_raw_f_) / static_cast<double>(os_);
        data_raw_f_ = r;
    }
    fr.start_sample = origin_abs_ +
                      static_cast<size_t>(std::max(0.0, aligned0 * total_ratio_));

    cur_ = std::move(fr);
    data_sym_ = 0;
    timing_int_ = 0.0;
    prev_obs_.clear();
    slip_syms_.clear();
    slip_obs_.clear();
    sym_fracs_.clear();
    alt_syms_.clear();
    soft_mags_.clear();
    needed_syms_ = 8;
    have_len_ = false;
    fold_acc_ = {0.0, 0.0};
    fold_mag_ = 0.0;
    snr_acc_ = 0.0;
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
        if (wander_) {
            trackSymbol(step);
            if (!checkHeader()) {
                break;
            }
            continue;
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

        const auto ref_phys =
            static_cast<int32_t>(std::lround(wrapSigned(ref_int, static_cast<double>(n_chips_))));
        // Coherent fold once the rotation estimate is consistent; otherwise
        // the power fold. Timing (frac, refine) always reads the power fold,
        // so the loop behaves the same either way.
        const double fold_abs = std::abs(fold_acc_);
        const bool coherent =
            params_.coherent_fold && fold_mag_ > 0.0 && fold_abs > 0.6 * fold_mag_;
        const Sample rot = (fold_abs > 0.0)
                               ? Sample(static_cast<float>(fold_acc_.real() / fold_abs),
                                        static_cast<float>(fold_acc_.imag() / fold_abs))
                               : Sample(1.0f, 0.0f);
        ChipPeak pk_co;
        ChipPeak pk;
        sym_.demodData(win_.data(), ref_phys, rot, pk, pk_co);
        double rel = wrapSigned(pk.chip - ref_int, n_chips_);
        double rel_pos = (rel < 0.0) ? rel + n_chips_ : rel;
        double frac = chipFrac(rel_pos);

        // Same-symbol sub-chip refine (see refineSymbolWindow). Fold any
        // accepted shift into data_raw_f_ so the rest of the frame stays
        // centered on the corrected grid.
        {
            double start = start_rel;
            bool tried = false;
            const bool refine_ok = channelSnrDb(pk, n_chips_, params_.spreading_factor) >=
                                   params_.refine_min_snr_db - sf_gain_db_;
            if (refine_ok && refineSymbolWindow(pk, rel_pos, frac, start, ref_int, n_chips_, os_, total_ratio_,
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
                                       tried = true;
                                       derotate();
                                       out = sym_.demod(win_.data());
                                       return true;
                                   })) {
                data_raw_f_ += start - start_rel;
                // A shift of d raw samples moves the window d / ratio samples
                // and turns the inter-segment phase by 2 pi d / (ratio * os);
                // restart the estimate rather than track the jump.
                fold_acc_ = {0.0, 0.0};
                fold_mag_ = 0.0;
            }
            if (tried) {
                // win_ holds the last trial; redo the chosen window.
                resampleCatmullRom(buf_.data(), buf_.size(), start, total_ratio_, win_.data(),
                                   sps_);
                derotate();
                ChipPeak nc_unused;
                sym_.demodData(win_.data(), ref_phys, rot, nc_unused, pk_co);
            }
        }

        // Decision-directed rotation update at the decided peak, ~6-symbol
        // memory so the timing loop's slow sub-sample walk is tracked.
        {
            const ChipPeak& d = coherent ? pk_co : pk;
            const std::complex<double> s1(d.seg1);
            const std::complex<double> s2(d.seg2);
            fold_acc_ = 0.85 * fold_acc_ + s1 * std::conj(s2);
            fold_mag_ = 0.85 * fold_mag_ + std::abs(s1) * std::abs(s2);
        }
        ++data_syms_total_;
        coherent_syms_ += coherent;
        if (coherent) {
            // Value and metrics from the coherent fold. `frac` (timing loop)
            // stays on the power fold.
            const double rel_co = wrapSigned(pk_co.chip - ref_int, n_chips_);
            pk = pk_co;
            rel_pos = (rel_co < 0.0) ? rel_co + n_chips_ : rel_co;
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
        {
            // Value v sits at chip bin v + ref (mod N), the same map the hard
            // value went through.
            const std::vector<double>& f = coherent ? sym_.foldedCoherent() : sym_.foldedPower();
            const size_t base = soft_mags_.size();
            soft_mags_.resize(base + n_chips_);
            const int32_t n = static_cast<int32_t>(n_chips_);
            for (int32_t v = 0; v < n; ++v) {
                const int32_t j = ((v + ref_phys) % n + n) % n;
                soft_mags_[base + static_cast<size_t>(v)] =
                    static_cast<float>(std::sqrt(f[static_cast<size_t>(j)]));
            }
        }
        if (pk.total_power > 0.0) {
            snr_acc_ += channelSnrDb(pk, n_chips_, params_.spreading_factor);
            pwr_acc_ += pk.total_power;
            ++metric_n_;
        }

        // Decision-directed PI timing loop. Residual rate error shows up as
        // a fractional chip offset; P absorbs leftover sub-chip error after
        // the same-symbol refine, I learns the preamble rate residual
        // (~+/-15 ppm). A late window reads high, so positive frac steers
        // the next start earlier.
        timing_int_ += params_.timing_ki * frac;
        const double adj =
            -chipsToRawSamples(params_.timing_kp * frac + timing_int_, os_, total_ratio_);
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

        if (!checkHeader()) {
            break;  // bad header; stop early
        }
    }

    finalizeData();
    return true;
}

void Receiver::trackSymbol(double step) {
    const auto n = static_cast<int32_t>(n_chips_);
    const auto sps = static_cast<int32_t>(sps_);
    const size_t idx = cur_.raw_values.size();
    const bool reduced = idx < 8 || params_.ldro;

    // Remove the predicted carrier (ref_ at the symbol start, ramping by
    // trk_slope_ chips across it) and read the value against its integer part.
    const double ref_int = std::round(ref_);
    derotateRamp(win_.data(), ref_ - ref_int, trk_slope_);
    const auto ref_phys =
        static_cast<int32_t>(std::lround(wrapSigned(ref_int, static_cast<double>(n_chips_))));

    const double fold_abs = std::abs(fold_acc_);
    const bool coherent =
        params_.coherent_fold && fold_mag_ > 0.0 && fold_abs > 0.6 * fold_mag_;
    const Sample rot = (fold_abs > 0.0)
                           ? Sample(static_cast<float>(fold_acc_.real() / fold_abs),
                                    static_cast<float>(fold_acc_.imag() / fold_abs))
                           : Sample(1.0f, 0.0f);
    ChipPeak nc, co;
    sym_.demodData(win_.data(), ref_phys, rot, nc, co);
    const int32_t v0 = (((coherent ? co : nc).chip_int - ref_phys) % n + n) % n;
    const int32_t b1 = ((v0 + ref_phys) % sps + sps) % sps;

    // The FFT peak sits near the symbol's mean carrier, which can be chips
    // away from where the symbol started. The fit reads the residual
    // eps(t) = e(t) - d against the provisional peak, where e is the true
    // carrier error vs the prediction and d the decision error. Two priors
    // pick d: e(0) ~ 0 (carrier is continuous from the previous symbol) and
    // a small mean error. Reduced-rate symbols only take raw values 4q + 1.
    int32_t d_hat = 0;
    double r0 = 0.0, e1 = 0.0, slope1 = 0.0, var1 = 1e9;
    const auto obs0 = static_cast<uint32_t>(slip_obs_.size());
    if (idx == 0) {
        kf_on_ = kf_q_ > 0.0;
        kf_x_[0] = ref_;
        kf_ref0_ = ref_;
        kf_x_[1] = trk_slope_;
        kf_p_[0] = trk_var_;
        kf_p_[1] = 0.0;
        kf_p_[2] = sigma_w_ * sigma_w_ + 1.0;
        kf_t_ = 0.0;
    }
    const double ref_pred = ref_;
    const double slope_pred = trk_slope_;
    // value v folds at chip N - v
    const auto wrap = static_cast<int32_t>((n_chips_ - static_cast<uint32_t>(v0)) * os_);
    const bool fitted = fitWindow(win_.data(), false, b1, wrap, coherent ? co : nc);
    if (fitted) {
        const double e0 = wfit_.eval(0.0);
        const double cm = wfit_.mean();
        // Continuity term: -e0 is where this symbol's start says d sits.
        // Better, when the previous symbol's observations are kept: fit one
        // smooth carrier across both symbols with d as a free offset on this
        // one. The boundary is then inside the fit instead of at the edge of
        // two separate ones, which is what keeps cycle slips rare.
        double d_cont = -e0;
        double vc = trk_var_ + wfit_.var(0.0) + 1e-3;
        if (!prev_obs_.empty()) {
            // The measured wander has ~70 Hz RMS at 60-150 Hz, which no
            // low-order carrier follows across two 8+ ms symbols; fitted
            // that way, the step reads 0.3+ chip biased at SF11. Near the
            // boundary the carrier is close to a quadratic, so weight the
            // observations by a Gaussian taper of joint_w_ms_ around it.
            constexpr int kMaxJoint = 8;
            const bool local = joint_frac_ < 1.0;
            const int pj = local ? 2 : std::min(wfit_.order() + 1, kMaxJoint - 2);
            const int np = pj + 2;
            const size_t n_cur = wfit_.observations();
            const size_t rows = prev_obs_.size() + n_cur;
            joint_a_.assign(rows * static_cast<size_t>(np), 0.0);
            joint_y_.resize(rows);
            joint_w_.resize(rows);
            double dp[kMaxJoint];
            auto basis = [&](double t, double* row) {
                if (local) {
                    const double u = t / joint_frac_;
                    row[0] = 1.0;
                    row[1] = u;
                    row[2] = u * u;
                    return std::exp(-u * u);
                }
                legendreAt(t, pj, row, dp);
                return 1.0;
            };
            size_t r = 0;
            for (const auto& o : prev_obs_) {
                const double taper = basis(o[0] - 1.0, joint_a_.data() + r * np);
                joint_y_[r] = o[1];
                joint_w_[r] = o[2] * taper;
                ++r;
            }
            for (size_t k = 0; k < n_cur; ++k, ++r) {
                const double t = wfit_.obsT(k);
                const double taper = basis(t, joint_a_.data() + r * np);
                joint_a_[r * np + static_cast<size_t>(np - 1)] = -1.0;
                joint_y_[r] = ref_pred + slope_pred * t + wfit_.obsVal(k);
                joint_w_[r] = wfit_.obsW(k) * taper;
            }
            double prior[kMaxJoint] = {};
            const double span = local ? joint_frac_ : 1.0;
            for (int i = 1; i <= pj; ++i) {
                const double s = std::max(0.05, 2.0 * sigma_w_ * span / (1.0 + i));
                prior[i] = 1.0 / (s * s);
            }
            double x[kMaxJoint], cov[kMaxJoint * kMaxJoint];
            if (ridgeSolve(joint_a_.data(), joint_y_.data(), joint_w_.data(), rows, np, prior, x,
                           cov)) {
                d_cont = x[np - 1];
                vc = cov[np * np - 1] + 1e-4;
            }
        }
        // Carrier Kalman (integrated random walk, see slipSearch) with this
        // symbol's decision offset as a third state: the continuity estimate
        // then uses every earlier observation with the right weights instead
        // of a fixed window around the boundary.
        double kx[3] = {kf_x_[0], kf_x_[1], 0.0};
        double kp[3][3] = {{kf_p_[0], kf_p_[1], 0.0}, {kf_p_[1], kf_p_[2], 0.0}, {0.0, 0.0, 1e6}};
        double kt = kf_t_;
        if (kf_on_) {
            for (size_t k = 0; k < wfit_.fineObservations(); ++k) {
                const double t = static_cast<double>(idx) + wfit_.fineT(k);
                const double dt = t - kt;
                kt = t;
                kx[0] += kx[1] * dt;
                const double p00 = kp[0][0] + 2.0 * dt * kp[0][1] + dt * dt * kp[1][1] +
                                   kf_q_ * dt * dt * dt / 3.0;
                const double p01 = kp[0][1] + dt * kp[1][1] + kf_q_ * dt * dt / 2.0;
                const double p02 = kp[0][2] + dt * kp[1][2];
                kp[0][0] = p00;
                kp[0][1] = kp[1][0] = p01;
                kp[0][2] = kp[2][0] = p02;
                kp[1][1] += kf_q_ * dt;
                // z = f - d + noise
                const double z = ref_pred + slope_pred * wfit_.fineT(k) + wfit_.fineVal(k);
                const double ph[3] = {kp[0][0] - kp[0][2], kp[1][0] - kp[1][2], kp[2][0] - kp[2][2]};
                const double sv = ph[0] - ph[2] + 1.0 / std::max(wfit_.fineW(k), 1e-9) + 0.03 * 0.03;
                const double nu = z - (kx[0] - kx[2]);
                // Short-block phase steps wrap or land on the wrong fold
                // segment now and then; one such outlier taken at full weight
                // walks the carrier off by tens of kHz.
                if (nu * nu > 16.0 * sv && k > 0) {
                    continue;
                }
                for (int i = 0; i < 3; ++i) {
                    kx[i] += ph[i] / sv * nu;
                }
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        kp[i][j] -= ph[i] * ph[j] / sv;
                    }
                }
            }
            d_cont = kx[2];
            vc = kp[2][2] + 1e-4;
        }
        const double vm = kf_on_ ? 1e12 : trk_var_ + 0.25 * sigma_w_ * sigma_w_ + 0.01;
        // Search around the cost minimum, not around 0: OTA SF11/125k
        // symbols see 2 kHz (35 chip) carrier swings in one symbol.
        const double v_post = 1.0 / (1.0 / vc + 1.0 / vm);
        const double d_star = v_post * (d_cont / vc - cm / vm);
        const int32_t span =
            std::min<int32_t>(n / 8, static_cast<int32_t>(std::ceil(3.0 * std::sqrt(v_post))) + 4);
        const auto d_mid = static_cast<int32_t>(std::lround(std::clamp(d_star, -n / 4.0, n / 4.0)));
        double best = std::numeric_limits<double>::infinity();
        for (int32_t d = d_mid - span; d <= d_mid + span; ++d) {
            if (reduced && (((v0 - d) % 4) + 4) % 4 != 1) {
                continue;
            }
            const double a = d - d_cont;
            const double b = cm + d;
            const double cost = a * a / vc + b * b / vm;
            if (cost < best) {
                best = cost;
                d_hat = d;
            }
        }
        r0 = d_hat - d_cont;
        if (kf_on_) {
            smoothSymbol(idx, ref_pred, slope_pred, static_cast<double>(d_hat));
        }
#ifdef RX_TRACE
        std::fprintf(stderr, "    d_cont=%+7.3f sd=%.3f cm=%+7.3f sdm=%.2f nobs=%zu prev=%zu obs:", d_cont,
                     std::sqrt(vc), cm, std::sqrt(vm), wfit_.observations(), prev_obs_.size());
        for (size_t k = 0; k < wfit_.observations(); ++k) {
            std::fprintf(stderr, " %.3f:%+.3f", wfit_.obsT(k), wfit_.obsVal(k));
        }
        std::fprintf(stderr, "\n");
#endif
        e1 = wfit_.eval(1.0) + d_hat;
        slope1 = wfit_.meanSlope();
        var1 = wfit_.var(1.0);
        prev_obs_.resize(wfit_.observations());
        for (size_t k = 0; k < prev_obs_.size(); ++k) {
            const double t = wfit_.obsT(k);
            prev_obs_[k] = {t, ref_pred + slope_pred * t + wfit_.obsVal(k) + d_hat, wfit_.obsW(k)};
        }
        for (const auto& o : prev_obs_) {
            slip_obs_.push_back({static_cast<double>(idx) + o[0], o[1], o[2]});
        }
        // Take the whole trajectory out so the re-FFT puts one sharp tone at
        // the decided bin for the soft rows and metrics.
        if (kf_on_ && trk_t_.size() >= 2) {
            derotateTrack(win_.data());
        } else {
            wfit_.derotate(win_.data(), static_cast<double>(d_hat));
        }
        sym_.demodData(win_.data(), ref_phys, rot, nc, co);
    } else {
        prev_obs_.clear();
    }
    slip_syms_.push_back({obs0, static_cast<uint32_t>(slip_obs_.size()), reduced});
    const ChipPeak& pk = coherent ? co : nc;
    const auto value = static_cast<uint16_t>(((v0 - d_hat) % n + n) % n);

    {
        const std::complex<double> s1(pk.seg1);
        const std::complex<double> s2(pk.seg2);
        fold_acc_ = 0.85 * fold_acc_ + s1 * std::conj(s2);
        fold_mag_ = 0.85 * fold_mag_ + std::abs(s1) * std::abs(s2);
    }
    ++data_syms_total_;
    coherent_syms_ += coherent;
    sym_fracs_.push_back(std::clamp(r0, -0.5, 0.5));

    if (pk.mag2 > 0.30 * pk.magnitude) {
        const auto alt =
            static_cast<uint16_t>(((static_cast<int32_t>(pk.chip2_int) - ref_phys) % n + n) % n);
        if (alt != value) {
            alt_syms_.push_back({idx, alt, pk.mag2 / pk.magnitude});
        }
    }
    cur_.raw_values.push_back(value);
    {
        const std::vector<double>& f = coherent ? sym_.foldedCoherent() : sym_.foldedPower();
        const size_t base = soft_mags_.size();
        soft_mags_.resize(base + n_chips_);
        for (int32_t v = 0; v < n; ++v) {
            const int32_t j = ((v + ref_phys) % n + n) % n;
            soft_mags_[base + static_cast<size_t>(v)] =
                static_cast<float>(std::sqrt(f[static_cast<size_t>(j)]));
        }
    }
    if (pk.total_power > 0.0) {
        snr_acc_ += channelSnrDb(pk, n_chips_, params_.spreading_factor);
        pwr_acc_ += pk.total_power;
        ++metric_n_;
    }

    // Carrier at the next symbol start: prediction plus the measured end
    // error, weighted by how well this symbol pinned it down.
    double g = 1.0;
    if (kf_on_) {
        // Inter-unit carrier spread is ~+-2 kHz around the sync estimate.
        const double band = 3000.0 * static_cast<double>(n_chips_) / params_.bandwidth_hz;
        const double slope_lim = 4.0 * sigma_w_ + 1.0;
        kf_x_[0] = std::clamp(kf_x_[0], kf_ref0_ - band, kf_ref0_ + band);
        kf_x_[1] = std::clamp(kf_x_[1], -slope_lim, slope_lim);
        const double dt = static_cast<double>(idx + 1) - kf_t_;
        ref_ = kf_x_[0] + kf_x_[1] * dt;
        trk_slope_ = kf_x_[1];
        trk_var_ = kf_p_[0] + 2.0 * dt * kf_p_[1] + dt * dt * kf_p_[2] + kf_q_ * dt * dt * dt / 3.0;
    } else {
        const double p_prior = trk_var_ + sigma_w_ * sigma_w_;
        g = fitted ? p_prior / (p_prior + var1) : 0.0;
        ref_ += trk_slope_ + g * e1;
        trk_var_ = (1.0 - g) * p_prior;
        const double slope_lim = 4.0 * sigma_w_ + 1.0;
        trk_slope_ = std::clamp(trk_beta_ * (trk_slope_ + g * slope1), -slope_lim, slope_lim);
    }
#ifdef RX_TRACE
    std::fprintf(stderr, "  sym %2zu v0=%4d d=%+3d val=%4u r0=%+6.2f e1=%+7.2f g=%.2f ref=%9.3f slope=%+6.2f\n",
                 data_sym_, v0, d_hat, value, r0, e1, g, ref_, trk_slope_);
#endif

    data_raw_f_ += step;
    ++data_sym_;
    const double keep = data_raw_f_ - 8.0;
    if (keep > 0.0) {
        trimFront(static_cast<size_t>(keep));
    }
}

// Re-run the carrier filter over this symbol's short-block observations with
// the decided offset, then a Rauch-Tung-Striebel pass back over them. The
// smoothed carrier, less the prediction the window was derotated with, is
// what derotateTrack takes out; the filtered end state carries on.
void Receiver::smoothSymbol(size_t idx, double ref_pred, double slope_pred, double d) {
    const size_t m = wfit_.fineObservations();
    trk_t_.clear();
    trk_e_.clear();
    sm_x_.resize(m);
    sm_p_.resize(m);
    sm_xp_.resize(m);
    sm_pp_.resize(m);
    sm_dt_.resize(m);
    double x0 = kf_x_[0], x1 = kf_x_[1];
    double p00 = kf_p_[0], p01 = kf_p_[1], p11 = kf_p_[2];
    double t_last = kf_t_;
    size_t used = 0;
    for (size_t k = 0; k < m; ++k) {
        const double t = static_cast<double>(idx) + wfit_.fineT(k);
        const double dt = t - t_last;
        const double q = kf_q_;
        const double a0 = x0 + x1 * dt, a1 = x1;
        const double a00 = p00 + 2.0 * dt * p01 + dt * dt * p11 + q * dt * dt * dt / 3.0;
        const double a01 = p01 + dt * p11 + q * dt * dt / 2.0;
        const double a11 = p11 + q * dt;
        const double z = ref_pred + slope_pred * wfit_.fineT(k) + wfit_.fineVal(k) + d;
        const double sv = a00 + 1.0 / std::max(wfit_.fineW(k), 1e-9) + 0.03 * 0.03;
        const double nu = z - a0;
        if (nu * nu > 16.0 * sv && k > 0) {
            continue;
        }
        const double k0 = a00 / sv, k1 = a01 / sv;
        x0 = a0 + k0 * nu;
        x1 = a1 + k1 * nu;
        p00 = a00 - k0 * a00;
        p01 = a01 - k0 * a01;
        p11 = a11 - k1 * a01;
        sm_xp_[used] = {a0, a1};
        sm_pp_[used] = {a00, a01, a11};
        sm_x_[used] = {x0, x1};
        sm_p_[used] = {p00, p01, p11};
        sm_dt_[used] = t;
        t_last = t;
        ++used;
    }
    kf_x_[0] = x0;
    kf_x_[1] = x1;
    kf_p_[0] = p00;
    kf_p_[1] = p01;
    kf_p_[2] = p11;
    kf_t_ = t_last;
    if (used < 2) {
        return;
    }
    // RTS: C = P_k F' inv(P_{k+1|k}), 2x2.
    std::array<double, 2> xs = sm_x_[used - 1];
    trk_t_.resize(used);
    trk_e_.resize(used);
    auto store = [&](size_t k, const std::array<double, 2>& x) {
        const double ts = sm_dt_[k] - static_cast<double>(idx);
        trk_t_[k] = ts;
        trk_e_[k] = x[0] - (ref_pred + slope_pred * ts);
    };
    store(used - 1, xs);
    for (size_t k = used - 1; k-- > 0;) {
        const double dt = sm_dt_[k + 1] - sm_dt_[k];
        const auto& p = sm_p_[k];
        const auto& pp = sm_pp_[k + 1];
        // P_k F' with F = [1 dt; 0 1]
        const double b00 = p[0] + dt * p[1], b01 = p[1];
        const double b10 = p[1] + dt * p[2], b11 = p[2];
        const double det = pp[0] * pp[2] - pp[1] * pp[1];
        if (std::abs(det) < 1e-18) {
            store(k, sm_x_[k]);
            xs = sm_x_[k];
            continue;
        }
        const double i00 = pp[2] / det, i01 = -pp[1] / det, i11 = pp[0] / det;
        const double c00 = b00 * i00 + b01 * i01, c01 = b00 * i01 + b01 * i11;
        const double c10 = b10 * i00 + b11 * i01, c11 = b10 * i01 + b11 * i11;
        const double r0 = xs[0] - sm_xp_[k + 1][0], r1 = xs[1] - sm_xp_[k + 1][1];
        xs = {sm_x_[k][0] + c00 * r0 + c01 * r1, sm_x_[k][1] + c10 * r0 + c11 * r1};
        store(k, xs);
    }
}

void Receiver::derotateTrack(Sample* window) const {
    // trk_e_: carrier minus the window's derotation ramp, chips, at trk_t_
    // (symbol fractions); linear in between, held flat past the ends.
    const double inv = 1.0 / static_cast<double>(sps_);
    double phase = 0.0;
    size_t j = 0;
    const size_t m = trk_t_.size();
    for (uint32_t n = 0; n < sps_; ++n) {
        const double t = (n + 0.5) * inv;
        while (j + 1 < m && trk_t_[j + 1] < t) {
            ++j;
        }
        double e;
        if (t <= trk_t_[0]) {
            e = trk_e_[0];
        } else if (j + 1 >= m) {
            e = trk_e_[m - 1];
        } else {
            const double a = (t - trk_t_[j]) / (trk_t_[j + 1] - trk_t_[j]);
            e = trk_e_[j] + a * (trk_e_[j + 1] - trk_e_[j]);
        }
        const double c = std::cos(phase), s = std::sin(phase);
        window[n] *= Sample(static_cast<float>(c), static_cast<float>(-s));
        phase += 2.0 * M_PI * e * inv;
    }
}

bool Receiver::checkHeader() {
    if (have_len_ || cur_.raw_values.size() < 8) {
        return true;
    }
    DecodeResult head = decodeHard(cur_.raw_values);
    if (!head.header.valid && params_.soft_decoding) {
        DecodeResult hs = decodeSoft(softRows(8));
        if (hs.header.valid) {
            head = std::move(hs);
            ++soft_headers_;
        }
    }
    if (!head.header.valid) {
        // A stream step between the sync calibration and the data
        // start (nothing observes that span) offsets the whole grid
        // by a chip; the header checksum resolves the hypothesis. Under
        // wander the tracker snaps header symbols onto the 4q+1 lattice,
        // so a start error past +-2 chips shows up as whole lattice steps.
        // The carrier carried across the 2.25-symbol SFD is off by up to
        // ~1.5 sigma_w (OTA SF11/125k: 8-16 chips). Smallest shifts first,
        // since each extra try is another ~1/32 chance of a false header.
        std::vector<int> shifts{+1, -1};
        if (wander_) {
            const int k_max = std::clamp(static_cast<int>(std::ceil(1.5 * sigma_w_ / 4.0)), 1, 8);
            for (int k = 1; k <= k_max; ++k) {
                shifts.push_back(4 * k);
                shifts.push_back(-4 * k);
            }
        }
        for (const int sh : shifts) {
            const auto shifted = shiftValues(cur_.raw_values, sh, n_chips_);
            DecodeResult h2 = decodeHard(shifted);
            if (h2.header.valid) {
                cur_.raw_values = shifted;
                for (size_t r = 0; r < cur_.raw_values.size() &&
                                   (r + 1) * n_chips_ <= soft_mags_.size();
                     ++r) {
                    rotateSoftRow(soft_mags_.data() + r * n_chips_, n_chips_, sh);
                }
                ref_ -= sh;  // future symbols read consistently
                for (auto& o : prev_obs_) {
                    o[1] -= sh;
                }
                for (auto& o : slip_obs_) {
                    o[1] -= sh;
                }
                kf_x_[0] -= sh;
                kf_ref0_ -= sh;
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
            DecodeResult h2 = decodeHard(cand);
            if (h2.header.valid) {
                cur_.raw_values = std::move(cand);
                head = std::move(h2);
                break;
            }
        }
    }
    if (!head.header.valid) {
        return false;
    }
    LoraParams pl = params_;
    pl.cr = head.header.cr;
    pl.has_crc = head.header.has_crc;
    needed_syms_ = frameSymbolCount(pl, head.header.payload_len);
    have_len_ = true;
    return true;
}

void Receiver::slipSearch(std::vector<std::vector<int>>& paths) const {
    paths.clear();
    const size_t n_sym = slip_syms_.size();
    if (n_sym == 0 || params_.wander_irw_q <= 0.0) {
        return;
    }
    // Carrier model in chips and symbol time: f'' is white with intensity
    // q T^5 (Hz^2/s^3 -> chips^2/symbol^3). The covariance recursion does
    // not depend on the data, so it runs once; only means and metrics are
    // per path. Each path carries a cumulative slip o: symbol k's carrier
    // reads y + o_k and its value v - o_k.
    const double t_sym = static_cast<double>(n_chips_) / params_.bandwidth_hz;
    const double q = params_.wander_irw_q * std::pow(t_sym, 5.0);
    constexpr double kObsFloor = 0.03 * 0.03;  // point vs block-average model error, chips^2
    constexpr double kSlipCost = 6.0;          // chi^2 units per slip
    constexpr size_t kPaths = 48;

    struct Path {
        double f, fd, metric;
        int o;
        uint32_t parent;
    };
    std::vector<Path> cur{{0.0, 0.0, 0.0, 0, 0}}, next;
    std::vector<std::vector<std::pair<uint32_t, int>>> trace(n_sym);
    double p00 = 1e6, p01 = 0.0, p11 = std::pow(4.0 * sigma_w_ + 1.0, 2.0);
    double t_last = 0.0;
    std::vector<double> gains;  // per obs: dt, k0, k1, s
    for (size_t k = 0; k < n_sym; ++k) {
        const SlipSym& sym = slip_syms_[k];
        gains.clear();
        for (uint32_t j = sym.obs0; j < sym.obs1; ++j) {
            const double dt = slip_obs_[j][0] - t_last;
            t_last = slip_obs_[j][0];
            const double a00 = p00 + 2.0 * dt * p01 + dt * dt * p11 + q * dt * dt * dt / 3.0;
            const double a01 = p01 + dt * p11 + q * dt * dt / 2.0;
            const double a11 = p11 + q * dt;
            const double sv = a00 + 1.0 / std::max(slip_obs_[j][2], 1e-9) + kObsFloor;
            const double k0 = a00 / sv, k1 = a01 / sv;
            p00 = a00 - k0 * a00;
            p01 = a01 - k0 * a01;
            p11 = a11 - k1 * a01;
            gains.insert(gains.end(), {dt, k0, k1, sv});
        }
        const int step = sym.reduced ? 4 : 1;
        next.clear();
        for (uint32_t pi = 0; pi < cur.size(); ++pi) {
            for (const int s : {0, -step, step}) {
                Path p = cur[pi];
                p.o += s;
                p.metric += s != 0 ? kSlipCost : 0.0;
                p.parent = pi;
                for (uint32_t j = sym.obs0, g = 0; j < sym.obs1; ++j, g += 4) {
                    p.f += p.fd * gains[g];
                    const double nu = slip_obs_[j][1] + p.o - p.f;
                    p.metric += nu * nu / gains[g + 3];
                    p.f += gains[g + 1] * nu;
                    p.fd += gains[g + 2] * nu;
                }
                next.push_back(p);
            }
        }
        const size_t keep = std::min(kPaths, next.size());
        std::partial_sort(next.begin(), next.begin() + static_cast<std::ptrdiff_t>(keep), next.end(),
                          [](const Path& a, const Path& b) { return a.metric < b.metric; });
        next.resize(keep);
        trace[k].resize(keep);
        for (size_t i = 0; i < keep; ++i) {
            trace[k][i] = {next[i].parent, next[i].o};
        }
        cur.swap(next);
    }
    for (uint32_t i = 0; i < cur.size(); ++i) {
        std::vector<int> offs(n_sym);
        uint32_t at = i;
        for (size_t k = n_sym; k-- > 0;) {
            offs[k] = trace[k][at].second;
            at = trace[k][at].first;
        }
        paths.push_back(std::move(offs));
    }
}

void Receiver::finalizeData() {
    if (wander_ && !slip_syms_.empty()) {
        const DecodeResult d0 = decodeHard(cur_.raw_values);
        if (d0.header.valid && d0.header.has_crc && !d0.crc_ok) {
            std::vector<std::vector<int>> paths;
            slipSearch(paths);
            const size_t n_rows = std::min({cur_.raw_values.size(), slip_syms_.size(),
                                            soft_mags_.size() / n_chips_});
            auto applied = [&](const std::vector<int>& offs) {
                auto cand = cur_.raw_values;
                for (size_t s = 0; s < n_rows; ++s) {
                    cand[s] = static_cast<uint16_t>(
                        ((static_cast<int>(cand[s]) - offs[s]) % static_cast<int>(n_chips_) +
                         static_cast<int>(n_chips_)) %
                        static_cast<int>(n_chips_));
                }
                return cand;
            };
#ifdef RX_TRACE
            if (!paths.empty()) {
                std::fprintf(stderr, "SLIP best:");
                for (const int o : paths[0]) {
                    std::fprintf(stderr, " %d", o);
                }
                std::fprintf(stderr, "\n");
            }
#endif
            const std::vector<int>* pick = paths.empty() ? nullptr : &paths[0];
            for (const auto& offs : paths) {
                const DecodeResult d2 = decodeHard(applied(offs));
                if (d2.header.valid && d2.crc_ok) {
                    pick = &offs;
                    break;
                }
            }
            if (pick) {
                cur_.raw_values = applied(*pick);
                for (size_t s = 0; s < n_rows; ++s) {
                    if ((*pick)[s] != 0) {
                        rotateSoftRow(soft_mags_.data() + s * n_chips_, n_chips_, -(*pick)[s]);
                    }
                }
            }
        }
    }
    const std::vector<uint16_t> hard_in = cur_.raw_values;
    cur_.decode = decodeHard(cur_.raw_values);
    // Chip-lattice offsets can slip past the header (the reduced-rate
    // header drops the two LSBs, so +/-1 chip usually leaves it valid);
    // the payload CRC is the checksum that actually resolves them. Try the
    // whole frame shifted, then tail or single-symbol shifts anchored at
    // the symbols whose readings sat closest to the rounding boundary
    // (that is where a stream step or slow drift flipped the lattice).
    if (cur_.decode.header.valid && cur_.decode.header.has_crc && cur_.decode.crc_ok) {
        cur_.synced = true;
    }
    if (params_.soft_decoding && cur_.decode.header.valid && cur_.decode.header.has_crc &&
        !cur_.decode.crc_ok) {
        DecodeResult ds = decodeSoft(softRows(cur_.raw_values.size()));
        if (ds.header.valid && ds.crc_ok) {
            cur_.decode = std::move(ds);
            cur_.synced = true;
            cur_.soft_decoded = true;
            ++soft_frames_;
        }
    }
    if (cur_.decode.header.valid && cur_.decode.header.has_crc && !cur_.decode.crc_ok) {
        auto tryDecode = [&](std::vector<uint16_t>&& cand) {
            DecodeResult d2 = decodeHard(cand);
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
                // Head shift up to and including the anchor: a sync grid a
                // chip off that the timing loop pulls back during the frame.
                auto head = cur_.raw_values;
                for (size_t s = 0; s <= m && s < head.size(); ++s) {
                    head[s] = static_cast<uint16_t>((head[s] + sh + n_chips_) % n_chips_);
                }
                if ((fixed = tryDecode(std::move(head)))) {
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
    if (params_.soft_decoding && cur_.decode.header.valid && cur_.decode.header.has_crc &&
        !cur_.decode.crc_ok) {
        const SoftSymbols soft = softRows(cur_.raw_values.size());
        for (const int sh : {-1, +1}) {
            DecodeResult ds = decodeSoft(soft, sh);
            if (ds.header.valid && ds.crc_ok) {
                cur_.decode = std::move(ds);
                cur_.synced = true;
                cur_.soft_decoded = true;
                ++soft_frames_;
                break;
            }
        }
    }
    // The payload CRC only XORs the last two bytes into the check, so one
    // wrong symbol in the tail block can flip matching bits in a payload byte
    // and the CRC nibbles and still pass (~1% of near-threshold hard passes).
    // Below the gate a hard-path pass must decode to the same payload on the
    // soft path too; two different CRC-passing payloads means one is wrong.
    if (params_.soft_decoding && !cur_.soft_decoded && cur_.decode.header.valid &&
        cur_.decode.crc_ok && metric_n_ > 0 &&
        snr_acc_ / static_cast<double>(metric_n_) < params_.soft_verify_below_snr_db - sf_gain_db_) {
        // A whole-frame, head or tail shift is a lattice slip; give the soft
        // check the same lattice so it judges the payload, not the shift. A
        // lone +-1 flip is one of ~2N guesses and must stand on the soft
        // rows as received, or the check just repeats the hard decision.
        const size_t n_rows = std::min(cur_.raw_values.size(), soft_mags_.size() / n_chips_);
        std::vector<std::pair<size_t, int>> slips;
        for (size_t s = 0; s < n_rows && s < hard_in.size(); ++s) {
            const int d = static_cast<int>(
                wrapSigned(static_cast<double>(cur_.raw_values[s]) - hard_in[s], n_chips_));
            if (d == 1 || d == -1) {
                slips.emplace_back(s, d);
            }
        }
        if (slips.size() >= 2) {
            for (const auto& [s, d] : slips) {
                rotateSoftRow(soft_mags_.data() + s * n_chips_, n_chips_, d);
            }
        }
        const DecodeResult dc = decodeSoft(softRows(cur_.raw_values.size()));
        if (!(dc.header.valid && dc.crc_ok && dc.payload == cur_.decode.payload)) {
            cur_.decode.crc_ok = false;
            cur_.synced = false;
        }
    }
    const double noise_bw_db = (params_.rx_noise_bw_hz > 0.0)
                                   ? 10.0 * std::log10(params_.rx_noise_bw_hz / params_.bandwidth_hz)
                                   : 0.0;
    if (metric_n_ > 0) {
        cur_.snr_db = snr_acc_ / static_cast<double>(metric_n_) + noise_bw_db;
        cur_.lvl_dbfs = levelDbfs(pwr_acc_ / static_cast<double>(metric_n_), sps_);
    }
    if (cur_.decode.header.valid && cur_.decode.crc_ok) {
        LoraParams pl = params_;
        pl.cr = cur_.decode.header.cr;
        pl.has_crc = cur_.decode.header.has_crc;
        const std::vector<uint16_t> sent = encodeFrame(pl, cur_.decode.payload);
        const FrameMetrics fm = decisionAidedMetrics(soft_mags_.data(), soft_mags_.size() / n_chips_,
                                                     n_chips_, sent);
        cur_.snr_db = fm.snr_db + noise_bw_db;
        cur_.sir_db = fm.sir_db;
    }
    out_.push_back(std::move(cur_));
    cur_ = ReceivedFrame{};
    resetToSearch(static_cast<size_t>(data_raw_f_) + sps_);
}

}  // namespace phy::lora
