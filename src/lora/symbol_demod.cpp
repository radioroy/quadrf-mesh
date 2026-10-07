#include <phy/lora/symbol_demod.hpp>

#include <phy/dsp/dechirper.hpp>
#include <phy/lora/modulator.hpp>

#include <algorithm>
#include <cmath>

namespace phy::lora {

SymbolDemod::SymbolDemod(const LoraParams& params) {
    sps_ = samplesPerSymbol(params);
    n_chips_ = chipCount(params);
    up_ = buildUpchirp(0, params.spreading_factor, osFactor(params));
    down_.resize(up_.size());
    for (size_t i = 0; i < up_.size(); ++i) {
        down_[i] = std::conj(up_[i]);
    }
    fft_ = std::make_unique<FftEngine>(sps_);
    work_.resize(sps_);
    folded_.resize(n_chips_);
}

ChipPeak SymbolDemod::demod(const Sample* window, bool downchirp_ref) {
    const Sample* ref = downchirp_ref ? up_.data() : down_.data();
    dechirp(window, ref, work_.data(), sps_);
    fft_->forward(work_.data());
    const IQBuffer& spec = fft_->spectrum();

    // Fold oversampling images into the chip domain (LoRa SDR demodulation
    // method per gr-lora / Robyns et al. and gr-lora_sdr / Tapparel et al.).
    // Kept in double — float accumulators were enough to bias the parabolic refine
    // and walk the timing loop off a clean frame.
    const uint32_t os = sps_ / n_chips_;
    for (uint32_t k = 0; k < n_chips_; ++k) {
        double p = 0.0;
        for (uint32_t im = 0; im < os; ++im) {
            p += std::norm(spec[k + im * n_chips_]);
        }
        folded_[k] = p;
    }
    return peakOf(folded_);
}

void SymbolDemod::demodData(const Sample* window, int32_t ref_phys, Sample rot, ChipPeak& nc,
                            ChipPeak& co) {
    dechirp(window, down_.data(), work_.data(), sps_);
    fft_->forward(work_.data());
    const IQBuffer& spec = fft_->spectrum();

    const uint32_t os = sps_ / n_chips_;
    const int32_t n = static_cast<int32_t>(n_chips_);
    const int32_t sps = static_cast<int32_t>(sps_);
    folded_co_.resize(n_chips_);
    // Folded bin j holds symbol value v = j - ref (mod N); its first segment
    // sits at unfolded bin v + ref_phys (mod sps), the second one N lower.
    std::vector<uint32_t> b1_of(n_chips_);
    for (int32_t j = 0; j < n; ++j) {
        double p = 0.0;
        for (uint32_t im = 0; im < os; ++im) {
            p += std::norm(spec[static_cast<size_t>(j) + im * n_chips_]);
        }
        folded_[static_cast<size_t>(j)] = p;

        const int32_t v = ((j - ref_phys) % n + n) % n;
        const int32_t b1 = ((v + ref_phys) % sps + sps) % sps;
        const int32_t b2 = (b1 - n + sps) % sps;
        b1_of[static_cast<size_t>(j)] = static_cast<uint32_t>(b1);
        const std::complex<double> y = std::complex<double>(spec[static_cast<size_t>(b1)]) +
                                       std::complex<double>(rot) *
                                           std::complex<double>(spec[static_cast<size_t>(b2)]);
        folded_co_[static_cast<size_t>(j)] = std::norm(y);
    }
    nc = peakOf(folded_);
    co = peakOf(folded_co_);
    for (ChipPeak* pk : {&nc, &co}) {
        const uint32_t b1 = b1_of[pk->chip_int];
        pk->seg1 = spec[b1];
        pk->seg2 = spec[(b1 + sps_ - n_chips_) % sps_];
    }
}

ChipPeak SymbolDemod::peakOf(const std::vector<double>& folded) const {
    uint32_t peak = 0;
    double peak_p = 0.0;
    double tot = 0.0;
    for (uint32_t k = 0; k < n_chips_; ++k) {
        tot += folded[k];
        if (folded[k] > peak_p) {
            peak_p = folded[k];
            peak = k;
        }
    }

    double lobe = 0.0;
    for (int d = -2; d <= 2; ++d) {
        const uint32_t k = (peak + n_chips_ - 2u + static_cast<uint32_t>(d + 2)) % n_chips_;
        lobe += folded[k];
    }

    // second peak, excluding the main lobe (+-2 chips circular)
    uint32_t peak2 = peak;
    double peak2_p = 0.0;
    for (uint32_t k = 0; k < n_chips_; ++k) {
        const uint32_t d = (k >= peak) ? k - peak : peak - k;
        if (std::min(d, n_chips_ - d) <= 2) {
            continue;
        }
        if (folded[k] > peak2_p) {
            peak2_p = folded[k];
            peak2 = k;
        }
    }

    // parabolic sub-chip refinement
    const double a = folded[(peak + n_chips_ - 1) % n_chips_];
    const double b = folded[peak];
    const double c = folded[(peak + 1) % n_chips_];
    const double denom = a - 2.0 * b + c;
    const double delta = (std::abs(denom) > 1e-20) ? 0.5 * (a - c) / denom : 0.0;

    // rough noise floor: median of folded powers
    std::vector<double> tmp = folded;
    std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());

    ChipPeak result;
    result.chip_int = static_cast<uint16_t>(peak);
    result.chip = static_cast<double>(peak) + delta;
    result.magnitude = peak_p;
    result.noise = tmp[tmp.size() / 2];
    result.chip2_int = static_cast<uint16_t>(peak2);
    result.mag2 = peak2_p;
    result.lobe_power = lobe;
    result.total_power = tot;
    return result;
}

FrameMetrics decisionAidedMetrics(const float* rows, size_t n_rows, uint32_t n_chips,
                                  const std::vector<uint16_t>& values) {
    FrameMetrics fm;
    const size_t k_syms = std::min(n_rows, values.size());
    if (k_syms == 0 || n_chips < 32) {
        return fm;
    }
    const auto n = static_cast<int32_t>(n_chips);
    auto at = [&](const float* r, int32_t u) {
        const double m = r[((u % n) + n) % n];
        return m * m;
    };
    auto lobe = [&](const float* r, int32_t c) {
        double s = 0.0;
        for (int32_t d = -2; d <= 2; ++d) s += at(r, c + d);
        return s;
    };
    double s_acc = 0.0, n_acc = 0.0, i_acc = 0.0;
    for (size_t k = 0; k < k_syms; ++k) {
        const float* r = rows + k * n_chips;
        // A +-1 chip lattice slip that the CRC search corrected leaves the
        // stored row one bin off the sent value; take the strongest of the three.
        int32_t c = values[k];
        double best = -1.0;
        for (int32_t d = -1; d <= 1; ++d) {
            const double l = lobe(r, values[k] + d);
            if (l > best) {
                best = l;
                c = values[k] + d;
            }
        }
        auto apart = [&](int32_t u, int32_t v) {
            int32_t dist = std::abs(((u % n) + n) % n - ((v % n) + n) % n);
            return std::min(dist, n - dist) > 4;
        };
        double tot = 0.0;
        double m2 = -1.0;
        int32_t c2 = c;
        for (int32_t u = 0; u < n; ++u) {
            const double p = at(r, u);
            tot += p;
            if (apart(u, c) && p > m2) {
                m2 = p;
                c2 = u;
            }
        }
        // An unsynchronised co-SF interferer straddles our window, so its
        // energy lands in two partial tones (fractions a and 1-a of the
        // symbol); count the runner-up lobe as well.
        double m3 = -1.0;
        int32_t c3 = c;
        for (int32_t u = 0; u < n; ++u) {
            const double p = at(r, u);
            if (apart(u, c) && apart(u, c2) && p > m3) {
                m3 = p;
                c3 = u;
            }
        }
        const double nm = (tot - best) / static_cast<double>(n - 5);
        s_acc += best - 5.0 * nm;
        n_acc += nm;
        i_acc += lobe(r, c2) + lobe(r, c3) - 10.0 * nm;
    }
    const double kk = static_cast<double>(k_syms);
    const double s_mean = s_acc / kk;
    const double n_mean = n_acc / kk;
    if (n_mean <= 0.0) {
        fm.snr_db = 40.0;
        return fm;
    }
    fm.snr_db = (s_mean > 0.0)
                    ? 10.0 * std::log10(s_mean / (n_mean * static_cast<double>(n_chips)))
                    : -30.0;
    // Noise alone: the largest of m = N - 9 exponential bins averages
    // n * H_m and the runner-up n * (H_m - 1), neighbours n each, so the two
    // lobes less 10n average n * (2 H_m - 3). Spread per symbol is about
    // n * sqrt(pi^2/6 + (pi^2/6 - 1) + 8).
    const double m = static_cast<double>(n_chips - 9);
    const double h_m = std::log(m) + 0.5772156649 + 1.0 / (2.0 * m);
    const double excess = i_acc / kk - n_mean * (2.0 * h_m - 3.0);
    const double sigma = n_mean * std::sqrt(M_PI * M_PI / 3.0 + 7.0) / std::sqrt(kk);
    if (s_mean > 0.0 && excess > 3.0 * sigma) {
        fm.sir_db = std::min(kSirCeilingDb, 10.0 * std::log10(s_mean / excess));
    }
    return fm;
}

double preambleSlope(SymbolDemod& sym, const Sample* buf, size_t len, size_t start,
                     uint16_t preamble_len, std::vector<double>* traj_out,
                     std::vector<double>* mags_out) {
    const uint32_t sps = sym.sps();
    const double n_chips = sym.chipsPerSymbol();
    const double half_chips = n_chips / 2.0;

    const size_t usable = std::min<size_t>(
        static_cast<size_t>(preamble_len) - 1, (len > start) ? (len - start) / sps : 0);
    if (usable < 5) {
        return std::nan("");
    }
    std::vector<double> traj(usable);
    std::vector<double> mags(usable);
    for (size_t s = 0; s < usable; ++s) {
        const ChipPeak pk = sym.demod(buf + start + s * sps);
        traj[s] = pk.chip;
        mags[s] = pk.magnitude;
    }
    for (size_t s = 1; s < usable; ++s) {
        double d = traj[s] - traj[s - 1];
        while (d > half_chips) {
            traj[s] -= n_chips;
            d = traj[s] - traj[s - 1];
        }
        while (d < -half_chips) {
            traj[s] += n_chips;
            d = traj[s] - traj[s - 1];
        }
    }
    // per-symbol steps; median is the robust drift-rate reference
    std::vector<double> steps(usable - 1);
    for (size_t s = 1; s < usable; ++s) {
        steps[s - 1] = traj[s] - traj[s - 1];
    }
    std::vector<double> sorted = steps;
    std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
    const double med_step = sorted[sorted.size() / 2];

    // fit the contiguous run; skip the first window (straddles the
    // detection boundary), stop at sync word / SFD outliers
    const size_t fit_begin = 1;
    size_t fit_end = fit_begin + 1;
    while (fit_end < usable && mags[fit_end] > 0.25 * mags[fit_begin] &&
           std::abs(steps[fit_end - 1] - med_step) < 4.0) {
        ++fit_end;
    }
    const size_t n_fit = fit_end - fit_begin;
    if (traj_out) {
        *traj_out = traj;
    }
    if (mags_out) {
        *mags_out = mags;
    }
    if (n_fit < 4) {
        // A single glitched read (plateau + double-step pair) can cut the
        // contiguous run short even though the drift itself is clean. The
        // median step is robust to those; average the steps that agree with
        // it instead of abandoning the frame.
        double acc = 0.0;
        size_t cnt = 0;
        for (const double st : steps) {
            if (std::abs(st - med_step) <= 1.0) {
                acc += st;
                ++cnt;
            }
        }
        if (cnt >= 3) {
            return acc / static_cast<double>(cnt);
        }
        return std::nan("");
    }
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (size_t s = fit_begin; s < fit_end; ++s) {
        sx += s;
        sy += traj[s];
        sxx += static_cast<double>(s) * s;
        sxy += s * traj[s];
    }
    return (n_fit * sxy - sx * sy) / (n_fit * sxx - sx * sx);
}

}  // namespace phy::lora
