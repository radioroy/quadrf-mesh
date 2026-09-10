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

    uint32_t peak = 0;
    double peak_p = 0.0;
    double tot = 0.0;
    for (uint32_t k = 0; k < n_chips_; ++k) {
        tot += folded_[k];
        if (folded_[k] > peak_p) {
            peak_p = folded_[k];
            peak = k;
        }
    }

    double lobe = 0.0;
    for (int d = -2; d <= 2; ++d) {
        const uint32_t k = (peak + n_chips_ - 2u + static_cast<uint32_t>(d + 2)) % n_chips_;
        lobe += folded_[k];
    }

    // second peak, excluding the main lobe (+-2 chips circular)
    uint32_t peak2 = peak;
    double peak2_p = 0.0;
    for (uint32_t k = 0; k < n_chips_; ++k) {
        const uint32_t d = (k >= peak) ? k - peak : peak - k;
        if (std::min(d, n_chips_ - d) <= 2) {
            continue;
        }
        if (folded_[k] > peak2_p) {
            peak2_p = folded_[k];
            peak2 = k;
        }
    }

    // parabolic sub-chip refinement
    const double a = folded_[(peak + n_chips_ - 1) % n_chips_];
    const double b = folded_[peak];
    const double c = folded_[(peak + 1) % n_chips_];
    const double denom = a - 2.0 * b + c;
    const double delta = (std::abs(denom) > 1e-20) ? 0.5 * (a - c) / denom : 0.0;

    // rough noise floor: median of folded powers
    std::vector<double> tmp = folded_;
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
