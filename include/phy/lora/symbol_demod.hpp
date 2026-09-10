#pragma once

#include <phy/dsp/fft_engine.hpp>
#include <phy/lora/params.hpp>
#include <phy/types.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace phy::lora {

// Wrap into (-period/2, period/2]; the chip domain is circular.
inline double wrapSigned(double v, double period) {
    v = std::fmod(v, period);
    if (v > period / 2.0) {
        v -= period;
    } else if (v < -period / 2.0) {
        v += period;
    }
    return v;
}

// Fractional chip error vs the nearest integer, in (-0.5, 0.5].
inline double chipFrac(double rel_pos) {
    return rel_pos - static_cast<double>(std::lround(rel_pos));
}

// Chip timing error -> raw-stream samples. A late window reads high by
// 1/os chip per raw sample; callers typically subtract this from the
// window start. Same scale the PI loop and Catmull-Rom start offset share.
inline double chipsToRawSamples(double chips, uint32_t os, double ratio) {
    return chips * static_cast<double>(os) * ratio;
}

// One demodulated symbol: folded chip-domain peak.
struct ChipPeak {
    double chip = 0.0;  // sub-bin refined position, 0 .. 2^sf
    uint16_t chip_int = 0;
    double magnitude = 0.0;  // |peak|^2
    double noise = 0.0;      // median bin power (SEARCH detector; leave alone)
    // Runner-up peak outside the main lobe. LO supply-ripple FM puts
    // sidebands +-f_ripple/bin chips from the true tone; when the ripple
    // deepens mid-symbol the sideband can out-vote the carrier, so the
    // decoder needs the loser as a CRC-checked alternate.
    uint16_t chip2_int = 0;
    double mag2 = 0.0;  // |second peak|^2
    double lobe_power = 0.0;   // sum of peak +-2 chip bins
    double total_power = 0.0;  // sum of all folded bins
};

// Channel SNR: main-lobe mean vs off-lobe mean, minus 10*log10(2^SF).
// Comparable to Semtech PktSnr / PER-curve SNR (SF7 CR4/5 cliff ~ -7.5 dB).
inline double channelSnrDb(const ChipPeak& pk, uint32_t n_chips, uint8_t sf) {
    constexpr double kLobeBins = 5.0;
    if (n_chips <= 5 || sf == 0 || pk.total_power <= 0.0) {
        return 0.0;
    }
    const double noise = pk.total_power - pk.lobe_power;
    const double n_noise = static_cast<double>(n_chips) - kLobeBins;
    if (noise <= 0.0 || pk.lobe_power <= 0.0) {
        return 40.0;
    }
    const double n_mean = noise / n_noise;
    // Lobe *sum* is the tone energy (neighbors catch scallop); subtracting
    // 5*n_mean removes the noise that also sits in those bins. Using the
    // lobe mean instead would dilute a 1-bin tone by 10*log10(5) ~ 7 dB.
    const double s = pk.lobe_power - kLobeBins * n_mean;
    if (s <= 0.0 || n_mean <= 0.0) {
        return -20.0;
    }
    return 10.0 * std::log10(s / n_mean) -
           10.0 * std::log10(static_cast<double>(1u << sf));
}

// Winner vs second tone. High = clean; low = collision or LO-ripple sideband.
inline double sirDb(const ChipPeak& pk) {
    if (pk.lobe_power <= 0.0) {
        return 0.0;
    }
    if (pk.mag2 <= 0.0) {
        return 40.0;
    }
    return 10.0 * std::log10(pk.lobe_power / pk.mag2);
}

// Uncalibrated signal level from folded power. Amplitude 1.0 -> 0 dBFS.
inline double levelDbfs(double mean_total_power, uint32_t n_fft) {
    if (mean_total_power <= 0.0 || n_fft == 0) {
        return -99.0;
    }
    return 10.0 * std::log10(mean_total_power) -
           20.0 * std::log10(static_cast<double>(n_fft));
}

// Same-symbol sub-chip refine. When |frac| is large the peak sits near a
// rounding boundary (half a chip at os=2 for a one-sample stream step);
// frac's sign is then a coin toss, so a proportional nudge alone can steer
// the wrong way. Try ±1-sample grids (DSI slip scale) and, when |frac| is
// clearly interior, a proportional nudge. Keep the candidate with the
// tightest frac / strongest peak. `try_shift(delta)` resamples+demods at
// start+delta; return false when the window is unavailable.
//
// Policy lives here; the only DSP atom involved is the caller's resampler
// (Catmull-Rom / Farrow resampler in phy::resampleCatmullRom).
template <typename TryShift>
inline bool refineSymbolWindow(ChipPeak& pk, double& rel_pos, double& frac, double& start,
                               double ref, uint32_t n_chips, uint32_t os, double ratio,
                               TryShift&& try_shift) {
    if (std::abs(frac) <= 0.25) {
        return false;
    }
    ChipPeak best_pk = pk;
    double best_rel = rel_pos;
    double best_frac = frac;
    double best_shift = 0.0;
    double best_mag = pk.magnitude;

    auto consider = [&](double shift) {
        ChipPeak pk2;
        if (!try_shift(shift, pk2)) {
            return;
        }
        const double rel2 = wrapSigned(pk2.chip - ref, n_chips);
        const double rel2_pos = (rel2 < 0.0) ? rel2 + n_chips : rel2;
        const double frac2 = chipFrac(rel2_pos);
        const bool tighter = std::abs(frac2) < std::abs(best_frac) - 1e-6;
        const bool tied_frac = std::abs(std::abs(frac2) - std::abs(best_frac)) <= 1e-6;
        const bool louder = pk2.magnitude > best_mag * 1.02;
        // DSI drops are the common step; on a pure |frac| tie prefer the
        // earlier window (negative shift).
        const bool drop_tie =
            tied_frac && shift < best_shift && pk2.magnitude >= best_mag * 0.98;
        if (louder || tighter || drop_tie) {
            best_pk = pk2;
            best_rel = rel2_pos;
            best_frac = frac2;
            best_shift = shift;
            best_mag = pk2.magnitude;
        }
    };

    consider(-1.0);
    consider(+1.0);
    if (std::abs(std::abs(frac) - 0.5) > 0.08) {
        consider(-chipsToRawSamples(frac, os, ratio));
    }
    if (best_shift == 0.0) {
        return false;
    }
    pk = best_pk;
    rel_pos = best_rel;
    frac = best_frac;
    start += best_shift;
    return true;
}

// Per-symbol kernel of the receive path: dechirp -> FFT -> fold oversampled
// bins -> peak detection with parabolic interpolation.
class SymbolDemod {
public:
    explicit SymbolDemod(const LoraParams& params);

    ChipPeak demod(const Sample* window, bool downchirp_ref = false);

    uint32_t sps() const { return sps_; }
    uint32_t chipsPerSymbol() const { return n_chips_; }
    uint32_t oversampling() const { return sps_ / n_chips_; }

private:
    uint32_t sps_ = 0;
    uint32_t n_chips_ = 0;
    IQBuffer up_;
    IQBuffer down_;
    std::unique_ptr<FftEngine> fft_;
    std::vector<Sample> work_;
    std::vector<double> folded_;
};

// Chip drift per symbol across a preamble, measured on a symbol grid at
// `start`. Robust to the sync word / SFD at the tail (median-step outlier
// gate) and to a straddling first window. Returns NaN when no usable run.
// Optionally reports the per-symbol trajectory and magnitudes.
double preambleSlope(SymbolDemod& sym, const Sample* buf, size_t len, size_t start,
                     uint16_t preamble_len, std::vector<double>* traj_out = nullptr,
                     std::vector<double>* mags_out = nullptr);

}  // namespace phy::lora
