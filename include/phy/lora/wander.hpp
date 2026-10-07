#pragma once

#include <phy/types.hpp>

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace phy::lora {

// Expected carrier wander between two units, in chips (FFT bins) per symbol.
// QuadRF units free-run their MAX2850 crystals: the relative carrier
// between two units moves ~800 Hz RMS, flat to ~30 Hz FM rate and falling
// steeply above ~50 Hz (CW capture, 5.8 GHz). The change over one symbol
// grows ~rate * T until it saturates near the total spread, so in bins
// (1 bin = 1/T) it scales as min(rate * T, cap) * T.
inline double wanderChipsPerSymbol(double symbol_s, double rate_hz_per_s, double cap_hz) {
    if (rate_hz_per_s <= 0.0) {
        return 0.0;
    }
    const double hz = rate_hz_per_s * symbol_s;
    return (hz < cap_hz ? hz : cap_hz) * symbol_s;
}

// Legendre P_0..P_order at x in [-1, 1], and dP/dx.
void legendreAt(double x, int order, double* p, double* dp);

// Weighted ridge least squares: minimise
//   sum_r w[r] (y[r] - a[r,:] x)^2 + sum_i prior_inv[i] x[i]^2
// a: rows x np, row-major. cov: np x np posterior covariance. np <= 8.
bool ridgeSolve(const double* a, const double* y, const double* w, size_t rows, int np,
                const double* prior_inv, double* x, double* cov);

// Residual carrier trajectory inside one symbol. The window is dechirped and
// split into sub-blocks; each sub-block is correlated against the decided
// tone (both fold segments: bin b1 before the chirp wrap, b1 - N after),
// and phase steps between same-segment neighbours give the residual
// frequency at sub-block boundaries. A ridge-regularised Legendre fit on
// t in [0, 1] (symbol time) turns those into a smooth trajectory eps(t), in
// chips: eps(t) = sum c_i P_i(2t - 1). c_0 is the mean residual and is left
// unregularised; higher orders are pulled toward 0 with prior sigmas
// scaled from the expected wander.
class WanderFit {
public:
    static constexpr int kMaxOrder = 5;
    using Coeffs = std::array<double, kMaxOrder + 1>;

    // sigma_w: expected wander over one symbol, chips. order: 1..kMaxOrder.
    void configure(uint32_t sps, uint32_t n_chips, double sigma_w, int order);

    // dechirped: sps samples (window * reference chirp). b1: unfolded FFT
    // bin of the decided tone's first segment. wrap: sample where the tone
    // steps to b1 - N (sps = never), -1 if unknown. noise_var: per-sample
    // complex noise variance of `dechirped`. n_sub: sub-blocks (>= 4,
    // divides sps). Returns false when too few usable phase steps.
    bool fit(const Sample* dechirped, int32_t b1, int32_t wrap, double noise_var, uint32_t n_sub);

    int order() const { return order_; }
    double eval(double t) const;  // eps(t), chips
    double slope(double t) const; // d eps / dt, chips per symbol
    double var(double t) const;   // posterior variance of eps(t)
    double mean() const { return c_[0]; }
    double meanVar() const { return cov_[0][0]; }
    // Average d eps / dt over the symbol (the P_1 term), chips per symbol.
    // Far less noisy than slope(1), which extrapolates the fit's edge.
    double meanSlope() const { return order_ >= 1 ? 2.0 * c_[1] : 0.0; }

    // Point observations behind the last fit: residual eps at time t with
    // weight 1/variance (already scaled by the fit's chi^2).
    size_t observations() const { return n_obs_; }
    double obsT(size_t k) const { return obs_t_[k]; }
    double obsVal(size_t k) const { return obs_val_[k]; }
    double obsW(size_t k) const { return obs_w_[k]; }
    // First-stage (short-block) observations: noisier, but they reach to
    // within half a block of the symbol edges.
    size_t fineObservations() const { return n_fine_; }
    double fineT(size_t k) const { return fine_t_[k]; }
    double fineVal(size_t k) const { return fine_val_[k]; }
    double fineW(size_t k) const { return fine_w_[k]; }

    // Multiply window by exp(-j 2 pi int_0^t (eps + extra) dt'), the phase a
    // residual carrier of eps(t) + extra chips builds up over the symbol.
    void derotate(Sample* window, double extra_chips) const;

private:
    bool solve(const Sample* z, int32_t b1, int32_t wrap, double noise_var, uint32_t n_sub,
               const Coeffs& prior_inv);
    double evalWith(const Coeffs& c, double t) const;

    struct Block {
        std::complex<double> p;
        int seg;        // 1: before the chirp wrap, 2: after
        double center;  // samples
        double len;     // samples
    };

    uint32_t sps_ = 0;
    uint32_t n_chips_ = 0;
    int order_ = 3;
    Coeffs prior_inv_{};
    Coeffs c_{};
    std::array<Coeffs, kMaxOrder + 1> cov_{};
    size_t n_obs_ = 0;
    std::vector<double> obs_t_, obs_val_, obs_w_, rows_;
    size_t n_fine_ = 0;
    std::vector<double> fine_t_, fine_val_, fine_w_;
    std::vector<Block> blocks_;
    std::vector<Sample> tmp_;
};

}  // namespace phy::lora
