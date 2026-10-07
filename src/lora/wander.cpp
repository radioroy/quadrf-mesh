#include <phy/lora/wander.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

namespace phy::lora {

namespace {

constexpr double kTwoPi = 6.283185307179586;

}  // namespace

void legendreAt(double x, int order, double* p, double* dp) {
    p[0] = 1.0;
    dp[0] = 0.0;
    if (order >= 1) {
        p[1] = x;
        dp[1] = 1.0;
    }
    for (int i = 2; i <= order; ++i) {
        p[i] = ((2.0 * i - 1.0) * x * p[i - 1] - (i - 1.0) * p[i - 2]) / i;
        dp[i] = dp[i - 2] + (2.0 * i - 1.0) * p[i - 1];
    }
}

bool ridgeSolve(const double* a, const double* y, const double* w, size_t rows, int np,
                const double* prior_inv, double* x, double* cov) {
    constexpr int kMax = 8;
    if (np < 1 || np > kMax) {
        return false;
    }
    double h[kMax][kMax] = {};
    double g[kMax] = {};
    for (int i = 0; i < np; ++i) {
        h[i][i] = prior_inv[i];
    }
    for (size_t r = 0; r < rows; ++r) {
        const double* ar = a + r * static_cast<size_t>(np);
        for (int i = 0; i < np; ++i) {
            g[i] += w[r] * ar[i] * y[r];
            for (int j = 0; j < np; ++j) {
                h[i][j] += w[r] * ar[i] * ar[j];
            }
        }
    }
    // Gauss-Jordan on [H | I]: the inverse is the posterior covariance.
    double m[kMax][2 * kMax] = {};
    for (int i = 0; i < np; ++i) {
        for (int j = 0; j < np; ++j) {
            m[i][j] = h[i][j];
        }
        m[i][np + i] = 1.0;
    }
    for (int col = 0; col < np; ++col) {
        int piv = col;
        for (int r = col + 1; r < np; ++r) {
            if (std::abs(m[r][col]) > std::abs(m[piv][col])) {
                piv = r;
            }
        }
        if (std::abs(m[piv][col]) < 1e-300) {
            return false;
        }
        if (piv != col) {
            for (int j = 0; j < 2 * np; ++j) {
                std::swap(m[piv][j], m[col][j]);
            }
        }
        const double inv = 1.0 / m[col][col];
        for (int j = 0; j < 2 * np; ++j) {
            m[col][j] *= inv;
        }
        for (int r = 0; r < np; ++r) {
            if (r == col || m[r][col] == 0.0) {
                continue;
            }
            const double f = m[r][col];
            for (int j = 0; j < 2 * np; ++j) {
                m[r][j] -= f * m[col][j];
            }
        }
    }
    for (int i = 0; i < np; ++i) {
        double xi = 0.0;
        for (int j = 0; j < np; ++j) {
            cov[i * np + j] = m[i][np + j];
            xi += m[i][np + j] * g[j];
        }
        x[i] = xi;
    }
    return true;
}

void WanderFit::configure(uint32_t sps, uint32_t n_chips, double sigma_w, int order) {
    sps_ = sps;
    n_chips_ = n_chips;
    order_ = std::clamp(order, 1, kMaxOrder);
    prior_inv_.fill(0.0);
    for (int i = 1; i <= order_; ++i) {
        // P_1 spans +-1, so a start-to-end change of sigma_w is c_1 ~ sigma_w / 2.
        // Higher orders carry curvature; give them less room. The floor keeps a
        // well-behaved fit when the expected wander is tiny.
        const double sigma = std::max(sigma_w / (1.0 + i), 0.02);
        prior_inv_[static_cast<size_t>(i)] = 1.0 / (sigma * sigma);
    }
    c_.fill(0.0);
    for (auto& row : cov_) {
        row.fill(0.0);
    }
}

double WanderFit::evalWith(const Coeffs& c, double t) const {
    double p[kMaxOrder + 1], dp[kMaxOrder + 1];
    legendreAt(2.0 * t - 1.0, order_, p, dp);
    double v = 0.0;
    for (int i = 0; i <= order_; ++i) {
        v += c[static_cast<size_t>(i)] * p[i];
    }
    return v;
}

bool WanderFit::solve(const Sample* z, int32_t b1, int32_t wrap, double noise_var,
                      uint32_t n_sub, const Coeffs& prior_inv) {
    c_.fill(0.0);
    for (auto& row : cov_) {
        row.fill(0.0);
    }
    n_obs_ = 0;
    if (sps_ == 0 || n_sub < 4 || sps_ % n_sub != 0) {
        return false;
    }
    const uint32_t len = sps_ / n_sub;
    const auto sps = static_cast<int32_t>(sps_);
    const int32_t b2 = ((b1 - static_cast<int32_t>(n_chips_)) % sps + sps) % sps;

    const std::complex<double> w1 = std::polar(1.0, -kTwoPi * b1 / sps);
    const std::complex<double> w2 = std::polar(1.0, -kTwoPi * b2 / sps);
    // Known wrap: the tone is b1 before sample `wrap` and b1 - N after. The
    // phase step across it depends on sub-sample timing (pi per sample at
    // os = 2), so the block holding the wrap is split in two and pairs never
    // cross it. Unknown wrap: correlate each block against both tones and
    // keep the louder.
    const bool known = wrap >= 0;
    blocks_.clear();
    for (uint32_t m = 0; m < n_sub; ++m) {
        const uint32_t n0 = m * len;
        // Restart the oscillators per block so rounding drift stays bounded.
        std::complex<double> r1 = std::polar(1.0, -kTwoPi * static_cast<double>(b1) * n0 / sps);
        std::complex<double> r2 = std::polar(1.0, -kTwoPi * static_cast<double>(b2) * n0 / sps);
        std::complex<double> p1{}, p2{};
        const auto split = known ? std::clamp<int32_t>(wrap - static_cast<int32_t>(n0), 0,
                                                       static_cast<int32_t>(len))
                                 : static_cast<int32_t>(len);
        for (uint32_t n = 0; n < len; ++n) {
            const std::complex<double> s(z[n0 + n].real(), z[n0 + n].imag());
            if (!known || static_cast<int32_t>(n) < split) {
                p1 += s * r1;
            }
            if (!known || static_cast<int32_t>(n) >= split) {
                p2 += s * r2;
            }
            r1 *= w1;
            r2 *= w2;
        }
        if (!known) {
            const bool first = std::norm(p1) >= std::norm(p2);
            blocks_.push_back({first ? p1 : p2, first ? 1 : 2, n0 + 0.5 * len, double(len)});
            continue;
        }
        // Slivers under 1/8 block carry little signal; drop them.
        const double a1 = split, a2 = len - static_cast<double>(split);
        if (a1 >= len / 8.0) {
            blocks_.push_back({p1, 1, n0 + 0.5 * a1, a1});
        }
        if (a2 >= len / 8.0) {
            blocks_.push_back({p2, 2, n0 + a1 + 0.5 * a2, a2});
        }
    }

    const int np = order_ + 1;
    // Phase noise of a block phasor is ~1/(2 SNR) rad^2 at high SNR and
    // saturates at the uniform-phase pi^2/3.
    const double var_cap = kTwoPi * kTwoPi / 12.0;
    const size_t n_blk = blocks_.size();
    obs_t_.resize(n_blk);
    obs_val_.resize(n_blk);
    obs_w_.resize(n_blk);
    rows_.resize(n_blk * static_cast<size_t>(np));
    double dp[kMaxOrder + 1];
    for (size_t m = 0; m + 1 < n_blk; ++m) {
        const Block& q0 = blocks_[m];
        const Block& q1 = blocks_[m + 1];
        if (q0.seg != q1.seg) {
            continue;
        }
        const double snr0 = std::max(std::norm(q0.p) / std::max(noise_var * q0.len, 1e-30) - 1.0, 0.3);
        const double snr1 = std::max(std::norm(q1.p) / std::max(noise_var * q1.len, 1e-30) - 1.0, 0.3);
        const double var_d = std::min(0.5 / snr0 + 0.5 / snr1, var_cap);
        const double to_chips = sps_ / (kTwoPi * (q1.center - q0.center));
        const double t = 0.5 * (q0.center + q1.center) / sps_;
        obs_t_[n_obs_] = t;
        obs_val_[n_obs_] = std::arg(q1.p * std::conj(q0.p)) * to_chips;
        obs_w_[n_obs_] = 1.0 / (var_d * to_chips * to_chips);
        legendreAt(2.0 * t - 1.0, order_, rows_.data() + n_obs_ * np, dp);
        ++n_obs_;
    }
    if (n_obs_ < 2) {
        n_obs_ = 0;
        return false;
    }
    double cov[(kMaxOrder + 1) * (kMaxOrder + 1)];
    if (!ridgeSolve(rows_.data(), obs_val_.data(), obs_w_.data(), n_obs_, np, prior_inv.data(),
                    c_.data(), cov)) {
        n_obs_ = 0;
        return false;
    }
    // The phase-noise model only knows thermal noise; resampler and filter
    // ripple add scatter that is not in it. Scale the posterior (and the
    // exported weights) by chi^2 / dof, never below 1.
    double chi2 = 0.0;
    for (size_t k = 0; k < n_obs_; ++k) {
        const double r = obs_val_[k] - eval(obs_t_[k]);
        chi2 += obs_w_[k] * r * r;
    }
    const int dof = static_cast<int>(n_obs_) - np;
    const double scale = dof > 0 ? std::max(1.0, chi2 / dof) : 1.0;
    for (int i = 0; i < np; ++i) {
        for (int j = 0; j < np; ++j) {
            cov_[static_cast<size_t>(i)][static_cast<size_t>(j)] = cov[i * np + j] * scale;
        }
    }
    for (size_t k = 0; k < n_obs_; ++k) {
        obs_w_[k] /= scale;
    }
    return true;
}

bool WanderFit::fit(const Sample* z, int32_t b1, int32_t wrap, double noise_var,
                    uint32_t n_sub) {
    if (!solve(z, b1, wrap, noise_var, n_sub, prior_inv_)) {
        n_fine_ = 0;
        return false;
    }
    n_fine_ = n_obs_;
    fine_t_.assign(obs_t_.begin(), obs_t_.begin() + static_cast<std::ptrdiff_t>(n_obs_));
    fine_val_.assign(obs_val_.begin(), obs_val_.begin() + static_cast<std::ptrdiff_t>(n_obs_));
    fine_w_.assign(obs_w_.begin(), obs_w_.begin() + static_cast<std::ptrdiff_t>(n_obs_));
    if (n_sub < 16) {
        return true;
    }
    // Short blocks follow fast wander without phase wrap but resolve
    // frequency poorly (a phase step over L samples reads to S / (2 pi L)
    // chips per radian). Take the first fit out and re-fit what is left
    // with 4-8 long blocks; the first pass's uncertainty is the prior.
    const Coeffs c1 = c_;
    const auto cov1 = cov_;
    const size_t n1 = n_obs_;
    const std::vector<double> t1 = obs_t_, v1 = obs_val_, w1 = obs_w_;
    tmp_.assign(z, z + sps_);
    derotate(tmp_.data(), 0.0);
    Coeffs prior2{};
    for (int i = 1; i <= order_; ++i) {
        const double v = cov1[static_cast<size_t>(i)][static_cast<size_t>(i)];
        prior2[static_cast<size_t>(i)] = 1.0 / std::max(v, 1e-6);
    }
    const uint32_t n2 = std::min<uint32_t>(order_ >= 4 ? 32 : 8, n_sub / 2);
    if (!solve(tmp_.data(), b1, wrap, noise_var, n2, prior2)) {
        c_ = c1;
        cov_ = cov1;
        obs_t_ = t1;
        obs_val_ = v1;
        obs_w_ = w1;
        n_obs_ = n1;
        return true;
    }
    // Stage-2 observations are residuals against the stage-1 trajectory.
    for (size_t k = 0; k < n_obs_; ++k) {
        obs_val_[k] += evalWith(c1, obs_t_[k]);
    }
    for (int i = 0; i <= order_; ++i) {
        c_[static_cast<size_t>(i)] += c1[static_cast<size_t>(i)];
    }
    return true;
}

double WanderFit::eval(double t) const {
    return evalWith(c_, t);
}

double WanderFit::slope(double t) const {
    double p[kMaxOrder + 1], dp[kMaxOrder + 1];
    legendreAt(2.0 * t - 1.0, order_, p, dp);
    double v = 0.0;
    for (int i = 0; i <= order_; ++i) {
        v += c_[static_cast<size_t>(i)] * 2.0 * dp[i];
    }
    return v;
}

double WanderFit::var(double t) const {
    double p[kMaxOrder + 1], dp[kMaxOrder + 1];
    legendreAt(2.0 * t - 1.0, order_, p, dp);
    double v = 0.0;
    for (int i = 0; i <= order_; ++i) {
        for (int j = 0; j <= order_; ++j) {
            v += p[i] * cov_[static_cast<size_t>(i)][static_cast<size_t>(j)] * p[j];
        }
    }
    return v;
}

void WanderFit::derotate(Sample* window, double extra_chips) const {
    double phase = 0.0;
    const double inv = 1.0 / sps_;
    for (uint32_t n = 0; n < sps_; ++n) {
        const double f = eval((n + 0.5) * inv) + extra_chips;
        const double c = std::cos(phase), s = std::sin(phase);
        window[n] *= Sample(static_cast<float>(c), static_cast<float>(-s));
        phase += kTwoPi * f * inv;
    }
}

}  // namespace phy::lora
