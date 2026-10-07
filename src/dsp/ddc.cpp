#include <phy/dsp/ddc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#ifdef PHY_USE_NEON
#include <arm_neon.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace phy {
namespace {

double besselI0(double x) {
    double sum = 1.0, term = 1.0;
    const double q = x * x / 4.0;
    for (int k = 1; k < 64; ++k) {
        term *= q / (static_cast<double>(k) * k);
        sum += term;
        if (term < 1e-12 * sum) {
            break;
        }
    }
    return sum;
}

inline Sample dotReal(const float* h, const Sample* x, size_t n) {
#ifdef PHY_USE_NEON
    // n is a multiple of 4. Two accumulator pairs cover the 4-cycle FMA
    // latency on A76.
    float32x4_t ar0 = vdupq_n_f32(0.0f), ai0 = vdupq_n_f32(0.0f);
    float32x4_t ar1 = vdupq_n_f32(0.0f), ai1 = vdupq_n_f32(0.0f);
    const float* xf = reinterpret_cast<const float*>(x);
    size_t k = 0;
    for (; k + 8 <= n; k += 8) {
        const float32x4x2_t v0 = vld2q_f32(xf + 2 * k);
        const float32x4x2_t v1 = vld2q_f32(xf + 2 * k + 8);
        const float32x4_t h0 = vld1q_f32(h + k);
        const float32x4_t h1 = vld1q_f32(h + k + 4);
        ar0 = vfmaq_f32(ar0, v0.val[0], h0);
        ai0 = vfmaq_f32(ai0, v0.val[1], h0);
        ar1 = vfmaq_f32(ar1, v1.val[0], h1);
        ai1 = vfmaq_f32(ai1, v1.val[1], h1);
    }
    for (; k < n; k += 4) {
        const float32x4x2_t v0 = vld2q_f32(xf + 2 * k);
        const float32x4_t h0 = vld1q_f32(h + k);
        ar0 = vfmaq_f32(ar0, v0.val[0], h0);
        ai0 = vfmaq_f32(ai0, v0.val[1], h0);
    }
    return Sample(vaddvq_f32(vaddq_f32(ar0, ar1)), vaddvq_f32(vaddq_f32(ai0, ai1)));
#else
    float re = 0.0f, im = 0.0f;
    for (size_t k = 0; k < n; ++k) {
        re += h[k] * x[k].real();
        im += h[k] * x[k].imag();
    }
    return Sample(re, im);
#endif
}

}  // namespace

std::vector<float> designKaiserLowpass(double fs, double pass_hz, double stop_hz,
                                       double atten_db) {
    if (!(stop_hz > pass_hz) || !(pass_hz > 0.0) || stop_hz >= fs / 2.0) {
        throw std::invalid_argument("designKaiserLowpass: bad band edges");
    }
    const double a = atten_db;
    const double beta = (a > 50.0)   ? 0.1102 * (a - 8.7)
                        : (a >= 21.0) ? 0.5842 * std::pow(a - 21.0, 0.4) + 0.07886 * (a - 21.0)
                                      : 0.0;
    const double dw = 2.0 * M_PI * (stop_hz - pass_hz) / fs;
    size_t n = static_cast<size_t>(std::ceil((a - 7.95) / (2.285 * dw))) + 1;
    n |= 1;  // odd length: integer group delay
    const double fc = 0.5 * (pass_hz + stop_hz) / fs;  // cycles/sample
    const double mid = 0.5 * static_cast<double>(n - 1);
    const double i0b = besselI0(beta);
    std::vector<double> h(n);
    double sum = 0.0;
    for (size_t k = 0; k < n; ++k) {
        const double t = static_cast<double>(k) - mid;
        const double sinc = (t == 0.0) ? 2.0 * fc : std::sin(2.0 * M_PI * fc * t) / (M_PI * t);
        const double r = t / mid;
        const double w = besselI0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / i0b;
        h[k] = sinc * w;
        sum += h[k];
    }
    std::vector<float> out(n);
    for (size_t k = 0; k < n; ++k) {
        out[k] = static_cast<float>(h[k] / sum);
    }
    return out;
}

DecimatingFir::DecimatingFir(double fs, uint32_t decim, double pass_hz, double stop_hz,
                             double atten_db)
    : decim_(decim) {
    taps_ = designKaiserLowpass(fs, pass_hz, stop_hz, atten_db);
    taps_.resize((taps_.size() + 3) & ~size_t(3), 0.0f);
    reset();
}

void DecimatingFir::reset() {
    buf_.assign(taps_.empty() ? 0 : taps_.size() - 1, Sample(0.0f, 0.0f));
    phase_ = 0;
}

void DecimatingFir::process(const Sample* in, size_t n, std::vector<Sample>& out) {
    const size_t len = taps_.size();
    buf_.insert(buf_.end(), in, in + n);
    while (phase_ + len <= buf_.size()) {
        out.push_back(dotReal(taps_.data(), buf_.data() + phase_, len));
        phase_ += decim_;
    }
    const size_t drop = std::min(phase_, buf_.size());
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<ptrdiff_t>(drop));
    phase_ -= drop;
}

Ddc::Ddc(const Config& cfg) : cfg_(cfg) {
    const double r = cfg.fs_in / cfg.fs_out;
    const auto ratio = static_cast<uint32_t>(std::lround(r));
    if (ratio < 1 || std::abs(r - ratio) > 1e-9) {
        throw std::invalid_argument("Ddc: fs_in must be an integer multiple of fs_out");
    }
    if (ratio == 1) {
        if (cfg.f_if != 0.0) {
            throw std::invalid_argument("Ddc: IF offset needs fs_in > fs_out");
        }
        passthrough_ = true;
        noise_bw_hz_ = cfg.fs_in;
        return;
    }
    if (ratio % 2 != 0) {
        throw std::invalid_argument("Ddc: fs_in / fs_out must be even");
    }
    if (std::abs(cfg.f_if) + cfg.pass_hz >= cfg.fs_in / 2.0) {
        throw std::invalid_argument("Ddc: IF + passband exceed input Nyquist");
    }
    passthrough_ = false;
    const double fs_mid = 2.0 * cfg.fs_out;
    use_st1_ = ratio > 2;
    if (use_st1_) {
        // Protect only [-stop, +stop] at fs_mid; stage 2 removes the rest.
        st1_ = DecimatingFir(cfg.fs_in, ratio / 2, cfg.stop_hz, fs_mid - cfg.stop_hz,
                             cfg.atten_db);
    }
    st2_ = DecimatingFir(fs_mid, 2, cfg.pass_hz, cfg.stop_hz, cfg.atten_db);

    // Cascade impulse response at fs_in: h1 * (h2 upsampled by ratio/2).
    // Both stages have unit DC gain, so white input of density N0 leaves
    // N0 * fs_in * sum(h^2) per output sample.
    {
        const std::vector<float>& h2 = st2_.coefficients();
        std::vector<double> h;
        if (use_st1_) {
            const std::vector<float>& h1 = st1_.coefficients();
            const size_t m1 = ratio / 2;
            h.assign(h1.size() + (h2.size() - 1) * m1, 0.0);
            for (size_t j = 0; j < h2.size(); ++j) {
                for (size_t i = 0; i < h1.size(); ++i) {
                    h[i + j * m1] += static_cast<double>(h1[i]) * h2[j];
                }
            }
        } else {
            h.assign(h2.begin(), h2.end());
        }
        double e = 0.0;
        for (const double v : h) {
            e += v * v;
        }
        noise_bw_hz_ = cfg.fs_in * e;
    }

    if (cfg.f_if != 0.0) {
        const double cyc = cfg.f_if / cfg.fs_in;
        size_t period = 0;
        for (size_t q = 1; q <= 4096; ++q) {
            const double v = cyc * static_cast<double>(q);
            if (std::abs(v - std::round(v)) < 1e-9) {
                period = q;
                break;
            }
        }
        if (period) {
            // Repeat the period out to >= 1024 entries so the mix loop runs
            // long contiguous spans.
            const size_t reps = (1024 + period - 1) / period;
            nco_.resize(period * reps);
            for (size_t k = 0; k < nco_.size(); ++k) {
                const double ph = -2.0 * M_PI * std::fmod(cyc * static_cast<double>(k % period), 1.0);
                nco_[k] = Sample(static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
            }
        } else {
            const double w = -2.0 * M_PI * cyc;
            rot_step_ = Sample(static_cast<float>(std::cos(w)), static_cast<float>(std::sin(w)));
        }
    }
}

void Ddc::reset() {
    nco_pos_ = 0;
    rot_ = Sample(1.0f, 0.0f);
    rot_count_ = 0;
    if (use_st1_) {
        st1_.reset();
    }
    st2_.reset();
}

void Ddc::process(const Sample* in, size_t n, std::vector<Sample>& out) {
    if (passthrough_) {
        out.insert(out.end(), in, in + n);
        return;
    }
    const Sample* src = in;
    if (cfg_.f_if != 0.0) {
        mixed_.resize(n);
        if (!nco_.empty()) {
            size_t i = 0;
            while (i < n) {
                const size_t span = std::min(n - i, nco_.size() - nco_pos_);
                const Sample* lo = nco_.data() + nco_pos_;
                Sample* dst = mixed_.data() + i;
                const Sample* x = in + i;
                size_t k = 0;
#ifdef PHY_USE_NEON
                for (; k + 4 <= span; k += 4) {
                    const float32x4x2_t a = vld2q_f32(reinterpret_cast<const float*>(x + k));
                    const float32x4x2_t b = vld2q_f32(reinterpret_cast<const float*>(lo + k));
                    float32x4x2_t c;
                    c.val[0] = vfmsq_f32(vmulq_f32(a.val[0], b.val[0]), a.val[1], b.val[1]);
                    c.val[1] = vfmaq_f32(vmulq_f32(a.val[0], b.val[1]), a.val[1], b.val[0]);
                    vst2q_f32(reinterpret_cast<float*>(dst + k), c);
                }
#endif
                for (; k < span; ++k) {
                    dst[k] = x[k] * lo[k];
                }
                i += span;
                nco_pos_ += span;
                if (nco_pos_ == nco_.size()) {
                    nco_pos_ = 0;
                }
            }
        } else {
            for (size_t k = 0; k < n; ++k) {
                mixed_[k] = in[k] * rot_;
                rot_ *= rot_step_;
                if (++rot_count_ == 1024) {
                    rot_ /= std::abs(rot_);
                    rot_count_ = 0;
                }
            }
        }
        src = mixed_.data();
    }
    if (use_st1_) {
        mid_.clear();
        st1_.process(src, n, mid_);
        st2_.process(mid_.data(), mid_.size(), out);
    } else {
        st2_.process(src, n, out);
    }
}

}  // namespace phy
