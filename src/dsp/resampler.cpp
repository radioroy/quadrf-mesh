#include <phy/dsp/resampler.hpp>

#include <cmath>
#include <cstddef>

#ifdef PHY_USE_NEON
#include <arm_neon.h>
#endif

namespace phy {
namespace {

// Catmull-Rom / cubic Hermite as Farrow weights on {p0,p1,p2,p3}, mu in [0,1).
inline void farrowWeights(float mu, float& w0, float& w1, float& w2, float& w3) {
    const float mu2 = mu * mu;
    const float mu3 = mu2 * mu;
    w0 = 0.5f * (-mu + 2.0f * mu2 - mu3);
    w1 = 1.0f + 0.5f * (-5.0f * mu2 + 3.0f * mu3);
    w2 = 0.5f * (mu + 4.0f * mu2 - 3.0f * mu3);
    w3 = 0.5f * (-mu2 + mu3);
}

inline Sample farrowTap(const Sample* p, float mu) {
    float w0, w1, w2, w3;
    farrowWeights(mu, w0, w1, w2, w3);
#ifdef PHY_USE_NEON
    // Four contiguous CF32 -> deinterleaved re/im, then horizontal weighted sum.
    const float32x4x2_t v = vld2q_f32(reinterpret_cast<const float*>(p));
    const float32x4_t w = {w0, w1, w2, w3};
    return Sample(vaddvq_f32(vmulq_f32(v.val[0], w)), vaddvq_f32(vmulq_f32(v.val[1], w)));
#else
    return w0 * p[0] + w1 * p[1] + w2 * p[2] + w3 * p[3];
#endif
}

}  // namespace

size_t resampleCatmullRom(const Sample* in, size_t in_size, double start, double ratio,
                          Sample* out, size_t out_count) {
    for (size_t k = 0; k < out_count; ++k) {
        const double pos = start + static_cast<double>(k) * ratio;
        const auto i1 = static_cast<ptrdiff_t>(std::floor(pos));
        if (i1 < 1 || i1 + 2 >= static_cast<ptrdiff_t>(in_size)) {
            return k;
        }
        const float mu = static_cast<float>(pos - static_cast<double>(i1));
        // p0..p3 = in[i1-1 .. i1+2], contiguous — Farrow NEON loads them as one vld2.
        out[k] = farrowTap(in + (i1 - 1), mu);
    }
    return out_count;
}

}  // namespace phy
