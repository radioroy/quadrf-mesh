#include <phy/dsp/dechirper.hpp>

#ifdef PHY_USE_NEON
#include <arm_neon.h>
#endif

namespace phy {

void dechirp(const Sample* rx, const Sample* ref_down, Sample* out, size_t count) {
    size_t i = 0;

#ifdef PHY_USE_NEON
    for (; i + 4 <= count; i += 4) {
        const float32x4x2_t rx_val = vld2q_f32(reinterpret_cast<const float*>(rx + i));
        const float32x4x2_t ref_val = vld2q_f32(reinterpret_cast<const float*>(ref_down + i));

        float32x4_t out_r = vmulq_f32(rx_val.val[0], ref_val.val[0]);
        out_r = vmlsq_f32(out_r, rx_val.val[1], ref_val.val[1]);

        float32x4_t out_i = vmulq_f32(rx_val.val[0], ref_val.val[1]);
        out_i = vmlaq_f32(out_i, rx_val.val[1], ref_val.val[0]);

        float32x4x2_t out_val;
        out_val.val[0] = out_r;
        out_val.val[1] = out_i;
        vst2q_f32(reinterpret_cast<float*>(out + i), out_val);
    }
#endif

    for (; i < count; ++i) {
        out[i] = rx[i] * ref_down[i];
    }
}

}  // namespace phy
