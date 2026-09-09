#pragma once

#include <phy/types.hpp>

#include <cmath>
#include <cstddef>

namespace phy {

struct RxStats {
    double rms = 0.0;
    double peak = 0.0;
    double rms_dbfs = -100.0;
    double peak_dbfs = -100.0;
    double clip_fraction = 0.0;
};

inline RxStats computeStats(const Sample* samples, size_t count, float full_scale = 127.0f,
                            float clip_threshold = 120.0f) {
    RxStats stats;
    if (count == 0) {
        return stats;
    }

    double sum_sq = 0.0;
    double peak_sq = 0.0;
    size_t clips = 0;

    for (size_t idx = 0; idx < count; ++idx) {
        const Sample& s = samples[idx];
        const double mag_i = std::abs(s.real());
        const double mag_q = std::abs(s.imag());
        const double mag_sq = mag_i * mag_i + mag_q * mag_q;
        sum_sq += mag_sq;
        peak_sq = std::max(peak_sq, mag_sq);
        if (mag_i > clip_threshold || mag_q > clip_threshold) {
            ++clips;
        }
    }

    stats.rms = std::sqrt(sum_sq / static_cast<double>(count));
    stats.peak = std::sqrt(peak_sq);
    stats.rms_dbfs = 20.0 * std::log10(stats.rms / full_scale + 1e-12);
    stats.peak_dbfs = 20.0 * std::log10(stats.peak / full_scale + 1e-12);
    stats.clip_fraction = static_cast<double>(clips) / static_cast<double>(count);
    return stats;
}

}  // namespace phy
