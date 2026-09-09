#pragma once

#include <phy/types.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace phy {

class ToneGenerator {
public:
    ToneGenerator(double sample_rate_hz, double tone_hz, float amplitude = 0.8f)
        : sample_rate_hz_(sample_rate_hz), tone_hz_(tone_hz), amplitude_(amplitude) {}

    void fill(Sample* buffer, size_t count) {
        const double phase_inc = 2.0 * M_PI * tone_hz_ / sample_rate_hz_;
        for (size_t i = 0; i < count; ++i) {
            buffer[i] = Sample(amplitude_ * static_cast<float>(std::cos(phase_)),
                               amplitude_ * static_cast<float>(std::sin(phase_)));
            phase_ += phase_inc;
            if (phase_ > 2.0 * M_PI) {
                phase_ -= 2.0 * M_PI;
            }
        }
    }

    void reset() { phase_ = 0.0; }

    std::vector<Sample> buffer(size_t count) {
        std::vector<Sample> out(count);
        fill(out.data(), count);
        return out;
    }

private:
    double sample_rate_hz_;
    double tone_hz_;
    float amplitude_;
    double phase_ = 0.0;
};

}  // namespace phy
