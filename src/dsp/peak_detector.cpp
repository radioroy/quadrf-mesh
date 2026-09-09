#include <phy/dsp/peak_detector.hpp>

#include <cmath>

namespace phy {

PeakResult findPeakBin(const Sample* spectrum, size_t count) {
    PeakResult result;
    for (size_t i = 0; i < count; ++i) {
        const double mag = std::norm(spectrum[i]);
        if (mag > result.magnitude) {
            result.magnitude = mag;
            result.bin = i;
        }
    }
    return result;
}

}  // namespace phy
