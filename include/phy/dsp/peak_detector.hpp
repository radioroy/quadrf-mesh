#pragma once

#include <phy/types.hpp>

#include <cstddef>
#include <utility>

namespace phy {

struct PeakResult {
    size_t bin = 0;
    double magnitude = 0.0;
};

PeakResult findPeakBin(const Sample* spectrum, size_t count);

}  // namespace phy
