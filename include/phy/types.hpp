#pragma once

#include <complex>
#include <cstdint>
#include <vector>

namespace phy {

using Sample = std::complex<float>;
using IQBuffer = std::vector<Sample>;

}  // namespace phy
