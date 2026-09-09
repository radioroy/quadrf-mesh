#pragma once

#include <phy/types.hpp>

#include <cstddef>

namespace phy {

// Catmull-Rom fractional resampler: out[k] = in(start + k * ratio).
// Needs one input sample of history and two of lookahead; stops early at
// the input edge and returns the number of outputs actually written.
//
// Implemented as a 4-tap Farrow FIR (equivalent weights to cubic Hermite
// interpolation). When PHY_USE_NEON is enabled, the polynomial evaluation is SIMD-accelerated.
size_t resampleCatmullRom(const Sample* in, size_t in_size, double start, double ratio,
                          Sample* out, size_t out_count);

}  // namespace phy
