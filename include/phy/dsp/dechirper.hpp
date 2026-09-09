#pragma once

#include <phy/types.hpp>

#include <cstddef>

namespace phy {

void dechirp(const Sample* rx, const Sample* ref_down, Sample* out, size_t count);

}  // namespace phy
