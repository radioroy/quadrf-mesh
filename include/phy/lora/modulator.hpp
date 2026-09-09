#pragma once

#include <phy/lora/params.hpp>
#include <phy/types.hpp>

#include <cstdint>
#include <vector>

namespace phy::lora {

// Analytic modulated upchirp, following gr-lora_sdr build_upchirp convention
// (Tapparel et al., EPFL; https://github.com/tapparelj/gr-lora_sdr):
// starts at (id/N - 0.5)*bw and folds down by bw when it reaches +bw/2.
IQBuffer buildUpchirp(uint16_t id, uint8_t sf, uint32_t os, float amplitude = 1.0f);

class Modulator {
public:
    explicit Modulator(const LoraParams& params, float amplitude = 0.8f);

    // preamble + sync word + 2.25 downchirp SFD + data chirps
    IQBuffer frame(const std::vector<uint16_t>& chips) const;

    const IQBuffer& upChirp() const { return up_; }
    const IQBuffer& downChirp() const { return down_; }
    uint32_t samplesPerSymbol() const { return sps_; }

private:
    LoraParams params_;
    float amplitude_;
    uint32_t sps_ = 0;
    IQBuffer up_;
    IQBuffer down_;
};

}  // namespace phy::lora
