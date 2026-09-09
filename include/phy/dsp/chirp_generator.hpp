#pragma once

#include <phy/config.hpp>
#include <phy/types.hpp>

namespace phy {

class ChirpGenerator {
public:
    explicit ChirpGenerator(const PhyConfig& cfg);

    const IQBuffer& upChirp() const { return up_; }
    const IQBuffer& downChirp() const { return down_; }

    IQBuffer upChirpWithOffset(size_t bin_offset) const;
    IQBuffer preamble(size_t count) const;
    IQBuffer syncWord() const;
    IQBuffer sfd() const;

    uint32_t fftSize() const { return n_fft_; }

private:
    void generate();

    PhyConfig cfg_;
    uint32_t n_fft_ = 0;
    IQBuffer up_;
    IQBuffer down_;
};

}  // namespace phy
