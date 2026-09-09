#pragma once

#include <phy/types.hpp>

#include <fftw3.h>

#include <cstddef>
#include <vector>

namespace phy {

class FftEngine {
public:
    explicit FftEngine(size_t size);
    ~FftEngine();

    FftEngine(const FftEngine&) = delete;
    FftEngine& operator=(const FftEngine&) = delete;

    size_t size() const { return size_; }

    const IQBuffer& forward(const Sample* input);
    const IQBuffer& spectrum() const { return output_; }

    size_t peakBin() const;

private:
    size_t size_ = 0;
    IQBuffer input_;
    IQBuffer output_;
    fftwf_plan plan_ = nullptr;
    fftwf_complex* fftw_in_ = nullptr;
    fftwf_complex* fftw_out_ = nullptr;
};

}  // namespace phy
