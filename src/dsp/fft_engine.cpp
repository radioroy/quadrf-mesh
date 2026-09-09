#include <phy/dsp/fft_engine.hpp>

#include <cmath>
#include <stdexcept>

namespace phy {

FftEngine::FftEngine(size_t size) : size_(size), input_(size), output_(size) {
    if (size_ == 0) {
        throw std::invalid_argument("FftEngine: size must be > 0");
    }

    fftw_in_ = fftwf_alloc_complex(static_cast<int>(size_));
    fftw_out_ = fftwf_alloc_complex(static_cast<int>(size_));
    if (!fftw_in_ || !fftw_out_) {
        throw std::runtime_error("FftEngine: fftwf_alloc_complex failed");
    }

    plan_ = fftwf_plan_dft_1d(static_cast<int>(size_), fftw_in_, fftw_out_, FFTW_FORWARD, FFTW_MEASURE);
    if (!plan_) {
        throw std::runtime_error("FftEngine: fftwf_plan_dft_1d failed");
    }
}

FftEngine::~FftEngine() {
    if (plan_) {
        fftwf_destroy_plan(plan_);
    }
    if (fftw_in_) {
        fftwf_free(fftw_in_);
    }
    if (fftw_out_) {
        fftwf_free(fftw_out_);
    }
}

const IQBuffer& FftEngine::forward(const Sample* input) {
    for (size_t i = 0; i < size_; ++i) {
        fftw_in_[i][0] = input[i].real();
        fftw_in_[i][1] = input[i].imag();
    }

    fftwf_execute(plan_);

    for (size_t i = 0; i < size_; ++i) {
        output_[i] = Sample(fftw_out_[i][0], fftw_out_[i][1]);
    }
    return output_;
}

size_t FftEngine::peakBin() const {
    size_t peak = 0;
    double peak_mag = 0.0;
    for (size_t i = 0; i < output_.size(); ++i) {
        const double mag = std::norm(output_[i]);
        if (mag > peak_mag) {
            peak_mag = mag;
            peak = i;
        }
    }
    return peak;
}

}  // namespace phy
