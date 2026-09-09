#include <phy/dsp/chirp_generator.hpp>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace phy {

ChirpGenerator::ChirpGenerator(const PhyConfig& cfg) : cfg_(cfg) {
    n_fft_ = phy::fftSize(cfg_);
    generate();
}

void ChirpGenerator::generate() {
    up_.resize(n_fft_);
    down_.resize(n_fft_);

    const double sr = cfg_.sample_rate_hz;
    const double bw = cfg_.bandwidth_hz;
    const double T_s = symbolDurationSec(cfg_);

    for (uint32_t n = 0; n < n_fft_; ++n) {
        const double t = static_cast<double>(n) / sr;
        const double phase = 2.0 * M_PI * (-0.5 * bw * t + 0.5 * (bw / T_s) * t * t);
        up_[n] = Sample(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
        down_[n] = std::conj(up_[n]);
    }
}

IQBuffer ChirpGenerator::upChirpWithOffset(size_t bin_offset) const {
    IQBuffer out(n_fft_);
    const double sr = cfg_.sample_rate_hz;
    const double bw = cfg_.bandwidth_hz;
    const double T_s = symbolDurationSec(cfg_);

    double phase = 0.0;
    for (uint32_t n = 0; n < n_fft_; ++n) {
        out[n] = Sample(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));

        double t = static_cast<double>(n) / sr;
        double f_inst = -0.5 * bw + (static_cast<double>(bin_offset) / n_fft_) * bw + (bw / T_s) * t;

        f_inst = std::fmod(f_inst + 0.5 * bw, bw);
        if (f_inst < 0.0) f_inst += bw;
        f_inst -= 0.5 * bw;

        phase += 2.0 * M_PI * f_inst / sr;
    }
    return out;
}

IQBuffer ChirpGenerator::preamble(size_t count) const {
    IQBuffer out;
    out.reserve(static_cast<size_t>(n_fft_) * count);
    for (size_t i = 0; i < count; ++i) {
        out.insert(out.end(), up_.begin(), up_.end());
    }
    return out;
}

IQBuffer ChirpGenerator::syncWord() const {
    const size_t ms = syncNibbleOffset(cfg_, (cfg_.sync_word >> 4) & 0x0F);
    const size_t ls = syncNibbleOffset(cfg_, cfg_.sync_word & 0x0F);

    IQBuffer out;
    const IQBuffer sym0 = upChirpWithOffset(ms);
    const IQBuffer sym1 = upChirpWithOffset(ls);
    out.insert(out.end(), sym0.begin(), sym0.end());
    out.insert(out.end(), sym1.begin(), sym1.end());
    return out;
}

IQBuffer ChirpGenerator::sfd() const {
    IQBuffer out;
    out.reserve(static_cast<size_t>(n_fft_) * 2 + n_fft_ / 4);
    out.insert(out.end(), down_.begin(), down_.end());
    out.insert(out.end(), down_.begin(), down_.end());
    out.insert(out.end(), down_.begin(), down_.begin() + n_fft_ / 4);
    return out;
}

}  // namespace phy
