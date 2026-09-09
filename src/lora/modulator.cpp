#include <phy/lora/modulator.hpp>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace phy::lora {

IQBuffer buildUpchirp(uint16_t id, uint8_t sf, uint32_t os, float amplitude) {
    const double n_chips = static_cast<double>(1u << sf);
    const uint32_t total = static_cast<uint32_t>(n_chips) * os;
    const uint32_t n_fold = total - id * os;
    const double os_d = static_cast<double>(os);

    IQBuffer out(total);
    for (uint32_t n = 0; n < total; ++n) {
        const double nd = static_cast<double>(n);
        const double quad = nd * nd / (2.0 * n_chips) / (os_d * os_d);
        const double lin = (n < n_fold) ? (id / n_chips - 0.5) : (id / n_chips - 1.5);
        const double phase = 2.0 * M_PI * (quad + lin * nd / os_d);
        out[n] = Sample(amplitude * static_cast<float>(std::cos(phase)),
                        amplitude * static_cast<float>(std::sin(phase)));
    }
    return out;
}

Modulator::Modulator(const LoraParams& params, float amplitude)
    : params_(params), amplitude_(amplitude) {
    sps_ = phy::lora::samplesPerSymbol(params_);
    up_ = buildUpchirp(0, params_.spreading_factor, osFactor(params_), amplitude_);
    down_.resize(up_.size());
    for (size_t i = 0; i < up_.size(); ++i) {
        down_[i] = std::conj(up_[i]);
    }
}

IQBuffer Modulator::frame(const std::vector<uint16_t>& chips) const {
    const uint32_t os = osFactor(params_);

    IQBuffer out;
    out.reserve((params_.preamble_len + 4 + chips.size()) * sps_ + sps_ / 4);

    for (uint16_t i = 0; i < params_.preamble_len; ++i) {
        out.insert(out.end(), up_.begin(), up_.end());
    }

    const IQBuffer sync0 = buildUpchirp(syncChip0(params_), params_.spreading_factor, os, amplitude_);
    const IQBuffer sync1 = buildUpchirp(syncChip1(params_), params_.spreading_factor, os, amplitude_);
    out.insert(out.end(), sync0.begin(), sync0.end());
    out.insert(out.end(), sync1.begin(), sync1.end());

    // SFD: 2.25 downchirps
    out.insert(out.end(), down_.begin(), down_.end());
    out.insert(out.end(), down_.begin(), down_.end());
    out.insert(out.end(), down_.begin(), down_.begin() + sps_ / 4);

    for (const uint16_t chip : chips) {
        const IQBuffer sym = buildUpchirp(chip, params_.spreading_factor, os, amplitude_);
        out.insert(out.end(), sym.begin(), sym.end());
    }
    return out;
}

}  // namespace phy::lora
