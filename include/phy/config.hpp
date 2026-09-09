#pragma once

#include <cmath>
#include <cstdint>

namespace phy {

struct PhyConfig {
    double center_freq_hz = 5.8e9;
    double bandwidth_hz = 500e3;
    uint8_t spreading_factor = 11;
    uint8_t sync_word = 0x2B;
    uint16_t preamble_chirps = 16;
    double sample_rate_hz = 1e6;
};

inline uint32_t samplesPerSymbol(const PhyConfig& cfg) {
    return 1u << cfg.spreading_factor;
}

inline double oversampleFactor(const PhyConfig& cfg) {
    return cfg.sample_rate_hz / cfg.bandwidth_hz;
}

inline uint32_t fftSize(const PhyConfig& cfg) {
    const double n_eff = cfg.sample_rate_hz * static_cast<double>(samplesPerSymbol(cfg)) / cfg.bandwidth_hz;
    return static_cast<uint32_t>(std::lround(n_eff));
}

inline double symbolDurationSec(const PhyConfig& cfg) {
    return static_cast<double>(samplesPerSymbol(cfg)) / cfg.bandwidth_hz;
}

inline size_t syncNibbleOffset(const PhyConfig& cfg, uint8_t nibble) {
    return (static_cast<size_t>(nibble & 0x0Fu) * fftSize(cfg)) / 16u;
}

inline size_t syncNibbleBin(const PhyConfig& cfg, uint8_t nibble) {
    return (static_cast<size_t>(nibble & 0x0Fu) * samplesPerSymbol(cfg)) / 16u;
}

}  // namespace phy
