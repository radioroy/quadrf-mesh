#pragma once

#include <cstdint>

namespace phy::lora {

// LoRa PHY parameters, gr-lora_sdr / SX127x conventions.
// cr is the coding-rate index: 1..4 -> 4/5..4/8.
struct LoraParams {
    uint8_t spreading_factor = 11;
    uint8_t cr = 1;
    bool has_crc = true;
    bool ldro = false;  // low data rate optimization (reduced-rate payload blocks)
    uint8_t sync_word = 0x2B;
    uint16_t preamble_len = 16;
    double bandwidth_hz = 500e3;
    double sample_rate_hz = 1e6;
    // Physical clock-offset bound for the preamble rate fit, in ppm.
    // 0 disables the clamp (offline tests use synthetic drifts of 1000+ ppm);
    // live hardware TCXOs stay within ~+/-20 ppm, so noise-driven slope fits
    // beyond the bound get clamped instead of warping the payload resample.
    double max_rate_ppm = 0.0;
};

inline uint32_t chipCount(const LoraParams& p) {
    return 1u << p.spreading_factor;
}

inline uint32_t osFactor(const LoraParams& p) {
    return static_cast<uint32_t>(p.sample_rate_hz / p.bandwidth_hz + 0.5);
}

inline uint32_t samplesPerSymbol(const LoraParams& p) {
    return chipCount(p) * osFactor(p);
}

// Sync word nibbles as modulated chip values.
inline uint16_t syncChip0(const LoraParams& p) {
    return static_cast<uint16_t>(((p.sync_word >> 4) & 0x0F) << 3);
}

inline uint16_t syncChip1(const LoraParams& p) {
    return static_cast<uint16_t>((p.sync_word & 0x0F) << 3);
}

}  // namespace phy::lora
