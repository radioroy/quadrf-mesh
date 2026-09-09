#pragma once

#include <phy/lora/params.hpp>

#include <array>
#include <cctype>
#include <string>
#include <utility>

namespace phy::lora {

// Meshtastic modem presets.
// All use sync word 0x2B, 16-symbol preamble, explicit header, CRC on.
enum class MeshtasticPreset {
    kShortTurbo,   // BW 500, SF 7, CR 4/5 (default)
    kShortFast,    // BW 250, SF 7, CR 4/5
};

inline constexpr std::array<MeshtasticPreset, 2> kMeshtasticPresets = {
    MeshtasticPreset::kShortTurbo,
    MeshtasticPreset::kShortFast,
};

inline const char* meshtasticPresetKey(MeshtasticPreset preset) {
    switch (preset) {
        case MeshtasticPreset::kShortTurbo:
            return "shortturbo";
        case MeshtasticPreset::kShortFast:
            return "shortfast";
    }
    return "shortturbo";
}

inline const char* meshtasticModemPresetName(MeshtasticPreset preset) {
    switch (preset) {
        case MeshtasticPreset::kShortTurbo:
            return "SHORT_TURBO";
        case MeshtasticPreset::kShortFast:
            return "SHORT_FAST";
    }
    return "SHORT_TURBO";
}

inline bool parseMeshtasticPreset(const std::string& name, MeshtasticPreset& out) {
    std::string key;
    key.reserve(name.size());
    for (unsigned char c : name) {
        if (c == '-' || c == '_' || c == ' ') {
            continue;
        }
        key.push_back(static_cast<char>(std::tolower(c)));
    }
    static const std::pair<const char*, MeshtasticPreset> table[] = {
        {"shortturbo", MeshtasticPreset::kShortTurbo},
        {"shortfast", MeshtasticPreset::kShortFast},
    };
    for (const auto& [k, preset] : table) {
        if (key == k) {
            out = preset;
            return true;
        }
    }
    return false;
}

inline LoraParams meshtasticParams(MeshtasticPreset preset, double sample_rate_hz = 1e6) {
    LoraParams p;
    p.sync_word = 0x2B;
    p.preamble_len = 16;
    p.has_crc = true;
    p.sample_rate_hz = sample_rate_hz;
    p.max_rate_ppm = 15.0;
    switch (preset) {
        case MeshtasticPreset::kShortTurbo:
            p.bandwidth_hz = 500e3; p.spreading_factor = 7; p.cr = 1;
            break;
        case MeshtasticPreset::kShortFast:
            p.bandwidth_hz = 250e3; p.spreading_factor = 7; p.cr = 1;
            break;
    }
    // SX126x auto-LDRO rule: enable reduced-rate payload blocks when
    // symbol duration >= 16.384 ms.
    const double t_sym = static_cast<double>(1u << p.spreading_factor) / p.bandwidth_hz;
    p.ldro = t_sym >= 16.384e-3;
    return p;
}

inline bool parseMeshtasticPreset(const std::string& name, LoraParams& out,
                                  double sample_rate_hz = 1e6) {
    MeshtasticPreset preset;
    if (!parseMeshtasticPreset(name, preset)) {
        return false;
    }
    out = meshtasticParams(preset, sample_rate_hz);
    return true;
}

}  // namespace phy::lora
