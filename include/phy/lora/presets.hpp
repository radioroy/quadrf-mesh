#pragma once

#include <phy/lora/params.hpp>

#include <array>
#include <cctype>
#include <cstdint>
#include <string>
#include <utility>

namespace phy::lora {

// Meshtastic modem presets (firmware v2.7 modemPresetToParams, non-wide
// regions). All use sync word 0x2B, 16-symbol preamble, explicit header,
// CRC on. VERY_LONG_SLOW is deprecated upstream (2.5) and v2.7 falls back
// to LONG_FAST parameters for it; this table keeps its last definition.
//
// The enum value is also the Air-IPC SetModem preset id; append only.
enum class MeshtasticPreset : uint8_t {
    kShortTurbo = 0,    // BW 500,  SF 7,  CR 4/5 (default)
    kShortFast = 1,     // BW 250,  SF 7,  CR 4/5
    kShortSlow = 2,     // BW 250,  SF 8,  CR 4/5
    kMediumFast = 3,    // BW 250,  SF 9,  CR 4/5
    kMediumSlow = 4,    // BW 250,  SF 10, CR 4/5
    kLongFast = 5,      // BW 250,  SF 11, CR 4/5
    kLongModerate = 6,  // BW 125,  SF 11, CR 4/8, LDRO
    kLongSlow = 7,      // BW 125,  SF 12, CR 4/8, LDRO
    kVeryLongSlow = 8,  // BW 62.5, SF 12, CR 4/8, LDRO
    kLongTurbo = 9,     // BW 500,  SF 11, CR 4/8
};

inline constexpr std::array<MeshtasticPreset, 10> kMeshtasticPresets = {
    MeshtasticPreset::kShortTurbo,   MeshtasticPreset::kShortFast,
    MeshtasticPreset::kShortSlow,    MeshtasticPreset::kMediumFast,
    MeshtasticPreset::kMediumSlow,   MeshtasticPreset::kLongFast,
    MeshtasticPreset::kLongModerate, MeshtasticPreset::kLongSlow,
    MeshtasticPreset::kVeryLongSlow, MeshtasticPreset::kLongTurbo,
};

struct MeshtasticPresetInfo {
    MeshtasticPreset preset;
    const char* key;        // CLI / config spelling
    const char* modem_name; // Meshtastic lora.modem_preset
    double bandwidth_hz;
    uint8_t sf;
    uint8_t cr;             // 1..4 -> 4/5..4/8
};

inline constexpr std::array<MeshtasticPresetInfo, 10> kMeshtasticPresetInfo = {{
    {MeshtasticPreset::kShortTurbo, "shortturbo", "SHORT_TURBO", 500e3, 7, 1},
    {MeshtasticPreset::kShortFast, "shortfast", "SHORT_FAST", 250e3, 7, 1},
    {MeshtasticPreset::kShortSlow, "shortslow", "SHORT_SLOW", 250e3, 8, 1},
    {MeshtasticPreset::kMediumFast, "mediumfast", "MEDIUM_FAST", 250e3, 9, 1},
    {MeshtasticPreset::kMediumSlow, "mediumslow", "MEDIUM_SLOW", 250e3, 10, 1},
    {MeshtasticPreset::kLongFast, "longfast", "LONG_FAST", 250e3, 11, 1},
    {MeshtasticPreset::kLongModerate, "longmoderate", "LONG_MODERATE", 125e3, 11, 4},
    {MeshtasticPreset::kLongSlow, "longslow", "LONG_SLOW", 125e3, 12, 4},
    {MeshtasticPreset::kVeryLongSlow, "verylongslow", "VERY_LONG_SLOW", 62.5e3, 12, 4},
    {MeshtasticPreset::kLongTurbo, "longturbo", "LONG_TURBO", 500e3, 11, 4},
}};

inline const MeshtasticPresetInfo& meshtasticPresetInfo(MeshtasticPreset preset) {
    for (const auto& info : kMeshtasticPresetInfo) {
        if (info.preset == preset) {
            return info;
        }
    }
    return kMeshtasticPresetInfo[0];
}

inline bool meshtasticPresetFromId(uint8_t id, MeshtasticPreset& out) {
    if (id >= kMeshtasticPresetInfo.size()) {
        return false;
    }
    out = static_cast<MeshtasticPreset>(id);
    return true;
}

inline const char* meshtasticPresetKey(MeshtasticPreset preset) {
    return meshtasticPresetInfo(preset).key;
}

inline const char* meshtasticModemPresetName(MeshtasticPreset preset) {
    return meshtasticPresetInfo(preset).modem_name;
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
    for (const auto& info : kMeshtasticPresetInfo) {
        if (key == info.key) {
            out = info.preset;
            return true;
        }
    }
    return false;
}

// RX baseband rate for a preset: 2x BW. The receiver's coherent two-segment
// fold and the timing/rotation loops were tuned at os = 2; higher os only
// adds images of DDC-filtered noise to the power fold.
inline double meshtasticRxRate(const LoraParams& p) {
    return 2.0 * p.bandwidth_hz;
}

inline LoraParams meshtasticParams(MeshtasticPreset preset, double sample_rate_hz = 1e6) {
    const MeshtasticPresetInfo& info = meshtasticPresetInfo(preset);
    LoraParams p;
    p.sync_word = 0x2B;
    p.preamble_len = 16;
    p.has_crc = true;
    p.sample_rate_hz = sample_rate_hz;
    p.max_rate_ppm = 15.0;
    // QuadRF pairs read 13-16 kHz apart at 5.8 GHz (~2.7 ppm combined).
    p.max_cfo_hz = 20e3;
    // Unit-to-unit crystal wander, CW captures quadrf-2 <-> quadrf-3:
    // ~800 Hz RMS, 0.8 kHz p-p in 16 ms, 2.5-4 kHz p-p in 150 ms.
    p.wander_rate_hz_per_s = 75e3;
    p.wander_cap_hz = 750.0;
    p.wander_irw_q = 2e12;
    p.bandwidth_hz = info.bandwidth_hz;
    p.spreading_factor = info.sf;
    p.cr = info.cr;
    // SX126x auto-LDRO rule: enable reduced-rate payload blocks when
    // symbol duration >= 16.384 ms (SF11/125k, SF12/125k, SF12/62.5k).
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
