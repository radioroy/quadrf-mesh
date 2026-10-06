#pragma once

#include <cstdint>
#include <vector>

// lora_beacon test payload (32 B): 'Q' 'B' seq_hi seq_lo + 28 B LCG keyed by seq.
inline std::vector<uint8_t> beaconPayload(uint16_t seq) {
    std::vector<uint8_t> p(32);
    p[0] = 'Q';
    p[1] = 'B';
    p[2] = static_cast<uint8_t>(seq >> 8);
    p[3] = static_cast<uint8_t>(seq);
    uint32_t s = 0x9E3779B9u ^ (static_cast<uint32_t>(seq) * 2654435761u);
    for (size_t i = 4; i < p.size(); ++i) {
        s = s * 1664525u + 1013904223u;
        p[i] = static_cast<uint8_t>(s >> 24);
    }
    return p;
}

inline bool beaconSeq(const std::vector<uint8_t>& p, uint16_t& seq) {
    if (p.size() != 32 || p[0] != 'Q' || p[1] != 'B') {
        return false;
    }
    seq = static_cast<uint16_t>((p[2] << 8) | p[3]);
    return p == beaconPayload(seq);
}
