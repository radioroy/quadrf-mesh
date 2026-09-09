#pragma once

// Meshtastic channel AES-CTR (firmware CryptoEngine). PSK expand + encrypt
// Data protobuf <-> MeshPacket.encrypted. No PKI — channel traffic only.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace phy::bridge {

// Default LongFast PSK (psk shorthand byte 0x01).
inline constexpr uint8_t kDefaultPsk[16] = {0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
                                            0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01};

// Expand 0/1/16/32-byte PSK shorthand into the actual AES key.
// empty / [0] => empty (no crypto); [1] => default; [n>1] => default with last += n-1.
bool expandPsk(const uint8_t* psk, size_t len, std::vector<uint8_t>& key_out);

inline uint8_t xorHash(const uint8_t* data, size_t len) {
    uint8_t h = 0;
    for (size_t i = 0; i < len; ++i) {
        h = static_cast<uint8_t>(h ^ data[i]);
    }
    return h;
}

// channel.hash = xor(name) ^ xor(expanded_key), matching firmware
// generateHash behavior for expanded channel keys and UTF-8 channel names.
uint8_t channelHash(const std::string& name, const std::vector<uint8_t>& key);

// AES-CTR in-place (encrypt == decrypt). nonce = packet_id u64 LE || from u32 LE || 0.
bool aesCtr(const std::vector<uint8_t>& key, uint32_t packet_id, uint32_t from_node,
            uint8_t* data, size_t len);

bool aesCtr(const std::vector<uint8_t>& key, uint32_t packet_id, uint32_t from_node,
            std::vector<uint8_t>& data);

}  // namespace phy::bridge
