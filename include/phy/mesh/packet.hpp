#pragma once

// Meshtastic on-air packet header (Layer 1) and the hop/relay helpers that
// sit on top of a decoded LoRa PHY payload.
//
// Wire layout matches firmware PacketHeader / mesh-algo docs: 16 fixed bytes
// (to, from, id, flags, channel, next_hop, relay_node), then an opaque
// encrypted SubPacket blob. Kept independent of radio and DSP layers.

#include <cstddef>
#include <cstdint>
#include <vector>

namespace phy::mesh {

inline constexpr size_t kHeaderLength = 16;
inline constexpr size_t kMaxPayloadLength = 255;  // SX12xx LoRa max
inline constexpr size_t kMaxEncryptedLength = kMaxPayloadLength - kHeaderLength;

inline constexpr uint32_t kBroadcastNode = 0xFFFFFFFFu;
inline constexpr uint8_t kHopMax = 7;
inline constexpr uint8_t kHopReliable = 3;
inline constexpr uint8_t kNoNextHop = 0;
inline constexpr uint8_t kNoRelayNode = 0;

// Default LongFast channel hash (hash of channel name + default PSK).
inline constexpr uint8_t kDefaultChannelHash = 8;

inline constexpr uint8_t kFlagsHopLimitMask = 0x07;
inline constexpr uint8_t kFlagsWantAckMask = 0x08;
inline constexpr uint8_t kFlagsViaMqttMask = 0x10;
inline constexpr uint8_t kFlagsHopStartMask = 0xE0;
inline constexpr uint8_t kFlagsHopStartShift = 5;

struct PacketHeader {
    uint32_t to = kBroadcastNode;
    uint32_t from = 0;
    uint32_t id = 0;
    uint8_t hop_limit = 0;
    uint8_t hop_start = 0;
    bool want_ack = false;
    bool via_mqtt = false;
    uint8_t channel = kDefaultChannelHash;
    uint8_t next_hop = kNoNextHop;
    uint8_t relay_node = kNoRelayNode;
};

struct Packet {
    PacketHeader header;
    std::vector<uint8_t> encrypted;  // opaque SubPacket ciphertext
};

inline uint8_t nodeIdByte(uint32_t node) {
    return static_cast<uint8_t>(node & 0xFF);
}

inline bool isBroadcast(uint32_t to) {
    return to == kBroadcastNode;
}

inline uint8_t packFlags(const PacketHeader& h) {
    uint8_t flags = static_cast<uint8_t>(h.hop_limit & kFlagsHopLimitMask);
    if (h.want_ack) {
        flags = static_cast<uint8_t>(flags | kFlagsWantAckMask);
    }
    if (h.via_mqtt) {
        flags = static_cast<uint8_t>(flags | kFlagsViaMqttMask);
    }
    flags = static_cast<uint8_t>(
        flags | ((h.hop_start << kFlagsHopStartShift) & kFlagsHopStartMask));
    return flags;
}

inline void unpackFlags(uint8_t flags, PacketHeader& h) {
    h.hop_limit = static_cast<uint8_t>(flags & kFlagsHopLimitMask);
    h.want_ack = (flags & kFlagsWantAckMask) != 0;
    h.via_mqtt = (flags & kFlagsViaMqttMask) != 0;
    h.hop_start = static_cast<uint8_t>((flags & kFlagsHopStartMask) >> kFlagsHopStartShift);
}

// Little-endian 16-byte header <-> structured fields.
void packHeader(const PacketHeader& h, uint8_t out[kHeaderLength]);
bool unpackHeader(const uint8_t* data, size_t len, PacketHeader& out);

// Full air frame: header || encrypted. Returns false if too short / too long.
bool packPacket(const Packet& pkt, std::vector<uint8_t>& out);
bool unpackPacket(const uint8_t* data, size_t len, Packet& out);

// Managed-flood / next-hop relay decision (firmware FloodingRouter /
// NextHopRouter core, without role/favorite special cases):
//   - not addressed to us, not originated by us
//   - hop_limit > 0
//   - next_hop is unset, or matches our node-id LSB
bool shouldRelay(const PacketHeader& h, uint32_t our_node);

// Prepare a copy for rebroadcast: decrement hop_limit, stamp relay_node.
// Returns false if shouldRelay would reject the packet.
bool prepareRelay(Packet& pkt, uint32_t our_node);

}  // namespace phy::mesh
