#include <phy/mesh/packet.hpp>

#include <cstring>

namespace phy::mesh {
namespace {

void writeLe32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

uint32_t readLe32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

}  // namespace

void packHeader(const PacketHeader& h, uint8_t out[kHeaderLength]) {
    writeLe32(out + 0, h.to);
    writeLe32(out + 4, h.from);
    writeLe32(out + 8, h.id);
    out[12] = packFlags(h);
    out[13] = h.channel;
    out[14] = h.next_hop;
    out[15] = h.relay_node;
}

bool unpackHeader(const uint8_t* data, size_t len, PacketHeader& out) {
    if (data == nullptr || len < kHeaderLength) {
        return false;
    }
    out.to = readLe32(data + 0);
    out.from = readLe32(data + 4);
    out.id = readLe32(data + 8);
    unpackFlags(data[12], out);
    out.channel = data[13];
    out.next_hop = data[14];
    out.relay_node = data[15];
    return out.from != 0;
}

bool packPacket(const Packet& pkt, std::vector<uint8_t>& out) {
    if (pkt.header.from == 0 || pkt.encrypted.size() > kMaxEncryptedLength) {
        return false;
    }
    out.resize(kHeaderLength + pkt.encrypted.size());
    packHeader(pkt.header, out.data());
    if (!pkt.encrypted.empty()) {
        std::memcpy(out.data() + kHeaderLength, pkt.encrypted.data(), pkt.encrypted.size());
    }
    return true;
}

bool unpackPacket(const uint8_t* data, size_t len, Packet& out) {
    if (!unpackHeader(data, len, out.header)) {
        return false;
    }
    out.encrypted.assign(data + kHeaderLength, data + len);
    return out.encrypted.size() <= kMaxEncryptedLength;
}

bool shouldRelay(const PacketHeader& h, uint32_t our_node) {
    if (h.from == 0 || h.from == our_node) {
        return false;
    }
    if (h.to == our_node) {
        return false;
    }
    if (h.hop_limit == 0 || h.id == 0) {
        return false;
    }
    const uint8_t us = nodeIdByte(our_node);
    return h.next_hop == kNoNextHop || h.next_hop == us;
}

bool prepareRelay(Packet& pkt, uint32_t our_node) {
    if (!shouldRelay(pkt.header, our_node)) {
        return false;
    }
    --pkt.header.hop_limit;
    pkt.header.relay_node = nodeIdByte(our_node);
    return true;
}

}  // namespace phy::mesh
