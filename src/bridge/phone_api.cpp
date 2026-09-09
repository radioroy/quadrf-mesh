#include <phy/bridge/phone_api.hpp>

#include <phy/bridge/crypto.hpp>
#include <phy/bridge/pb_wire.hpp>

#include <cstdio>
#include <cstring>
#include <ctime>

namespace phy::bridge {
namespace {

using phy::pb::Field;
using phy::pb::WireType;

std::vector<uint8_t> encodeMyInfo(const MyNodeInfo& m) {
    std::vector<uint8_t> out;
    pb::writeU32(out, 1, m.my_node_num);
    pb::writeU32(out, 8, m.reboot_count);
    pb::writeU32(out, 11, m.min_app_version);
    return out;
}

std::vector<uint8_t> encodeMetadata(const DeviceMetadata& m) {
    std::vector<uint8_t> out;
    pb::writeString(out, 1, m.firmware_version);
    pb::writeU32(out, 2, m.device_state_version);
    pb::writeBool(out, 3, m.can_shutdown);
    pb::writeBool(out, 4, m.has_wifi);
    pb::writeBool(out, 5, m.has_bluetooth);
    pb::writeU32(out, 7, m.role);
    pb::writeU32(out, 9, m.hw_model);
    return out;
}

std::vector<uint8_t> encodeUser(const NodeUser& u) {
    std::vector<uint8_t> out;
    pb::writeString(out, 1, u.id);
    pb::writeString(out, 2, u.long_name);
    pb::writeString(out, 3, u.short_name);
    pb::writeU32(out, 5, u.hw_model);
    pb::writeU32(out, 7, u.role);
    return out;
}

std::vector<uint8_t> encodeNodeInfo(const NodeInfo& n) {
    std::vector<uint8_t> out;
    pb::writeU32(out, 1, n.num);
    pb::writeSubmessage(out, 2, encodeUser(n.user));
    pb::writeFloat(out, 4, n.snr);
    pb::writeFixed32(out, 5, n.last_heard);
    return out;
}

std::vector<uint8_t> encodeChannelSettings(const ChannelSettings& s) {
    std::vector<uint8_t> out;
    pb::writeBytes(out, 1, s.psk);
    pb::writeString(out, 2, s.name);
    return out;
}

std::vector<uint8_t> encodeChannel(const Channel& c) {
    std::vector<uint8_t> out;
    // index 0 is omitted by proto3 defaults; clients fix up — but set it when
    // non-zero. For primary we still write settings + role.
    if (c.index != 0) {
        // sint32 encoded as zigzag varint; for small non-neg ints same as u32
        pb::writeU32(out, 1, static_cast<uint32_t>(c.index));
    }
    pb::writeSubmessage(out, 2, encodeChannelSettings(c.settings));
    pb::writeU32(out, 3, static_cast<uint32_t>(c.role));
    return out;
}

std::vector<uint8_t> encodeQueueStatus(const FromRadio& m) {
    std::vector<uint8_t> out;
    pb::writeU32(out, 1, m.res);
    pb::writeU32(out, 2, m.free);
    pb::writeU32(out, 3, m.maxlen);
    pb::writeU32(out, 4, m.mesh_packet_id);
    return out;
}

}  // namespace

std::vector<uint8_t> encodeData(const Data& d) {
    std::vector<uint8_t> out;
    pb::writeU32(out, 1, d.portnum);
    pb::writeBytes(out, 2, d.payload);
    pb::writeBool(out, 3, d.want_response);
    pb::writeFixed32(out, 4, d.dest);
    pb::writeFixed32(out, 5, d.source);
    pb::writeFixed32(out, 6, d.request_id);
    pb::writeFixed32(out, 7, d.reply_id);
    return out;
}

bool decodeData(const uint8_t* data, size_t len, Data& out) {
    out = Data{};
    std::vector<Field> fields;
    if (!pb::parseFields(data, len, fields)) {
        return false;
    }
    for (const Field& f : fields) {
        switch (f.number) {
            case 1:
                out.portnum = static_cast<uint32_t>(f.varint);
                break;
            case 2:
                out.payload.assign(f.data, f.data + f.len);
                break;
            case 3:
                out.want_response = f.varint != 0;
                break;
            case 4:
                out.dest = f.fixed32;
                break;
            case 5:
                out.source = f.fixed32;
                break;
            case 6:
                out.request_id = f.fixed32;
                break;
            case 7:
                out.reply_id = f.fixed32;
                break;
            default:
                break;
        }
    }
    return true;
}

std::vector<uint8_t> encodeMeshPacket(const MeshPacketPb& p) {
    std::vector<uint8_t> out;
    pb::writeFixed32(out, 1, p.from);
    pb::writeFixed32(out, 2, p.to);
    pb::writeU32(out, 3, p.channel);
    if (p.decoded) {
        pb::writeSubmessage(out, 4, encodeData(*p.decoded));
    } else if (!p.encrypted.empty()) {
        pb::writeBytes(out, 5, p.encrypted);
    }
    pb::writeFixed32(out, 6, p.id);
    pb::writeFixed32(out, 7, p.rx_time);
    pb::writeFloat(out, 8, p.rx_snr);
    pb::writeU32(out, 9, p.hop_limit);
    pb::writeBool(out, 10, p.want_ack);
    pb::writeU32(out, 12, p.rx_rssi);
    // Meshtastic 2.5/2.6 protobuf field numbers: via_mqtt=14, hop_start=15, next_hop=18,
    // relay_node=19 (field 13 is Delayed).
    pb::writeBool(out, 14, p.via_mqtt);
    pb::writeU32(out, 15, p.hop_start);
    pb::writeU32(out, 18, p.next_hop);
    pb::writeU32(out, 19, p.relay_node);
    return out;
}

bool decodeMeshPacket(const uint8_t* data, size_t len, MeshPacketPb& out) {
    out = MeshPacketPb{};
    std::vector<Field> fields;
    if (!pb::parseFields(data, len, fields)) {
        return false;
    }
    for (const Field& f : fields) {
        switch (f.number) {
            case 1:
                out.from = f.fixed32;
                break;
            case 2:
                out.to = f.fixed32;
                break;
            case 3:
                out.channel = static_cast<uint32_t>(f.varint);
                break;
            case 4: {
                Data d;
                if (!decodeData(f.data, f.len, d)) {
                    return false;
                }
                out.decoded = d;
                break;
            }
            case 5:
                out.encrypted.assign(f.data, f.data + f.len);
                break;
            case 6:
                out.id = f.fixed32;
                break;
            case 7:
                out.rx_time = f.fixed32;
                break;
            case 8: {
                float snr = 0.0f;
                std::memcpy(&snr, &f.fixed32, 4);
                out.rx_snr = snr;
                break;
            }
            case 9:
                out.hop_limit = static_cast<uint32_t>(f.varint);
                break;
            case 10:
                out.want_ack = f.varint != 0;
                break;
            case 12:
                out.rx_rssi = static_cast<uint32_t>(f.varint);
                break;
            case 14:
                out.via_mqtt = f.varint != 0;
                break;
            case 15:
                out.hop_start = static_cast<uint32_t>(f.varint);
                break;
            case 18:
                out.next_hop = static_cast<uint32_t>(f.varint);
                break;
            case 19:
                out.relay_node = static_cast<uint32_t>(f.varint);
                break;
            default:
                break;
        }
    }
    return out.from != 0 || out.id != 0 || out.decoded || !out.encrypted.empty();
}

std::vector<uint8_t> encodeToRadio(const ToRadio& m) {
    std::vector<uint8_t> out;
    switch (m.kind) {
        case ToRadioKind::kPacket:
            pb::writeSubmessage(out, 1, encodeMeshPacket(m.packet));
            break;
        case ToRadioKind::kWantConfig:
            pb::writeU32(out, 3, m.want_config_id);
            break;
        case ToRadioKind::kDisconnect:
            pb::writeBool(out, 4, true);
            break;
        case ToRadioKind::kHeartbeat:
            // Heartbeat is an empty submessage at field 7
            pb::writeSubmessage(out, 7, {}, /*force=*/true);
            break;
        default:
            break;
    }
    return out;
}

bool decodeToRadio(const uint8_t* data, size_t len, ToRadio& out) {
    out = ToRadio{};
    std::vector<Field> fields;
    if (!pb::parseFields(data, len, fields)) {
        return false;
    }
    for (const Field& f : fields) {
        switch (f.number) {
            case 1:
                out.kind = ToRadioKind::kPacket;
                if (!decodeMeshPacket(f.data, f.len, out.packet)) {
                    return false;
                }
                break;
            case 3:
                out.kind = ToRadioKind::kWantConfig;
                out.want_config_id = static_cast<uint32_t>(f.varint);
                break;
            case 4:
                out.kind = ToRadioKind::kDisconnect;
                out.disconnect = f.varint != 0;
                break;
            case 7:
                out.kind = ToRadioKind::kHeartbeat;
                break;
            default:
                // Unhandled message types (e.g. MQTT / XMODEM packets)
                break;
        }
    }
    return out.kind != ToRadioKind::kNone;
}

std::vector<uint8_t> encodeFromRadio(const FromRadio& m) {
    std::vector<uint8_t> out;
    pb::writeU32(out, 1, m.id);
    switch (m.kind) {
        case FromRadioKind::kPacket:
            pb::writeSubmessage(out, 2, encodeMeshPacket(m.packet));
            break;
        case FromRadioKind::kMyInfo:
            pb::writeSubmessage(out, 3, encodeMyInfo(m.my_info));
            break;
        case FromRadioKind::kNodeInfo:
            pb::writeSubmessage(out, 4, encodeNodeInfo(m.node_info));
            break;
        case FromRadioKind::kConfig:
            pb::writeBytes(out, 5, m.config_bytes);
            break;
        case FromRadioKind::kConfigComplete:
            pb::writeU32(out, 7, m.config_complete_id);
            break;
        case FromRadioKind::kRebooted:
            pb::writeBool(out, 8, true);
            break;
        case FromRadioKind::kModuleConfig:
            pb::writeBytes(out, 9, m.module_config_bytes);
            break;
        case FromRadioKind::kChannel:
            pb::writeSubmessage(out, 10, encodeChannel(m.channel));
            break;
        case FromRadioKind::kQueueStatus:
            pb::writeSubmessage(out, 11, encodeQueueStatus(m));
            break;
        case FromRadioKind::kMetadata:
            pb::writeSubmessage(out, 13, encodeMetadata(m.metadata));
            break;
        default:
            break;
    }
    return out;
}

bool meshPacketToAir(const MeshPacketPb& in, const std::vector<uint8_t>& channel_key,
                     uint8_t channel_hash, uint32_t our_node, phy::mesh::Packet& out) {
    out = phy::mesh::Packet{};
    out.header.to = in.to ? in.to : phy::mesh::kBroadcastNode;
    out.header.from = in.from ? in.from : our_node;
    out.header.id = in.id;
    out.header.hop_limit = static_cast<uint8_t>(in.hop_limit & 0x7);
    out.header.hop_start =
        static_cast<uint8_t>((in.hop_start ? in.hop_start : in.hop_limit) & 0x7);
    out.header.want_ack = in.want_ack;
    out.header.via_mqtt = in.via_mqtt;
    out.header.channel = channel_hash;
    out.header.next_hop = static_cast<uint8_t>(in.next_hop & 0xFF);
    out.header.relay_node = static_cast<uint8_t>(in.relay_node & 0xFF);

    if (!in.encrypted.empty()) {
        out.encrypted = in.encrypted;
        return out.header.from != 0 && out.header.id != 0;
    }
    if (!in.decoded) {
        return false;
    }
    out.encrypted = encodeData(*in.decoded);
    if (!aesCtr(channel_key, out.header.id, out.header.from, out.encrypted)) {
        return false;
    }
    return out.header.from != 0 && out.header.id != 0 &&
           out.encrypted.size() <= phy::mesh::kMaxEncryptedLength;
}

bool airToMeshPacket(const phy::mesh::Packet& in, const std::vector<uint8_t>& channel_key,
                     MeshPacketPb& out) {
    out = MeshPacketPb{};
    out.from = in.header.from;
    out.to = in.header.to;
    out.channel = 0;  // phone index for primary
    out.id = in.header.id;
    out.hop_limit = in.header.hop_limit;
    out.hop_start = in.header.hop_start;
    out.want_ack = in.header.want_ack;
    out.via_mqtt = in.header.via_mqtt;
    out.next_hop = in.header.next_hop;
    out.relay_node = in.header.relay_node;
    out.rx_time = static_cast<uint32_t>(std::time(nullptr));

    std::vector<uint8_t> plain = in.encrypted;
    if (!aesCtr(channel_key, in.header.id, in.header.from, plain)) {
        return false;
    }
    Data d;
    if (decodeData(plain.data(), plain.size(), d) && d.portnum != 0) {
        out.decoded = std::move(d);
    } else {
        // leave encrypted for the host if we cannot decode
        out.encrypted = in.encrypted;
    }
    return true;
}

}  // namespace phy::bridge
