#pragma once

// PhoneAPI envelopes (ToRadio / FromRadio / MeshPacket / Data) data
// structures and protobuf serialization codecs.

#include <phy/mesh/packet.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace phy::bridge {

inline constexpr uint32_t kPortTextMessage = 1;
inline constexpr uint32_t kPortNodeInfo = 4;
inline constexpr uint32_t kPortRouting = 5;
inline constexpr uint32_t kPortAdmin = 6;

inline constexpr uint32_t kNonceOnlyConfig = 69420;
inline constexpr uint32_t kNonceOnlyNodes = 69421;

struct Data {
    uint32_t portnum = 0;
    std::vector<uint8_t> payload;
    bool want_response = false;
    uint32_t dest = 0;
    uint32_t source = 0;
    uint32_t request_id = 0;
    uint32_t reply_id = 0;
};

struct MeshPacketPb {
    uint32_t from = 0;
    uint32_t to = phy::mesh::kBroadcastNode;
    uint32_t channel = 0;  // index on phone API; hash on air when encrypted
    std::optional<Data> decoded;
    std::vector<uint8_t> encrypted;
    uint32_t id = 0;
    uint32_t rx_time = 0;
    float rx_snr = 0.0f;
    uint32_t hop_limit = 3;
    bool want_ack = false;
    uint32_t hop_start = 0;
    bool via_mqtt = false;
    uint32_t next_hop = 0;
    uint32_t relay_node = 0;
    uint32_t rx_rssi = 0;
};

enum class ToRadioKind { kNone, kPacket, kWantConfig, kDisconnect, kHeartbeat };

struct ToRadio {
    ToRadioKind kind = ToRadioKind::kNone;
    MeshPacketPb packet;
    uint32_t want_config_id = 0;
    bool disconnect = false;
};

enum class FromRadioKind {
    kNone,
    kPacket,
    kMyInfo,
    kNodeInfo,
    kConfig,
    kConfigComplete,
    kRebooted,
    kModuleConfig,
    kChannel,
    kQueueStatus,
    kMetadata,
};

struct MyNodeInfo {
    uint32_t my_node_num = 0;
    uint32_t reboot_count = 0;
    uint32_t min_app_version = 30200;  // python client ~2.x
};

struct DeviceMetadata {
    std::string firmware_version = "quadrf-lora-phy-0.1";
    uint32_t device_state_version = 24;
    bool can_shutdown = false;
    bool has_wifi = false;
    bool has_bluetooth = false;
    // Role CLIENT = 0; HwModel UNSET = 0
    uint32_t role = 0;
    uint32_t hw_model = 0;
};

struct ChannelSettings {
    std::vector<uint8_t> psk;  // shorthand or full key
    std::string name = "LongFast";
};

struct Channel {
    int32_t index = 0;
    ChannelSettings settings;
    int32_t role = 1;  // PRIMARY
};

struct NodeUser {
    std::string id;  // "!aabbccdd"
    std::string long_name = "QuadRF";
    std::string short_name = "QRF";
    uint32_t hw_model = 0;
    uint32_t role = 0;
};

struct NodeInfo {
    uint32_t num = 0;
    NodeUser user;
    float snr = 0.0f;
    uint32_t last_heard = 0;
};

struct FromRadio {
    uint32_t id = 0;
    FromRadioKind kind = FromRadioKind::kNone;
    MeshPacketPb packet;
    MyNodeInfo my_info;
    NodeInfo node_info;
    // Opaque already-encoded Config / ModuleConfig / Channel submessages so
    // the daemon can emit curated stubs without modeling every field.
    std::vector<uint8_t> config_bytes;
    std::vector<uint8_t> module_config_bytes;
    Channel channel;
    DeviceMetadata metadata;
    uint32_t config_complete_id = 0;
    bool rebooted = false;
    uint32_t free = 0;       // QueueStatus.free
    uint32_t maxlen = 16;    // QueueStatus.maxlen
    uint32_t res = 0;        // QueueStatus.res
    uint32_t mesh_packet_id = 0;
};

std::vector<uint8_t> encodeData(const Data& d);
bool decodeData(const uint8_t* data, size_t len, Data& out);

std::vector<uint8_t> encodeMeshPacket(const MeshPacketPb& p);
bool decodeMeshPacket(const uint8_t* data, size_t len, MeshPacketPb& out);

std::vector<uint8_t> encodeToRadio(const ToRadio& m);
bool decodeToRadio(const uint8_t* data, size_t len, ToRadio& out);

std::vector<uint8_t> encodeFromRadio(const FromRadio& m);

// Map phone MeshPacket (+ channel key) <-> on-air phy::mesh::Packet bytes.
bool meshPacketToAir(const MeshPacketPb& in, const std::vector<uint8_t>& channel_key,
                     uint8_t channel_hash, uint32_t our_node, phy::mesh::Packet& out);
bool airToMeshPacket(const phy::mesh::Packet& in, const std::vector<uint8_t>& channel_key,
                     MeshPacketPb& out);

}  // namespace phy::bridge
