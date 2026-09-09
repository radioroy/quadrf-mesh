#include <phy/bridge/session.hpp>

#include <phy/bridge/crypto.hpp>
#include <phy/bridge/pb_wire.hpp>

#include <cstdio>
#include <cstring>

namespace phy::bridge {
namespace {

// Config.lora (field 6) wrapping LoRaConfig with use_preset + SHORT_TURBO.
std::vector<uint8_t> wrapConfigLoRa(const std::vector<uint8_t>& lora) {
    std::vector<uint8_t> out;
    pb::writeSubmessage(out, 6, lora);
    return out;
}

std::vector<uint8_t> wrapConfigDevice(const std::vector<uint8_t>& device) {
    std::vector<uint8_t> out;
    pb::writeSubmessage(out, 1, device);
    return out;
}

}  // namespace

Session::Session(SessionConfig cfg) : cfg_(std::move(cfg)) {
    if (!expandPsk(cfg_.psk_shorthand.data(), cfg_.psk_shorthand.size(), channel_key_)) {
        channel_key_.assign(kDefaultPsk, kDefaultPsk + 16);
    }
    // Hash uses the *settings* psk bytes for 16/32, but expanded key for the
    // default shorthand. Matching LongFast: name xor key → 8.
    if (cfg_.psk_shorthand.size() == 1) {
        channel_hash_ = phy::bridge::channelHash(cfg_.channel_name, channel_key_);
    } else {
        channel_hash_ = phy::bridge::channelHash(cfg_.channel_name, cfg_.psk_shorthand);
    }
}

std::string Session::nodeIdString() const {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "!%08x", cfg_.node_num);
    return buf;
}

size_t Session::fromRadioQueued() const {
    std::lock_guard<std::mutex> lk(mu_);
    return from_q_.size();
}

void Session::enqueueFrom(FromRadio fr) {
    fr.id = from_seq_++;
    std::vector<uint8_t> bytes = encodeFromRadio(fr);
    std::lock_guard<std::mutex> lk(mu_);
    from_q_.push_back(std::move(bytes));
}

bool Session::popFromRadio(std::vector<uint8_t>& out) {
    std::lock_guard<std::mutex> lk(mu_);
    if (from_q_.empty()) {
        return false;
    }
    out = std::move(from_q_.front());
    from_q_.pop_front();
    return true;
}

std::vector<uint8_t> Session::makeLoRaConfig() const {
    std::vector<uint8_t> lora;
    pb::writeBool(lora, 1, true);  // use_preset
    // modem_preset SHORT_TURBO = 8
    pb::writeTag(lora, 2, pb::WireType::kVarint);
    pb::writeVarint(lora, 8);
    pb::writeU32(lora, 8, 3);      // hop_limit
    pb::writeBool(lora, 9, true);  // tx_enabled
    return wrapConfigLoRa(lora);
}

std::vector<uint8_t> Session::makeDeviceConfig() const {
    std::vector<uint8_t> device;
    // role CLIENT = 0 (default)
    pb::writeU32(device, 7, 10800);  // node_info_broadcast_secs
    return wrapConfigDevice(device);
}

void Session::emitOwnNodeInfo() {
    FromRadio fr;
    fr.kind = FromRadioKind::kNodeInfo;
    fr.node_info.num = cfg_.node_num;
    fr.node_info.user.id = nodeIdString();
    fr.node_info.user.long_name = cfg_.long_name;
    fr.node_info.user.short_name = cfg_.short_name;
    enqueueFrom(std::move(fr));
}

void Session::startConfig(uint32_t nonce) {
    const bool nodes_only = (nonce == kNonceOnlyNodes);
    const bool config_only = (nonce == kNonceOnlyConfig);

    if (!nodes_only) {
        FromRadio my;
        my.kind = FromRadioKind::kMyInfo;
        my.my_info.my_node_num = cfg_.node_num;
        my.my_info.min_app_version = 30200;
        enqueueFrom(std::move(my));

        emitOwnNodeInfo();

        FromRadio meta;
        meta.kind = FromRadioKind::kMetadata;
        meta.metadata.firmware_version = "quadrf-lora-phy-0.1";
        meta.metadata.device_state_version = 24;
        enqueueFrom(std::move(meta));

        FromRadio ch;
        ch.kind = FromRadioKind::kChannel;
        ch.channel.index = 0;
        ch.channel.role = 1;  // PRIMARY
        ch.channel.settings.name = cfg_.channel_name;
        ch.channel.settings.psk = cfg_.psk_shorthand;
        enqueueFrom(std::move(ch));

        FromRadio device;
        device.kind = FromRadioKind::kConfig;
        device.config_bytes = makeDeviceConfig();
        enqueueFrom(std::move(device));

        FromRadio lora;
        lora.kind = FromRadioKind::kConfig;
        lora.config_bytes = makeLoRaConfig();
        enqueueFrom(std::move(lora));
    } else {
        // Stage 2: NodeDB only (own node again is fine / expected)
        FromRadio my;
        my.kind = FromRadioKind::kMyInfo;
        my.my_info.my_node_num = cfg_.node_num;
        enqueueFrom(std::move(my));
        emitOwnNodeInfo();
    }

    (void)config_only;
    FromRadio done;
    done.kind = FromRadioKind::kConfigComplete;
    done.config_complete_id = nonce;
    enqueueFrom(std::move(done));
}

void Session::handleToRadio(const uint8_t* payload, size_t len,
                            std::vector<std::vector<uint8_t>>& tx_air) {
    ToRadio msg;
    if (!decodeToRadio(payload, len, msg)) {
        return;
    }
    switch (msg.kind) {
        case ToRadioKind::kWantConfig:
            startConfig(msg.want_config_id);
            break;
        case ToRadioKind::kHeartbeat:
            // no-op; connection stays alive
            break;
        case ToRadioKind::kDisconnect:
            break;
        case ToRadioKind::kPacket: {
            // Assign id / from if the host left them blank.
            MeshPacketPb pkt = msg.packet;
            if (pkt.from == 0) {
                pkt.from = cfg_.node_num;
            }
            if (pkt.id == 0) {
                pkt.id = from_seq_++;  // reuse seq as packet id source
            }
            if (pkt.hop_limit == 0) {
                pkt.hop_limit = 3;
            }
            phy::mesh::Packet air;
            if (!meshPacketToAir(pkt, channel_key_, channel_hash_, cfg_.node_num, air)) {
                break;
            }
            std::vector<uint8_t> bytes;
            if (!phy::mesh::packPacket(air, bytes)) {
                break;
            }
            tx_air.push_back(std::move(bytes));
            ++tx_enqueued_;

            FromRadio qs;
            qs.kind = FromRadioKind::kQueueStatus;
            qs.free = cfg_.tx_queue_maxlen > 1 ? cfg_.tx_queue_maxlen - 1 : 0;
            qs.maxlen = cfg_.tx_queue_maxlen;
            qs.mesh_packet_id = pkt.id;
            enqueueFrom(std::move(qs));
            break;
        }
        default:
            break;
    }
}

void Session::handleAirFrame(const uint8_t* payload, size_t len, float snr_db) {
    phy::mesh::Packet air;
    if (!phy::mesh::unpackPacket(payload, len, air)) {
        return;
    }
    // Retain self-frames for loopback verification; filter empty encrypted payloads.
    if (air.encrypted.empty()) {
        return;
    }
    MeshPacketPb pkt;
    if (!airToMeshPacket(air, channel_key_, pkt)) {
        return;
    }
    pkt.rx_snr = snr_db;
    FromRadio fr;
    fr.kind = FromRadioKind::kPacket;
    fr.packet = std::move(pkt);
    enqueueFrom(std::move(fr));
    ++rx_delivered_;
}

}  // namespace phy::bridge
