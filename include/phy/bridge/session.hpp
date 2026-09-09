#pragma once

// Standalone PhoneAPI session implementation. Bridges ToRadio.packet to air
// frames and RX air frames to FromRadio.packet.

#include <phy/bridge/phone_api.hpp>
#include <phy/lora/params.hpp>
#include <phy/mesh/packet.hpp>

#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace phy::bridge {

struct SessionConfig {
    uint32_t node_num = 0xE58F0001;  // QuadRF-looking id
    std::string long_name = "QuadRF";
    std::string short_name = "QRF";
    // Primary channel: default PSK shorthand (0x01) + LongFast name → hash 8
    std::vector<uint8_t> psk_shorthand = {0x01};
    std::string channel_name = "LongFast";
    phy::lora::LoraParams lora;
    uint32_t tx_queue_maxlen = 16;
};

class Session {
public:
    explicit Session(SessionConfig cfg);

    // Handle one decoded ToRadio protobuf payload. Air frames to TX are
    // appended to `tx_air`; FromRadio replies are queued internally.
    void handleToRadio(const uint8_t* payload, size_t len, std::vector<std::vector<uint8_t>>& tx_air);

    // A decoded LoRa payload (header||encrypted) becomes a FromRadio.packet.
    void handleAirFrame(const uint8_t* payload, size_t len, float snr_db = 0.0f);

    // Pop next FromRadio protobuf (unframed). False when empty.
    bool popFromRadio(std::vector<uint8_t>& out);

    uint32_t nodeNum() const { return cfg_.node_num; }
    uint8_t channelHash() const { return channel_hash_; }
    const std::vector<uint8_t>& channelKey() const { return channel_key_; }
    size_t fromRadioQueued() const;
    size_t txEnqueued() const { return tx_enqueued_; }
    size_t rxDelivered() const { return rx_delivered_; }

private:
    void enqueueFrom(FromRadio fr);
    void startConfig(uint32_t nonce);
    void emitOwnNodeInfo();
    std::vector<uint8_t> makeLoRaConfig() const;
    std::vector<uint8_t> makeDeviceConfig() const;
    std::string nodeIdString() const;

    SessionConfig cfg_;
    std::vector<uint8_t> channel_key_;
    uint8_t channel_hash_ = 0;
    uint32_t from_seq_ = 1;
    size_t tx_enqueued_ = 0;
    size_t rx_delivered_ = 0;

    mutable std::mutex mu_;
    std::deque<std::vector<uint8_t>> from_q_;
};

}  // namespace phy::bridge
