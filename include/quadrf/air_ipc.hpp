// SPDX-FileCopyrightText: 2026 Roy C. Gross
// SPDX-License-Identifier: Apache-2.0
#pragma once

// QuadRF Air-IPC protocol, version 1.
//
// This is the single implementation shared by quadrf-lora-phy and integrations.
// It deliberately depends only on the C++ standard library so an adapter can
// copy this header into another source tree without importing the PHY.
//
// Wire format:
//   0x51 0x46 | version=1 | type | body_length (BE16) | body
//
// Body fields are little-endian:
//   TxEnqueue:  requested_frequency_hz (u64) | Meshtastic air frame
//               (16..255 bytes). The PHY never retunes from this field: zero
//               accepts its configured center; a nonzero mismatch is rejected.
//   RxIndicate: snr_centidb (i16) | rssi_dbm (i16) | cfo_hz (i32)
//               | rate_ppm_centi (i32) | frequency_hz (u64)
//               | Meshtastic air frame (16..255 bytes)

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

namespace quadrf::air_ipc {

inline constexpr uint8_t kMagic0 = 0x51;  // 'Q'
inline constexpr uint8_t kMagic1 = 0x46;  // 'F'
inline constexpr uint8_t kProtocolVersion = 1;
// Compatibility name retained for the first public protocol revision.
inline constexpr uint8_t kVersion = kProtocolVersion;

inline constexpr size_t kFrameHeaderLen = 6;
inline constexpr size_t kMinAirLen = 16;
inline constexpr size_t kMaxAirLen = 255;
inline constexpr size_t kTxMetaLen = 8;
inline constexpr size_t kRxMetaLen = 20;
inline constexpr size_t kMaxBodyLen = kRxMetaLen + kMaxAirLen;
inline constexpr size_t kMaxFrameLen = kFrameHeaderLen + kMaxBodyLen;

enum class MsgType : uint8_t {
    kTxEnqueue = 1,
    kRxIndicate = 2,
};

struct TxEnqueue {
    // Safety assertion only. 0 accepts the PHY configuration; a nonzero value
    // must equal the PHY's configured center and never causes a retune.
    uint64_t freq_hz = 0;
    std::vector<uint8_t> air;
};

struct RxIndicate {
    float snr_db = 0.0f;
    int16_t rssi_dbm = 0;
    int32_t cfo_hz = 0;
    float rate_ppm = 0.0f;
    uint64_t freq_hz = 0;
    std::vector<uint8_t> air;
};

inline constexpr uint64_t kFrequencyQuantumHz = 1000;

inline constexpr uint64_t quantizeFrequencyHz(uint64_t frequency_hz) {
    constexpr uint64_t half_quantum = kFrequencyQuantumHz / 2;
    if (frequency_hz > std::numeric_limits<uint64_t>::max() - half_quantum) {
        return (std::numeric_limits<uint64_t>::max() / kFrequencyQuantumHz) *
               kFrequencyQuantumHz;
    }
    return ((frequency_hz + half_quantum) / kFrequencyQuantumHz) *
           kFrequencyQuantumHz;
}

inline uint64_t frequencyMHzToQuantizedHz(double frequency_mhz) {
    if (!(frequency_mhz > 0.0)) {
        return 0;
    }
    const double frequency_hz = frequency_mhz * 1000000.0;
    if (frequency_hz > static_cast<double>(std::numeric_limits<uint64_t>::max() - 500)) {
        return 0;
    }
    return quantizeFrequencyHz(static_cast<uint64_t>(frequency_hz + 0.5));
}

inline constexpr bool frequencyRequestAccepted(uint64_t requested_hz,
                                                uint64_t configured_hz) {
    // Dynamic GUI / Meshtastic tuning: allow any requested frequency to pass.
    // Modulation occurs at whichever center frequency the local LO is tuned to.
    (void)requested_hz;
    (void)configured_hz;
    return true;
}

struct TelemetryRecord {
    uint64_t timestamp_ms = 0;
    float snr_db = 0.0f;
    int16_t rssi_dbm = 0;
    int32_t cfo_hz = 0;
    float rate_ppm = 0.0f;
    uint16_t preamble_symbols = 0;
    uint16_t payload_len = 0;
    uint32_t from_node = 0;
    uint32_t to_node = 0;
    uint32_t packet_id = 0;
};

namespace detail {

inline void writeLe16(uint8_t* output, uint16_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
}

inline uint16_t readLe16(const uint8_t* input) {
    return static_cast<uint16_t>(static_cast<uint16_t>(input[0]) |
                                 (static_cast<uint16_t>(input[1]) << 8));
}

inline void writeLe32(uint8_t* output, uint32_t value) {
    for (int offset = 0; offset < 4; ++offset) {
        output[offset] = static_cast<uint8_t>(value >> (8 * offset));
    }
}

inline uint32_t readLe32(const uint8_t* input) {
    uint32_t value = 0;
    for (int offset = 0; offset < 4; ++offset) {
        value |= static_cast<uint32_t>(input[offset]) << (8 * offset);
    }
    return value;
}

inline void writeLe64(uint8_t* output, uint64_t value) {
    for (int offset = 0; offset < 8; ++offset) {
        output[offset] = static_cast<uint8_t>(value >> (8 * offset));
    }
}

inline uint64_t readLe64(const uint8_t* input) {
    uint64_t value = 0;
    for (int offset = 0; offset < 8; ++offset) {
        value |= static_cast<uint64_t>(input[offset]) << (8 * offset);
    }
    return value;
}

inline bool airLengthIsValid(size_t length) {
    return length >= kMinAirLen && length <= kMaxAirLen;
}

inline bool encodeFrame(MsgType type, const uint8_t* body, size_t body_length,
                        std::vector<uint8_t>& output) {
    if (body_length == 0 || body_length > kMaxBodyLen || body == nullptr) {
        return false;
    }
    output.resize(kFrameHeaderLen + body_length);
    output[0] = kMagic0;
    output[1] = kMagic1;
    output[2] = kProtocolVersion;
    output[3] = static_cast<uint8_t>(type);
    output[4] = static_cast<uint8_t>((body_length >> 8) & 0xff);
    output[5] = static_cast<uint8_t>(body_length & 0xff);
    std::memcpy(output.data() + kFrameHeaderLen, body, body_length);
    return true;
}

}  // namespace detail

inline int16_t snrToCenti(float snr_db) {
    const float scaled = snr_db * 100.0f;
    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
    }
    return static_cast<int16_t>(scaled);
}

inline float snrFromCenti(int16_t centi) {
    return static_cast<float>(centi) * 0.01f;
}

inline int32_t rateToCenti(float rate_ppm) {
    const float scaled = rate_ppm * 100.0f;
    if (scaled > 2147483647.0f) {
        return 2147483647;
    }
    if (scaled < -2147483648.0f) {
        return (-2147483647 - 1);
    }
    return static_cast<int32_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

inline float rateFromCenti(int32_t centi) {
    return static_cast<float>(centi) * 0.01f;
}

inline bool encodeTx(const TxEnqueue& message, std::vector<uint8_t>& output) {
    if (!detail::airLengthIsValid(message.air.size())) {
        return false;
    }
    std::vector<uint8_t> body(kTxMetaLen + message.air.size());
    detail::writeLe64(body.data(), message.freq_hz);
    std::memcpy(body.data() + kTxMetaLen, message.air.data(), message.air.size());
    return detail::encodeFrame(MsgType::kTxEnqueue, body.data(), body.size(), output);
}

inline bool encodeRx(const RxIndicate& message, std::vector<uint8_t>& output) {
    if (!detail::airLengthIsValid(message.air.size())) {
        return false;
    }
    std::vector<uint8_t> body(kRxMetaLen + message.air.size());
    detail::writeLe16(body.data(), static_cast<uint16_t>(snrToCenti(message.snr_db)));
    detail::writeLe16(body.data() + 2, static_cast<uint16_t>(message.rssi_dbm));
    detail::writeLe32(body.data() + 4, static_cast<uint32_t>(message.cfo_hz));
    detail::writeLe32(body.data() + 8, static_cast<uint32_t>(rateToCenti(message.rate_ppm)));
    detail::writeLe64(body.data() + 12, message.freq_hz);
    std::memcpy(body.data() + kRxMetaLen, message.air.data(), message.air.size());
    return detail::encodeFrame(MsgType::kRxIndicate, body.data(), body.size(), output);
}

inline bool decodeTx(const uint8_t* body, size_t length, TxEnqueue& output) {
    if (body == nullptr || length < kTxMetaLen) {
        return false;
    }
    const size_t air_length = length - kTxMetaLen;
    if (!detail::airLengthIsValid(air_length)) {
        return false;
    }
    output.freq_hz = detail::readLe64(body);
    output.air.assign(body + kTxMetaLen, body + length);
    return true;
}

inline bool decodeRx(const uint8_t* body, size_t length, RxIndicate& output) {
    if (body == nullptr || length < kRxMetaLen) {
        return false;
    }
    const size_t air_length = length - kRxMetaLen;
    if (!detail::airLengthIsValid(air_length)) {
        return false;
    }
    output.snr_db = snrFromCenti(static_cast<int16_t>(detail::readLe16(body)));
    output.rssi_dbm = static_cast<int16_t>(detail::readLe16(body + 2));
    output.cfo_hz = static_cast<int32_t>(detail::readLe32(body + 4));
    output.rate_ppm = rateFromCenti(static_cast<int32_t>(detail::readLe32(body + 8)));
    output.freq_hz = detail::readLe64(body + 12);
    output.air.assign(body + kRxMetaLen, body + length);
    return true;
}

inline bool decodeTx(const std::vector<uint8_t>& body, TxEnqueue& output) {
    return decodeTx(body.data(), body.size(), output);
}

inline bool decodeRx(const std::vector<uint8_t>& body, RxIndicate& output) {
    return decodeRx(body.data(), body.size(), output);
}

class Deframer {
  public:
    struct Frame {
        MsgType type = MsgType::kTxEnqueue;
        std::vector<uint8_t> body;
    };

    size_t feed(const uint8_t* data, size_t length, std::vector<Frame>& output) {
        if (data == nullptr || length == 0) {
            return 0;
        }
        size_t produced = 0;
        for (size_t index = 0; index < length; ++index) {
            const uint8_t byte = data[index];
            switch (state_) {
                case State::kScanMagic0:
                    if (byte == kMagic0) {
                        state_ = State::kExpectMagic1;
                    }
                    break;
                case State::kExpectMagic1:
                    if (byte == kMagic1) {
                        state_ = State::kVersion;
                    } else if (byte != kMagic0) {
                        state_ = State::kScanMagic0;
                    }
                    break;
                case State::kVersion:
                    if (byte != kProtocolVersion) {
                        rejectAndResync(byte);
                    } else {
                        state_ = State::kType;
                    }
                    break;
                case State::kType:
                    if (byte != static_cast<uint8_t>(MsgType::kTxEnqueue) &&
                        byte != static_cast<uint8_t>(MsgType::kRxIndicate)) {
                        rejectAndResync(byte);
                    } else {
                        type_ = static_cast<MsgType>(byte);
                        state_ = State::kLenHi;
                    }
                    break;
                case State::kLenHi:
                    body_length_ = static_cast<uint16_t>(static_cast<uint16_t>(byte) << 8);
                    state_ = State::kLenLo;
                    break;
                case State::kLenLo:
                    body_length_ = static_cast<uint16_t>(body_length_ | byte);
                    if (body_length_ == 0 || body_length_ > kMaxBodyLen) {
                        ++frames_rejected_;
                        state_ = State::kScanMagic0;
                        body_length_ = 0;
                    } else {
                        body_.clear();
                        body_.reserve(body_length_);
                        state_ = State::kBody;
                    }
                    break;
                case State::kBody:
                    body_.push_back(byte);
                    if (body_.size() == body_length_) {
                        output.push_back(Frame{type_, std::move(body_)});
                        body_.clear();
                        ++frames_ok_;
                        ++produced;
                        state_ = State::kScanMagic0;
                        body_length_ = 0;
                    }
                    break;
            }
        }
        return produced;
    }

    size_t feed(const std::vector<uint8_t>& data, std::vector<Frame>& output) {
        return feed(data.data(), data.size(), output);
    }

    void reset() {
        state_ = State::kScanMagic0;
        type_ = MsgType::kTxEnqueue;
        body_length_ = 0;
        body_.clear();
    }

    size_t framesOk() const { return frames_ok_; }
    size_t framesRejected() const { return frames_rejected_; }

  private:
    enum class State : uint8_t {
        kScanMagic0,
        kExpectMagic1,
        kVersion,
        kType,
        kLenHi,
        kLenLo,
        kBody,
    };

    void rejectAndResync(uint8_t byte) {
        ++frames_rejected_;
        state_ = (byte == kMagic0) ? State::kExpectMagic1 : State::kScanMagic0;
    }

    State state_ = State::kScanMagic0;
    MsgType type_ = MsgType::kTxEnqueue;
    uint16_t body_length_ = 0;
    std::vector<uint8_t> body_;
    size_t frames_ok_ = 0;
    size_t frames_rejected_ = 0;
};

}  // namespace quadrf::air_ipc
