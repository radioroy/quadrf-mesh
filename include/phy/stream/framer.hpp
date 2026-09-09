#pragma once

// Meshtastic StreamAPI framing: 0x94 0xC3 + BE16 length + protobuf payload.
// Max payload 512 (MAX_TO_FROM_RADIO_SIZE). Resyncs on corrupt headers the
// same way firmware StreamAPI.cpp does. No protobuf knowledge here.

#include <cstddef>
#include <cstdint>
#include <vector>

namespace phy::stream {

inline constexpr uint8_t kStart1 = 0x94;
inline constexpr uint8_t kStart2 = 0xC3;
inline constexpr size_t kHeaderLen = 4;
inline constexpr size_t kMaxPayload = 512;

// Encode one protobuf payload into a framed StreamAPI packet.
// Returns false if payload exceeds kMaxPayload.
bool frame(const uint8_t* payload, size_t len, std::vector<uint8_t>& out);
inline bool frame(const std::vector<uint8_t>& payload, std::vector<uint8_t>& out) {
    return frame(payload.data(), payload.size(), out);
}

// Incremental byte deframer. Feed arbitrary chunks; completed payloads are
// appended to `out`. Bytes that never match a frame are discarded (they may
// be legacy debug console text on a real serial link).
class Deframer {
public:
    // Returns number of complete frames appended to out this call.
    size_t feed(const uint8_t* data, size_t n, std::vector<std::vector<uint8_t>>& out);
    size_t feed(const std::vector<uint8_t>& data, std::vector<std::vector<uint8_t>>& out) {
        return feed(data.data(), data.size(), out);
    }

    void reset();
    size_t framesOk() const { return frames_ok_; }
    size_t framesRejected() const { return frames_rejected_; }

private:
    enum class State : uint8_t {
        kScanStart1,
        kExpectStart2,
        kLenHi,
        kLenLo,
        kPayload,
    };

    State state_ = State::kScanStart1;
    uint16_t len_ = 0;
    std::vector<uint8_t> buf_;
    size_t frames_ok_ = 0;
    size_t frames_rejected_ = 0;
};

}  // namespace phy::stream
