#include <phy/stream/framer.hpp>

#include <cstring>

namespace phy::stream {

bool frame(const uint8_t* payload, size_t len, std::vector<uint8_t>& out) {
    if (payload == nullptr && len != 0) {
        return false;
    }
    if (len > kMaxPayload) {
        return false;
    }
    out.resize(kHeaderLen + len);
    out[0] = kStart1;
    out[1] = kStart2;
    out[2] = static_cast<uint8_t>((len >> 8) & 0xFF);
    out[3] = static_cast<uint8_t>(len & 0xFF);
    if (len != 0) {
        std::memcpy(out.data() + kHeaderLen, payload, len);
    }
    return true;
}

void Deframer::reset() {
    state_ = State::kScanStart1;
    len_ = 0;
    buf_.clear();
}

size_t Deframer::feed(const uint8_t* data, size_t n, std::vector<std::vector<uint8_t>>& out) {
    if (data == nullptr || n == 0) {
        return 0;
    }
    size_t produced = 0;
    for (size_t i = 0; i < n; ++i) {
        const uint8_t b = data[i];
        switch (state_) {
            case State::kScanStart1:
                if (b == kStart1) {
                    state_ = State::kExpectStart2;
                }
                break;
            case State::kExpectStart2:
                if (b == kStart2) {
                    state_ = State::kLenHi;
                } else if (b == kStart1) {
                    // stay — wake / repeated START1
                } else {
                    state_ = State::kScanStart1;
                }
                break;
            case State::kLenHi:
                len_ = static_cast<uint16_t>(static_cast<uint16_t>(b) << 8);
                state_ = State::kLenLo;
                break;
            case State::kLenLo: {
                len_ = static_cast<uint16_t>(len_ | b);
                if (len_ > kMaxPayload) {
                    ++frames_rejected_;
                    state_ = State::kScanStart1;
                    len_ = 0;
                } else if (len_ == 0) {
                    out.emplace_back();
                    ++frames_ok_;
                    ++produced;
                    state_ = State::kScanStart1;
                } else {
                    buf_.clear();
                    buf_.reserve(len_);
                    state_ = State::kPayload;
                }
                break;
            }
            case State::kPayload:
                buf_.push_back(b);
                if (buf_.size() >= len_) {
                    out.push_back(std::move(buf_));
                    buf_.clear();
                    ++frames_ok_;
                    ++produced;
                    state_ = State::kScanStart1;
                    len_ = 0;
                }
                break;
        }
    }
    return produced;
}

}  // namespace phy::stream
