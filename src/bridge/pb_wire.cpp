#include <phy/bridge/pb_wire.hpp>

#include <cstring>

namespace phy::pb {

void writeVarint(std::vector<uint8_t>& out, uint64_t v) {
    while (v >= 0x80) {
        out.push_back(static_cast<uint8_t>((v & 0x7F) | 0x80));
        v >>= 7;
    }
    out.push_back(static_cast<uint8_t>(v));
}

void writeTag(std::vector<uint8_t>& out, uint32_t field, WireType wt) {
    writeVarint(out, tag(field, wt));
}

void writeU32(std::vector<uint8_t>& out, uint32_t field, uint32_t v) {
    if (v == 0) {
        return;  // proto3 default
    }
    writeTag(out, field, WireType::kVarint);
    writeVarint(out, v);
}

void writeU64(std::vector<uint8_t>& out, uint32_t field, uint64_t v) {
    if (v == 0) {
        return;
    }
    writeTag(out, field, WireType::kVarint);
    writeVarint(out, v);
}

void writeBool(std::vector<uint8_t>& out, uint32_t field, bool v) {
    if (!v) {
        return;
    }
    writeTag(out, field, WireType::kVarint);
    writeVarint(out, 1);
}

void writeFixed32(std::vector<uint8_t>& out, uint32_t field, uint32_t v) {
    if (v == 0) {
        return;
    }
    writeTag(out, field, WireType::kFixed32);
    out.push_back(static_cast<uint8_t>(v));
    out.push_back(static_cast<uint8_t>(v >> 8));
    out.push_back(static_cast<uint8_t>(v >> 16));
    out.push_back(static_cast<uint8_t>(v >> 24));
}

void writeFloat(std::vector<uint8_t>& out, uint32_t field, float v) {
    if (v == 0.0f) {
        return;
    }
    uint32_t bits = 0;
    static_assert(sizeof(float) == 4, "IEEE float");
    std::memcpy(&bits, &v, 4);
    writeTag(out, field, WireType::kFixed32);
    out.push_back(static_cast<uint8_t>(bits));
    out.push_back(static_cast<uint8_t>(bits >> 8));
    out.push_back(static_cast<uint8_t>(bits >> 16));
    out.push_back(static_cast<uint8_t>(bits >> 24));
}

void writeBytes(std::vector<uint8_t>& out, uint32_t field, const uint8_t* data, size_t len,
                bool force) {
    if (!force && (data == nullptr || len == 0)) {
        return;
    }
    writeTag(out, field, WireType::kLength);
    writeVarint(out, len);
    if (len != 0 && data != nullptr) {
        out.insert(out.end(), data, data + len);
    }
}

void writeBytes(std::vector<uint8_t>& out, uint32_t field, const std::vector<uint8_t>& data,
                bool force) {
    writeBytes(out, field, data.data(), data.size(), force);
}

void writeString(std::vector<uint8_t>& out, uint32_t field, const std::string& s) {
    writeBytes(out, field, reinterpret_cast<const uint8_t*>(s.data()), s.size());
}

void writeSubmessage(std::vector<uint8_t>& out, uint32_t field, const std::vector<uint8_t>& msg,
                     bool force) {
    writeBytes(out, field, msg.data(), msg.size(), force);
}

bool readVarint(const uint8_t*& p, const uint8_t* end, uint64_t& v) {
    v = 0;
    int shift = 0;
    while (p < end && shift <= 63) {
        const uint8_t b = *p++;
        v |= static_cast<uint64_t>(b & 0x7F) << shift;
        if ((b & 0x80) == 0) {
            return true;
        }
        shift += 7;
    }
    return false;
}

bool parseFields(const uint8_t* data, size_t len, std::vector<Field>& out) {
    out.clear();
    if (data == nullptr && len != 0) {
        return false;
    }
    const uint8_t* p = data;
    const uint8_t* end = data + len;
    while (p < end) {
        uint64_t raw_tag = 0;
        if (!readVarint(p, end, raw_tag)) {
            return false;
        }
        Field f;
        f.number = static_cast<uint32_t>(raw_tag >> 3);
        f.type = static_cast<WireType>(raw_tag & 7);
        switch (f.type) {
            case WireType::kVarint:
                if (!readVarint(p, end, f.varint)) {
                    return false;
                }
                break;
            case WireType::kFixed64:
                if (end - p < 8) {
                    return false;
                }
                f.fixed64 = static_cast<uint64_t>(p[0]) | (static_cast<uint64_t>(p[1]) << 8) |
                            (static_cast<uint64_t>(p[2]) << 16) | (static_cast<uint64_t>(p[3]) << 24) |
                            (static_cast<uint64_t>(p[4]) << 32) | (static_cast<uint64_t>(p[5]) << 40) |
                            (static_cast<uint64_t>(p[6]) << 48) | (static_cast<uint64_t>(p[7]) << 56);
                p += 8;
                break;
            case WireType::kLength: {
                uint64_t slen = 0;
                if (!readVarint(p, end, slen) || slen > static_cast<uint64_t>(end - p)) {
                    return false;
                }
                f.data = p;
                f.len = static_cast<size_t>(slen);
                p += static_cast<size_t>(slen);
                break;
            }
            case WireType::kFixed32:
                if (end - p < 4) {
                    return false;
                }
                f.fixed32 = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                            (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
                p += 4;
                break;
            default:
                return false;
        }
        out.push_back(f);
    }
    return true;
}

bool parseFields(const std::vector<uint8_t>& data, std::vector<Field>& out) {
    return parseFields(data.data(), data.size(), out);
}

}  // namespace phy::pb
