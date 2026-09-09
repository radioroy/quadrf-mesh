#pragma once

// Minimal protobuf wire encode/decode for PhoneAPI bridge fields,
// without requiring external code generation.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace phy::pb {

enum class WireType : uint8_t { kVarint = 0, kFixed64 = 1, kLength = 2, kFixed32 = 5 };

inline constexpr uint32_t tag(uint32_t field, WireType wt) {
    return (field << 3) | static_cast<uint32_t>(wt);
}

void writeVarint(std::vector<uint8_t>& out, uint64_t v);
void writeTag(std::vector<uint8_t>& out, uint32_t field, WireType wt);
void writeU32(std::vector<uint8_t>& out, uint32_t field, uint32_t v);
void writeU64(std::vector<uint8_t>& out, uint32_t field, uint64_t v);
void writeBool(std::vector<uint8_t>& out, uint32_t field, bool v);
void writeFixed32(std::vector<uint8_t>& out, uint32_t field, uint32_t v);
void writeFloat(std::vector<uint8_t>& out, uint32_t field, float v);
void writeBytes(std::vector<uint8_t>& out, uint32_t field, const uint8_t* data, size_t len,
                bool force = false);
void writeBytes(std::vector<uint8_t>& out, uint32_t field, const std::vector<uint8_t>& data,
                bool force = false);
void writeString(std::vector<uint8_t>& out, uint32_t field, const std::string& s);
void writeSubmessage(std::vector<uint8_t>& out, uint32_t field, const std::vector<uint8_t>& msg,
                     bool force = false);

struct Field {
    uint32_t number = 0;
    WireType type = WireType::kVarint;
    uint64_t varint = 0;
    uint32_t fixed32 = 0;
    uint64_t fixed64 = 0;
    const uint8_t* data = nullptr;
    size_t len = 0;
};

// Walk raw protobuf bytes. Returns false on truncated / invalid wire.
bool parseFields(const uint8_t* data, size_t len, std::vector<Field>& out);
bool parseFields(const std::vector<uint8_t>& data, std::vector<Field>& out);

bool readVarint(const uint8_t*& p, const uint8_t* end, uint64_t& v);

}  // namespace phy::pb
