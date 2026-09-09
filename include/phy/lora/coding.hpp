#pragma once

// LoRa bit-level coding chain (SX127x conventions, adapted from gr-lora_sdr
// [Tapparel et al., EPFL] and gr-lora [Robyns et al.]):
// whitening -> explicit header -> payload CRC -> Hamming -> diagonal
// interleaver -> gray demap -> chirp values, and the inverse path.
//
// Everything here is plain integer math on small arrays, decoupled from DSP
// kernels.

#include <phy/lora/params.hpp>

#include <array>
#include <cstdint>
#include <vector>

namespace phy::lora {

extern const uint8_t kWhiteningSeq[255];

// CRC16 poly 0x1021, init 0x0000, MSB-first (no reflection).
uint16_t crc16(uint16_t crc, uint8_t byte);

// LoRa payload CRC: CRC16 over the first len-2 bytes, then XORed with the
// last two payload bytes.
uint16_t payloadCrc(const uint8_t* payload, size_t len);

// Explicit header: 5 nibbles (len_hi, len_lo, cr<<1|crc, chk_hi, chk_lo).
std::array<uint8_t, 5> buildHeader(uint8_t payload_len, uint8_t cr, bool has_crc);

struct HeaderInfo {
    bool valid = false;
    uint8_t payload_len = 0;
    uint8_t cr = 0;
    bool has_crc = false;
};
HeaderInfo parseHeader(const uint8_t* nibbles);

// Hamming: nibble -> codeword of (4 + cr_app) bits, cr_app in 1..4.
// Bit layout (MSB first): [b0 b1 b2 b3 p0 p1 p2 p3] truncated, where b0 is
// the nibble LSB. cr_app==1 uses a single parity bit instead.
uint8_t hammingEncode(uint8_t nibble, uint8_t cr_app);
uint8_t hammingDecode(uint8_t codeword, uint8_t cr_app);

// gray demap (TX): gray -> binary; gray map (RX): binary -> gray
uint32_t grayDecode(uint32_t g, uint8_t bits);
uint32_t grayEncode(uint32_t b);

// Diagonal interleaver for one block: sf_app codewords of cw_len bits each
// -> cw_len symbols. Reduced-rate blocks (header / LDRO) append a parity
// bit and a zero so symbols are always sf bits wide.
void interleaveBlock(const uint8_t* cw, uint8_t sf, uint8_t sf_app, uint8_t cw_len,
                     bool reduced, uint16_t* out_symbols);
void deinterleaveBlock(const uint16_t* symbols, uint8_t sf_app, uint8_t cw_len,
                       uint8_t* out_cw);

// Total number of data symbols (header block included) for a frame.
size_t frameSymbolCount(const LoraParams& p, size_t payload_len);

// Full encode: payload bytes -> chirp values (0 .. 2^sf-1), ready for the
// modulator. Explicit header, whitened payload, optional CRC.
std::vector<uint16_t> encodeFrame(const LoraParams& p, const std::vector<uint8_t>& payload);

struct DecodeResult {
    HeaderInfo header;
    bool crc_ok = false;
    uint16_t crc_expected = 0;
    uint16_t crc_received = 0;
    std::vector<uint8_t> payload;
    size_t symbols_consumed = 0;
};

// Full decode from raw demodulated chirp values (argmax bins, 0 .. 2^sf-1).
DecodeResult decodeFrame(const LoraParams& p, const std::vector<uint16_t>& raw_values);

}  // namespace phy::lora
