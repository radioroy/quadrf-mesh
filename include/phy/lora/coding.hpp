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
    // Soft decode only. Per nibble, margin = ML log-likelihood of the best
    // codeword minus the runner-up. The CRC only XORs the last two payload
    // bytes in, so equal errors in a tail nibble and its CRC partner nibble
    // pass; this is the min over the 4 pairs of max(margin, partner margin).
    double tail_margin = 1e300;
};

// Full decode from raw demodulated chirp values (argmax bins, 0 .. 2^sf-1).
DecodeResult decodeFrame(const LoraParams& p, const std::vector<uint16_t>& raw_values);

// Soft-decision input: one row of 2^sf folded bin magnitudes (sqrt of bin
// power) per data symbol, row-major, indexed by chip value exactly like
// raw_values (raw_values[s] is the argmax of row s). `tone_amp` and
// `noise_pow` are the per-frame tone amplitude and per-bin noise power in
// the same units (tone_amp^2 ~ peak bin power above the floor).
struct SoftSymbols {
    const float* mags = nullptr;
    size_t n_syms = 0;
    double tone_amp = 0.0;
    double noise_pow = 0.0;
};

// Per-frame tone amplitude / bin noise power from the stored rows: noise is
// the mean off-lobe (peak +-2 excluded) power, amplitude from the mean peak
// power above it.
void estimateSoftScale(const float* mags, size_t n_syms, uint32_t n_chips, double& tone_amp,
                       double& noise_pow);

// Bit LLRs (log P(1)/P(0)) of one symbol's gray word, MSB first, sf_app
// bits (sf - 2 for reduced-rate blocks). Non-coherent tone likelihood
// ln I0(2 a m_v / sigma^2) per chip value, summed exactly (log-sum-exp) over
// the values carrying each bit. `shift` reads value u from bin u - shift,
// the soft analogue of adding `shift` to the hard value.
void symbolLlrs(const float* row, uint8_t sf, bool reduced, double tone_amp, double noise_pow,
                int shift, double* llr_out);

// ML nibble over the 16 Hamming codewords given cw_len codeword-bit LLRs
// (MSB first, same layout as hammingDecode's input). `margin`, if given,
// receives best minus second-best codeword log-likelihood.
uint8_t hammingDecodeSoft(const double* llr, uint8_t cr_app, double* margin = nullptr);

// Soft counterpart of decodeFrame: same block structure, header handling and
// CRC check, with LLR deinterleaving and ML Hamming decoding. Decodes as far
// as the rows allow (8 rows are enough for the header).
DecodeResult decodeFrameSoft(const LoraParams& p, const SoftSymbols& soft, int shift = 0);

}  // namespace phy::lora
