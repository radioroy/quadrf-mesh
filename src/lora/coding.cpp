#include <phy/lora/coding.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace phy::lora {

// SX127x whitening sequence (LFSR x^8+x^6+x^5+x^4+1, seed 0xFF), one byte
// per payload byte. Matches gr-lora_sdr tables.h (Tapparel et al., EPFL),
// derived from gr-lora whitening conventions (Robyns et al.).
const uint8_t kWhiteningSeq[255] = {
    0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE1, 0xC2, 0x85, 0x0B, 0x17, 0x2F, 0x5E, 0xBC, 0x78, 0xF1,
    0xE3, 0xC6, 0x8D, 0x1A, 0x34, 0x68, 0xD0, 0xA0, 0x40, 0x80, 0x01, 0x02, 0x04, 0x08, 0x11,
    0x23, 0x47, 0x8E, 0x1C, 0x38, 0x71, 0xE2, 0xC4, 0x89, 0x12, 0x25, 0x4B, 0x97, 0x2E, 0x5C,
    0xB8, 0x70, 0xE0, 0xC0, 0x81, 0x03, 0x06, 0x0C, 0x19, 0x32, 0x64, 0xC9, 0x92, 0x24, 0x49,
    0x93, 0x26, 0x4D, 0x9B, 0x37, 0x6E, 0xDC, 0xB9, 0x72, 0xE4, 0xC8, 0x90, 0x20, 0x41, 0x82,
    0x05, 0x0A, 0x15, 0x2B, 0x56, 0xAD, 0x5B, 0xB6, 0x6D, 0xDA, 0xB5, 0x6B, 0xD6, 0xAC, 0x59,
    0xB2, 0x65, 0xCB, 0x96, 0x2C, 0x58, 0xB0, 0x61, 0xC3, 0x87, 0x0F, 0x1F, 0x3E, 0x7D, 0xFB,
    0xF6, 0xED, 0xDB, 0xB7, 0x6F, 0xDE, 0xBD, 0x7A, 0xF5, 0xEB, 0xD7, 0xAE, 0x5D, 0xBA, 0x74,
    0xE8, 0xD1, 0xA2, 0x44, 0x88, 0x10, 0x21, 0x43, 0x86, 0x0D, 0x1B, 0x36, 0x6C, 0xD8, 0xB1,
    0x63, 0xC7, 0x8F, 0x1E, 0x3C, 0x79, 0xF3, 0xE7, 0xCE, 0x9C, 0x39, 0x73, 0xE6, 0xCC, 0x98,
    0x31, 0x62, 0xC5, 0x8B, 0x16, 0x2D, 0x5A, 0xB4, 0x69, 0xD2, 0xA4, 0x48, 0x91, 0x22, 0x45,
    0x8A, 0x14, 0x29, 0x52, 0xA5, 0x4A, 0x95, 0x2A, 0x54, 0xA9, 0x53, 0xA7, 0x4E, 0x9D, 0x3B,
    0x77, 0xEE, 0xDD, 0xBB, 0x76, 0xEC, 0xD9, 0xB3, 0x67, 0xCF, 0x9E, 0x3D, 0x7B, 0xF7, 0xEF,
    0xDF, 0xBF, 0x7E, 0xFD, 0xFA, 0xF4, 0xE9, 0xD3, 0xA6, 0x4C, 0x99, 0x33, 0x66, 0xCD, 0x9A,
    0x35, 0x6A, 0xD4, 0xA8, 0x51, 0xA3, 0x46, 0x8C, 0x18, 0x30, 0x60, 0xC1, 0x83, 0x07, 0x0E,
    0x1D, 0x3A, 0x75, 0xEA, 0xD5, 0xAA, 0x55, 0xAB, 0x57, 0xAF, 0x5F, 0xBE, 0x7C, 0xF9, 0xF2,
    0xE5, 0xCA, 0x94, 0x28, 0x50, 0xA1, 0x42, 0x84, 0x09, 0x13, 0x27, 0x4F, 0x9F, 0x3F, 0x7F};

uint16_t crc16(uint16_t crc, uint8_t byte) {
    for (int i = 0; i < 8; ++i) {
        if (((crc & 0x8000) >> 8) ^ (byte & 0x80)) {
            crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
        } else {
            crc = static_cast<uint16_t>(crc << 1);
        }
        byte = static_cast<uint8_t>(byte << 1);
    }
    return crc;
}

uint16_t payloadCrc(const uint8_t* payload, size_t len) {
    uint16_t crc = 0x0000;
    if (len < 2) {
        // degenerate but keep the same structure as the reference
        for (size_t i = 0; i < len; ++i) {
            crc = crc16(crc, payload[i]);
        }
        return crc;
    }
    for (size_t i = 0; i + 2 < len; ++i) {
        crc = crc16(crc, payload[i]);
    }
    crc = static_cast<uint16_t>(crc ^ payload[len - 1] ^ (payload[len - 2] << 8));
    return crc;
}

namespace {

uint8_t headerChecksum(const uint8_t* h) {
    const bool c4 = ((h[0] & 0x8) >> 3) ^ ((h[0] & 0x4) >> 2) ^ ((h[0] & 0x2) >> 1) ^ (h[0] & 0x1);
    const bool c3 = ((h[0] & 0x8) >> 3) ^ ((h[1] & 0x8) >> 3) ^ ((h[1] & 0x4) >> 2) ^
                    ((h[1] & 0x2) >> 1) ^ (h[2] & 0x1);
    const bool c2 = ((h[0] & 0x4) >> 2) ^ ((h[1] & 0x8) >> 3) ^ (h[1] & 0x1) ^
                    ((h[2] & 0x8) >> 3) ^ ((h[2] & 0x2) >> 1);
    const bool c1 = ((h[0] & 0x2) >> 1) ^ ((h[1] & 0x4) >> 2) ^ (h[1] & 0x1) ^
                    ((h[2] & 0x4) >> 2) ^ ((h[2] & 0x2) >> 1) ^ (h[2] & 0x1);
    const bool c0 = (h[0] & 0x1) ^ ((h[1] & 0x2) >> 1) ^ ((h[2] & 0x8) >> 3) ^
                    ((h[2] & 0x4) >> 2) ^ ((h[2] & 0x2) >> 1) ^ (h[2] & 0x1);
    return static_cast<uint8_t>((c4 << 4) | (c3 << 3) | (c2 << 2) | (c1 << 1) | c0);
}

}  // namespace

std::array<uint8_t, 5> buildHeader(uint8_t payload_len, uint8_t cr, bool has_crc) {
    std::array<uint8_t, 5> h{};
    h[0] = static_cast<uint8_t>(payload_len >> 4);
    h[1] = static_cast<uint8_t>(payload_len & 0x0F);
    h[2] = static_cast<uint8_t>((cr << 1) | (has_crc ? 1 : 0));
    const uint8_t chk = headerChecksum(h.data());
    h[3] = static_cast<uint8_t>((chk >> 4) & 0x1);
    h[4] = static_cast<uint8_t>(chk & 0x0F);
    return h;
}

HeaderInfo parseHeader(const uint8_t* n) {
    HeaderInfo info;
    info.payload_len = static_cast<uint8_t>((n[0] << 4) | (n[1] & 0x0F));
    info.has_crc = (n[2] & 0x1) != 0;
    info.cr = static_cast<uint8_t>((n[2] >> 1) & 0x7);
    const uint8_t chk_rx = static_cast<uint8_t>(((n[3] & 0x1) << 4) | (n[4] & 0x0F));
    const uint8_t chk = headerChecksum(n);
    info.valid = (chk_rx == chk) && info.payload_len != 0 && info.cr >= 1 && info.cr <= 4;
    return info;
}

uint8_t hammingEncode(uint8_t nibble, uint8_t cr_app) {
    const bool b0 = nibble & 0x1;
    const bool b1 = (nibble >> 1) & 0x1;
    const bool b2 = (nibble >> 2) & 0x1;
    const bool b3 = (nibble >> 3) & 0x1;
    if (cr_app == 1) {
        const bool p4 = b0 ^ b1 ^ b2 ^ b3;
        return static_cast<uint8_t>((b0 << 4) | (b1 << 3) | (b2 << 2) | (b3 << 1) | p4);
    }
    const bool p0 = b0 ^ b1 ^ b2;
    const bool p1 = b1 ^ b2 ^ b3;
    const bool p2 = b0 ^ b1 ^ b3;
    const bool p3 = b0 ^ b2 ^ b3;
    const uint8_t full = static_cast<uint8_t>((b0 << 7) | (b1 << 6) | (b2 << 5) | (b3 << 4) |
                                              (p0 << 3) | (p1 << 2) | (p2 << 1) | p3);
    return static_cast<uint8_t>(full >> (4 - cr_app));
}

uint8_t hammingDecode(uint8_t codeword, uint8_t cr_app) {
    const uint8_t cw_len = static_cast<uint8_t>(cr_app + 4);
    // c[0] is the MSB of the codeword (= data LSB per the TX layout)
    bool c[8] = {};
    for (int i = 0; i < cw_len; ++i) {
        c[i] = (codeword >> (cw_len - 1 - i)) & 0x1;
    }
    bool d0 = c[0], d1 = c[1], d2 = c[2], d3 = c[3];  // data bits LSB..MSB

    auto popcount = [&]() {
        int cnt = 0;
        for (int i = 0; i < cw_len; ++i) {
            cnt += c[i] ? 1 : 0;
        }
        return cnt;
    };

    switch (cr_app) {
        case 4:
            if (popcount() % 2 == 0) {
                break;  // even parity mismatch pattern: don't attempt correction
            }
            [[fallthrough]];
        case 3: {
            const bool s0 = c[0] ^ c[1] ^ c[2] ^ c[4];
            const bool s1 = c[1] ^ c[2] ^ c[3] ^ c[5];
            const bool s2 = c[0] ^ c[1] ^ c[3] ^ c[6];
            const int syndrome = s0 + (s1 << 1) + (s2 << 2);
            switch (syndrome) {
                case 5: d0 = !d0; break;
                case 7: d1 = !d1; break;
                case 3: d2 = !d2; break;
                case 6: d3 = !d3; break;
                default: break;
            }
            break;
        }
        default:
            break;  // cr 1/2: detection only
    }
    return static_cast<uint8_t>((d3 << 3) | (d2 << 2) | (d1 << 1) | (d0 ? 1 : 0));
}

uint32_t grayDecode(uint32_t g, uint8_t bits) {
    uint32_t b = g;
    for (int j = 1; j < bits; ++j) {
        b ^= g >> j;
    }
    return b;
}

uint32_t grayEncode(uint32_t b) {
    return b ^ (b >> 1);
}

void interleaveBlock(const uint8_t* cw, uint8_t sf, uint8_t sf_app, uint8_t cw_len,
                     bool reduced, uint16_t* out_symbols) {
    for (int i = 0; i < cw_len; ++i) {
        uint32_t sym = 0;
        int ones = 0;
        // bit j (MSB first within the sf_app-wide part)
        for (int j = 0; j < sf_app; ++j) {
            const int src = ((i - j - 1) % sf_app + sf_app) % sf_app;
            const int bit = (cw[src] >> (cw_len - 1 - i)) & 0x1;
            sym = (sym << 1) | bit;
            ones += bit;
        }
        if (reduced) {
            // parity bit then a zero, keeping the symbol sf bits wide
            sym = (sym << 1) | (ones % 2);
            sym <<= (sf - sf_app - 1);
        }
        out_symbols[i] = static_cast<uint16_t>(sym);
    }
}

void deinterleaveBlock(const uint16_t* symbols, uint8_t sf_app, uint8_t cw_len,
                       uint8_t* out_cw) {
    std::fill(out_cw, out_cw + sf_app, 0);
    for (int i = 0; i < cw_len; ++i) {
        for (int j = 0; j < sf_app; ++j) {
            const int bit = (symbols[i] >> (sf_app - 1 - j)) & 0x1;
            const int dst = ((i - j - 1) % sf_app + sf_app) % sf_app;
            out_cw[dst] = static_cast<uint8_t>(out_cw[dst] | (bit << (cw_len - 1 - i)));
        }
    }
}

size_t frameSymbolCount(const LoraParams& p, size_t payload_len) {
    const size_t nibbles = 5 + 2 * payload_len + (p.has_crc ? 4 : 0);
    const int sf = p.spreading_factor;
    const size_t first_block = static_cast<size_t>(sf - 2);
    if (nibbles <= first_block) {
        return 8;
    }
    const size_t sf_app = p.ldro ? sf - 2 : sf;
    const size_t rest = nibbles - first_block;
    const size_t blocks = (rest + sf_app - 1) / sf_app;
    return 8 + blocks * (p.cr + 4);
}

std::vector<uint16_t> encodeFrame(const LoraParams& p, const std::vector<uint8_t>& payload) {
    const uint8_t sf = p.spreading_factor;
    const uint32_t n_chips = chipCount(p);

    // nibble stream: header, whitened payload (low nibble first), CRC
    std::vector<uint8_t> nibbles;
    nibbles.reserve(5 + 2 * payload.size() + 4);

    const auto header = buildHeader(static_cast<uint8_t>(payload.size()), p.cr, p.has_crc);
    nibbles.insert(nibbles.end(), header.begin(), header.end());

    for (size_t i = 0; i < payload.size(); ++i) {
        const uint8_t w = payload[i] ^ kWhiteningSeq[i % sizeof(kWhiteningSeq)];
        nibbles.push_back(w & 0x0F);
        nibbles.push_back(w >> 4);
    }

    if (p.has_crc) {
        const uint16_t crc = payloadCrc(payload.data(), payload.size());
        nibbles.push_back(crc & 0x000F);
        nibbles.push_back((crc & 0x00F0) >> 4);
        nibbles.push_back((crc & 0x0F00) >> 8);
        nibbles.push_back((crc & 0xF000) >> 12);
    }

    std::vector<uint16_t> chips;
    size_t idx = 0;
    bool first_block = true;
    while (idx < nibbles.size()) {
        const bool reduced = first_block || p.ldro;
        const uint8_t cr_app = first_block ? 4 : p.cr;
        const uint8_t cw_len = static_cast<uint8_t>(cr_app + 4);
        const uint8_t sf_app = reduced ? sf - 2 : sf;

        uint8_t cw[16] = {};
        for (int i = 0; i < sf_app; ++i) {
            const uint8_t nib = (idx < nibbles.size()) ? nibbles[idx] : 0;
            cw[i] = hammingEncode(nib, cr_app);
            ++idx;
        }

        uint16_t symbols[16] = {};
        interleaveBlock(cw, sf, sf_app, cw_len, reduced, symbols);

        for (int i = 0; i < cw_len; ++i) {
            const uint32_t chip = (grayDecode(symbols[i], sf) + 1) % n_chips;
            chips.push_back(static_cast<uint16_t>(chip));
        }
        first_block = false;
    }
    return chips;
}

namespace {

// Nibble stream (header + whitened payload + CRC) -> payload bytes, CRC check.
void finishPayload(const std::vector<uint8_t>& nibbles, DecodeResult& result) {
    const size_t len = result.header.payload_len;
    result.payload.resize(len);
    for (size_t i = 0; i < len; ++i) {
        const uint8_t w =
            static_cast<uint8_t>((nibbles[5 + 2 * i] & 0x0F) | (nibbles[5 + 2 * i + 1] << 4));
        result.payload[i] = w ^ kWhiteningSeq[i % sizeof(kWhiteningSeq)];
    }

    if (result.header.has_crc) {
        const size_t c = 5 + 2 * len;
        result.crc_received = static_cast<uint16_t>(
            (nibbles[c] & 0x0F) | (nibbles[c + 1] << 4) | (nibbles[c + 2] << 8) |
            (nibbles[c + 3] << 12));
        result.crc_expected = payloadCrc(result.payload.data(), result.payload.size());
        result.crc_ok = (result.crc_received == result.crc_expected);
    } else {
        result.crc_ok = true;
    }
}

}  // namespace

DecodeResult decodeFrame(const LoraParams& p, const std::vector<uint16_t>& raw_values) {
    DecodeResult result;
    const uint8_t sf = p.spreading_factor;
    const uint32_t n_chips = chipCount(p);

    std::vector<uint8_t> nibbles;
    size_t sym_idx = 0;
    bool first_block = true;
    size_t nibbles_needed = static_cast<size_t>(sf - 2);  // until header decoded

    while (nibbles.size() < nibbles_needed && sym_idx < raw_values.size()) {
        const bool reduced = first_block || p.ldro;
        const uint8_t cr_app = first_block ? 4 : result.header.cr;
        const uint8_t cw_len = static_cast<uint8_t>(cr_app + 4);
        const uint8_t sf_app = reduced ? sf - 2 : sf;

        if (sym_idx + cw_len > raw_values.size()) {
            break;
        }

        uint16_t block_syms[16] = {};
        for (int i = 0; i < cw_len; ++i) {
            uint32_t v = (raw_values[sym_idx + i] + n_chips - 1) % n_chips;
            if (reduced) {
                v >>= 2;
            }
            block_syms[i] = static_cast<uint16_t>(grayEncode(v));
        }
        sym_idx += cw_len;

        uint8_t cw[16] = {};
        deinterleaveBlock(block_syms, sf_app, cw_len, cw);
        for (int i = 0; i < sf_app; ++i) {
            nibbles.push_back(hammingDecode(cw[i], cr_app));
        }

        if (first_block) {
            result.header = parseHeader(nibbles.data());
            if (!result.header.valid) {
                result.symbols_consumed = sym_idx;
                return result;
            }
            nibbles_needed =
                5 + 2 * static_cast<size_t>(result.header.payload_len) +
                (result.header.has_crc ? 4 : 0);
            first_block = false;
        }
    }

    result.symbols_consumed = sym_idx;
    if (result.header.valid && nibbles.size() >= nibbles_needed) {
        finishPayload(nibbles, result);
    }
    return result;
}

namespace {

// ln I0(x), Abramowitz & Stegun 9.8.1 / 9.8.2 (rel. err < 2e-7).
double logBesselI0(double x) {
    const double ax = std::abs(x);
    if (ax < 3.75) {
        double t = x / 3.75;
        t *= t;
        return std::log(1.0 + t * (3.5156229 + t * (3.0899424 + t * (1.2067492 +
                                   t * (0.2659732 + t * (0.0360768 + t * 0.0045813))))));
    }
    const double t = 3.75 / ax;
    const double poly =
        0.39894228 +
        t * (0.01328592 +
             t * (0.00225319 +
                  t * (-0.00157565 +
                       t * (0.00916281 +
                            t * (-0.02057706 + t * (0.02635537 + t * (-0.01647633 + t * 0.00392377)))))));
    return ax - 0.5 * std::log(ax) + std::log(poly);
}

}  // namespace

void estimateSoftScale(const float* mags, size_t n_syms, uint32_t n_chips, double& tone_amp,
                       double& noise_pow) {
    double peak_acc = 0.0;
    double noise_acc = 0.0;
    for (size_t s = 0; s < n_syms; ++s) {
        const float* row = mags + s * n_chips;
        uint32_t pk = 0;
        double pk_p = 0.0;
        double tot = 0.0;
        for (uint32_t k = 0; k < n_chips; ++k) {
            const double p = static_cast<double>(row[k]) * row[k];
            tot += p;
            if (p > pk_p) {
                pk_p = p;
                pk = k;
            }
        }
        double lobe = 0.0;
        for (int d = -2; d <= 2; ++d) {
            const double m = row[(pk + n_chips + static_cast<uint32_t>(d + 2) - 2u) % n_chips];
            lobe += m * m;
        }
        peak_acc += pk_p;
        noise_acc += (tot - lobe) / static_cast<double>(n_chips - 5);
    }
    if (n_syms == 0) {
        tone_amp = 0.0;
        noise_pow = 0.0;
        return;
    }
    noise_pow = noise_acc / static_cast<double>(n_syms);
    const double sig = peak_acc / static_cast<double>(n_syms) - noise_pow;
    // Near threshold the mean peak is mostly noise maxima; keep the scale
    // positive so the LLRs degrade toward hard decisions instead of vanishing.
    tone_amp = std::sqrt(std::max(sig, 0.25 * noise_pow));
}

void symbolLlrs(const float* row, uint8_t sf, bool reduced, double tone_amp, double noise_pow,
                int shift, double* llr_out) {
    const uint32_t n = 1u << sf;
    const uint8_t sf_app = reduced ? static_cast<uint8_t>(sf - 2) : sf;
    const double k = (noise_pow > 0.0) ? 2.0 * tone_amp / noise_pow : 0.0;
    const uint32_t sh = static_cast<uint32_t>(((shift % static_cast<int>(n)) + static_cast<int>(n)) %
                                              static_cast<int>(n));

    thread_local std::vector<double> metric;
    metric.resize(n);
    double mmax = -1e300;
    for (uint32_t u = 0; u < n; ++u) {
        const double m = logBesselI0(k * row[(u + n - sh) % n]);
        metric[u] = m;
        mmax = std::max(mmax, m);
    }
    double s1[16] = {};
    double s0[16] = {};
    for (uint32_t u = 0; u < n; ++u) {
        const double e = std::exp(metric[u] - mmax);
        uint32_t v = (u + n - 1) % n;
        if (reduced) {
            v >>= 2;
        }
        const uint32_t g = grayEncode(v);
        for (int j = 0; j < sf_app; ++j) {
            if ((g >> (sf_app - 1 - j)) & 0x1) {
                s1[j] += e;
            } else {
                s0[j] += e;
            }
        }
    }
    constexpr double kTiny = 1e-300;
    constexpr double kClamp = 60.0;
    for (int j = 0; j < sf_app; ++j) {
        const double l = std::log(s1[j] + kTiny) - std::log(s0[j] + kTiny);
        llr_out[j] = std::clamp(l, -kClamp, kClamp);
    }
}

uint8_t hammingDecodeSoft(const double* llr, uint8_t cr_app, double* margin) {
    const uint8_t cw_len = static_cast<uint8_t>(cr_app + 4);
    uint8_t best = 0;
    double best_score = -1e300;
    double second_score = -1e300;
    for (uint8_t nib = 0; nib < 16; ++nib) {
        const uint8_t cw = hammingEncode(nib, cr_app);
        double score = 0.0;
        for (int i = 0; i < cw_len; ++i) {
            if ((cw >> (cw_len - 1 - i)) & 0x1) {
                score += llr[i];
            }
        }
        if (score > best_score) {
            second_score = best_score;
            best_score = score;
            best = nib;
        } else if (score > second_score) {
            second_score = score;
        }
    }
    if (margin) {
        *margin = best_score - second_score;
    }
    return best;
}

DecodeResult decodeFrameSoft(const LoraParams& p, const SoftSymbols& soft, int shift) {
    DecodeResult result;
    const uint8_t sf = p.spreading_factor;
    const uint32_t n_chips = chipCount(p);

    std::vector<uint8_t> nibbles;
    std::vector<double> margins;
    size_t sym_idx = 0;
    bool first_block = true;
    size_t nibbles_needed = static_cast<size_t>(sf - 2);

    while (nibbles.size() < nibbles_needed && sym_idx < soft.n_syms) {
        const bool reduced = first_block || p.ldro;
        const uint8_t cr_app = first_block ? 4 : result.header.cr;
        const uint8_t cw_len = static_cast<uint8_t>(cr_app + 4);
        const uint8_t sf_app = reduced ? sf - 2 : sf;
        if (sym_idx + cw_len > soft.n_syms) {
            break;
        }

        // cw_llr[c][i]: codeword c, bit i MSB first. Inverse of the diagonal
        // map in deinterleaveBlock: symbol i bit j -> codeword (i - j - 1).
        double cw_llr[16][8] = {};
        for (int i = 0; i < cw_len; ++i) {
            double llr[16];
            symbolLlrs(soft.mags + (sym_idx + i) * n_chips, sf, reduced, soft.tone_amp,
                       soft.noise_pow, shift, llr);
            for (int j = 0; j < sf_app; ++j) {
                const int dst = ((i - j - 1) % sf_app + sf_app) % sf_app;
                cw_llr[dst][i] = llr[j];
            }
        }
        sym_idx += cw_len;
        for (int c = 0; c < sf_app; ++c) {
            double m = 0.0;
            nibbles.push_back(hammingDecodeSoft(cw_llr[c], cr_app, &m));
            margins.push_back(m);
        }

        if (first_block) {
            result.header = parseHeader(nibbles.data());
            if (!result.header.valid) {
                result.symbols_consumed = sym_idx;
                return result;
            }
            nibbles_needed = 5 + 2 * static_cast<size_t>(result.header.payload_len) +
                             (result.header.has_crc ? 4 : 0);
            first_block = false;
        }
    }

    result.symbols_consumed = sym_idx;
    if (result.header.valid && nibbles.size() >= nibbles_needed) {
        finishPayload(nibbles, result);
        // Header is nibbles 0-4, payload byte k is nibbles 5+2k (lo) and
        // 6+2k (hi), CRC bits 4m..4m+3 are nibble c+m. CRC = crc16(head) ^
        // (byte[n-2] << 8 | byte[n-1]), so an undetected error needs the same
        // flip in a tail nibble and its CRC partner; both must be uncertain.
        const size_t len = result.header.payload_len;
        if (result.header.has_crc && len >= 2) {
            const size_t a = 5 + 2 * (len - 2);
            const size_t c = 5 + 2 * len;
            const size_t pairs[4][2] = {{a + 2, c}, {a + 3, c + 1}, {a, c + 2}, {a + 1, c + 3}};
            for (const auto& pr : pairs) {
                result.tail_margin =
                    std::min(result.tail_margin, std::max(margins[pr[0]], margins[pr[1]]));
            }
        }
    }
    return result;
}

}  // namespace phy::lora
