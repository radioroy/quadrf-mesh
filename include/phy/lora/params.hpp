#pragma once

#include <cstdint>

namespace phy::lora {

// LoRa PHY parameters, gr-lora_sdr / SX127x conventions.
// cr is the coding-rate index: 1..4 -> 4/5..4/8.
struct LoraParams {
    uint8_t spreading_factor = 11;
    uint8_t cr = 1;
    bool has_crc = true;
    bool ldro = false;  // low data rate optimization (reduced-rate payload blocks)
    uint8_t sync_word = 0x2B;
    uint16_t preamble_len = 16;
    double bandwidth_hz = 500e3;
    double sample_rate_hz = 1e6;
    // Physical clock-offset bound for the preamble rate fit, in ppm.
    // 0 disables the clamp (offline tests use synthetic drifts of 1000+ ppm);
    // live hardware TCXOs stay within ~+/-20 ppm, so noise-driven slope fits
    // beyond the bound get clamped instead of warping the payload resample.
    double max_rate_ppm = 0.0;
    // Largest carrier offset to expect, Hz. The up/down-chirp solve reads
    // CFO modulo BW/2, so when this exceeds BW/4 (62.5 kHz BW against the
    // ~15 kHz inter-unit offset at 5.8 GHz) sync also tries the alias
    // cfo +- BW/2. The PHY channel filter is opened by the same amount.
    // 0 = no alias hypotheses.
    double max_cfo_hz = 0.0;
    // Noise bandwidth of the RX front end ahead of the receiver (DDC
    // Ddc::noiseBandwidthHz()). The dechirped fold collects noise from that
    // whole band, so the per-symbol SNR reads 10*log10(noise_bw / BW) low
    // against S / (N0 * BW); ReceivedFrame::snr_db adds it back.
    // 0 = treat as BW (no correction).
    double rx_noise_bw_hz = 0.0;
    // Data symbols: coherent two-image fold (SymbolDemod::demodData) once the
    // per-frame rotation estimate locks. false = power fold over all images.
    bool coherent_fold = true;
    // Data-symbol PI timing loop gains on the fractional chip error. AWGN
    // sweeps put 10% PER at -7.6 dB (S/(N0*BW), SF7) with 0.2/0.03 vs
    // -5.9 dB with 0.7/0.15: near threshold a wrong peak yields a uniform
    // frac and high gains walk the grid off the next symbols.
    double timing_kp = 0.2;
    double timing_ki = 0.03;
    // Same-symbol +/-1 sample refine (DSI slip recovery) only above this
    // per-symbol channelSnrDb. Below it, noise alone passes the louder/tighter
    // test and each accepted trial permanently shifts the grid. On the OTA
    // captures any gate >= 3 dB matched never refining, at all noise levels.
    // Both SNR gates here are SF7 channel SNR; the receiver lowers them by
    // 10*log10(2^SF / 128) so they sit at the same per-symbol SNR at any SF.
    double refine_min_snr_db = 6.0;
    // Soft-decision header/payload decode (LLR deinterleave + ML Hamming)
    // when the hard path fails its checksum / CRC.
    bool soft_decoding = true;
    // Frame SNR (dB) below which a hard-decision CRC pass is only accepted if
    // the soft decode also passes; it catches tail-block CRC false accepts.
    double soft_verify_below_snr_db = 0.0;
    // Soft CRC passes need DecodeResult::tail_margin >= this (nats). At 2.0
    // ~0.6% of correct soft decodes near the cliff are dropped.
    double soft_tail_margin = 2.0;
    // Relative carrier wander between TX and RX: RMS rate (Hz/s) and the
    // spread it saturates at (Hz). See wanderChipsPerSymbol. When the
    // expected change per symbol reaches ~0.05 chip the receiver tracks the
    // carrier inside each symbol instead of holding the sync estimate, and
    // drops the preamble rate fit (wander reads as rate). 0 = off.
    double wander_rate_hz_per_s = 0.0;
    double wander_cap_hz = 750.0;
    // Integrated-random-walk intensity of the same wander, Hz^2/s^3: the
    // carrier's second derivative as white noise. The CW capture's second
    // differences give ~2e12 at 2-8 ms lags. Drives the frame-end slip search.
    double wander_irw_q = 0.0;
};

inline uint32_t chipCount(const LoraParams& p) {
    return 1u << p.spreading_factor;
}

inline uint32_t osFactor(const LoraParams& p) {
    return static_cast<uint32_t>(p.sample_rate_hz / p.bandwidth_hz + 0.5);
}

inline uint32_t samplesPerSymbol(const LoraParams& p) {
    return chipCount(p) * osFactor(p);
}

// Sync word nibbles as modulated chip values.
inline uint16_t syncChip0(const LoraParams& p) {
    return static_cast<uint16_t>(((p.sync_word >> 4) & 0x0F) << 3);
}

inline uint16_t syncChip1(const LoraParams& p) {
    return static_cast<uint16_t>((p.sync_word & 0x0F) << 3);
}

}  // namespace phy::lora
