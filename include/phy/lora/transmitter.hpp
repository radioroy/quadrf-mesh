#pragma once

#include <phy/lora/modulator.hpp>
#include <phy/lora/params.hpp>
#include <phy/types.hpp>

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

namespace phy::lora {

// Streaming LoRa transmitter: the TX counterpart of Receiver.
//
// Upper layers enqueue payload bytes (or a pre-built IQ frame). The radio
// thread pulls continuous CF32 samples so the SoapySDR TX stream never
// underruns: when a frame is pending it emits
//   [warmup carrier][modulated frame][gap silence]
// otherwise idle silence. Warmup is a DC carrier — settles RX AGC without
// looking like a preamble once dechirped.
//
// Modulation happens on enqueue (producer side) so the pull path is a
// straight sample copy. Chirp generation is handled in Modulator.
class Transmitter {
public:
    explicit Transmitter(const LoraParams& params, float amplitude = 0.8f);

    void setWarmupSymbols(uint32_t n) { warmup_syms_ = n; }
    void setGapSymbols(uint32_t n) { gap_syms_ = n; }
    uint32_t warmupSymbols() const { return warmup_syms_; }
    uint32_t gapSymbols() const { return gap_syms_; }
    uint32_t samplesPerSymbol() const { return mod_.samplesPerSymbol(); }
    float amplitude() const { return amplitude_; }

    // Encode + modulate and queue. Returns false if the payload is too
    // long for the LoRa PHY ( > 255 bytes ).
    bool enqueue(const std::vector<uint8_t>& payload);

    // Queue an already-modulated frame (tests / custom waveforms).
    void enqueueFrame(IQBuffer frame);

    // Fill out[0..n) with the next TX samples. Always writes n samples
    // (silence when idle). Safe to call from a different thread than enqueue.
    void pull(Sample* out, size_t n);

    // Frames waiting to start (not counting the one currently on the air).
    size_t queued() const;
    // True when nothing is queued and no burst is mid-air (incl. gap).
    bool isIdle() const;
    size_t framesStarted() const { return frames_started_; }
    size_t samplesEmitted() const { return samples_emitted_; }

    // Drop pending frames; finishes the current burst if one is mid-air.
    void clearQueue();

private:
    void startNextBurstLocked();
    void fillFromBurst(Sample* out, size_t n, size_t& written);

    LoraParams params_;
    float amplitude_;
    Modulator mod_;
    uint32_t sps_ = 0;
    uint32_t warmup_syms_ = 0;
    uint32_t gap_syms_ = 12;

    mutable std::mutex mu_;
    std::deque<IQBuffer> queue_;

    // Current on-air burst: warmup + frame + gap, assembled at start.
    IQBuffer burst_;
    size_t burst_pos_ = 0;

    size_t frames_started_ = 0;
    size_t samples_emitted_ = 0;
};

}  // namespace phy::lora
