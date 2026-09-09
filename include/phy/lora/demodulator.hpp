#pragma once

#include <phy/lora/coding.hpp>
#include <phy/lora/params.hpp>
#include <phy/lora/symbol_demod.hpp>
#include <phy/types.hpp>

#include <cstdint>
#include <vector>

namespace phy::lora {

struct FrameResult {
    bool synced = false;
    size_t start_sample = 0;        // preamble start in the raw capture
    size_t preamble_symbols = 0;
    double rate_ratio = 1.0;        // RX samples per nominal sample
    double rate_ppm = 0.0;
    double ref_chip = 0.0;          // preamble chip position (CFO+timing ref)
    double sync_err0 = 0.0;         // sync word chip errors
    double sync_err1 = 0.0;
    bool sync_ok = false;
    bool sfd_ok = false;
    std::vector<double> preamble_chips;   // per-symbol trajectory (raw capture)
    std::vector<double> preamble_mags;
    std::vector<uint16_t> raw_values;     // data chips after ref removal
    std::vector<double> raw_chip_pos;     // sub-bin data chip positions
    std::vector<double> data_mags;
    DecodeResult decode;
};

// Whole-capture frame analyzer. Detects, syncs, and decodes one frame per
// call from a buffered capture; used by the offline suite and capture
// forensics. The chunk-fed real-time path is Receiver.
class Demodulator {
public:
    explicit Demodulator(const LoraParams& params);

    // Demodulate one dechirped window (folded over the oversampling images).
    ChipPeak demodWindow(const Sample* window, bool downchirp_ref = false);

    // Find and decode the next frame at or after `from`. Returns a result
    // with synced=false when no preamble is found.
    FrameResult processFrame(const IQBuffer& rx, size_t from = 0);

    uint32_t sps() const { return sps_; }

private:
    ChipPeak demodAt(const IQBuffer& rx, size_t pos, bool downchirp_ref = false);

    LoraParams params_;
    uint32_t sps_ = 0;
    uint32_t n_chips_ = 0;
    SymbolDemod sym_;
};

// Cubic (Catmull-Rom) fractional resampler: out[k] = in(start + k * ratio).
IQBuffer resampleCubic(const IQBuffer& in, double start, double ratio, size_t out_count);

}  // namespace phy::lora
