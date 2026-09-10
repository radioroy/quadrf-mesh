#pragma once

#include <phy/lora/coding.hpp>
#include <phy/lora/params.hpp>
#include <phy/lora/symbol_demod.hpp>
#include <phy/types.hpp>

#include <cstdint>
#include <deque>
#include <vector>

namespace phy::lora {

struct ReceivedFrame {
    size_t start_sample = 0;   // absolute stream index of the preamble start
    size_t preamble_symbols = 0;
    double rate_ratio = 1.0;   // RX samples per nominal sample
    double rate_ppm = 0.0;
    double cfo_chips = 0.0;    // carrier offset, one chip = bw / 2^sf Hz
    double cfo_hz = 0.0;
    double tau_samples = 0.0;  // fine timing offset solved from up/down peaks
    double snr_db = 0.0;       // mean channel SNR over data symbols (PG removed)
    double sir_db = 0.0;       // worst-symbol 10*log10(lobe/mag2)
    double lvl_dbfs = 0.0;     // mean folded power, dBFS (not dBm)
    double sync_err0 = 0.0;
    double sync_err1 = 0.0;
    bool sync_ok = false;
    bool sfd_ok = false;
    bool synced = false;
    std::vector<uint16_t> raw_values;  // data chips after reference removal
    DecodeResult decode;
};

// Streaming LoRa receiver: a chunk-fed state machine, the real-time
// counterpart of Demodulator::processFrame.
//
//   SEARCH  one dechirp FFT per symbol period on a fixed grid, looking for
//           a run of strong windows with a slowly moving peak.
//   SYNC    once enough of the burst is buffered: iterative rate estimate
//           from the preamble drift, then joint CFO/timing from the
//           preamble up-chirp and SFD down-chirp peaks
//           (cfo = (u+d)/2, tau = os*(u-d)/2) - a handful of FFTs instead
//           of an offset scan.
//   DATA    one window per symbol, cubic-resampled straight out of the raw
//           stream at the estimated rate, with same-symbol sub-chip window
//           refinement and a decision-directed PI timing loop. Frames are
//           queued as they complete.
//
// Memory is bounded: the raw buffer holds at most the preamble/sync span
// while acquiring and roughly one symbol while in DATA.
class Receiver {
public:
    explicit Receiver(const LoraParams& params);

    // Feed an arbitrary-size chunk of contiguous RX samples.
    void feed(const Sample* chunk, size_t n);

    // Reset receiver internal buffers and sync state (e.g. after stream mode switch).
    void reset();

    // End of stream: finalize a frame still being demodulated.
    void flush();

    // Dequeue the next completed frame; false when none pending.
    bool pop(ReceivedFrame& out);

    size_t framesDetected() const { return frames_detected_; }
    size_t samplesConsumed() const { return next_abs_; }

private:
    enum class State { kSearch, kSync, kData };

    void process();
    bool stepSearch();
    bool stepSync();
    bool stepData();
    void finalizeData();
    void resetToSearch(size_t scan_from_abs);
    void trimFront(size_t keep_from_abs);

    // window read straight from the raw buffer (integer position)
    const Sample* rawWindow(size_t abs_pos) const;

    LoraParams params_;
    SymbolDemod sym_;
    uint32_t sps_ = 0;
    uint32_t n_chips_ = 0;
    uint32_t os_ = 0;

    // raw sample buffer; buf_[0] is absolute stream index base_
    std::vector<Sample> buf_;
    size_t base_ = 0;
    size_t next_abs_ = 0;  // total samples fed so far

    State state_ = State::kSearch;

    // SEARCH
    size_t scan_abs_ = 0;
    size_t run_start_abs_ = 0;
    int run_len_ = 0;
    double prev_chip_ = 0.0;

    // SYNC / DATA context
    size_t found_abs_ = 0;    // first strong window of the run
    size_t origin_abs_ = 0;   // resample origin (found - back margin)
    double total_ratio_ = 1.0;
    IQBuffer resampled_;      // sync-span working buffer

    // DATA
    double data_raw_f_ = 0.0;  // next symbol start, fractional raw-stream index
    double timing_int_ = 0.0;  // integral term of the symbol timing loop, chips
    std::vector<double> sym_fracs_;  // per-symbol fractional chip error, pre-correction
    // Alternate candidate symbols for CRC retry when runner-up peak magnitude
    // approaches the primary peak: {symbol index, alternate value, mag2/mag1}.
    struct AltSym {
        size_t idx;
        uint16_t value;
        double ratio;
    };
    std::vector<AltSym> alt_syms_;
    size_t data_sym_ = 0;
    size_t needed_syms_ = 8;
    bool have_len_ = false;
    double ref_ = 0.0;
    double snr_acc_ = 0.0;
    double sir_min_ = 0.0;
    double pwr_acc_ = 0.0;
    size_t metric_n_ = 0;
    ReceivedFrame cur_;
    IQBuffer win_;

    std::deque<ReceivedFrame> out_;
    size_t frames_detected_ = 0;
};

}  // namespace phy::lora
