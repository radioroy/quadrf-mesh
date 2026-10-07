#pragma once

#include <phy/lora/coding.hpp>
#include <phy/lora/params.hpp>
#include <phy/lora/symbol_demod.hpp>
#include <phy/lora/wander.hpp>
#include <phy/types.hpp>

#include <array>
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
    // S / (N0 * BW), dB. CRC-passing frames: decision-aided at the sent
    // values (decisionAidedMetrics); others: mean per-symbol channelSnrDb.
    // Both corrected for the front-end noise bandwidth (rx_noise_bw_hz).
    double snr_db = 0.0;
    double sir_db = kSirCeilingDb;  // signal vs strongest other tone; CRC-passing frames only
    double lvl_dbfs = 0.0;     // mean folded power, dBFS (not dBm)
    double sync_err0 = 0.0;
    double sync_err1 = 0.0;
    bool sync_ok = false;
    bool sfd_ok = false;
    bool synced = false;
    bool soft_decoded = false;  // CRC passed on the soft path, not the hard one
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
    // Data symbols demodulated so far, and how many used the coherent fold.
    size_t dataSymbols() const { return data_syms_total_; }
    size_t coherentSymbols() const { return coherent_syms_; }
    // Headers / frames the hard path lost and soft decisions recovered.
    size_t softHeaders() const { return soft_headers_; }
    size_t softFrames() const { return soft_frames_; }
    size_t samplesConsumed() const { return next_abs_; }

private:
    enum class State { kSearch, kSync, kData };

    void process();
    bool stepSearch();
    bool stepSync();
    bool stepData();
    // Wander-mode replacement for the per-symbol body of stepData; win_
    // holds the resampled window.
    void trackSymbol(double step);
    // Parse the header once 8 symbols are in; false = invalid header.
    bool checkHeader();
    // Frame-end search over cumulative +-1 chip slips (+-4 on reduced-rate
    // symbols) against a Kalman carrier model; fills offset paths, best first.
    void slipSearch(std::vector<std::vector<int>>& paths) const;
    void smoothSymbol(size_t idx, double ref_pred, double slope_pred, double d);
    void derotateTrack(Sample* window) const;
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
    double sf_gain_db_ = 0.0;  // processing gain over SF7, for the SNR gates

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
    // Soft-decision rows, one per raw value: folded magnitudes in value order
    // (row[v] belongs to value v), from whichever fold made the decision.
    std::vector<float> soft_mags_;
    size_t data_sym_ = 0;
    size_t needed_syms_ = 8;
    bool have_len_ = false;
    double ref_ = 0.0;
    // Coherent image fold (SymbolDemod::demodData): decision-directed estimate
    // of the inter-segment rotation, sum of seg1 * conj(seg2) at decided peaks,
    // and the matching sum of |seg1||seg2| (coherence = |acc| / mag).
    std::complex<double> fold_acc_{0.0, 0.0};
    double fold_mag_ = 0.0;

    // Carrier tracking under crystal wander (LoraParams::wander_*). ref_ is
    // then the predicted carrier at the next symbol start, trk_slope_ its
    // predicted change across the symbol, trk_var_ the variance of ref_.
    bool wander_ = false;
    double sigma_w_ = 0.0;      // expected carrier change per symbol, chips
    uint32_t n_sub_max_ = 4;    // sub-blocks per symbol at high SNR
    double trk_beta_ = 0.0;     // slope carried into the next symbol
    double joint_frac_ = 1.0;   // boundary taper width, symbols
    // Data-carrier Kalman: absolute carrier (chips) and its rate (chips per
    // symbol) at kf_t_ (symbols from data start); p = {P00, P01, P11}.
    bool kf_on_ = false;
    double kf_q_ = 0.0;
    double kf_x_[2] = {0.0, 0.0};
    double kf_p_[3] = {0.0, 0.0, 0.0};
    double kf_t_ = 0.0;
    double kf_ref0_ = 0.0;  // carrier at data start
    std::vector<double> trk_t_, trk_e_;  // smoothed in-symbol carrier vs the derotation ramp
    std::vector<std::array<double, 2>> sm_x_, sm_xp_;
    std::vector<std::array<double, 3>> sm_p_, sm_pp_;
    std::vector<double> sm_dt_;
    double step_tol_ = 0.0;     // SEARCH / preamble peak-step bound, chips
    double sync_tol_ = 0.0;     // sync-word / calibration residual bound, chips
    double trk_slope_ = 0.0;
    double trk_var_ = 0.0;
    WanderFit wfit_;
    IQBuffer dech_;
    // Previous symbol's carrier observations at its decided value:
    // {symbol time t, carrier in chips, weight}.
    std::vector<std::array<double, 3>> prev_obs_;
    std::vector<double> joint_a_, joint_y_, joint_w_;
    struct SlipSym {
        uint32_t obs0, obs1;  // range in slip_obs_
        bool reduced;
    };
    std::vector<SlipSym> slip_syms_;
    std::vector<std::array<double, 3>> slip_obs_;  // t (symbols from data start), carrier (chips), weight
    // Sub-blocks for one window: as many as the wander needs, as few as
    // keep ~6 dB SNR per block.
    uint32_t subBlocks(const ChipPeak& pk) const;
    // Fit the carrier trajectory of the tone at unfolded bin b1 in window w
    // (down: SFD window, dechirped by the up-chirp). wrap: see WanderFit::fit.
    bool fitWindow(const Sample* w, bool down, int32_t b1, int32_t wrap, const ChipPeak& pk);
    void derotateRamp(Sample* w, double f0, double slope) const;
    double snr_acc_ = 0.0;
    double pwr_acc_ = 0.0;
    size_t metric_n_ = 0;
    ReceivedFrame cur_;
    IQBuffer win_;

    std::deque<ReceivedFrame> out_;
    size_t frames_detected_ = 0;
    size_t data_syms_total_ = 0;
    size_t coherent_syms_ = 0;
    size_t soft_headers_ = 0;
    size_t soft_frames_ = 0;

    SoftSymbols softRows(size_t n_syms) const;
    void enforceCrcFlag(DecodeResult& d) const;
    DecodeResult decodeHard(const std::vector<uint16_t>& values) const;
    DecodeResult decodeSoft(const SoftSymbols& soft, int shift = 0) const;
};

}  // namespace phy::lora
