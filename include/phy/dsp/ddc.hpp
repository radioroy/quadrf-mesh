#pragma once

#include <phy/types.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace phy {

// Real-tap decimating FIR on CF32. Kaiser-windowed sinc, taps padded to a
// multiple of 4 for the NEON dot product. History is carried across calls.
class DecimatingFir {
public:
    DecimatingFir() = default;
    // fs: input rate; pass/stop: one-sided band edges in Hz; atten_db: stopband.
    DecimatingFir(double fs, uint32_t decim, double pass_hz, double stop_hz, double atten_db);

    // Appends floor((hist + n) / decim) outputs to out.
    void process(const Sample* in, size_t n, std::vector<Sample>& out);
    void reset();

    size_t taps() const { return taps_.size(); }
    uint32_t decim() const { return decim_; }
    const std::vector<float>& coefficients() const { return taps_; }

private:
    std::vector<float> taps_;  // reversed, so out[m] = sum taps_[k] * buf[m*decim + k]
    uint32_t decim_ = 1;
    std::vector<Sample> buf_;  // pending input incl. taps-1 history
    size_t phase_ = 0;         // offset into buf_ of the next output window
};

// RX digital down-converter: host-rate stream with the channel at +f_if Hz
// (RX LO tuned f_if below the channel) -> complex baseband at fs_out with the
// channel centered on 0 Hz.
//
//   x[n] * exp(-j 2 pi f_if n / fs_in)  ->  FIR/M1  ->  FIR/2  ->  y[m]
//
// The mix moves the RX LO feedthrough / ADC DC term to -f_if, where the
// second stage stopband removes it. Stage 1 only has to protect the final
// passband from aliasing (stop at fs_mid - stop2); stage 2 is the channel
// filter. With fs_in == 2*fs_out stage 1 is skipped.
class Ddc {
public:
    struct Config {
        double fs_in = 8e6;
        double fs_out = 1e6;
        double f_if = 0.0;      // channel offset in the input stream, Hz
        double pass_hz = 270e3;  // final one-sided passband edge
        double stop_hz = 330e3;  // final one-sided stopband edge
        double atten_db = 70.0;
    };

    Ddc() = default;
    explicit Ddc(const Config& cfg);

    // Appends fs_out samples to out. Any chunk size.
    void process(const Sample* in, size_t n, std::vector<Sample>& out);
    void reset();

    const Config& config() const { return cfg_; }
    bool passthrough() const { return passthrough_; }
    size_t stage1Taps() const { return st1_.taps(); }
    size_t stage2Taps() const { return st2_.taps(); }
    // Equivalent noise bandwidth of the whole chain, Hz: output noise power
    // is N0 * noiseBandwidthHz() for white input of density N0. Passthrough
    // reports fs_in.
    double noiseBandwidthHz() const { return noise_bw_hz_; }

private:
    Config cfg_;
    bool passthrough_ = true;
    double noise_bw_hz_ = 0.0;
    bool use_st1_ = false;
    // NCO as a table of one full period when f_if / fs_in is a ratio with a
    // small denominator, else a renormalized recursive phasor.
    std::vector<Sample> nco_;
    size_t nco_pos_ = 0;
    Sample rot_{1.0f, 0.0f};
    Sample rot_step_{1.0f, 0.0f};
    size_t rot_count_ = 0;
    DecimatingFir st1_;
    DecimatingFir st2_;
    std::vector<Sample> mixed_;
    std::vector<Sample> mid_;
};

// Kaiser-window lowpass prototype, unit DC gain. Exposed for tests.
std::vector<float> designKaiserLowpass(double fs, double pass_hz, double stop_hz, double atten_db);

}  // namespace phy
