#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace phy {

// MAX2850 (TX) / MAX2851 (RX) fractional-N synth, 80 MHz reference:
// f_LO = 80 MHz * (IDIV + FDIV / 2^20), so one FDIV LSB is 76.3 Hz.
// Words go through the FPGA SPI tunnel as (reg << 10) | data10.
struct SynthWords {
    uint16_t w15 = 0;  // Main15: D9 = 1 (frac mode), IDIV[6:0]
    uint16_t w16 = 0;  // Main16: FDIV[19:10]
    uint16_t w17 = 0;  // Main17: FDIV[9:0]
};

SynthWords synthWords(double lo_mhz);
// Inverse of synthWords from the 10-bit readbacks of Main15/16/17.
double synthLoMhz(uint16_t r15, uint16_t r16, uint16_t r17);
// MAX2851 Main2 LNA_BAND (D6:D5) for the RX LO, same table as quadrf-jtag.
uint16_t max2851LnaBandReg2(double lo_mhz);

// Keeps TX on the channel and RX at channel - IF while the GUI (or anything
// else calling quadrf-jtag) moves either LO. The user-facing knob is the
// channel: a TX slider move sets it directly, an RX slider move means
// "receive here", so both map to channel = new LO. Expected values are the
// readbacks after our own programming, so synth quantization never looks
// like an external move.
class LoFollower {
public:
    LoFollower(double channel_mhz, double if_mhz, double tol_mhz = 0.005);

    double channelMhz() const { return channel_mhz_; }
    double rxLoMhz() const { return channel_mhz_ - if_mhz_; }

    struct Retune {
        bool tx_moved = false;
        bool rx_moved = false;
        double channel_mhz = 0.0;
    };
    // Compare fresh readbacks against what we last programmed. Returns the
    // new channel when an external write moved one of the LOs. If both moved
    // (GUI "Tx follow Rx"), TX wins; it is the same value then anyway.
    std::optional<Retune> observe(double tx_lo_mhz, double rx_lo_mhz);
    // Record readbacks after (re)programming both synths.
    void programmed(double tx_lo_mhz, double rx_lo_mhz);

private:
    double channel_mhz_;
    double if_mhz_;
    double tol_mhz_;
    double tx_expect_mhz_;
    double rx_expect_mhz_;
};

// Direct synth access via /dev/csi_stream0 JTAG register ioctls. Each call is
// one short transaction under the driver's JTAG lease; if quadrf-jtag holds
// the lease the call fails immediately instead of waiting, and the caller
// retries on its next poll.
class SynthAccess {
public:
    enum class Chip : uint8_t { kTx = 0x42, kRx = 0x43 };

    explicit SynthAccess(std::string dev = "/dev/csi_stream0");

    // Reads Main15/16/17 (and optionally further main registers) with DOUT
    // switched to SPI readback, then restores DOUT to PLL lock detect.
    bool readLo(Chip chip, double& lo_mhz, uint16_t* main0 = nullptr,
                uint16_t* main6 = nullptr) const;
    // Writes Main19 back to automatic VCO sub-band select, then Main15/16/17
    // (+ Main2 LNA band on RX); the Main17 write starts VAS. Optional RX
    // Main0/Main6 restores: the max285x.c in the appliance source tree
    // rewrites both on any `--rx` (all four antennas, 40 MHz); quadrf-fpga
    // 1.0.32 does not.
    bool programLo(Chip chip, double lo_mhz, std::optional<uint16_t> main0 = std::nullopt,
                   std::optional<uint16_t> main6 = std::nullopt) const;

    // Main19 = VAS_RELOCK_SEL (D7), VAS_MODE (D6), VAS_SPI[5:0]. With D6
    // clear the VCO stays on VAS_SPI no matter what Main15-17 ask for.
    // PhaseGaze / rf-vision leave the MAX2851 like that (pinned to the band
    // of their last hop), and `quadrf-jtag --rx freq=` never rewrites Main19,
    // so the RX LO cannot reach the requested frequency: measured as a deaf
    // receiver at sub-band 63, and a cliff below ~5760 MHz at sub-band 55.
    struct VcoState {
        uint16_t main19 = 0;   // written value
        uint8_t band = 0;      // sub-band in use (Main27 VAS_VCO_READ)
        uint8_t tune_adc = 0;  // 3-bit tune-voltage ADC; 0 or 7 = at a rail
        bool autoSelect() const { return (main19 & 0x40u) != 0; }
        bool tuneInRange() const { return tune_adc >= 1 && tune_adc <= 6; }
    };
    bool readVco(Chip chip, VcoState& st) const;

    const std::string& lastError() const { return err_; }

private:
    std::string dev_;
    mutable std::string err_;
};

}  // namespace phy
