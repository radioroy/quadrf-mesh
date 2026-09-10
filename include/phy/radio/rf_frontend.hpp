#pragma once

#include <cstdint>
#include <string>

namespace phy {

// Mosaic TX is RHCP-only on this hardware; RX matches that sense for mesh coupling.
enum class RfPolarization { Rhcp, Lhcp };

class RfFrontend {
public:
    explicit RfFrontend(std::string jtag_path = PHY_JTAG_PATH);

    // antenna_mask: 4-bit enable mask (1=ant1, 2=ant2, 4=ant3, 8=ant4; 15=all).
    // pol is RX-only (jtag has no TX pol). Default Rhcp matches fixed RHCP TX.
    void configureRx(double freq_mhz, int gain_db, int bw_mhz = 40, int antenna_mask = 1,
                     RfPolarization pol = RfPolarization::Rhcp);
    // Full TX program — call once at startup (gain/freq/bw/phases/ants).
    void configureTx(double freq_mhz, int gain_db, int bw_mhz = 40, int antenna_mask = 1);

    // Exit TX standby after txOff. Restores ants + tone_en=0. Optional freq/gain
    // re-arm the LO so the PLL has a setpoint after standby.
    void enableTx(int antenna_mask) const;
    void enableTx(int antenna_mask, double freq_mhz, int gain_db) const;

    bool txPllLocked() const;

    // Keep MAX2850 in TX mode (PLL warm). Gate PA_BIAS + FPGA disable_tx only.
    // Mute:  0x42=0x2800 then 0x23=1.  Unmute: 0x23=0 then 0x42=0x2801.
    void paMute() const;
    void paUnmute() const;

    // Lightweight helpers (each programs only the named field).
    void setTxAntennaMask(int antenna_mask) const;
    void setTxGain(int gain_db) const;
    // Parse `jtag --status tx` antenna enable mask (0 if none / off).
    int readTxAntennaMask() const;

    void setRxTone(bool enable, double tone_freq_mhz = 0.1);
    void setTxTone(bool enable, double tone_freq_mhz = 0.1);

    void rxOff();
    void txOff();

    uint16_t readRegister(uint8_t addr) const;
    void writeRegister(uint8_t addr, uint16_t value) const;
    // Fast-path register access without entering/exiting CSI JTAG setup.
    uint16_t readRegisterNoSetup(uint8_t addr) const;
    void writeRegisterNoSetup(uint8_t addr, uint16_t value) const;

    // FPGA reg 0x2E bit 2: route TX baseband into RX (bypasses RF).
    void setDigitalLoopback(bool enable) const;

    // Fast-path RX gain poke via FPGA reg 0x6A (MAX2851 LNA/VGA/digital gain).
    // Does not run jtag --status / setup; safe on the unmute/mute path.
    int readRxGainNoSetup() const;
    void setRxGainNoSetup(int gain_db) const;

    const std::string& jtagPath() const { return jtag_path_; }

private:
    int runJtag(const std::string& args) const;
    std::string runJtagCapture(const std::string& args) const;

    std::string jtag_path_;
};

}  // namespace phy
