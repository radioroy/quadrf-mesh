#include <phy/radio/rf_frontend.hpp>

#include <algorithm>
#include <array>
#include <cstdio>
#include <sstream>
#include <stdexcept>

namespace phy {

RfFrontend::RfFrontend(std::string jtag_path) : jtag_path_(std::move(jtag_path)) {}

int RfFrontend::runJtag(const std::string& args) const {
    const std::string cmd = jtag_path_ + " " + args + " 2>&1";
    const int rc = std::system(cmd.c_str());
    if (rc != 0) {
        std::ostringstream oss;
        oss << "jtag command failed (rc=" << rc << "): " << cmd;
        throw std::runtime_error(oss.str());
    }
    return rc;
}

void RfFrontend::configureRx(double freq_mhz, int gain_db, int bw_mhz, int antenna_mask,
                             RfPolarization pol) {
    std::ostringstream spec;
    // Default mask 1: single antenna avoids multi-path self-cancellation.
    // autosteer=0 + zero phases: beamforming off / centered.
    // pol=rhcp: match fixed RHCP TX for co-polarized mesh hops.
    const char* pol_s = (pol == RfPolarization::Lhcp) ? "lhcp" : "rhcp";
    spec << "antennas=" << antenna_mask
         << ",interleave=0,tone_en=0,autosteer=0,p1=0,p2=0,p3=0,p4=0,pol=" << pol_s
         << ",freq=" << freq_mhz << ",gain=" << gain_db << ",bw=" << bw_mhz;
    runJtag("--rx " + spec.str());
}

void RfFrontend::configureTx(double freq_mhz, int gain_db, int bw_mhz, int antenna_mask) {
    std::ostringstream spec;
    // Clear sticky follow/phases — leftover TX state interferes with transmission.
    spec << "antennas=" << antenna_mask
         << ",tone_en=0,tx_follow_rx=0,p1=0,p2=0,p3=0,p4=0,freq="
         << freq_mhz << ",gain=" << gain_db << ",bw=" << bw_mhz;
    runJtag("--tx " + spec.str());
}

void RfFrontend::enableTx(int antenna_mask) const {
    // txOff clears PA enables and leaves standby. Re-enter TX mode with the
    // minimum keys to radiate and lock PLL. Omitting gain/freq/bw/phases/follow
    // preserves frontend tuning across mute cycles.
    std::ostringstream spec;
    spec << "antennas=" << antenna_mask << ",tone_en=0";
    runJtag("--tx " + spec.str());
}

void RfFrontend::enableTx(int antenna_mask, double freq_mhz, int gain_db) const {
    std::ostringstream spec;
    spec << "antennas=" << antenna_mask << ",tone_en=0,freq=" << freq_mhz
         << ",gain=" << gain_db;
    runJtag("--tx " + spec.str());
}

bool RfFrontend::txPllLocked() const {
    try {
        const std::string status = runJtagCapture("--status tx");
        return status.find("PLL Lock: LOCKED") != std::string::npos;
    } catch (...) {
        return false;
    }
}

// MAX2850 SPI via FPGA reg 0x42: word = (reg << 10) | data10. Main10 bit0 is PA_BIAS.
// FPGA reg 0x23 bit0 is disable_tx, per QuadRF hardware register specifications.
constexpr uint16_t kSpiMain10PaOn = 0x2801;
constexpr uint16_t kSpiMain10PaOff = 0x2800;
constexpr uint8_t kFpgaSpi = 0x42;
constexpr uint8_t kFpgaDisableTx = 0x23;

void RfFrontend::paMute() const {
    writeRegisterNoSetup(kFpgaSpi, kSpiMain10PaOff);
    writeRegisterNoSetup(kFpgaDisableTx, 0x0001);
}

void RfFrontend::paUnmute() const {
    writeRegisterNoSetup(kFpgaDisableTx, 0x0000);
    writeRegisterNoSetup(kFpgaSpi, kSpiMain10PaOn);
}

void RfFrontend::setTxAntennaMask(int antenna_mask) const {
    std::ostringstream spec;
    spec << "antennas=" << antenna_mask;
    runJtag("--tx " + spec.str());
}

void RfFrontend::setTxGain(int gain_db) const {
    std::ostringstream spec;
    spec << "gain=" << gain_db;
    runJtag("--tx " + spec.str());
}

int RfFrontend::readTxAntennaMask() const {
    const std::string status = runJtagCapture("--status tx");
    const auto pos = status.find("Antennas enabled:");
    if (pos == std::string::npos) {
        return 0;
    }
    const auto line_end = status.find('\n', pos);
    const std::string line = status.substr(pos, line_end == std::string::npos ? std::string::npos : line_end - pos);
    if (line.find("None") != std::string::npos) {
        return 0;
    }
    // e.g. "Antennas enabled: 1" or "1, 4"
    int mask = 0;
    for (size_t i = 0; i < line.size(); ++i) {
        if (line[i] >= '1' && line[i] <= '4' && (i == 0 || line[i - 1] < '0' || line[i - 1] > '9')) {
            const int ant = line[i] - '0';
            mask |= (1 << (ant - 1));
        }
    }
    return mask;
}

void RfFrontend::setRxTone(bool enable, double tone_freq_mhz) {
    std::ostringstream spec;
    spec << "antennas=1,interleave=0,autosteer=0,tone_en=" << (enable ? 1 : 0)
         << ",tone_freq=" << tone_freq_mhz;
    runJtag("--rx " + spec.str());
}

void RfFrontend::setTxTone(bool enable, double tone_freq_mhz) {
    std::ostringstream spec;
    spec << "antennas=1,tone_en=" << (enable ? 1 : 0)
         << ",tone_freq=" << tone_freq_mhz;
    runJtag("--tx " + spec.str());
}

void RfFrontend::rxOff() {
    runJtag("--rx off");
}

void RfFrontend::txOff() {
    runJtag("--tx off");
}

std::string RfFrontend::runJtagCapture(const std::string& args) const {
    const std::string cmd = jtag_path_ + " " + args + " 2>&1";
    std::array<char, 256> buf{};
    std::string output;
    FILE* raw_pipe = popen(cmd.c_str(), "r");
    if (!raw_pipe) {
        throw std::runtime_error("failed to run jtag: " + cmd);
    }
    while (fgets(buf.data(), static_cast<int>(buf.size()), raw_pipe) != nullptr) {
        output += buf.data();
    }
    const int rc = pclose(raw_pipe);
    if (rc != 0) {
        std::ostringstream oss;
        oss << "jtag command failed (rc=" << rc << "): " << cmd << "\n" << output;
        throw std::runtime_error(oss.str());
    }
    return output;
}

uint16_t RfFrontend::readRegister(uint8_t addr) const {
    std::ostringstream args;
    args << "read 0x" << std::hex << static_cast<unsigned>(addr);
    const std::string output = runJtagCapture(args.str());
    const auto pos = output.rfind("0x");
    if (pos == std::string::npos) {
        throw std::runtime_error("jtag read parse failed: " + output);
    }
    return static_cast<uint16_t>(std::stoul(output.substr(pos), nullptr, 16));
}

uint16_t RfFrontend::readRegisterNoSetup(uint8_t addr) const {
    std::ostringstream args;
    args << "--no-setup read 0x" << std::hex << static_cast<unsigned>(addr);
    const std::string output = runJtagCapture(args.str());
    const auto pos = output.rfind("0x");
    if (pos == std::string::npos) {
        throw std::runtime_error("jtag read parse failed: " + output);
    }
    return static_cast<uint16_t>(std::stoul(output.substr(pos), nullptr, 16));
}

void RfFrontend::writeRegister(uint8_t addr, uint16_t value) const {
    std::ostringstream args;
    args << "write 0x" << std::hex << static_cast<unsigned>(addr) << " 0x" << value;
    runJtag(args.str());
}

void RfFrontend::writeRegisterNoSetup(uint8_t addr, uint16_t value) const {
    std::ostringstream args;
    args << "--no-setup write 0x" << std::hex << static_cast<unsigned>(addr)
         << " 0x" << value;
    runJtag(args.str());
}

void RfFrontend::setDigitalLoopback(bool enable) const {
    writeRegister(0x2E, enable ? 0x0004 : 0x0000);
}

void RfFrontend::setRxGainNoSetup(int gain_db) const {
    const uint16_t clamped = static_cast<uint16_t>(std::max(0, std::min(gain_db, 63)));
    writeRegisterNoSetup(0x6A, clamped);
}

}  // namespace phy
