#include <phy/radio/synth.hpp>

#include <cerrno>
#include <cmath>
#include <cstring>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace phy {

namespace {

// fpga-csi driver ABI (sources/fpga/drivers/csi/fpga_csi.h).
struct CsiJtagReg {
    uint8_t addr;
    uint16_t value;
    uint16_t pad;
};
constexpr unsigned long kIocJtagRegWrite = _IOW('C', 0x12, CsiJtagReg);
constexpr unsigned long kIocJtagRegRead = _IOWR('C', 0x13, CsiJtagReg);
constexpr unsigned long kIocJtagAcquireLease = _IO('C', 0x15);
constexpr unsigned long kIocJtagReleaseLease = _IO('C', 0x16);

constexpr double kRefMhz = 80.0;
constexpr uint32_t kFracOne = 1u << 20;
// Main14 default on both chips; D1 = DOUT_SEL (0 = PLL lock detect,
// 1 = SPI readback). quadrf-jtag assumes lock detect between commands.
constexpr uint16_t kMain14 = 0x160;
constexpr uint16_t kReadCmd = 0x8000;
// quadrf-jtag --init templates: VAS_MODE (D6) = auto, VAS_SPI = 31 start.
constexpr uint16_t kMain19AutoRx = 0x0DF;
constexpr uint16_t kMain19AutoTx = 0x05F;
// Main27 default; D5 = VAS_VCO_READ swaps the Main19 readback to the VAS
// result: band in D5:0, tune ADC in D8:6.
constexpr uint16_t kMain27 = 0x180;
constexpr uint16_t kMain27VcoRead = kMain27 | 0x020;

uint16_t word(unsigned reg, unsigned data10) {
    return static_cast<uint16_t>(((reg & 0x3Fu) << 10) | (data10 & 0x3FFu));
}

// One open/lease/close per transaction: the driver drops the lease and JTAG
// attach on close, so a crash here cannot wedge quadrf-jtag.
class Transaction {
public:
    Transaction(const std::string& dev, std::string& err) : err_(err) {
        fd_ = ::open(dev.c_str(), O_RDWR | O_CLOEXEC);
        if (fd_ < 0) {
            err_ = "open " + dev + ": " + std::strerror(errno);
            return;
        }
        if (::ioctl(fd_, kIocJtagAcquireLease) != 0) {
            if (errno == EBUSY) {
                err_ = "jtag lease busy";
                ::close(fd_);
                fd_ = -1;
                return;
            }
            // ENOTTY: driver predates leases, same as quadrf-jtag.
            if (errno != ENOTTY) {
                err_ = std::string("lease: ") + std::strerror(errno);
                ::close(fd_);
                fd_ = -1;
                return;
            }
        } else {
            leased_ = true;
        }
    }
    ~Transaction() {
        if (fd_ >= 0) {
            if (leased_) {
                ::ioctl(fd_, kIocJtagReleaseLease);
            }
            ::close(fd_);
        }
    }
    bool ok() const { return fd_ >= 0 && good_; }

    void write(uint8_t addr, uint16_t value) {
        if (!ok()) {
            return;
        }
        CsiJtagReg r{addr, value, 0};
        if (::ioctl(fd_, kIocJtagRegWrite, &r) != 0) {
            fail("reg write");
        }
    }
    uint16_t read(uint8_t addr) {
        if (!ok()) {
            return 0;
        }
        CsiJtagReg r{addr, 0, 0};
        if (::ioctl(fd_, kIocJtagRegRead, &r) != 0) {
            fail("reg read");
            return 0;
        }
        return r.value;
    }

private:
    void fail(const char* what) {
        err_ = std::string(what) + ": " + std::strerror(errno);
        good_ = false;
    }

    std::string& err_;
    int fd_ = -1;
    bool leased_ = false;
    bool good_ = true;
};

}  // namespace

SynthWords synthWords(double lo_mhz) {
    const double ratio = lo_mhz / kRefMhz;
    long long idiv = static_cast<long long>(std::floor(ratio));
    long long fdiv = std::llround((ratio - static_cast<double>(idiv)) * kFracOne);
    if (fdiv == static_cast<long long>(kFracOne)) {
        ++idiv;
        fdiv = 0;
    }
    SynthWords w;
    w.w15 = word(15, (1u << 9) | (static_cast<unsigned>(idiv) & 0x7Fu));
    w.w16 = word(16, static_cast<unsigned>(fdiv >> 10));
    w.w17 = word(17, static_cast<unsigned>(fdiv));
    return w;
}

double synthLoMhz(uint16_t r15, uint16_t r16, uint16_t r17) {
    const unsigned idiv = r15 & 0x7Fu;
    const uint32_t fdiv = (static_cast<uint32_t>(r16 & 0x3FFu) << 10) | (r17 & 0x3FFu);
    return kRefMhz * (static_cast<double>(idiv) + static_cast<double>(fdiv) / kFracOne);
}

uint16_t max2851LnaBandReg2(double lo_mhz) {
    if (lo_mhz < 5200.0) return 0x180;
    if (lo_mhz < 5500.0) return 0x1A0;
    if (lo_mhz < 5800.0) return 0x1C0;
    return 0x1E0;
}

LoFollower::LoFollower(double channel_mhz, double if_mhz, double tol_mhz)
    : channel_mhz_(channel_mhz),
      if_mhz_(if_mhz),
      tol_mhz_(tol_mhz),
      tx_expect_mhz_(channel_mhz),
      rx_expect_mhz_(channel_mhz - if_mhz) {}

std::optional<LoFollower::Retune> LoFollower::observe(double tx_lo_mhz, double rx_lo_mhz) {
    Retune r;
    r.tx_moved = std::fabs(tx_lo_mhz - tx_expect_mhz_) > tol_mhz_;
    r.rx_moved = std::fabs(rx_lo_mhz - rx_expect_mhz_) > tol_mhz_;
    if (!r.tx_moved && !r.rx_moved) {
        return std::nullopt;
    }
    r.channel_mhz = r.tx_moved ? tx_lo_mhz : rx_lo_mhz;
    channel_mhz_ = r.channel_mhz;
    return r;
}

void LoFollower::programmed(double tx_lo_mhz, double rx_lo_mhz) {
    tx_expect_mhz_ = tx_lo_mhz;
    rx_expect_mhz_ = rx_lo_mhz;
}

SynthAccess::SynthAccess(std::string dev) : dev_(std::move(dev)) {}

bool SynthAccess::readLo(Chip chip, double& lo_mhz, uint16_t* main0, uint16_t* main6) const {
    Transaction t(dev_, err_);
    const uint8_t a = static_cast<uint8_t>(chip);
    auto readMain = [&](unsigned reg) {
        t.write(a, static_cast<uint16_t>(kReadCmd | ((reg & 0x1Fu) << 10)));
        return static_cast<uint16_t>(t.read(a) & 0x3FFu);
    };
    t.write(a, word(14, kMain14 | 0x2u));
    const uint16_t r15 = readMain(15);
    const uint16_t r16 = readMain(16);
    const uint16_t r17 = readMain(17);
    if (main0) {
        *main0 = readMain(0);
    }
    if (main6) {
        *main6 = readMain(6);
    }
    t.write(a, word(14, kMain14));
    if (!t.ok()) {
        return false;
    }
    // Main15 D9 is always set by us and by quadrf-jtag; an all-zero or
    // all-ones word means DOUT never switched to readback.
    if ((r15 & 0x200u) == 0 || (r15 == 0x3FFu && r16 == 0x3FFu && r17 == 0x3FFu)) {
        err_ = "implausible synth readback";
        return false;
    }
    lo_mhz = synthLoMhz(r15, r16, r17);
    return true;
}

bool SynthAccess::programLo(Chip chip, double lo_mhz, std::optional<uint16_t> main0,
                            std::optional<uint16_t> main6) const {
    Transaction t(dev_, err_);
    const uint8_t a = static_cast<uint8_t>(chip);
    if (main6) {
        t.write(a, word(6, *main6));
    }
    if (main0) {
        t.write(a, word(0, *main0));
    }
    t.write(a, word(19, chip == Chip::kRx ? kMain19AutoRx : kMain19AutoTx));
    // Same order as quadrf-jtag (max285x_set_freq_common).
    const SynthWords w = synthWords(lo_mhz);
    t.write(a, w.w15);
    t.write(a, w.w16);
    t.write(a, w.w17);
    if (chip == Chip::kRx) {
        t.write(a, word(2, max2851LnaBandReg2(lo_mhz)));
    }
    return t.ok();
}

bool SynthAccess::readVco(Chip chip, VcoState& st) const {
    Transaction t(dev_, err_);
    const uint8_t a = static_cast<uint8_t>(chip);
    auto readMain = [&](unsigned reg) {
        t.write(a, static_cast<uint16_t>(kReadCmd | ((reg & 0x1Fu) << 10)));
        return static_cast<uint16_t>(t.read(a) & 0x3FFu);
    };
    t.write(a, word(14, kMain14 | 0x2u));
    st.main19 = readMain(19);
    t.write(a, word(27, kMain27VcoRead));
    const uint16_t vas = readMain(19);
    t.write(a, word(27, kMain27));
    t.write(a, word(14, kMain14));
    st.band = static_cast<uint8_t>(vas & 0x3Fu);
    st.tune_adc = static_cast<uint8_t>((vas >> 6) & 0x7u);
    return t.ok();
}

}  // namespace phy
