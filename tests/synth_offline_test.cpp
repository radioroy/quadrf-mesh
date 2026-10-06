// Host checks for the MAX285x synth word encoding and the LO follow policy
// used by quadrf-lora-phy to track appliance GUI retunes.

#include <phy/radio/synth.hpp>

#include <cmath>
#include <cstdio>

using phy::LoFollower;

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) {
        ++g_failures;
    }
}

double roundTrip(double mhz) {
    const phy::SynthWords w = phy::synthWords(mhz);
    return phy::synthLoMhz(w.w15 & 0x3FF, w.w16 & 0x3FF, w.w17 & 0x3FF);
}

}  // namespace

int main() {
    std::printf("== synth words ==\n");
    {
        // 5800 / 80 = 72.5: IDIV 72, FDIV 2^19.
        const phy::SynthWords w = phy::synthWords(5800.0);
        check(w.w15 == ((15u << 10) | (1u << 9) | 72u) && w.w16 == ((16u << 10) | 512u) &&
                  w.w17 == (17u << 10),
              "5800 MHz -> IDIV 72, FDIV 0x80000");

        // Half an LSB of FDIV is 38.15 Hz.
        double worst_hz = 0.0;
        for (double f = 4900.0; f <= 5900.0; f += 0.0137) {
            worst_hz = std::fmax(worst_hz, std::fabs(roundTrip(f) - f) * 1e6);
        }
        char buf[96];
        std::snprintf(buf, sizeof(buf), "4.9-5.9 GHz round trip, worst %.1f Hz", worst_hz);
        check(worst_hz <= 38.2, buf);

        const double near_int = 80.0 * 73.0 - 1e-6;
        const phy::SynthWords c = phy::synthWords(near_int);
        check((c.w15 & 0x7F) == 73 && (c.w16 & 0x3FF) == 0 && (c.w17 & 0x3FF) == 0,
              "FDIV rounding to 2^20 carries into IDIV");

        check(phy::max2851LnaBandReg2(5199.9) == 0x180 && phy::max2851LnaBandReg2(5200.0) == 0x1A0 &&
                  phy::max2851LnaBandReg2(5799.5) == 0x1C0 && phy::max2851LnaBandReg2(5800.0) == 0x1E0,
              "LNA band edges match quadrf-jtag");
    }

    std::printf("== LO follower ==\n");
    {
        LoFollower f(5800.0, 0.5);
        f.programmed(roundTrip(5800.0), roundTrip(5799.5));
        check(!f.observe(roundTrip(5800.0), roundTrip(5799.5)), "own programming is not a move");
        check(f.rxLoMhz() == 5799.5, "RX LO = channel - IF");

        auto r = f.observe(roundTrip(5800.0), roundTrip(5750.0));
        check(r && r->rx_moved && !r->tx_moved && r->channel_mhz == roundTrip(5750.0) &&
                  std::fabs(f.rxLoMhz() - 5749.5) < 1e-4,
              "RX slider to 5750 -> channel 5750, RX LO 5749.5");
        f.programmed(roundTrip(5750.0), roundTrip(f.rxLoMhz()));
        check(!f.observe(roundTrip(5750.0), roundTrip(5749.5)), "settled after retune");

        r = f.observe(roundTrip(5820.0), roundTrip(5749.5));
        check(r && r->tx_moved && !r->rx_moved && std::fabs(f.channelMhz() - 5820.0) < 1e-4,
              "TX slider to 5820 -> channel 5820");
        f.programmed(roundTrip(5820.0), roundTrip(f.rxLoMhz()));

        r = f.observe(roundTrip(5700.0), roundTrip(5700.0));
        check(r && r->tx_moved && r->rx_moved && std::fabs(f.channelMhz() - 5700.0) < 1e-4,
              "Tx-follow-Rx move of both -> channel follows TX");

        LoFollower z(5800.0, 0.0);
        z.programmed(roundTrip(5800.0), roundTrip(5800.0));
        r = z.observe(roundTrip(5800.0), roundTrip(5810.0));
        check(r && std::fabs(z.rxLoMhz() - 5810.0) < 1e-4, "zero-IF: RX LO = channel");
    }

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
    return g_failures == 0 ? 0 : 1;
}
