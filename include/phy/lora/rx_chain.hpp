#pragma once

#include <phy/dsp/ddc.hpp>
#include <phy/lora/params.hpp>

namespace phy::lora {

// DDC for the receiver at rx.sample_rate_hz from a host stream at fs_in with
// the channel at +f_if. Passband BW/2 + rx.max_cfo_hz (20 kHz if unset),
// transition 0.12 BW; at 500 kHz that is the 270 / 330 kHz pair the SF7 OTA
// tuning used.
inline Ddc::Config rxChannelFilter(const LoraParams& rx, double fs_in, double f_if) {
    Ddc::Config c;
    c.fs_in = fs_in;
    c.fs_out = rx.sample_rate_hz;
    c.f_if = f_if;
    c.pass_hz = 0.5 * rx.bandwidth_hz + (rx.max_cfo_hz > 0.0 ? rx.max_cfo_hz : 20e3);
    c.stop_hz = c.pass_hz + 0.12 * rx.bandwidth_hz;
    return c;
}

}  // namespace phy::lora
