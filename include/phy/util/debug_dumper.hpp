#pragma once

#include <phy/types.hpp>

#include <cstddef>
#include <string>

namespace phy {

struct DumpManifest {
    double sample_rate_hz = 0.0;
    double bandwidth_hz = 0.0;
    uint32_t fft_size = 0;
    uint16_t preamble_chirps = 0;
    size_t sync_word_bin0 = 0;
    size_t sync_word_bin1 = 0;
    size_t align_offset = 0;
    size_t tx_samples = 0;
    size_t rx_samples = 0;
    size_t dechirped_symbol_index = 0;
    bool fft_is_frequency_domain = true;
};

class DebugDumper {
public:
    DebugDumper(bool enabled, std::string out_dir = "phy_dumps");

    bool enabled() const { return enabled_; }

    void write(const char* name, const Sample* data, size_t count);
    void writeManifest(const DumpManifest& manifest);

private:
    bool enabled_ = false;
    std::string out_dir_;
};

}  // namespace phy
