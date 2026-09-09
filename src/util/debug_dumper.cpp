#include <phy/util/debug_dumper.hpp>

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace phy {

DebugDumper::DebugDumper(bool enabled, std::string out_dir)
    : enabled_(enabled), out_dir_(std::move(out_dir)) {}

void DebugDumper::write(const char* name, const Sample* data, size_t count) {
    if (!enabled_ || count == 0) {
        return;
    }

    std::filesystem::create_directories(out_dir_);

    const std::string path = out_dir_ + "/" + name + ".iq";
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        throw std::runtime_error("DebugDumper: failed to open " + path);
    }

    for (size_t i = 0; i < count; ++i) {
        const float i_val = data[i].real();
        const float q_val = data[i].imag();
        out.write(reinterpret_cast<const char*>(&i_val), sizeof(float));
        out.write(reinterpret_cast<const char*>(&q_val), sizeof(float));
    }

    if (!out) {
        throw std::runtime_error("DebugDumper: write failed for " + path);
    }
}

void DebugDumper::writeManifest(const DumpManifest& m) {
    if (!enabled_) {
        return;
    }

    std::filesystem::create_directories(out_dir_);

    const std::string path = out_dir_ + "/manifest.json";
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        throw std::runtime_error("DebugDumper: failed to open " + path);
    }

    out << "{\n"
        << "  \"sample_rate_hz\": " << m.sample_rate_hz << ",\n"
        << "  \"bandwidth_hz\": " << m.bandwidth_hz << ",\n"
        << "  \"fft_size\": " << m.fft_size << ",\n"
        << "  \"preamble_chirps\": " << m.preamble_chirps << ",\n"
        << "  \"sync_word_bin0\": " << m.sync_word_bin0 << ",\n"
        << "  \"sync_word_bin1\": " << m.sync_word_bin1 << ",\n"
        << "  \"align_offset\": " << m.align_offset << ",\n"
        << "  \"tx_samples\": " << m.tx_samples << ",\n"
        << "  \"rx_samples\": " << m.rx_samples << ",\n"
        << "  \"dechirped_symbol_index\": " << m.dechirped_symbol_index << ",\n"
        << "  \"fft_is_frequency_domain\": "
        << (m.fft_is_frequency_domain ? "true" : "false") << "\n"
        << "}\n";

    if (!out) {
        throw std::runtime_error("DebugDumper: write failed for " + path);
    }
}

}  // namespace phy
