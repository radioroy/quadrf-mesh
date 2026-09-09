#include <phy/config.hpp>
#include <phy/dsp/chirp_generator.hpp>
#include <phy/dsp/dechirper.hpp>
#include <phy/dsp/fft_engine.hpp>
#include <phy/dsp/peak_detector.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <vector>

namespace {

bool testRoundTrip(const phy::PhyConfig& cfg) {
    phy::ChirpGenerator gen(cfg);
    const uint32_t n = gen.fftSize();

    std::vector<phy::Sample> work(n);
    phy::dechirp(gen.upChirp().data(), gen.downChirp().data(), work.data(), n);

    phy::FftEngine fft(n);
    fft.forward(work.data());
    const phy::PeakResult peak = phy::findPeakBin(fft.spectrum().data(), n);

    std::cout << "Round-trip peak bin: " << peak.bin << " (expected 0)\n";
    return peak.bin == 0;
}

bool testShift(const phy::PhyConfig& cfg, size_t chip_bin) {
    phy::ChirpGenerator gen(cfg);
    const uint32_t n = gen.fftSize();
    const size_t osr = static_cast<size_t>(std::lround(phy::oversampleFactor(cfg)));
    const size_t offset = chip_bin * osr;

    const phy::IQBuffer shifted = gen.upChirpWithOffset(offset);
    std::vector<phy::Sample> work(n);
    phy::dechirp(shifted.data(), gen.downChirp().data(), work.data(), n);

    phy::FftEngine fft(n);
    fft.forward(work.data());
    const phy::PeakResult peak = phy::findPeakBin(fft.spectrum().data(), n);

    const size_t expected = chip_bin % n;
    std::cout << "Shift test chip_bin=" << chip_bin << " peak bin: " << peak.bin << " (expected "
              << expected << ")\n";
    return peak.bin == expected;
}

bool testAwgn(const phy::PhyConfig& cfg) {
    phy::ChirpGenerator gen(cfg);
    const uint32_t n = gen.fftSize();
    const size_t osr = static_cast<size_t>(std::lround(phy::oversampleFactor(cfg)));
    const size_t chip_bin = 42;
    const size_t shift = chip_bin * osr;

    phy::IQBuffer noisy = gen.upChirpWithOffset(shift);
    std::mt19937 rng(12345);
    std::normal_distribution<float> noise(0.0f, 0.05f);

    for (auto& s : noisy) {
        s += phy::Sample(noise(rng), noise(rng));
    }

    std::vector<phy::Sample> work(n);
    phy::dechirp(noisy.data(), gen.downChirp().data(), work.data(), n);

    phy::FftEngine fft(n);
    fft.forward(work.data());
    const phy::PeakResult peak = phy::findPeakBin(fft.spectrum().data(), n);

    const size_t expected = chip_bin % n;
    std::cout << "AWGN test peak bin: " << peak.bin << " (expected " << expected << ")\n";
    return peak.bin == expected;
}

}  // namespace

int main() {
    phy::PhyConfig cfg;
    cfg.sample_rate_hz = 1e6;
    cfg.bandwidth_hz = 500e3;
    cfg.spreading_factor = 11;

    std::cout << "FFT size: " << phy::fftSize(cfg) << "\n";
    std::cout << "Sync nibble bins (0x2B): " << phy::syncNibbleBin(cfg, 2) << ", "
              << phy::syncNibbleBin(cfg, 11) << "\n";

    int failures = 0;
    if (!testRoundTrip(cfg)) {
        ++failures;
    }
    if (!testShift(cfg, 100)) {
        ++failures;
    }
    if (!testAwgn(cfg)) {
        ++failures;
    }

    if (failures == 0) {
        std::cout << "PASS: all offline chirp tests\n";
        return 0;
    }

    std::cout << "FAIL: " << failures << " test(s) failed\n";
    return 1;
}
