#pragma once

#include <phy/types.hpp>

#include <SoapySDR/Formats.hpp>

#include <cstddef>
#include <vector>

namespace SoapySDR {
class Device;
class Stream;
}

namespace phy {

class StreamPair {
public:
    StreamPair() = default;
    ~StreamPair();

    StreamPair(const StreamPair&) = delete;
    StreamPair& operator=(const StreamPair&) = delete;

    void setupRx(SoapySDR::Device* dev, size_t channel = 0,
                 const char* format = SOAPY_SDR_CF32);
    void setupTx(SoapySDR::Device* dev, size_t channel = 0,
                 const char* format = SOAPY_SDR_CF32);

    void activate();
    void deactivate();

    size_t mtu() const { return mtu_; }

    int read(Sample* buffer, size_t numSamples, long timeoutUs = 100000);
    int write(const Sample* buffer, size_t numSamples, long timeoutUs = 1000000);

    bool rxActive() const { return rxStream_ != nullptr; }
    bool txActive() const { return txStream_ != nullptr; }

private:
    SoapySDR::Device* dev_ = nullptr;
    SoapySDR::Stream* rxStream_ = nullptr;
    SoapySDR::Stream* txStream_ = nullptr;
    size_t mtu_ = 0;
};

}  // namespace phy
