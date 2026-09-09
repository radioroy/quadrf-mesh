#pragma once

#include <memory>
#include <string>

namespace SoapySDR {
class Device;
}

namespace phy {

class RadioSession {
public:
    RadioSession() = default;
    ~RadioSession();

    RadioSession(const RadioSession&) = delete;
    RadioSession& operator=(const RadioSession&) = delete;

    void open(const std::string& driver = "mipi");
    void close();

    SoapySDR::Device* device() { return dev_.get(); }
    const SoapySDR::Device* device() const { return dev_.get(); }

    bool isOpen() const { return dev_ != nullptr; }

    void setSampleRate(int direction, size_t channel, double rateHz);
    double sampleRate(int direction, size_t channel) const;

    void printInfo() const;

private:
    struct DeviceDeleter {
        void operator()(SoapySDR::Device* d) const;
    };

    std::unique_ptr<SoapySDR::Device, DeviceDeleter> dev_;
};

}  // namespace phy
