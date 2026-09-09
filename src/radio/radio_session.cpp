#include <phy/radio/radio_session.hpp>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>

#include <iostream>
#include <stdexcept>

namespace phy {

void RadioSession::DeviceDeleter::operator()(SoapySDR::Device* d) const {
    if (d) {
        SoapySDR::Device::unmake(d);
    }
}

RadioSession::~RadioSession() {
    close();
}

void RadioSession::open(const std::string& driver) {
    close();

    SoapySDR::Kwargs args;
    args["driver"] = driver;

    SoapySDR::Device* raw = SoapySDR::Device::make(args);
    if (!raw) {
        throw std::runtime_error("SoapySDR::Device::make failed for driver=" + driver);
    }

    dev_.reset(raw);
}

void RadioSession::close() {
    dev_.reset();
}

void RadioSession::setSampleRate(int direction, size_t channel, double rateHz) {
    if (!dev_) {
        throw std::runtime_error("RadioSession not open");
    }
    dev_->setSampleRate(direction, channel, rateHz);
}

double RadioSession::sampleRate(int direction, size_t channel) const {
    if (!dev_) {
        throw std::runtime_error("RadioSession not open");
    }
    return dev_->getSampleRate(direction, channel);
}

void RadioSession::printInfo() const {
    if (!dev_) {
        std::cout << "RadioSession: not open\n";
        return;
    }

    std::cout << "Hardware: " << dev_->getHardwareInfo()["hardware"] << "\n";
    std::cout << "Driver:   " << dev_->getDriverKey() << "\n";
    std::cout << "RX rate:  " << dev_->getSampleRate(SOAPY_SDR_RX, 0) / 1e6 << " MSps\n";
    std::cout << "TX rate:  " << dev_->getSampleRate(SOAPY_SDR_TX, 0) / 1e6 << " MSps\n";
}

}  // namespace phy
