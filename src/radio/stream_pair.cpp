#include <phy/radio/stream_pair.hpp>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>

#include <stdexcept>
#include <vector>

namespace phy {

StreamPair::~StreamPair() {
    deactivate();
    if (dev_) {
        if (rxStream_) {
            dev_->closeStream(rxStream_);
            rxStream_ = nullptr;
        }
        if (txStream_) {
            dev_->closeStream(txStream_);
            txStream_ = nullptr;
        }
    }
}

void StreamPair::setupRx(SoapySDR::Device* dev, size_t channel, const char* format) {
    if (!dev) {
        throw std::runtime_error("StreamPair::setupRx: null device");
    }
    dev_ = dev;

    if (rxStream_) {
        dev_->closeStream(rxStream_);
        rxStream_ = nullptr;
    }

    std::vector<size_t> channels{channel};
    rxStream_ = dev_->setupStream(SOAPY_SDR_RX, format, channels);
    if (!rxStream_) {
        throw std::runtime_error("StreamPair::setupRx: setupStream failed");
    }

    mtu_ = dev_->getStreamMTU(rxStream_);
    if (mtu_ == 0) {
        mtu_ = 4096;
    }
}

void StreamPair::setupTx(SoapySDR::Device* dev, size_t channel, const char* format) {
    if (!dev) {
        throw std::runtime_error("StreamPair::setupTx: null device");
    }
    dev_ = dev;

    if (txStream_) {
        dev_->closeStream(txStream_);
        txStream_ = nullptr;
    }

    std::vector<size_t> channels{channel};
    txStream_ = dev_->setupStream(SOAPY_SDR_TX, format, channels);
    if (!txStream_) {
        throw std::runtime_error("StreamPair::setupTx: setupStream failed");
    }

    const size_t tx_mtu = dev_->getStreamMTU(txStream_);
    if (tx_mtu > mtu_) {
        mtu_ = tx_mtu;
    }
    if (mtu_ == 0) {
        mtu_ = 4096;
    }
}

void StreamPair::activate() {
    if (!dev_) {
        throw std::runtime_error("StreamPair::activate: not configured");
    }
    if (rxStream_) {
        const int rc = dev_->activateStream(rxStream_);
        if (rc != 0) {
            throw std::runtime_error("StreamPair::activate: RX activate failed");
        }
    }
    if (txStream_) {
        const int rc = dev_->activateStream(txStream_);
        if (rc != 0) {
            throw std::runtime_error("StreamPair::activate: TX activate failed");
        }
    }
}

void StreamPair::deactivate() {
    if (!dev_) {
        return;
    }
    if (rxStream_) {
        dev_->deactivateStream(rxStream_);
    }
    if (txStream_) {
        dev_->deactivateStream(txStream_);
    }
}

int StreamPair::read(Sample* buffer, size_t numSamples, long timeoutUs) {
    if (!rxStream_ || !dev_) {
        return SOAPY_SDR_NOT_SUPPORTED;
    }

    void* buffs[1] = {buffer};
    int flags = 0;
    long long timeNs = 0;
    return dev_->readStream(rxStream_, buffs, numSamples, flags, timeNs, timeoutUs);
}

int StreamPair::write(const Sample* buffer, size_t numSamples, long timeoutUs) {
    if (!txStream_ || !dev_) {
        return SOAPY_SDR_NOT_SUPPORTED;
    }

    const void* buffs[1] = {buffer};
    int flags = 0;
    return dev_->writeStream(txStream_, buffs, numSamples, flags, 0, timeoutUs);
}

}  // namespace phy
