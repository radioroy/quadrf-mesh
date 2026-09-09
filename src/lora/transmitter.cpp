#include <phy/lora/transmitter.hpp>

#include <phy/lora/coding.hpp>

#include <algorithm>
#include <utility>

namespace phy::lora {

Transmitter::Transmitter(const LoraParams& params, float amplitude)
    : params_(params), amplitude_(amplitude), mod_(params, amplitude) {
    sps_ = mod_.samplesPerSymbol();
}

bool Transmitter::enqueue(const std::vector<uint8_t>& payload) {
    if (payload.size() > 255) {
        return false;
    }
    const std::vector<uint16_t> chips = encodeFrame(params_, payload);
    IQBuffer frame = mod_.frame(chips);
    std::lock_guard<std::mutex> lk(mu_);
    queue_.push_back(std::move(frame));
    return true;
}

void Transmitter::enqueueFrame(IQBuffer frame) {
    std::lock_guard<std::mutex> lk(mu_);
    queue_.push_back(std::move(frame));
}

size_t Transmitter::queued() const {
    std::lock_guard<std::mutex> lk(mu_);
    return queue_.size();
}

bool Transmitter::isIdle() const {
    std::lock_guard<std::mutex> lk(mu_);
    return queue_.empty() && burst_pos_ >= burst_.size();
}

void Transmitter::clearQueue() {
    std::lock_guard<std::mutex> lk(mu_);
    queue_.clear();
}

void Transmitter::startNextBurstLocked() {
    if (queue_.empty() || burst_pos_ < burst_.size()) {
        return;
    }
    IQBuffer frame = std::move(queue_.front());
    queue_.pop_front();

    const size_t warmup_n = static_cast<size_t>(warmup_syms_) * sps_;
    const size_t gap_n = static_cast<size_t>(gap_syms_) * sps_;
    burst_.clear();
    burst_.reserve(warmup_n + frame.size() + gap_n);
    burst_.insert(burst_.end(), warmup_n, Sample(amplitude_, 0.0f));
    burst_.insert(burst_.end(), frame.begin(), frame.end());
    burst_.insert(burst_.end(), gap_n, Sample(0.0f, 0.0f));
    burst_pos_ = 0;
    ++frames_started_;
}

void Transmitter::fillFromBurst(Sample* out, size_t n, size_t& written) {
    const size_t avail = burst_.size() - burst_pos_;
    const size_t take = std::min(n - written, avail);
    if (take == 0) {
        return;
    }
    std::copy_n(burst_.data() + burst_pos_, take, out + written);
    burst_pos_ += take;
    written += take;
    if (burst_pos_ >= burst_.size()) {
        burst_.clear();
        burst_pos_ = 0;
    }
}

void Transmitter::pull(Sample* out, size_t n) {
    size_t written = 0;
    {
        std::lock_guard<std::mutex> lk(mu_);
        while (written < n) {
            if (burst_pos_ >= burst_.size()) {
                startNextBurstLocked();
            }
            if (burst_pos_ < burst_.size()) {
                fillFromBurst(out, n, written);
                continue;
            }
            // Idle: fill the rest with silence and leave.
            std::fill(out + written, out + n, Sample(0.0f, 0.0f));
            written = n;
        }
    }
    samples_emitted_ += n;
}

}  // namespace phy::lora
