#pragma once

#include <phy/types.hpp>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <vector>

namespace phy {

// Single-producer single-consumer ring for RX capture threads.
class RingBuffer {
public:
    explicit RingBuffer(size_t capacity_samples) : data_(capacity_samples) {}

    bool push(const Sample* samples, size_t count) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (count > freeLocked()) {
            return false;
        }
        for (size_t i = 0; i < count; ++i) {
            data_[write_pos_] = samples[i];
            write_pos_ = (write_pos_ + 1) % data_.size();
        }
        size_ += count;
        cv_.notify_all();
        return true;
    }

    size_t pop(Sample* out, size_t max_count, long timeout_ms = 100) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                          [this] { return size_ > 0; })) {
            return 0;
        }
        const size_t n = std::min(max_count, size_);
        for (size_t i = 0; i < n; ++i) {
            out[i] = data_[read_pos_];
            read_pos_ = (read_pos_ + 1) % data_.size();
        }
        size_ -= n;
        return n;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return size_;
    }

private:
    size_t freeLocked() const { return data_.size() - size_; }

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<Sample> data_;
    size_t read_pos_ = 0;
    size_t write_pos_ = 0;
    size_t size_ = 0;
};

}  // namespace phy
