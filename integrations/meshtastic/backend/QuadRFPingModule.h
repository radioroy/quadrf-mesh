// SPDX-FileCopyrightText: 2026 Roy C. Gross
// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "SinglePortModule.h"
#include "configuration.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

class QuadRFPingModule : public SinglePortModule
{
public:
    static QuadRFPingModule *instance;

    QuadRFPingModule();
    ~QuadRFPingModule() override;

    void setIntervalSec(uint32_t sec);
    uint32_t getIntervalSec() const { return interval_sec_.load(); }
    void start();

protected:
    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

private:
    void threadMain();

    std::atomic<uint32_t> interval_sec_{0};
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::mutex mu_;
    std::condition_variable cv_;
    uint32_t seq_{0};
};
