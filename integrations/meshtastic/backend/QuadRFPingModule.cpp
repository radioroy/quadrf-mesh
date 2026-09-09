// SPDX-FileCopyrightText: 2026 Roy C. Gross
// SPDX-License-Identifier: GPL-3.0-only
#include "QuadRFPingModule.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"
#include "mesh/MeshTypes.h"
#include <Arduino.h>
#include <cstdio>
#include <cstring>

QuadRFPingModule *QuadRFPingModule::instance = nullptr;

QuadRFPingModule::QuadRFPingModule()
    : SinglePortModule("quadrf_ping", meshtastic_PortNum_RANGE_TEST_APP)
{
    instance = this;
    loopbackOk = true;
    start();
}

QuadRFPingModule::~QuadRFPingModule()
{
    running_ = false;
    cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
    if (instance == this) {
        instance = nullptr;
    }
}

void QuadRFPingModule::start()
{
    if (!running_.exchange(true)) {
        thread_ = std::thread(&QuadRFPingModule::threadMain, this);
    }
}

void QuadRFPingModule::setIntervalSec(uint32_t sec)
{
    interval_sec_.store(sec);
    moduleConfig.has_range_test = true;
    moduleConfig.range_test.enabled = (sec > 0);
    moduleConfig.range_test.sender = sec;

    if (sec > 0) {
        LOG_INFO("QuadRFPing: enabled with interval %u s", sec);
    } else {
        LOG_INFO("QuadRFPing: disabled");
    }
    {
        std::lock_guard<std::mutex> lk(mu_);
    }
    cv_.notify_all();
}

void QuadRFPingModule::threadMain()
{
    LOG_INFO("QuadRFPing: worker thread started");
    while (running_) {
        uint32_t sec = interval_sec_.load();
        if (sec == 0) {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait(lk, [this]() {
                return !running_ || interval_sec_.load() > 0;
            });
            continue;
        }

        meshtastic_MeshPacket *p = allocDataPacket();
        if (p) {
            p->to = NODENUM_BROADCAST;
            p->decoded.want_response = false;
            p->hop_limit = 0;
            p->want_ack = false;

            seq_++;
            char heartbeatString[64];
            std::snprintf(heartbeatString, sizeof(heartbeatString), "seq %u", seq_);
            p->decoded.payload.size = std::strlen(heartbeatString);
            std::memcpy(p->decoded.payload.bytes, heartbeatString, p->decoded.payload.size);

            LOG_INFO("QuadRFPing: sending RangeTest heartbeat seq %u (interval %u s)", seq_, sec);
            service->sendToMesh(p);
        } else {
            LOG_WARN("QuadRFPing: allocDataPacket returned null");
        }

        // Wait interval or wakeup
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait_for(lk, std::chrono::seconds(sec), [this, sec]() {
                return !running_ || interval_sec_.load() != sec;
            });
        }
    }
    LOG_INFO("QuadRFPing: worker thread exiting");
}

ProcessMessage QuadRFPingModule::handleReceived(const meshtastic_MeshPacket &/*mp*/)
{
    return ProcessMessage::CONTINUE;
}
