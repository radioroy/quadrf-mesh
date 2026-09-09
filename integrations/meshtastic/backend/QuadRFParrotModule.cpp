// SPDX-FileCopyrightText: 2026 Roy C. Gross
// SPDX-License-Identifier: GPL-3.0-only
#include "QuadRFParrotModule.h"
#include "QuadRFRadio.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"
#include "configuration.h"
#include <Arduino.h>
#include <algorithm>
#include <cstdio>
#include <cstring>

QuadRFParrotModule *QuadRFParrotModule::instance = nullptr;

QuadRFParrotModule::QuadRFParrotModule()
    : SinglePortModule("quadrf_parrot", meshtastic_PortNum_TEXT_MESSAGE_APP)
{
    instance = this;
    isPromiscuous = true;
}

QuadRFParrotModule::~QuadRFParrotModule()
{
    if (instance == this) {
        instance = nullptr;
    }
}

void QuadRFParrotModule::setEnabled(bool en)
{
    enabled_.store(en);
    LOG_INFO("QuadRFParrot: %s", en ? "ENABLED" : "DISABLED");
}

bool QuadRFParrotModule::wantPacket(const meshtastic_MeshPacket *p)
{
    if (!enabled_.load()) {
        return false;
    }
    return p && (p->decoded.portnum == ourPortNum ||
                 p->decoded.portnum == meshtastic_PortNum_RANGE_TEST_APP);
}

bool QuadRFParrotModule::isRateLimited(uint32_t from, uint32_t now_ms)
{
    for (auto &e : cooldown_) {
        if (e.from == from) {
            if ((uint32_t)(now_ms - e.last_ms) < 1000) {
                return true;
            }
            e.last_ms = now_ms;
            return false;
        }
    }
    cooldown_[cooldown_idx_].from = from;
    cooldown_[cooldown_idx_].last_ms = now_ms;
    cooldown_idx_ = (cooldown_idx_ + 1) % kCooldownSlots;
    return false;
}

ProcessMessage QuadRFParrotModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (!enabled_.load()) {
        return ProcessMessage::CONTINUE;
    }

    if (isFromUs(&mp)) {
        return ProcessMessage::CONTINUE;
    }

    if (mp.decoded.payload.size == 0) {
        return ProcessMessage::CONTINUE;
    }

    const char *text = reinterpret_cast<const char *>(mp.decoded.payload.bytes);
    size_t len = mp.decoded.payload.size;

    // Loop guard: never parrot a packet that already carries a parrot prefix
    if ((len >= 3 && std::strncmp(text, "[P:", 3) == 0) ||
        (len >= 7 && std::strncmp(text, "[PARROT", 7) == 0)) {
        LOG_DEBUG("QuadRFParrot: ignoring parrot loop packet id=0x%08x", mp.id);
        return ProcessMessage::CONTINUE;
    }

    uint32_t now = millis();
    if (isRateLimited(mp.from, now)) {
        LOG_DEBUG("QuadRFParrot: rate-limited from 0x%08x", (unsigned)mp.from);
        return ProcessMessage::CONTINUE;
    }

    QuadRFRadio::RxMetrics metrics;
    bool has_metrics = false;
    if (QuadRFRadio::instance) {
        has_metrics = QuadRFRadio::instance->getRxMetrics(mp.id, metrics);
    }

    char prefix[96];
    if (has_metrics) {
        float cfo_khz = static_cast<float>(metrics.cfo_hz) / 1000.0f;
        std::snprintf(prefix, sizeof(prefix), "[P: snr=%+.1fdB cfo=%+.1fkHz ppm=%+.1f] ",
                      metrics.snr_db, cfo_khz, metrics.rate_ppm);
    } else {
        std::snprintf(prefix, sizeof(prefix), "[P: snr=%+.1fdB] ", mp.rx_snr);
    }

    meshtastic_MeshPacket *reply = allocDataPacket();
    if (!reply) {
        LOG_WARN("QuadRFParrot: allocDataPacket returned null");
        return ProcessMessage::CONTINUE;
    }
    reply->to = NODENUM_BROADCAST;
    reply->channel = mp.channel;
    reply->want_ack = false;
    reply->hop_limit = 0;

    size_t prefix_len = std::strlen(prefix);
    size_t max_payload = sizeof(reply->decoded.payload.bytes);
    size_t copy_len = std::min(len, max_payload - prefix_len);

    std::memcpy(reply->decoded.payload.bytes, prefix, prefix_len);
    if (copy_len > 0) {
        std::memcpy(reply->decoded.payload.bytes + prefix_len, text, copy_len);
    }
    reply->decoded.payload.size = prefix_len + copy_len;

    LOG_INFO("QuadRFParrot: echoing packet id=0x%08x to 0x%08x (%zu bytes)",
             mp.id, (unsigned)reply->to, reply->decoded.payload.size);
    service->sendToMesh(reply);

    return ProcessMessage::CONTINUE;
}
