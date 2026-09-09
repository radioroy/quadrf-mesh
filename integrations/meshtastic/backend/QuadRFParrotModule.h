// SPDX-FileCopyrightText: 2026 Roy C. Gross
// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "SinglePortModule.h"
#include <array>
#include <atomic>
#include <cstdint>

class QuadRFParrotModule : public SinglePortModule
{
public:
    static QuadRFParrotModule *instance;

    QuadRFParrotModule();
    ~QuadRFParrotModule() override;

    void setEnabled(bool en);
    bool isEnabled() const { return enabled_.load(); }

protected:
    bool wantPacket(const meshtastic_MeshPacket *p) override;
    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

private:
    std::atomic<bool> enabled_{false};

    struct CooldownEntry {
        uint32_t from = 0;
        uint32_t last_ms = 0;
    };
    static constexpr size_t kCooldownSlots = 16;
    std::array<CooldownEntry, kCooldownSlots> cooldown_{};
    size_t cooldown_idx_ = 0;

    bool isRateLimited(uint32_t from, uint32_t now_ms);
};
