// SPDX-FileCopyrightText: 2026 Roy C. Gross
// SPDX-License-Identifier: GPL-3.0-only
// Minimal adapter-side compile and wire-compatibility smoke test.

#include <quadrf/air_ipc.hpp>
#include <tests/air_ipc_golden_vectors.hpp>

#include <algorithm>
#include <vector>

int main()
{
    static_assert(quadrf::air_ipc::kProtocolVersion == 1);
    const std::vector<uint8_t> wire(quadrf::air_ipc::golden::kTxV1.begin(),
                                    quadrf::air_ipc::golden::kTxV1.end());
    quadrf::air_ipc::Deframer deframer;
    std::vector<quadrf::air_ipc::Deframer::Frame> frames;
    if (deframer.feed(wire, frames) != 1 || frames.size() != 1)
        return 1;
    quadrf::air_ipc::TxEnqueue tx;
    if (!quadrf::air_ipc::decodeTx(frames.front().body, tx))
        return 2;
    if (tx.freq_hz != 0x1122334455667788ULL || tx.air.size() != 16)
        return 3;
    for (size_t i = 0; i < tx.air.size(); ++i)
        if (tx.air[i] != i)
            return 4;
    return 0;
}
