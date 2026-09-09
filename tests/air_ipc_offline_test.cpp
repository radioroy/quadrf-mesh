// Offline tests for the PHY to node air-frame IPC. No radio hardware.

#include <phy/mesh/packet.hpp>
#include <quadrf/air_ipc.hpp>
#include <tests/air_ipc_golden_vectors.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <vector>

using namespace quadrf::air_ipc;

namespace {

int g_fail = 0;

#define CHECK(cond, msg)                          \
    do {                                          \
        if (!(cond)) {                            \
            std::cerr << "FAIL: " << msg << "\n"; \
            ++g_fail;                             \
        } else {                                  \
            std::cout << "PASS: " << msg << "\n"; \
        }                                         \
    } while (0)

std::vector<uint8_t> makeAir(uint32_t from, uint32_t id, size_t enc_len) {
    phy::mesh::Packet pkt;
    pkt.header.to = phy::mesh::kBroadcastNode;
    pkt.header.from = from;
    pkt.header.id = id;
    pkt.header.hop_limit = 3;
    pkt.header.hop_start = 3;
    pkt.header.channel = phy::mesh::kDefaultChannelHash;
    pkt.encrypted.assign(enc_len, 0xA5);
    std::vector<uint8_t> air;
    if (!phy::mesh::packPacket(pkt, air)) {
        return {};
    }
    return air;
}

}  // namespace

int main() {
    {
        std::vector<uint8_t> vector_air(16);
        for (size_t i = 0; i < vector_air.size(); ++i) {
            vector_air[i] = static_cast<uint8_t>(i);
        }

        TxEnqueue tx{0x1122334455667788ULL, vector_air};
        std::vector<uint8_t> tx_wire;
        CHECK(encodeTx(tx, tx_wire), "encode frozen TX vector");
        CHECK(std::equal(tx_wire.begin(), tx_wire.end(), golden::kTxV1.begin(),
                         golden::kTxV1.end()),
              "full TX v1 golden bytes");

        RxIndicate rx;
        rx.snr_db = -7.25f;
        rx.rssi_dbm = -91;
        rx.cfo_hz = 12500;
        rx.rate_ppm = 1.5f;
        rx.freq_hz = 0x1122334455667788ULL;
        rx.air = vector_air;
        std::vector<uint8_t> rx_wire;
        CHECK(encodeRx(rx, rx_wire), "encode frozen RX vector");
        CHECK(std::equal(rx_wire.begin(), rx_wire.end(), golden::kRxV1.begin(),
                         golden::kRxV1.end()),
              "full RX v1 golden bytes");

        Deframer tx_deframer;
        std::vector<Deframer::Frame> frames;
        CHECK(tx_deframer.feed(golden::kTxV1.data(), golden::kTxV1.size(), frames) == 1,
              "decode frozen TX vector frame");
        TxEnqueue decoded_tx;
        CHECK(decodeTx(frames.front().body, decoded_tx) && decoded_tx.freq_hz == tx.freq_hz &&
                  decoded_tx.air == vector_air,
              "decode frozen TX vector body");

        Deframer rx_deframer;
        frames.clear();
        CHECK(rx_deframer.feed(golden::kRxV1.data(), golden::kRxV1.size(), frames) == 1,
              "decode frozen RX vector frame");
        RxIndicate decoded_rx;
        CHECK(decodeRx(frames.front().body, decoded_rx) && decoded_rx.freq_hz == rx.freq_hz &&
                  decoded_rx.rssi_dbm == -91 && decoded_rx.cfo_hz == 12500 &&
                  decoded_rx.rate_ppm > 1.49f && decoded_rx.rate_ppm < 1.51f &&
                  decoded_rx.air == vector_air,
              "decode frozen RX vector body");
    }

    const auto air = makeAir(0xE58F0001, 0xA11CE, 24);
    CHECK(!air.empty() && air.size() == 16 + 24, "build air frame");

    CHECK(frequencyMHzToQuantizedHz(5800.125) == 5800125000ULL,
          "MHz conversion preserves 1 kHz quantum");
    CHECK(frequencyMHzToQuantizedHz(static_cast<double>(5800.125f)) == 5800125000ULL,
          "float MHz converted in double has stable quantum");
    CHECK(quantizeFrequencyHz(5800125499ULL) == 5800125000ULL,
          "sub-quantum value rounds down");
    CHECK(quantizeFrequencyHz(5800125500ULL) == 5800126000ULL,
          "half-quantum value rounds up");
    CHECK(frequencyRequestAccepted(0, 5800000000ULL),
          "zero frequency accepts PHY authority");
    CHECK(frequencyRequestAccepted(5800000000ULL, 5800000000ULL),
          "matching frequency accepts PHY authority");
    CHECK(frequencyRequestAccepted(5800125001ULL, 5800125000ULL),
          "sub-quantum representation accepts same center");
    CHECK(frequencyRequestAccepted(5800126000ULL, 5800125000ULL),
          "dynamic tuning accepts requested frequency");

    // --- TX encode/decode ---
    {
        TxEnqueue tx;
        tx.freq_hz = 5800000000ull;
        tx.air = air;
        std::vector<uint8_t> framed;
        CHECK(encodeTx(tx, framed), "encode TX");
        CHECK(framed.size() == kFrameHeaderLen + kTxMetaLen + air.size(), "TX frame size");
        CHECK(framed[0] == kMagic0 && framed[1] == kMagic1, "TX magic");
        CHECK(framed[2] == kVersion, "TX version");
        CHECK(framed[3] == static_cast<uint8_t>(MsgType::kTxEnqueue), "TX type");

        Deframer d;
        std::vector<Deframer::Frame> frames;
        CHECK(d.feed(framed, frames) == 1, "deframe TX count");
        CHECK(frames[0].type == MsgType::kTxEnqueue, "deframe TX type");

        TxEnqueue got;
        CHECK(decodeTx(frames[0].body, got), "decode TX");
        CHECK(got.freq_hz == tx.freq_hz && got.air == air, "TX roundtrip");
    }

    // --- RX encode/decode (incl. negative SNR, negative CFO and negative PPM) ---
    {
        RxIndicate rx;
        rx.snr_db = -7.25f;
        rx.rssi_dbm = -91;
        rx.cfo_hz = -13500;
        rx.rate_ppm = -2.75f;
        rx.freq_hz = 5800000000ull;
        rx.air = air;
        std::vector<uint8_t> framed;
        CHECK(encodeRx(rx, framed), "encode RX");

        Deframer d;
        std::vector<Deframer::Frame> frames;
        // wake noise + frame
        std::vector<uint8_t> stream = {0x00, 0x51, 0x51, 0x00};
        stream.insert(stream.end(), framed.begin(), framed.end());
        CHECK(d.feed(stream, frames) == 1, "deframe RX amid noise");

        RxIndicate got;
        CHECK(decodeRx(frames[0].body, got), "decode RX");
        CHECK(got.rssi_dbm == rx.rssi_dbm && got.freq_hz == rx.freq_hz && got.cfo_hz == rx.cfo_hz, "RX meta");
        CHECK(got.air == air, "RX air");
        // centi-dB quantization: within 0.01
        CHECK(got.snr_db > -7.26f && got.snr_db < -7.24f, "RX SNR centi");
        CHECK(got.rate_ppm > -2.76f && got.rate_ppm < -2.74f, "RX rate PPM centi");
    }

    // --- reject oversize / short air / bad version ---
    {
        Deframer d;
        std::vector<Deframer::Frame> frames;
        const uint8_t oversize[] = {kMagic0, kMagic1, kVersion,
                                    static_cast<uint8_t>(MsgType::kTxEnqueue), 0x02, 0x00};
        CHECK(d.feed(oversize, sizeof(oversize), frames) == 0, "reject oversize body");
        CHECK(d.framesRejected() >= 1, "oversize counted");
    }
    {
        TxEnqueue tx;
        tx.air.assign(8, 0);  // shorter than header
        std::vector<uint8_t> framed;
        CHECK(!encodeTx(tx, framed), "reject short air");
    }
    {
        TxEnqueue tx;
        tx.air.assign(kMaxAirLen + 1, 0);
        std::vector<uint8_t> framed;
        CHECK(!encodeTx(tx, framed), "reject long air");
    }
    {
        // Valid framing but body too short for TX meta+air
        std::vector<uint8_t> framed = {kMagic0, kMagic1, kVersion,
                                       static_cast<uint8_t>(MsgType::kTxEnqueue), 0x00, 0x02,
                                       0x11, 0x22};
        Deframer d;
        std::vector<Deframer::Frame> frames;
        CHECK(d.feed(framed, frames) == 1, "short body still frames");
        TxEnqueue got;
        CHECK(!decodeTx(frames[0].body, got), "decode rejects short TX body");
    }

    // --- snr helpers ---
    {
        CHECK(snrToCenti(12.5f) == 1250, "snrToCenti");
        CHECK(snrFromCenti(-725) > -7.26f && snrFromCenti(-725) < -7.24f, "snrFromCenti");
    }

    if (g_fail != 0) {
        std::cerr << g_fail << " failure(s)\n";
        return 1;
    }
    std::cout << "ALL PASS\n";
    return 0;
}
