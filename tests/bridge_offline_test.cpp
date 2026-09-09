// Offline tests for StreamAPI framer + PhoneAPI encode/crypto/air roundtrip.
// No radio hardware.

#include <phy/bridge/crypto.hpp>
#include <phy/bridge/pb_wire.hpp>
#include <phy/bridge/phone_api.hpp>
#include <phy/bridge/session.hpp>
#include <phy/stream/framer.hpp>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

using namespace phy::stream;
using namespace phy::bridge;

namespace {

int g_fail = 0;

#define CHECK(cond, msg)                               \
    do {                                               \
        if (!(cond)) {                                 \
            std::cerr << "FAIL: " << msg << "\n";      \
            ++g_fail;                                  \
        } else {                                       \
            std::cout << "PASS: " << msg << "\n";      \
        }                                              \
    } while (0)

}  // namespace

int main() {
    // --- framer ---
    {
        const std::vector<uint8_t> payload = {0x08, 0x96, 0x9e, 0x04};
        std::vector<uint8_t> framed;
        CHECK(frame(payload, framed), "frame ok");
        CHECK(framed.size() == 4 + payload.size(), "frame size");
        CHECK(framed[0] == 0x94 && framed[1] == 0xC3, "start bytes");
        CHECK(framed[2] == 0 && framed[3] == payload.size(), "len");

        Deframer d;
        std::vector<std::vector<uint8_t>> out;
        // wake bytes + framed
        std::vector<uint8_t> stream = {0x94, 0x94, 0x94, 0x94};
        stream.insert(stream.end(), framed.begin(), framed.end());
        // noise then another frame
        stream.push_back(0x41);
        stream.insert(stream.end(), framed.begin(), framed.end());
        CHECK(d.feed(stream, out) == 2, "deframe count");
        CHECK(out[0] == payload && out[1] == payload, "deframe payload");
    }

    // reject oversize length
    {
        Deframer d;
        std::vector<std::vector<uint8_t>> out;
        const uint8_t bad[] = {0x94, 0xC3, 0x02, 0x01};  // len 513
        CHECK(d.feed(bad, sizeof(bad), out) == 0, "reject oversize");
        CHECK(d.framesRejected() == 1, "reject counted");
    }

    // --- channel hash ---
    {
        std::vector<uint8_t> key;
        CHECK(expandPsk(std::vector<uint8_t>{0x01}.data(), 1, key) && key.size() == 16,
              "expand default psk");
        CHECK(channelHash("LongFast", key) == 8, "LongFast hash == 8");
    }

    // --- AES-CTR roundtrip ---
    {
        std::vector<uint8_t> key;
        expandPsk(std::vector<uint8_t>{0x01}.data(), 1, key);
        std::vector<uint8_t> plain = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
        std::vector<uint8_t> ct = plain;
        CHECK(aesCtr(key, 0x12345678, 0xABCDEFFF, ct), "encrypt");
        CHECK(ct != plain, "ciphertext differs");
        CHECK(aesCtr(key, 0x12345678, 0xABCDEFFF, ct), "decrypt");
        CHECK(ct == plain, "roundtrip");
    }

    // --- PhoneAPI MeshPacket ↔ air ---
    {
        SessionConfig cfg;
        cfg.node_num = 0x11111111;
        Session session(cfg);

        MeshPacketPb pkt;
        pkt.from = cfg.node_num;
        pkt.to = phy::mesh::kBroadcastNode;
        pkt.id = 0x55AA55AA;
        pkt.hop_limit = 3;
        Data d;
        d.portnum = kPortTextMessage;
        const char* msg = "hello-quadrf";
        d.payload.assign(msg, msg + std::strlen(msg));
        pkt.decoded = d;

        phy::mesh::Packet air;
        CHECK(meshPacketToAir(pkt, session.channelKey(), session.channelHash(), cfg.node_num, air),
              "to air");
        CHECK(air.header.channel == 8, "air channel hash");
        CHECK(air.header.from == cfg.node_num && air.header.id == pkt.id, "air header");
        CHECK(!air.encrypted.empty(), "air ciphertext");

        MeshPacketPb back;
        CHECK(airToMeshPacket(air, session.channelKey(), back), "from air");
        CHECK(back.decoded.has_value(), "decoded");
        CHECK(back.decoded->portnum == kPortTextMessage, "port");
        CHECK(std::string(back.decoded->payload.begin(), back.decoded->payload.end()) == msg,
              "text");
    }

    // --- session want_config + packet enqueue ---
    {
        SessionConfig cfg;
        cfg.node_num = 0x22222222;
        Session session(cfg);

        ToRadio want;
        want.kind = ToRadioKind::kWantConfig;
        want.want_config_id = 69420;
        auto bytes = encodeToRadio(want);
        std::vector<std::vector<uint8_t>> air;
        session.handleToRadio(bytes.data(), bytes.size(), air);
        CHECK(air.empty(), "want_config no air");

        size_t n = 0;
        std::vector<uint8_t> fr;
        bool saw_my = false, saw_complete = false, saw_channel = false;
        while (session.popFromRadio(fr)) {
            ++n;
            FromRadio decoded;
            // peel kind via pb walk
            std::vector<phy::pb::Field> fields;
            phy::pb::parseFields(fr, fields);
            for (const auto& f : fields) {
                if (f.number == 3) {
                    saw_my = true;
                }
                if (f.number == 7 && f.varint == 69420) {
                    saw_complete = true;
                }
                if (f.number == 10) {
                    saw_channel = true;
                }
            }
        }
        CHECK(n >= 4, "config messages");
        CHECK(saw_my && saw_complete && saw_channel, "my_info+channel+complete");

        MeshPacketPb pkt;
        pkt.from = cfg.node_num;
        pkt.id = 99;
        pkt.hop_limit = 3;
        Data d;
        d.portnum = kPortTextMessage;
        d.payload = {'x'};
        pkt.decoded = d;
        ToRadio tor;
        tor.kind = ToRadioKind::kPacket;
        tor.packet = pkt;
        bytes = encodeToRadio(tor);
        air.clear();
        session.handleToRadio(bytes.data(), bytes.size(), air);
        CHECK(air.size() == 1, "packet → air");
        CHECK(air[0].size() >= phy::mesh::kHeaderLength + 1, "air size");

        session.handleAirFrame(air[0].data(), air[0].size(), 12.5f);
        CHECK(session.rxDelivered() == 1, "rx delivered");
        CHECK(session.popFromRadio(fr), "fromradio packet");
    }

    std::cout << "\n" << (g_fail ? "SOME FAILURES" : "ALL PASS") << " (" << g_fail
              << " failed)\n";
    return g_fail ? 1 : 0;
}
