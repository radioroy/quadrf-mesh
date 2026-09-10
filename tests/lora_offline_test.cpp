// Offline validation of the LoRa frame codec and demodulator: no hardware.
//
//   1. coding chain round trip (encode -> decode) across cr/crc/lengths
//   2. modulate -> demodulate a clean frame
//   3. same with an applied sample-rate offset (the digital loopback case)
//   4. same with AWGN
//
// Exit code 0 only if every stage passes.

#include <phy/dsp/resampler.hpp>
#include <phy/lora/coding.hpp>
#include <phy/lora/demodulator.hpp>
#include <phy/lora/modulator.hpp>
#include <phy/lora/presets.hpp>
#include <phy/lora/receiver.hpp>
#include <phy/lora/symbol_demod.hpp>
#include <phy/lora/transmitter.hpp>
#include <phy/mesh/packet.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace phy;
using namespace phy::lora;
using namespace phy::mesh;

namespace {

int g_failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
    if (!ok) {
        ++g_failures;
    }
}

std::vector<uint8_t> testPayload(size_t len) {
    std::vector<uint8_t> p(len);
    for (size_t i = 0; i < len; ++i) {
        p[i] = static_cast<uint8_t>(i * 37 + 11);
    }
    return p;
}

// Simulate a TX->RX rate mismatch by resampling the waveform.
IQBuffer applyRateOffset(const IQBuffer& in, double ppm) {
    const double ratio = 1.0 + ppm * 1e-6;
    const size_t out_n = static_cast<size_t>(static_cast<double>(in.size()) / ratio) - 4;
    return resampleCubic(in, 1.0, ratio, out_n);
}

IQBuffer addNoise(const IQBuffer& in, double snr_db, uint32_t seed) {
    std::mt19937 rng(seed);
    // signal amplitude 0.8 -> power 0.64
    const double sigma = std::sqrt(0.64 / std::pow(10.0, snr_db / 10.0) / 2.0);
    std::normal_distribution<float> gauss(0.0f, static_cast<float>(sigma));
    IQBuffer out(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        out[i] = in[i] + Sample(gauss(rng), gauss(rng));
    }
    return out;
}

IQBuffer pad(const IQBuffer& frame, size_t before, size_t after) {
    IQBuffer out(before, Sample(0.0f, 0.0f));
    out.insert(out.end(), frame.begin(), frame.end());
    out.insert(out.end(), after, Sample(0.0f, 0.0f));
    return out;
}

bool decodeMatches(const FrameResult& r, const std::vector<uint8_t>& expect) {
    return r.synced && r.decode.header.valid && r.decode.crc_ok && r.decode.payload == expect;
}

IQBuffer applyCfo(const IQBuffer& in, double cfo_hz, double fs) {
    IQBuffer out(in.size());
    for (size_t n = 0; n < in.size(); ++n) {
        const double ph = 2.0 * M_PI * cfo_hz / fs * static_cast<double>(n);
        out[n] = in[n] * Sample(static_cast<float>(std::cos(ph)),
                                static_cast<float>(std::sin(ph)));
    }
    return out;
}

// RX gain settling at burst start, as measured OTA: amplitude climbing
// from ~25% over roughly one symbol.
IQBuffer applyGainRamp(const IQBuffer& in, size_t pad_before, size_t ramp_len) {
    IQBuffer out = in;
    for (size_t n = pad_before; n < in.size(); ++n) {
        const double t = static_cast<double>(n - pad_before) / static_cast<double>(ramp_len);
        const double g = (t >= 1.0) ? 1.0 : 0.25 + 0.75 * t;
        out[n] *= static_cast<float>(g);
    }
    return out;
}

// Drop or duplicate one sample mid-capture (DSI boundary slip at os=2 is
// half a chip). Exercises same-symbol sub-chip window refinement.
IQBuffer injectSampleSlip(const IQBuffer& in, size_t at, int delta) {
    IQBuffer out = in;
    if (at >= out.size()) {
        return out;
    }
    if (delta < 0) {
        out.erase(out.begin() + static_cast<ptrdiff_t>(at));
    } else if (delta > 0) {
        out.insert(out.begin() + static_cast<ptrdiff_t>(at), out[at]);
    }
    return out;
}

// Feed a capture in irregular chunks and collect everything the receiver
// produces, exercising the chunk-boundary state machine.
std::vector<ReceivedFrame> runReceiver(Receiver& rx, const IQBuffer& capture, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<size_t> chunk(1, 5000);
    size_t pos = 0;
    std::vector<ReceivedFrame> frames;
    while (pos < capture.size()) {
        const size_t n = std::min(chunk(rng), capture.size() - pos);
        rx.feed(capture.data() + pos, n);
        pos += n;
        ReceivedFrame f;
        while (rx.pop(f)) {
            frames.push_back(std::move(f));
        }
    }
    rx.flush();
    ReceivedFrame f;
    while (rx.pop(f)) {
        frames.push_back(std::move(f));
    }
    return frames;
}

bool rxMatches(const std::vector<ReceivedFrame>& frames, const std::vector<uint8_t>& expect,
               size_t count = 1) {
    size_t good = 0;
    for (const auto& f : frames) {
        if (f.synced && f.decode.header.valid && f.decode.crc_ok && f.decode.payload == expect) {
            ++good;
        }
    }
    return good == count;
}

}  // namespace

int main() {
    std::printf("== 1. coding chain round trip ==\n");
    for (uint8_t cr = 1; cr <= 4; ++cr) {
        for (const bool crc : {false, true}) {
            for (const size_t len : {1u, 7u, 32u, 100u}) {
                LoraParams p;
                p.cr = cr;
                p.has_crc = crc;
                const auto payload = testPayload(len);
                const auto chips = encodeFrame(p, payload);
                const DecodeResult d = decodeFrame(p, chips);
                const bool ok = d.header.valid && d.header.payload_len == len &&
                                d.header.cr == cr && d.crc_ok && d.payload == payload &&
                                chips.size() == frameSymbolCount(p, len);
                char buf[64];
                std::snprintf(buf, sizeof(buf), "cr=%u crc=%d len=%zu (%zu syms)", cr, crc, len,
                              chips.size());
                check(ok, buf);
            }
        }
    }

    std::printf("== 1b. single chip errors are corrected (cr>=3) ==\n");
    {
        LoraParams p;
        p.cr = 3;
        const auto payload = testPayload(16);
        auto chips = encodeFrame(p, payload);
        // +-1 chip errors: the most common demod failure mode
        chips[9] = static_cast<uint16_t>((chips[9] + 1) % chipCount(p));
        chips[15] = static_cast<uint16_t>((chips[15] + chipCount(p) - 1) % chipCount(p));
        const DecodeResult d = decodeFrame(p, chips);
        check(d.header.valid && d.crc_ok && d.payload == payload, "2 corrupted chips, cr=3");
    }

    LoraParams p;  // defaults: SF11, BW 500k, 1 MSps, cr=1, crc
    const auto payload = testPayload(16);
    const auto chips = encodeFrame(p, payload);
    const Modulator mod(p);
    const IQBuffer frame = mod.frame(chips);
    const size_t sps = mod.samplesPerSymbol();
    Demodulator demod(p);

    std::printf("== 2. clean modulate -> demodulate ==\n");
    {
        const IQBuffer rx = pad(frame, 3 * sps + 517, 3 * sps);
        const FrameResult r = demod.processFrame(rx);
        std::printf("     synced=%d rate=%.1f ppm sync_err=(%.2f, %.2f) pre=%zu\n", r.synced,
                    r.rate_ppm, r.sync_err0, r.sync_err1, r.preamble_symbols);
        check(decodeMatches(r, payload), "clean frame decodes");
    }

    std::printf("== 3. rate offset (loopback residual is ~ -1000 ppm) ==\n");
    for (const double ppm : {-3000.0, -1000.0, +1000.0, +3000.0}) {
        const IQBuffer shifted = applyRateOffset(pad(frame, 2 * sps + 231, 3 * sps), ppm);
        const FrameResult r = demod.processFrame(shifted);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%+.0f ppm (est %+.0f ppm) sync=%d hdr=%d crc=%d", ppm, r.rate_ppm,
                      r.synced, r.decode.header.valid, r.decode.crc_ok);
        check(decodeMatches(r, payload), buf);
    }

    std::printf("== 4. AWGN ==\n");
    for (const double snr : {20.0, 10.0, 5.0}) {
        const IQBuffer noisy = addNoise(pad(frame, 2 * sps + 231, 3 * sps), snr, 42);
        const FrameResult r = demod.processFrame(noisy);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "SNR %.0f dB", snr);
        check(decodeMatches(r, payload), buf);
    }

    std::printf("== 5. rate offset + AWGN ==\n");
    {
        const IQBuffer rx =
            addNoise(applyRateOffset(pad(frame, 2 * sps + 231, 3 * sps), -1000.0), 10.0, 7);
        const FrameResult r = demod.processFrame(rx);
        check(decodeMatches(r, payload), "-1000 ppm + 10 dB SNR");
    }

    std::printf("== 6. streaming receiver: chunked feed, timing sweep ==\n");
    for (const size_t off : {0u, 231u, 1024u, 2048u, 3011u, 4095u}) {
        Receiver rx(p);
        const auto frames = runReceiver(rx, pad(frame, 3 * sps + off, 3 * sps), 100 + off);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "pad offset %zu (tau %.1f)", off,
                      frames.empty() ? 0.0 : frames[0].tau_samples);
        check(rxMatches(frames, payload), buf);
    }

    std::printf("== 6b. channel SNR / SIR / LVL (Short Turbo) ==\n");
    {
        LoraParams st = meshtasticParams(MeshtasticPreset::kShortTurbo);
        const auto st_payload = testPayload(16);
        const auto st_chips = encodeFrame(st, st_payload);
        const Modulator st_mod(st);
        const IQBuffer st_frame = st_mod.frame(st_chips);
        const size_t st_sps = st_mod.samplesPerSymbol();

        Receiver rx_clean(st);
        const auto clean = runReceiver(rx_clean, pad(st_frame, 2 * st_sps + 231, 3 * st_sps), 71);
        check(rxMatches(clean, st_payload), "clean ShortTurbo decode");
        if (!clean.empty()) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "clean snr=%.1f sir=%.1f lvl=%.1f",
                          clean[0].snr_db, clean[0].sir_db, clean[0].lvl_dbfs);
            // Noiseless: FFT leakage still sets a ceiling; SIR is a
            // second-bin ratio; LVL near 10*log10(0.64) = -1.9 dBFS.
            const bool snr_hi = clean[0].snr_db > 8.0;
            const bool sir_hi = clean[0].sir_db > 10.0;
            const bool lvl_ok = clean[0].lvl_dbfs > -20.0 && clean[0].lvl_dbfs < 5.0;
            check(snr_hi && sir_hi && lvl_ok, buf);
        }

        Receiver rx_n(st);
        const auto noisy = runReceiver(
            rx_n, addNoise(pad(st_frame, 2 * st_sps + 231, 3 * st_sps), 20.0, 77), 72);
        check(rxMatches(noisy, st_payload), "20 dB AWGN ShortTurbo decode");
        if (!noisy.empty()) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "20 dB AWGN snr=%.1f sir=%.1f lvl=%.1f",
                          noisy[0].snr_db, noisy[0].sir_db, noisy[0].lvl_dbfs);
            // Channel SNR (PG removed), not old peak/median (~40 dB).
            const bool snr_ok = noisy[0].snr_db > 5.0 && noisy[0].snr_db < 28.0;
            const bool sir_ok = noisy[0].sir_db > 8.0;
            check(snr_ok && sir_ok, buf);
        }
    }

    std::printf("== 7. streaming receiver: back-to-back frames ==\n");
    {
        IQBuffer capture(2 * sps + 517, Sample(0, 0));
        for (int k = 0; k < 5; ++k) {
            capture.insert(capture.end(), frame.begin(), frame.end());
            capture.insert(capture.end(), 12 * sps, Sample(0, 0));
        }
        Receiver rx(p);
        const auto frames = runReceiver(rx, capture, 9);
        check(rxMatches(frames, payload, 5), "5 frames in one stream");
    }

    std::printf("== 8. streaming receiver: rate offset / CFO / both ==\n");
    for (const double ppm : {-3000.0, -1000.0, +1000.0, +3000.0}) {
        Receiver rx(p);
        const auto frames =
            runReceiver(rx, applyRateOffset(pad(frame, 2 * sps + 231, 3 * sps), ppm), 11);
        char buf[80];
        std::snprintf(buf, sizeof(buf), "%+.0f ppm (est %+.0f ppm)", ppm,
                      frames.empty() ? 0.0 : frames[0].rate_ppm);
        check(rxMatches(frames, payload), buf);
    }
    for (const double cfo_hz : {-2000.0, 2000.0}) {
        Receiver rx(p);
        const auto frames = runReceiver(
            rx, applyCfo(pad(frame, 2 * sps + 231, 3 * sps), cfo_hz, p.sample_rate_hz), 12);
        char buf[80];
        std::snprintf(buf, sizeof(buf), "CFO %+.0f Hz (est %+.0f Hz)", cfo_hz,
                      frames.empty() ? 0.0 : frames[0].cfo_hz);
        check(rxMatches(frames, payload), buf);
    }
    {
        Receiver rx(p);
        const IQBuffer capture = addNoise(
            applyCfo(applyRateOffset(pad(frame, 2 * sps + 231, 3 * sps), -1100.0), 1500.0,
                     p.sample_rate_hz),
            10.0, 21);
        const auto frames = runReceiver(rx, capture, 13);
        check(rxMatches(frames, payload), "-1100 ppm + CFO 1.5 kHz + 10 dB SNR");
    }

    std::printf("== 9. streaming receiver: OTA gain-settling ramp ==\n");
    {
        const size_t before = 2 * sps + 231;
        Receiver rx(p);
        const auto frames =
            runReceiver(rx, applyGainRamp(pad(frame, before, 3 * sps), before, sps), 14);
        check(rxMatches(frames, payload), "amplitude ramp over first symbol");
    }

    // Inter-board LO offset at 5.8 GHz is ~±12 kHz (~2 ppm). Both positive
    // and negative CFO offsets must decode successfully.
    std::printf("== 9b. ShortFast large CFO (inter-board LO offset) ==\n");
    {
        const LoraParams sf = meshtasticParams(MeshtasticPreset::kShortFast);
        const auto sf_chips = encodeFrame(sf, payload);
        const Modulator sf_mod(sf);
        const IQBuffer sf_frame = sf_mod.frame(sf_chips);
        const size_t sf_sps = sf_mod.samplesPerSymbol();
        const IQBuffer padded = pad(sf_frame, 2 * sf_sps + 231, 3 * sf_sps);
        // Sweep includes half-bin fractional CFO (bin = bw/2^sf = 1953 Hz);
        // ±11719 Hz is ~exactly 6 bins, ±12700 Hz lands mid-bin.
        for (const double cfo_hz : {-12700.0, -11700.0, 11700.0, 12700.0, 13700.0}) {
            Receiver rx(sf);
            const auto frames =
                runReceiver(rx, applyCfo(padded, cfo_hz, sf.sample_rate_hz), 61);
            char buf[80];
            std::snprintf(buf, sizeof(buf), "ShortFast CFO %+.0f Hz (est %+.0f Hz)", cfo_hz,
                          frames.empty() ? 0.0 : frames[0].cfo_hz);
            check(rxMatches(frames, payload), buf);
        }
        for (const double cfo_hz : {-12700.0, 12700.0}) {
            Receiver rx(sf);
            const IQBuffer capture = addNoise(
                applyCfo(applyRateOffset(padded, -10.0), cfo_hz, sf.sample_rate_hz), 15.0, 62);
            const auto frames = runReceiver(rx, capture, 63);
            char buf[80];
            std::snprintf(buf, sizeof(buf), "ShortFast CFO %+.0f Hz -10 ppm 15 dB (est %+.0f Hz)",
                          cfo_hz, frames.empty() ? 0.0 : frames[0].cfo_hz);
            check(rxMatches(frames, payload), buf);
        }
    }

    std::printf("== 10. Meshtastic ShortTurbo preset (BW 500 kHz, os 2) ==\n");
    {
        const LoraParams st = meshtasticParams(MeshtasticPreset::kShortTurbo);
        const auto st_chips = encodeFrame(st, payload);
        const Modulator st_mod(st);
        const IQBuffer st_frame = st_mod.frame(st_chips);
        const size_t st_sps = st_mod.samplesPerSymbol();
        IQBuffer capture(2 * st_sps + 777, Sample(0, 0));
        capture.insert(capture.end(), st_frame.begin(), st_frame.end());
        capture.insert(capture.end(), 3 * st_sps, Sample(0, 0));
        Receiver rx(st);
        auto frames = runReceiver(rx, applyRateOffset(capture, -10.0), 15);
        check(rxMatches(frames, payload), "ShortTurbo, -10 ppm");
        Receiver rx2(st);
        frames = runReceiver(rx2, addNoise(capture, 8.0, 33), 16);
        check(rxMatches(frames, payload), "ShortTurbo, 8 dB SNR");
    }

    std::printf("== 11. mid-frame sample slip (sub-chip refine) ==\n");
    {
        // Slip after preamble+sync+SFD so sync calibration is clean and the
        // data path has to absorb a half-chip step on its own.
        const size_t before = 2 * sps + 231;
        const size_t slip_at = before + (p.preamble_len + 5) * sps + sps / 2;
        for (const int delta : {-1, +1}) {
            Receiver rx(p);
            const IQBuffer capture =
                injectSampleSlip(pad(frame, before, 3 * sps), slip_at, delta);
            const auto frames = runReceiver(rx, capture, 40 + static_cast<uint32_t>(delta + 1));
            char buf[64];
            std::snprintf(buf, sizeof(buf), "sample %s mid-data", delta < 0 ? "drop" : "dup");
            check(rxMatches(frames, payload), buf);
        }
        {
            // Same slip under the ~loopback rate residual.
            Receiver rx(p);
            const IQBuffer capture = applyRateOffset(
                injectSampleSlip(pad(frame, before, 3 * sps), slip_at, -1), -1100.0);
            const auto frames = runReceiver(rx, capture, 42);
            check(rxMatches(frames, payload), "sample drop + -1100 ppm");
        }
    }

    // Meshtastic interop: whitening/CRC match gr-lora_sdr (EPFL) and gr-lora
    // (the same tables SX127x / Meshtastic radios use), then the 16-byte
    // PacketHeader + hop helpers on top of a ShortTurbo PHY round-trip.
    std::printf("== 12. Meshtastic whitening / CRC conventions ==\n");
    {
        // First 32 bytes of gr-lora_sdr lib/tables.h whitening_seq[].
        static const uint8_t kRefWhiten[32] = {
            0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE1, 0xC2, 0x85, 0x0B, 0x17, 0x2F, 0x5E,
            0xBC, 0x78, 0xF1, 0xE3, 0xC6, 0x8D, 0x1A, 0x34, 0x68, 0xD0, 0xA0, 0x40,
            0x80, 0x01, 0x02, 0x04, 0x08, 0x11, 0x23, 0x47};
        check(std::memcmp(kWhiteningSeq, kRefWhiten, sizeof(kRefWhiten)) == 0,
              "whitening matches gr-lora_sdr tables.h");
        check(kWhiteningSeq[254] == 0x7F, "whitening[254] == 0x7F");

        // Published LoRa payload-CRC vectors (poly 0x1021, init 0, last-2 XOR).
        const uint8_t hello[] = {'h', 'e', 'l', 'l', 'o'};
        check(payloadCrc(hello, sizeof(hello)) == 0x0750, "CRC(\"hello\") == 0x0750");
        check(payloadCrc(payload.data(), payload.size()) == 0xBD0E,
              "CRC(i*37+11 x16) == 0xBD0E");
        const uint8_t zeros[8] = {};
        check(payloadCrc(zeros, sizeof(zeros)) == 0x0000, "CRC(zeros x8) == 0x0000");
        const uint8_t ones[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        check(payloadCrc(ones, sizeof(ones)) == 0x6820, "CRC(0xFF x8) == 0x6820");

        // Whitened first byte of a default-channel Meshtastic header (to=broadcast
        // starts 0xFF) must cancel the LFSR seed byte.
        check((static_cast<uint8_t>(0xFF) ^ kWhiteningSeq[0]) == 0x00,
              "broadcast LSB whitens to 0");
    }

    std::printf("== 13. Meshtastic PacketHeader + hop relay ==\n");
    {
        Packet pkt;
        pkt.header.to = kBroadcastNode;
        pkt.header.from = 0x78563412u;
        pkt.header.id = 0x01EFCDABU;
        pkt.header.hop_limit = 3;
        pkt.header.hop_start = 3;
        pkt.header.want_ack = false;
        pkt.header.via_mqtt = false;
        pkt.header.channel = kDefaultChannelHash;
        pkt.header.next_hop = kNoNextHop;
        pkt.header.relay_node = nodeIdByte(pkt.header.from);
        pkt.encrypted = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x11, 0x22, 0x33};

        // Exact 16-byte layout from mesh-algo / RadioInterface::beginSending.
        static const uint8_t kExpectHdr[16] = {
            0xFF, 0xFF, 0xFF, 0xFF,  // to = broadcast
            0x12, 0x34, 0x56, 0x78,  // from
            0xAB, 0xCD, 0xEF, 0x01,  // id
            0x63,                    // flags: hop_limit=3 | hop_start=3<<5
            0x08,                    // channel hash (default primary channel)
            0x00,                    // next_hop
            0x12,                    // relay_node = from LSB
        };
        uint8_t hdr[kHeaderLength] = {};
        packHeader(pkt.header, hdr);
        check(std::memcmp(hdr, kExpectHdr, sizeof(kExpectHdr)) == 0,
              "PacketHeader wire bytes");

        std::vector<uint8_t> air;
        check(packPacket(pkt, air) && air.size() == kHeaderLength + pkt.encrypted.size(),
              "packPacket length");
        Packet round;
        check(unpackPacket(air.data(), air.size(), round) && round.header.to == kBroadcastNode &&
                  round.header.from == pkt.header.from && round.header.id == pkt.header.id &&
                  round.header.hop_limit == 3 && round.header.hop_start == 3 &&
                  round.header.channel == kDefaultChannelHash &&
                  round.encrypted == pkt.encrypted,
              "pack/unpack round trip");

        const uint32_t us = 0xA1B2C3D4u;
        check(shouldRelay(pkt.header, us), "flood relay when next_hop unset");
        check(!shouldRelay(pkt.header, pkt.header.from), "no self-relay");
        PacketHeader to_us = pkt.header;
        to_us.to = us;
        check(!shouldRelay(to_us, us), "no relay when addressed to us");

        PacketHeader nh = pkt.header;
        nh.next_hop = 0x99;
        check(!shouldRelay(nh, us), "skip when next_hop is someone else");
        nh.next_hop = nodeIdByte(us);
        check(shouldRelay(nh, us), "relay when next_hop matches us");

        Packet relayed = pkt;
        check(prepareRelay(relayed, us) && relayed.header.hop_limit == 2 &&
                  relayed.header.relay_node == nodeIdByte(us),
              "prepareRelay decrements hop + stamps relay");
        PacketHeader dead = pkt.header;
        dead.hop_limit = 0;
        check(!shouldRelay(dead, us), "hop_limit 0 stops flood");

        // Flags: want_ack + via_mqtt + hop_start=5, hop_limit=2.
        PacketHeader f;
        f.hop_limit = 2;
        f.hop_start = 5;
        f.want_ack = true;
        f.via_mqtt = true;
        check(packFlags(f) == 0xBAu, "flags pack 0xBA");
        PacketHeader f2;
        unpackFlags(0xBA, f2);
        check(f2.hop_limit == 2 && f2.hop_start == 5 && f2.want_ack && f2.via_mqtt,
              "flags unpack 0xBA");
    }

    std::printf("== 14. Meshtastic airframe through ShortTurbo PHY ==\n");
    {
        Packet pkt;
        pkt.header.to = kBroadcastNode;
        pkt.header.from = 0x11223344u;
        pkt.header.id = 0x55667788u;
        pkt.header.hop_limit = kHopReliable;
        pkt.header.hop_start = kHopReliable;
        pkt.header.channel = kDefaultChannelHash;
        pkt.header.relay_node = nodeIdByte(pkt.header.from);
        pkt.encrypted = testPayload(24);

        std::vector<uint8_t> air;
        check(packPacket(pkt, air), "build airframe");

        const LoraParams st = meshtasticParams(MeshtasticPreset::kShortTurbo);
        const auto chips = encodeFrame(st, air);
        const DecodeResult d = decodeFrame(st, chips);
        check(d.header.valid && d.crc_ok && d.payload == air, "ShortTurbo encode/decode airframe");

        Packet decoded;
        check(unpackPacket(d.payload.data(), d.payload.size(), decoded) &&
                  decoded.header.from == pkt.header.from && decoded.header.id == pkt.header.id &&
                  decoded.header.hop_limit == kHopReliable &&
                  decoded.header.channel == kDefaultChannelHash &&
                  decoded.encrypted == pkt.encrypted,
              "PHY payload is a Meshtastic PacketHeader frame");

        // One managed-flood hop, then re-encode — what a rebroadcasting node
        // would put back on the air.
        check(prepareRelay(decoded, 0xAABBCCDDu), "relay decoded frame");
        std::vector<uint8_t> relayed_air;
        check(packPacket(decoded, relayed_air), "repack after relay");
        const auto rchips = encodeFrame(st, relayed_air);
        const DecodeResult rd = decodeFrame(st, rchips);
        Packet after;
        check(rd.crc_ok && unpackPacket(rd.payload.data(), rd.payload.size(), after) &&
                  after.header.hop_limit == kHopReliable - 1 &&
                  after.header.relay_node == 0xDD && after.encrypted == pkt.encrypted,
              "relayed frame survives ShortTurbo round trip");
    }

    // Streaming TX: queue payloads, pull continuous IQ (warmup / frame / gap
    // or idle silence). Mirrors Receiver's chunk-fed API on the other side.
    std::printf("== 15. streaming transmitter: queue / warmup / pull ==\n");
    {
        Transmitter tx(p, 0.8f);
        tx.setWarmupSymbols(2);
        tx.setGapSymbols(12);

        IQBuffer idle(sps);
        tx.pull(idle.data(), idle.size());
        bool idle_ok = true;
        for (const Sample& s : idle) {
            if (s != Sample(0.0f, 0.0f)) {
                idle_ok = false;
                break;
            }
        }
        check(idle_ok && tx.framesStarted() == 0, "idle pull is silence");

        check(tx.enqueue(payload) && tx.queued() == 1, "enqueue payload");
        const size_t burst_n =
            static_cast<size_t>(2 + 12) * sps + frame.size();
        IQBuffer got(burst_n + sps);  // +1 symbol of trailing idle
        // Irregular pull sizes, same idea as the RX chunked-feed test.
        std::mt19937 rng(77);
        std::uniform_int_distribution<size_t> chunk(1, 4096);
        size_t pos = 0;
        while (pos < got.size()) {
            const size_t n = std::min(chunk(rng), got.size() - pos);
            tx.pull(got.data() + pos, n);
            pos += n;
        }

        IQBuffer expect;
        expect.reserve(burst_n);
        expect.insert(expect.end(), 2 * sps, Sample(0.8f, 0.0f));
        expect.insert(expect.end(), frame.begin(), frame.end());
        expect.insert(expect.end(), 12 * sps, Sample(0.0f, 0.0f));

        bool match = got.size() >= expect.size();
        for (size_t i = 0; match && i < expect.size(); ++i) {
            if (got[i] != expect[i]) {
                match = false;
            }
        }
        for (size_t i = expect.size(); match && i < got.size(); ++i) {
            if (got[i] != Sample(0.0f, 0.0f)) {
                match = false;
            }
        }
        check(match && tx.framesStarted() == 1 && tx.queued() == 0,
              "warmup+frame+gap then idle");

        // Two queued frames into Receiver. Leading silence matches the
        // other streaming tests (search needs a quiet run-in).
        Transmitter tx2(p, 0.8f);
        tx2.setWarmupSymbols(0);
        tx2.setGapSymbols(12);
        tx2.enqueue(payload);
        tx2.enqueue(payload);
        const size_t lead = 2 * sps + 517;
        const size_t two_n = 2 * (frame.size() + 12 * sps) + 2 * sps;
        IQBuffer stream(lead + two_n, Sample(0.0f, 0.0f));
        tx2.pull(stream.data() + lead, two_n);
        Receiver rx(p);
        const auto frames = runReceiver(rx, stream, 88);
        check(rxMatches(frames, payload, 2), "TX pull -> RX feed, 2 frames");
    }

    // Resampler golden + ShortTurbo real-time CPU budget. The DATA path does
    // one Catmull-Rom window + SymbolDemod (dechirp/FFT/fold) per symbol;
    // that must fit in T_sym with headroom for SEARCH/SYNC and radio I/O.
    std::printf("== 16. NEON kernels + ShortTurbo CPU budget ==\n");
    {
        // Known cubic: samples at integer knots of f(x)=x^2 on CF32.
        // Catmull-Rom reproduces quadratics exactly → out = (1.5)^2 = 2.25.
        IQBuffer knots(8);
        for (size_t i = 0; i < knots.size(); ++i) {
            const float x = static_cast<float>(i);
            knots[i] = Sample(x * x, -x * x);
        }
        Sample got;
        const size_t n = resampleCatmullRom(knots.data(), knots.size(), 1.5, 1.0, &got, 1);
        check(n == 1 && std::abs(got.real() - 2.25f) < 1e-5f &&
                  std::abs(got.imag() + 2.25f) < 1e-5f,
              "Catmull-Rom Farrow reproduces x^2 at mu=0.5");

        const LoraParams st = meshtasticParams(MeshtasticPreset::kShortTurbo);
        const uint32_t sps = samplesPerSymbol(st);
        const double t_sym = static_cast<double>(sps) / st.sample_rate_hz;
        SymbolDemod sym(st);

        // Warmup: FFTW plan + first-touch caches (not counted).
        IQBuffer src(sps * 4 + 8, Sample(0.0f, 0.0f));
        for (size_t i = 0; i < src.size(); ++i) {
            const float ph = 0.01f * static_cast<float>(i);
            src[i] = Sample(std::cos(ph), std::sin(ph));
        }
        IQBuffer win(sps);
        resampleCatmullRom(src.data(), src.size(), 2.0, 1.0 + 200e-6, win.data(), sps);
        (void)sym.demod(win.data());

        constexpr size_t kIters = 64;
        const auto t0 = std::chrono::steady_clock::now();
        for (size_t i = 0; i < kIters; ++i) {
            const double start = 2.0 + 0.01 * static_cast<double>(i % 7);
            resampleCatmullRom(src.data(), src.size(), start, 1.0 + 200e-6, win.data(), sps);
            (void)sym.demod(win.data());
        }
        const auto t1 = std::chrono::steady_clock::now();
        const double elapsed =
            std::chrono::duration<double>(t1 - t0).count();
        const double per_sym = elapsed / static_cast<double>(kIters);
        const double load = per_sym / t_sym;
        // Half of airtime leaves room for acquisition, refine retries, TX.
        constexpr double kMaxLoad = 0.5;
        std::printf("  ShortTurbo: %.1f us/sym (T_sym=%.1f us) -> %.1f%% of realtime"
#ifdef PHY_USE_NEON
                    " [NEON]"
#else
                    " [scalar]"
#endif
                    "\n",
                    per_sym * 1e6, t_sym * 1e6, load * 100.0);
        check(load < kMaxLoad,
              "DATA path under 50% of ShortTurbo symbol period");
    }

    // Low-SNR validation: evaluate streaming Receiver performance under AWGN
    // and sample-rate offsets across multiple trials.
    std::printf("== 17. streaming low-SNR soak (PER vs SNR) ==\n");
    {
        constexpr size_t kTrials = 24;
        constexpr double kPpm = -1100.0;
        struct Point {
            double snr_db;
            double max_per;
        };
        // SF11 / BW 500 kHz: still solid at ~0 dB SNR; cliff below that.
        const Point points[] = {{8.0, 0.0}, {3.0, 0.05}, {0.0, 0.25}};
        const size_t before = 2 * sps + 231;
        const size_t after = 3 * sps;

        for (const Point& pt : points) {
            size_t ok = 0;
            double snr_sum = 0.0;
            size_t snr_n = 0;
            for (size_t t = 0; t < kTrials; ++t) {
                Receiver rx(p);
                const IQBuffer capture = addNoise(
                    applyRateOffset(pad(frame, before, after), kPpm), pt.snr_db,
                    static_cast<uint32_t>(1000 + t * 17 + static_cast<int>(pt.snr_db * 10)));
                const auto frames = runReceiver(rx, capture, static_cast<uint32_t>(200 + t));
                if (rxMatches(frames, payload, 1)) {
                    ++ok;
                }
                if (!frames.empty()) {
                    snr_sum += frames[0].snr_db;
                    ++snr_n;
                }
            }
            const double per = 1.0 - static_cast<double>(ok) / static_cast<double>(kTrials);
            const double mean_snr = snr_n ? snr_sum / static_cast<double>(snr_n) : -99.0;
            char buf[96];
            std::snprintf(buf, sizeof(buf),
                          "SNR %.0f dB: %zu/%zu OK (PER %.1f%%, meas SNR %.1f dB)", pt.snr_db,
                          ok, kTrials, per * 100.0, mean_snr);
            check(per <= pt.max_per + 1e-12, buf);
        }
    }

    // Coding + streaming RX for every supported Meshtastic preset. The ±12.7 kHz
    // CFO case is the inter-board LO offset at 5.8 GHz (ShortFast 9b).
    std::printf("== 18. Meshtastic presets (ShortTurbo, ShortFast) ==\n");
    {
        const auto air = testPayload(16);
        for (const MeshtasticPreset preset : kMeshtasticPresets) {
            const LoraParams pr = meshtasticParams(preset);
            const char* key = meshtasticPresetKey(preset);
            const bool want_ldro = (static_cast<double>(1u << pr.spreading_factor) /
                                    pr.bandwidth_hz) >= 16.384e-3;
            char buf[128];

            const auto pr_chips = encodeFrame(pr, air);
            const DecodeResult d = decodeFrame(pr, pr_chips);
            std::snprintf(buf, sizeof(buf),
                          "%s encode/decode SF%u BW%.0fk cr=%u ldro=%d os=%u", key,
                          pr.spreading_factor, pr.bandwidth_hz / 1e3, pr.cr, int(pr.ldro),
                          osFactor(pr));
            check(d.header.valid && d.crc_ok && d.payload == air && pr.ldro == want_ldro, buf);

            const Modulator pr_mod(pr);
            const IQBuffer pr_frame = pr_mod.frame(pr_chips);
            const size_t pr_sps = pr_mod.samplesPerSymbol();
            const IQBuffer padded = pad(pr_frame, 2 * pr_sps + 231, 3 * pr_sps);

            {
                Receiver rx(pr);
                const auto frames = runReceiver(rx, padded, 80);
                std::snprintf(buf, sizeof(buf), "%s streaming RX clean", key);
                check(rxMatches(frames, air), buf);
            }
            for (const double cfo_hz : {-12700.0, 12700.0}) {
                Receiver rx(pr);
                const auto frames =
                    runReceiver(rx, applyCfo(padded, cfo_hz, pr.sample_rate_hz), 81);
                std::snprintf(buf, sizeof(buf), "%s CFO %+.0f Hz (est %+.0f Hz)", key, cfo_hz,
                              frames.empty() ? 0.0 : frames[0].cfo_hz);
                check(rxMatches(frames, air), buf);
            }
            {
                Receiver rx(pr);
                const IQBuffer capture =
                    addNoise(applyCfo(padded, 12700.0, pr.sample_rate_hz), 15.0, 82);
                const auto frames = runReceiver(rx, capture, 83);
                std::snprintf(buf, sizeof(buf), "%s CFO +12700 Hz 15 dB", key);
                check(rxMatches(frames, air), buf);
            }
        }
    }

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
    return g_failures == 0 ? 0 : 1;
}
