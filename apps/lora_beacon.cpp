// Numbered LoRa test frames for over-the-air PER measurements.
//
// Payload (32 B): 'Q' 'B' seq_hi seq_lo + 28 B of an LCG keyed by seq, so a
// receiver can check content and count losses from the sequence numbers.
//
//   lora_beacon --count 500 --tx-gain 0 --amplitude 0.1
//
// --tx-lo-offset-khz K tunes the MAX2850 LO to (freq - K kHz) and rotates the
// baseband by +K kHz, so the frame stays at --freq on air while the TX LO
// feedthrough sits K kHz below the channel. Needs --tx-rate high enough to
// hold [K - bw/2, K + bw/2] inside +-rate/2.

#include "beacon_payload.hpp"

#include <phy/lora/coding.hpp>
#include <phy/lora/presets.hpp>
#include <phy/lora/transmitter.hpp>
#include <phy/radio/radio_session.hpp>
#include <phy/radio/rf_frontend.hpp>
#include <phy/radio/stream_pair.hpp>

#include <SoapySDR/Constants.h>
#include <SoapySDR/Errors.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace phy;
using namespace phy::lora;

namespace {

void usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --count <n>            Frames to send (default: 500)\n"
              << "  --start-seq <n>        First sequence number (default: 0)\n"
              << "  --freq <mhz>           On-air center (default: 5800)\n"
              << "  --tx-gain <db>         MAX2850 gain (default: 0)\n"
              << "  --amplitude <a>        Baseband amplitude (default: 0.7)\n"
              << "  --preset <name>        shortturbo|shortfast (default: shortturbo)\n"
              << "  --gap <syms>           Silence between frames (default: 64)\n"
              << "  --warmup <syms>        DC carrier before each frame (default: 2)\n"
              << "  --tx-rate <hz>         Host TX rate (default: 1e6)\n"
              << "  --tx-lo-offset-khz <k> TX LO below channel + digital upshift (default: 0)\n"
              << "  --tx-ant <mask>        TX antenna mask (default: 1)\n";
}

}  // namespace

int main(int argc, char** argv) {
    size_t count = 500;
    uint16_t start_seq = 0;
    double freq_mhz = 5800.0;
    int tx_gain = 0;
    float amplitude = 0.7f;
    std::string preset = "shortturbo";
    int gap = 64;
    int warmup = 2;
    double tx_rate = 1e6;
    double lo_off_khz = 0.0;
    int tx_ant = 1;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : ""; };
        if (a == "--count") count = std::stoul(next());
        else if (a == "--start-seq") start_seq = static_cast<uint16_t>(std::stoul(next()));
        else if (a == "--freq") freq_mhz = std::stod(next());
        else if (a == "--tx-gain") tx_gain = std::stoi(next());
        else if (a == "--amplitude") amplitude = std::stof(next());
        else if (a == "--preset") preset = next();
        else if (a == "--gap") gap = std::stoi(next());
        else if (a == "--warmup") warmup = std::stoi(next());
        else if (a == "--tx-rate") tx_rate = std::stod(next());
        else if (a == "--tx-lo-offset-khz") lo_off_khz = std::stod(next());
        else if (a == "--tx-ant") tx_ant = std::stoi(next());
        else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
        else { std::cerr << "Unknown argument: " << a << "\n"; usage(argv[0]); return 1; }
    }

    LoraParams params;
    if (!parseMeshtasticPreset(preset, params, tx_rate)) {
        std::cerr << "bad preset\n";
        return 1;
    }
    if (std::abs(lo_off_khz) * 1e3 + params.bandwidth_hz / 2 >= tx_rate / 2) {
        std::cerr << "--tx-rate too low for the LO offset\n";
        return 1;
    }

    Transmitter tx(params, amplitude);
    tx.setWarmupSymbols(static_cast<uint32_t>(warmup));
    tx.setGapSymbols(static_cast<uint32_t>(gap));

    RfFrontend rf;
    const uint16_t saved_reg = rf.readRegister(0x2E);
    try {
        RadioSession radio;
        radio.open();
        radio.setSampleRate(SOAPY_SDR_TX, 0, tx_rate);
        rf.setDigitalLoopback(false);
        rf.configureTx(freq_mhz - lo_off_khz * 1e-3, tx_gain, 20, tx_ant);
        rf.paMute();
        // Status read is safe here (PA muted, before streaming); not on the mute path.
        std::fprintf(stderr, "tx pll %s\n", rf.txPllLocked() ? "locked" : "UNLOCKED");
        StreamPair streams;
        streams.setupTx(radio.device());
        const size_t chunk_len = std::min<size_t>(streams.mtu(), 65536);
        std::vector<Sample> chunk(chunk_len);
        streams.activate();

        // Phase-continuous +lo_off rotation across chunks; renormalize the
        // recursive phasor every chunk to stop float magnitude drift.
        const double w = 2.0 * M_PI * lo_off_khz * 1e3 / tx_rate;
        double phase = 0.0;

        auto pushChunk = [&]() -> bool {
            tx.pull(chunk.data(), chunk_len);
            if (lo_off_khz != 0.0) {
                const Sample step(static_cast<float>(std::cos(w)), static_cast<float>(std::sin(w)));
                Sample rot(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
                for (size_t k = 0; k < chunk_len; ++k) {
                    chunk[k] *= rot;
                    rot *= step;
                }
                phase = std::fmod(phase + w * static_cast<double>(chunk_len), 2.0 * M_PI);
            }
            size_t sent = 0;
            while (sent < chunk_len) {
                const int ret = streams.write(chunk.data() + sent, chunk_len - sent);
                if (ret > 0) sent += static_cast<size_t>(ret);
                else if (ret != SOAPY_SDR_TIMEOUT) {
                    std::cerr << "TX error " << SoapySDR::errToStr(ret) << "\n";
                    return false;
                }
            }
            return true;
        };

        // prime silence so the DSI ring is running before the PA opens
        pushChunk();
        rf.paUnmute();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        size_t queued = 0;
        const auto t0 = std::chrono::steady_clock::now();
        while (queued < count || !tx.isIdle()) {
            while (queued < count && tx.queued() < 4) {
                tx.enqueue(beaconPayload(static_cast<uint16_t>(start_seq + queued)));
                ++queued;
            }
            if (!pushChunk()) break;
        }
        // two chunks of silence drain the driver ring before PA mute
        pushChunk();
        pushChunk();
        rf.paMute();
        streams.deactivate();
        rf.txOff();
        rf.writeRegister(0x2E, saved_reg);
        const double el =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::fprintf(stderr, "sent %zu frames (seq %u..%u) in %.1f s, gain %d amp %.3f lo_off %.0f kHz\n",
                     tx.framesStarted(), start_seq, static_cast<unsigned>(start_seq + count - 1),
                     el, tx_gain, amplitude, lo_off_khz);
    } catch (const std::exception& ex) {
        try {
            rf.paMute();
            rf.txOff();
            rf.writeRegister(0x2E, saved_reg);
        } catch (...) {}
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
    return 0;
}
