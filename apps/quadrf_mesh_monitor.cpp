// SPDX-License-Identifier: MIT
// QuadRF Mesh Monitor - Minimalist SDL2 Telemetry & Control Interface
#include "font8x16.hpp"

#include <SDL.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifndef QUADRF_MESH_VERSION
#define QUADRF_MESH_VERSION "v0.1.0"
#endif

namespace {

struct PacketRecord {
    uint64_t ts_ms = 0;
    std::string time_str;
    float snr_db = 0.0f;
    float sir_db = 0.0f;
    float lvl_dbfs = 0.0f;
    int32_t cfo_hz = 0;
    float rate_ppm = 0.0f;
    uint32_t from_node = 0;
    uint32_t to_node = 0;
    uint32_t packet_id = 0;
    uint16_t payload_len = 0;
    bool is_echo = false;
};

struct AppState {
    std::mutex mu;
    std::deque<PacketRecord> packets;
    std::atomic<bool> telemetry_connected{false};
    std::atomic<uint64_t> total_packets{0};
    PacketRecord last_packet;
    bool has_last_packet = false;

    std::atomic<bool> ble_active{false};
    std::atomic<int> rf_mode{0}; // 0 = Ch1 (Single), 1 = 4-Ch Sum
    std::atomic<uint32_t> range_sec{0}; // 0 = OFF, 5 = 5s, 10 = 10s
    std::atomic<bool> parrot_active{false};
    std::string callsign{"NOCALL"};
    std::string preset{"ShortTurbo"};

    int scroll_offset = 0;
};

// Dark minimal palette
struct Color {
    Uint8 r, g, b, a;
};

constexpr Color kBgColor        = {18, 18, 18, 255};
constexpr Color kCardBg         = {26, 26, 26, 255};
constexpr Color kCardBorder     = {44, 44, 44, 255};
constexpr Color kHeaderBg       = {32, 32, 32, 255};
constexpr Color kTextPrimary    = {235, 235, 235, 255};
constexpr Color kTextSecondary  = {140, 140, 140, 255};
constexpr Color kTextMuted      = {90, 90, 90, 255};
constexpr Color kAccentBlue     = {56, 189, 248, 255};
constexpr Color kStatusGreen    = {34, 197, 94, 255};
constexpr Color kStatusAmber    = {234, 179, 8, 255};
constexpr Color kStatusRed      = {239, 68, 68, 255};
constexpr Color kBtnNormal      = {38, 38, 38, 255};
constexpr Color kBtnHover       = {50, 50, 50, 255};

// Channel SNR: SF7 CR4/5 cliff is ~-7.5 dB. Green = ~7 dB of decode room.
Color snrColor(float snr_db, bool echo) {
    if (echo) {
        return kTextMuted;
    }
    if (snr_db >= 0.0f) {
        return kStatusGreen;
    }
    if (snr_db >= -7.5f) {
        return kStatusAmber;
    }
    return kStatusRed;
}

// SIR: 10 dB ~ clean; 6 dB is near the 0.30 mag2/peak CRC-alternate gate.
Color sirColor(float sir_db, bool echo) {
    if (echo) {
        return kTextMuted;
    }
    if (sir_db >= 10.0f) {
        return kStatusGreen;
    }
    if (sir_db >= 6.0f) {
        return kStatusAmber;
    }
    return kStatusRed;
}

void setColor(SDL_Renderer* ren, const Color& c) {
    SDL_SetRenderDrawColor(ren, c.r, c.g, c.b, c.a);
}

void drawChar(SDL_Renderer* ren, int x, int y, char ch, const Color& c) {
    setColor(ren, c);
    uint8_t uc = static_cast<uint8_t>(ch);
    for (int row = 0; row < 16; ++row) {
        uint8_t bits = g_font8x16[uc][row];
        for (int col = 0; col < 8; ++col) {
            if ((bits >> (7 - col)) & 1) {
                SDL_RenderDrawPoint(ren, x + col, y + row);
            }
        }
    }
}

void drawString(SDL_Renderer* ren, int x, int y, const std::string& str, const Color& c) {
    int cur_x = x;
    for (char ch : str) {
        drawChar(ren, cur_x, y, ch, c);
        cur_x += 8;
    }
}

void drawRect(SDL_Renderer* ren, int x, int y, int w, int h, const Color& c) {
    setColor(ren, c);
    SDL_Rect r = {x, y, w, h};
    SDL_RenderDrawRect(ren, &r);
}

void fillRect(SDL_Renderer* ren, int x, int y, int w, int h, const Color& c) {
    setColor(ren, c);
    SDL_Rect r = {x, y, w, h};
    SDL_RenderFillRect(ren, &r);
}

void drawParrotIcon(SDL_Renderer* ren, int x, int y) {
    // 16x16 pixel art representing a colorful Scarlet Macaw:
    // 1: Red (crest/body), 2: Yellow (beak/upper wing), 3: White (face patch)
    // 4: Black (eye/beak tip), 5: Green (mid-wing), 6: Blue (tail/flight feathers), 7: Grey (perch/feet)
    // Shifted to columns 4..11 to be horizontally centered in the 16x16 box.
    static const uint8_t kParrot[16][16] = {
        {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 1, 3, 3, 2, 2, 2, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 1, 3, 4, 2, 2, 4, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 4, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 1, 2, 2, 1, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 1, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 1, 5, 5, 5, 1, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 1, 5, 5, 5, 1, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 6, 6, 6, 6, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 6, 6, 6, 6, 7, 7, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 6, 6, 6, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 6, 6, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 6, 6, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    };

    static const Color kColors[8] = {
        {0, 0, 0, 0},
        {235, 40, 40, 255},    // 1: red
        {250, 200, 20, 255},   // 2: yellow
        {245, 245, 245, 255},  // 3: white
        {20, 20, 20, 255},     // 4: black
        {34, 197, 94, 255},    // 5: green
        {56, 189, 248, 255},   // 6: blue
        {160, 160, 160, 255},  // 7: feet grey
    };

    for (int r = 0; r < 16; ++r) {
        for (int c = 0; c < 16; ++c) {
            uint8_t idx = kParrot[r][c];
            if (idx > 0 && idx < 8) {
                fillRect(ren, x + c * 2, y + r * 2, 2, 2, kColors[idx]);
            }
        }
    }
}

struct Button {
    int x, y, w, h;
    std::string text;
    bool is_hovered = false;

    bool contains(int mx, int my) const {
        return mx >= x && mx < x + w && my >= y && my < y + h;
    }

    void draw(SDL_Renderer* ren, const Color& textColor = kTextPrimary, const Color& accentColor = kCardBorder) {
        fillRect(ren, x, y, w, h, is_hovered ? kBtnHover : kBtnNormal);
        drawRect(ren, x, y, w, h, accentColor);
        int text_x = x + (w - static_cast<int>(text.length()) * 8) / 2;
        int text_y = y + (h - 16) / 2;
        drawString(ren, text_x, text_y, text, textColor);
    }
};

std::string formatTime(uint64_t ts_ms) {
    time_t sec = static_cast<time_t>(ts_ms / 1000);
    struct tm tm_buf;
    localtime_r(&sec, &tm_buf);
    char buf[32];
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%01u",
             tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
             static_cast<unsigned>((ts_ms % 1000) / 100));
    return std::string(buf);
}

std::string presetDisplayName(const std::string& key) {
    if (key == "shortturbo") {
        return "ShortTurbo";
    }
    if (key == "shortfast") {
        return "ShortFast";
    }
    return key;
}

std::string formatNodeId(uint32_t node) {
    char buf[16];
    if (node == 0xFFFFFFFF) {
        return "Broadcast";
    }
    snprintf(buf, sizeof(buf), "%08X", node);
    return std::string(buf);
}

// Simple non-blocking JSON field parser for telemetry tap records
bool parseJsonField(const std::string& line, const std::string& key, std::string& val) {
    std::string needle = "\"" + key + "\":";
    size_t pos = line.find(needle);
    if (pos == std::string::npos) return false;
    pos += needle.length();
    while (pos < line.length() && (line[pos] == ' ' || line[pos] == '\"')) pos++;
    size_t end = pos;
    while (end < line.length() && line[end] != ',' && line[end] != '}' && line[end] != '\"' && line[end] != '\r' && line[end] != '\n') {
        end++;
    }
    // Trim trailing whitespace
    while (end > pos && line[end - 1] == ' ') {
        end--;
    }
    val = line.substr(pos, end - pos);
    return true;
}

void telemetryClientThread(AppState& state, const std::string& sock_path) {
    while (true) {
        int fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd < 0) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        struct sockaddr_un addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, sock_path.c_str(), sizeof(addr.sun_path) - 1);

        if (connect(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            close(fd);
            state.telemetry_connected = false;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        state.telemetry_connected = true;
        std::string buffer;
        char chunk[512];

        while (true) {
            ssize_t n = read(fd, chunk, sizeof(chunk) - 1);
            if (n <= 0) {
                break;
            }
            chunk[n] = '\0';
            buffer.append(chunk, n);

            size_t nl;
            while ((nl = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, nl);
                buffer.erase(0, nl + 1);

                if (line.empty()) continue;

                std::string val;
                if (parseJsonField(line, "type", val) && val == "modem") {
                    if (parseJsonField(line, "preset", val) && !val.empty()) {
                        std::lock_guard<std::mutex> lk(state.mu);
                        state.preset = presetDisplayName(val);
                    }
                    continue;
                }

                // PHY telemetry includes full-duplex self-TX (`echo:1`);
                // those rows stay visible even though Air-IPC never sees them.
                PacketRecord rec;
                if (parseJsonField(line, "ts", val)) rec.ts_ms = std::strtoull(val.c_str(), nullptr, 10);
                if (parseJsonField(line, "snr", val)) rec.snr_db = std::strtof(val.c_str(), nullptr);
                if (parseJsonField(line, "sir", val)) rec.sir_db = std::strtof(val.c_str(), nullptr);
                if (parseJsonField(line, "lvl", val)) {
                    rec.lvl_dbfs = std::strtof(val.c_str(), nullptr);
                } else if (parseJsonField(line, "rssi", val)) {
                    rec.lvl_dbfs = std::strtof(val.c_str(), nullptr);
                }
                if (parseJsonField(line, "cfo", val)) rec.cfo_hz = std::strtol(val.c_str(), nullptr, 10);
                if (parseJsonField(line, "ppm", val) || parseJsonField(line, "rate_ppm", val)) {
                    rec.rate_ppm = std::strtof(val.c_str(), nullptr);
                }
                if (parseJsonField(line, "from", val)) rec.from_node = std::strtoul(val.c_str(), nullptr, 0);
                if (parseJsonField(line, "to", val)) rec.to_node = std::strtoul(val.c_str(), nullptr, 0);
                if (parseJsonField(line, "id", val)) rec.packet_id = std::strtoul(val.c_str(), nullptr, 0);
                if (parseJsonField(line, "len", val)) rec.payload_len = static_cast<uint16_t>(std::strtoul(val.c_str(), nullptr, 10));
                if (parseJsonField(line, "echo", val)) {
                    rec.is_echo = (val != "0" && val != "false");
                }

                rec.time_str = formatTime(rec.ts_ms);

                {
                    std::lock_guard<std::mutex> lk(state.mu);
                    state.packets.push_front(rec);
                    if (state.packets.size() > 200) {
                        state.packets.pop_back();
                    }
                    state.last_packet = rec;
                    state.has_last_packet = true;
                    state.total_packets++;
                }
            }
        }

        close(fd);
        state.telemetry_connected = false;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

bool checkBleActive() {
    int ret = std::system("systemctl is-active --quiet quadrf-ble-bridge 2>/dev/null");
    return (ret == 0);
}

void toggleBleBridge(AppState& state) {
    if (state.ble_active) {
        std::system("sudo -n systemctl stop quadrf-ble-bridge 2>/dev/null");
    } else {
        std::system("sudo -n systemctl start quadrf-ble-bridge 2>/dev/null");
    }
    state.ble_active = checkBleActive();
}

int queryRfMode() {
    // Read FPGA register 0x25 via single-register read (no LO/RX disturbance).
    // quadrf-jtag prints "READ  addr=0x25 -> value=0x0001" (two spaces after READ).
    FILE* fp = popen("sudo quadrf-jtag --no-setup read 0x25 2>/dev/null", "r");
    if (!fp) return 0;
    char buf[128];
    int mode = 0;
    while (fgets(buf, sizeof(buf), fp)) {
        unsigned int val = 0;
        const char* p = std::strstr(buf, "value=0x");
        if (p && sscanf(p, "value=0x%x", &val) == 1) {
            mode = (val & 1) ? 1 : 0;
        }
    }
    pclose(fp);
    return mode;
}

bool sendMeshControl(const std::string& cmd, std::string* reply = nullptr) {
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return false;

    struct sockaddr_un addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, "/run/quadrf/mesh_control.sock", sizeof(addr.sun_path) - 1);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 250000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (connect(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(fd);
        return false;
    }

    std::string msg = cmd + "\n";
    if (write(fd, msg.data(), msg.size()) <= 0) {
        close(fd);
        return false;
    }

    if (reply) {
        char buf[256];
        ssize_t n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            *reply = std::string(buf);
        }
    }

    close(fd);
    return true;
}

std::string loadSystemCallsign() {
    std::string call;
    const char* conf_path = "/etc/quadrf/quadrf.conf";
    FILE* fp = std::fopen(conf_path, "r");
    if (fp) {
        char line[256];
        while (std::fgets(line, sizeof(line), fp)) {
            char* p = line;
            while (*p == ' ' || *p == '\t') p++;
            if (*p == '#' || *p == '\0') continue;
            if (std::strncmp(p, "CALLSIGN=", 9) == 0 || std::strncmp(p, "QUADRF_CALLSIGN=", 16) == 0) {
                char* eq = std::strchr(p, '=');
                if (eq) {
                    char* val = eq + 1;
                    while (*val == ' ' || *val == '\t' || *val == '\"' || *val == '\'') val++;
                    char* end = val + std::strlen(val);
                    while (end > val && (*(end - 1) == ' ' || *(end - 1) == '\t' ||
                                         *(end - 1) == '\"' || *(end - 1) == '\'' ||
                                         *(end - 1) == '\r' || *(end - 1) == '\n')) {
                        end--;
                    }
                    if (end > val) {
                        call.assign(val, end - val);
                    }
                }
            }
        }
        std::fclose(fp);
    }

    if (call.empty()) {
        const char* env_call = std::getenv("CALLSIGN");
        if (!env_call || !*env_call) {
            env_call = std::getenv("QUADRF_CALLSIGN");
        }
        if (env_call && *env_call) {
            call = env_call;
        }
    }

    if (call.empty()) {
        call = "NOCALL";
    }

    for (char& c : call) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return call;
}

void syncMeshControlStatus(AppState& state) {
    std::string resp;
    if (sendMeshControl("GET STATUS", &resp)) {
        size_t rpos = resp.find("RANGE=");
        if (rpos != std::string::npos) {
            uint32_t r = static_cast<uint32_t>(std::strtoul(resp.c_str() + rpos + 6, nullptr, 10));
            state.range_sec = r;
        }
        size_t ppos = resp.find("PARROT=");
        if (ppos != std::string::npos) {
            int p = std::atoi(resp.c_str() + ppos + 7);
            state.parrot_active = (p != 0);
        }
        size_t cpos = resp.find("CALLSIGN=");
        if (cpos != std::string::npos) {
            size_t val_start = cpos + 9;
            size_t val_end = resp.find_first_of(" \r\n", val_start);
            if (val_end == std::string::npos) val_end = resp.size();
            std::string call = resp.substr(val_start, val_end - val_start);
            for (char& c : call) {
                c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            }
            if (!call.empty()) {
                std::lock_guard<std::mutex> lk(state.mu);
                state.callsign = call;
            }
        }
    }
}

} // namespace

int main(int argc, char* argv[]) {
    std::string sock_path = "/run/quadrf/phy_telemetry.sock";
    if (argc > 1) {
        sock_path = argv[1];
    }

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    const int kWinWidth = 1060;
    const int kWinHeight = 560;

    SDL_Window* win = SDL_CreateWindow(
        "QuadRF Mesh Monitor",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        kWinWidth, kWinHeight,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );

    if (!win) {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* ren = SDL_CreateRenderer(
        win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );
    if (!ren) {
        ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE);
    }

    AppState state;
    state.callsign = loadSystemCallsign();
    state.ble_active = checkBleActive();
    state.rf_mode = queryRfMode();
    syncMeshControlStatus(state);

    std::thread telemetry_th(telemetryClientThread, std::ref(state), sock_path);
    telemetry_th.detach();

    // Buttons (fitted within 800px width with 20px left and right margins)
    Button btn_ble    = {20, 80, 105, 36, "BLE: OFF"};
    Button btn_mode   = {133, 80, 150, 36, "RX: Beamforming"};
    Button btn_range  = {291, 80, 125, 36, "RANGE: OFF"};
    Button btn_parrot = {424, 80, 54, 36, ""};
    Button btn_clear  = {486, 80, 106, 36, "Clear Log"};

    bool running = true;
    auto last_status_check = std::chrono::steady_clock::now();

    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) {
                running = false;
            } else if (ev.type == SDL_MOUSEMOTION) {
                int mx = ev.motion.x;
                int my = ev.motion.y;
                btn_ble.is_hovered = btn_ble.contains(mx, my);
                btn_mode.is_hovered = false;
                btn_range.is_hovered = btn_range.contains(mx, my);
                btn_parrot.is_hovered = btn_parrot.contains(mx, my);
                btn_clear.is_hovered = btn_clear.contains(mx, my);
            } else if (ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT) {
                int mx = ev.button.x;
                int my = ev.button.y;
                if (btn_ble.contains(mx, my)) {
                    toggleBleBridge(state);
                } else if (btn_range.contains(mx, my)) {
                    uint32_t cur = state.range_sec.load();
                    uint32_t next = (cur == 0) ? 5 : ((cur == 5) ? 10 : 0);
                    state.range_sec = next;
                    sendMeshControl("SET RANGE " + std::to_string(next));
                } else if (btn_parrot.contains(mx, my)) {
                    bool next = !state.parrot_active.load();
                    state.parrot_active = next;
                    sendMeshControl("SET PARROT " + std::string(next ? "1" : "0"));
                } else if (btn_clear.contains(mx, my)) {
                    std::lock_guard<std::mutex> lk(state.mu);
                    state.packets.clear();
                    state.has_last_packet = false;
                }
            } else if (ev.type == SDL_MOUSEWHEEL) {
                if (ev.wheel.y > 0) {
                    state.scroll_offset = std::max(0, state.scroll_offset - 2);
                } else if (ev.wheel.y < 0) {
                    state.scroll_offset += 2;
                }
            } else if (ev.type == SDL_KEYDOWN) {
                if (ev.key.keysym.sym == SDLK_UP) {
                    state.scroll_offset = std::max(0, state.scroll_offset - 1);
                } else if (ev.key.keysym.sym == SDLK_DOWN) {
                    state.scroll_offset += 1;
                }
            }
        }

        // Periodic background state refresh (every 2 seconds)
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_status_check).count() >= 2) {
            state.ble_active = checkBleActive();
            state.rf_mode = queryRfMode();
            syncMeshControlStatus(state);
            last_status_check = now;
        }

        // Update button labels
        btn_ble.text = state.ble_active ? "BLE: ON" : "BLE: OFF";
        btn_mode.text = (state.rf_mode == 0) ? "RX: Beamforming" : "RX: 4-Ch Sum";
        uint32_t rsec = state.range_sec.load();
        btn_range.text = (rsec == 0) ? "RANGE: OFF" : ("RANGE: " + std::to_string(rsec) + "s");

        // Render Frame
        fillRect(ren, 0, 0, kWinWidth, kWinHeight, kBgColor);

        // Header Bar
        fillRect(ren, 0, 0, kWinWidth, 68, kCardBg);
        fillRect(ren, 0, 67, kWinWidth, 1, kCardBorder);

        drawString(ren, 20, 16, "QUADRF MESH MONITOR", kTextPrimary);
        drawString(ren, 185, 16, QUADRF_MESH_VERSION, kTextMuted);

        std::string cur_call;
        std::string cur_preset;
        {
            std::lock_guard<std::mutex> lk(state.mu);
            cur_call = state.callsign;
            cur_preset = state.preset;
        }
        drawString(ren, 255, 16, "CALL:", kTextSecondary);
        drawString(ren, 300, 16, cur_call, kAccentBlue);

        // Status Badges in Header
        int badge_x = 440;
        drawString(ren, badge_x, 16, "TEL:", kTextSecondary);
        if (state.telemetry_connected) {
            drawString(ren, badge_x + 40, 16, "ONLINE", kStatusGreen);
        } else {
            drawString(ren, badge_x + 40, 16, "WAITING", kStatusAmber);
        }

        drawString(ren, badge_x + 110, 16, "FREQ:", kTextSecondary);
        drawString(ren, badge_x + 155, 16, "5800 MHz", kAccentBlue);

        drawString(ren, badge_x + 235, 16, "PRESET:", kTextSecondary);
        drawString(ren, badge_x + 295, 16, cur_preset, kTextPrimary);

        // Controls Area (Buttons)
        btn_ble.draw(ren, state.ble_active ? kStatusGreen : kTextSecondary,
                     state.ble_active ? kStatusGreen : kCardBorder);
        btn_mode.draw(ren, kTextPrimary, (state.rf_mode == 1) ? kAccentBlue : kCardBorder);
        btn_range.draw(ren, (rsec > 0) ? kStatusAmber : kTextSecondary,
                       (rsec > 0) ? kStatusAmber : kCardBorder);

        // Parrot Button: pixel-art icon with active status border
        bool parrot_on = state.parrot_active.load();
        fillRect(ren, btn_parrot.x, btn_parrot.y, btn_parrot.w, btn_parrot.h,
                 btn_parrot.is_hovered ? kBtnHover : kBtnNormal);
        drawRect(ren, btn_parrot.x, btn_parrot.y, btn_parrot.w, btn_parrot.h,
                 parrot_on ? kStatusGreen : kCardBorder);
        if (parrot_on) {
            // Subtle 1px inner border highlight when active
            drawRect(ren, btn_parrot.x + 1, btn_parrot.y + 1, btn_parrot.w - 2, btn_parrot.h - 2, kStatusGreen);
        }
        drawParrotIcon(ren, btn_parrot.x + 11, btn_parrot.y + 2);

        btn_clear.draw(ren, kTextSecondary, kCardBorder);

        // Telemetry Summary Card (y: 130 to 195)
        fillRect(ren, 20, 130, kWinWidth - 40, 65, kCardBg);
        drawRect(ren, 20, 130, kWinWidth - 40, 65, kCardBorder);

        drawString(ren, 35, 142, "LAST DECODED PACKET", kTextSecondary);

        PacketRecord last_rec;
        bool has_pkt = false;
        uint64_t total_pkts = 0;
        {
            std::lock_guard<std::mutex> lk(state.mu);
            has_pkt = state.has_last_packet;
            last_rec = state.last_packet;
            total_pkts = state.total_packets;
        }

        if (has_pkt) {
            char buf[128];
            if (last_rec.is_echo) {
                drawString(ren, 220, 142, "[SELF-TX ECHO]", kStatusAmber);
            } else {
                drawString(ren, 220, 142, "[PEER RX]", kStatusGreen);
            }

            std::string from_text = "From: " + formatNodeId(last_rec.from_node);
            if (last_rec.is_echo) {
                from_text += " [SELF]";
            }
            drawString(ren, 35, 165, from_text, last_rec.is_echo ? kTextMuted : kTextPrimary);

            snprintf(buf, sizeof(buf), "To: %s", formatNodeId(last_rec.to_node).c_str());
            drawString(ren, 220, 165, buf, last_rec.is_echo ? kTextMuted : kTextPrimary);

            snprintf(buf, sizeof(buf), "SNR: %+5.1f dB", last_rec.snr_db);
            drawString(ren, 345, 165, buf, snrColor(last_rec.snr_db, last_rec.is_echo));

            snprintf(buf, sizeof(buf), "SIR: %+5.1f dB", last_rec.sir_db);
            drawString(ren, 490, 165, buf, sirColor(last_rec.sir_db, last_rec.is_echo));

            snprintf(buf, sizeof(buf), "LVL: %+5.1f dBFS", last_rec.lvl_dbfs);
            drawString(ren, 635, 165, buf, last_rec.is_echo ? kTextMuted : kTextPrimary);

            snprintf(buf, sizeof(buf), "CFO: %+5d Hz", last_rec.cfo_hz);
            drawString(ren, 800, 165, buf, last_rec.is_echo ? kTextMuted : kTextPrimary);
        } else {
            drawString(ren, 35, 165, "Awaiting physical layer frame decodes...", kTextMuted);
        }

        char count_buf[64];
        snprintf(count_buf, sizeof(count_buf), "Total: %llu", static_cast<unsigned long long>(total_pkts));
        drawString(ren, kWinWidth - 145, 142, count_buf, kAccentBlue);

        // Recent Packet Log Card (y: 205 to 540)
        fillRect(ren, 20, 205, kWinWidth - 40, 335, kCardBg);
        drawRect(ren, 20, 205, kWinWidth - 40, 335, kCardBorder);

        // Table Header
        fillRect(ren, 21, 206, kWinWidth - 42, 28, kHeaderBg);
        fillRect(ren, 21, 234, kWinWidth - 42, 1, kCardBorder);

        drawString(ren, 30, 212, "TIME", kTextSecondary);
        drawString(ren, 110, 212, "FROM NODE", kTextSecondary);
        drawString(ren, 240, 212, "TO NODE", kTextSecondary);
        drawString(ren, 330, 212, "SNR (dB)", kTextSecondary);
        drawString(ren, 412, 212, "SIR (dB)", kTextSecondary);
        drawString(ren, 494, 212, "LVL (dBFS)", kTextSecondary);
        drawString(ren, 596, 212, "CFO (Hz)", kTextSecondary);
        drawString(ren, 688, 212, "RATE (ppm)", kTextSecondary);
        drawString(ren, 790, 212, "LEN", kTextSecondary);
        drawString(ren, 838, 212, "PACKET ID", kTextSecondary);

        // Table Rows
        std::vector<PacketRecord> snap_packets;
        {
            std::lock_guard<std::mutex> lk(state.mu);
            snap_packets.assign(state.packets.begin(), state.packets.end());
        }

        int start_idx = state.scroll_offset;
        int max_visible = 14;
        int y_row = 242;

        if (snap_packets.empty()) {
            drawString(ren, 35, y_row + 10, "No packets received yet. Mesh PHY is listening on 5800 MHz...", kTextMuted);
        } else {
            for (int i = start_idx; i < static_cast<int>(snap_packets.size()) && i < start_idx + max_visible; ++i) {
                const auto& p = snap_packets[i];

                Color time_c = p.is_echo ? kTextMuted : kTextSecondary;
                Color from_c = p.is_echo ? kTextMuted : kTextPrimary;
                Color to_c   = p.is_echo ? kTextMuted : kTextSecondary;
                Color snr_c  = snrColor(p.snr_db, p.is_echo);
                Color sir_c  = sirColor(p.sir_db, p.is_echo);
                Color lvl_c  = p.is_echo ? kTextMuted : kTextPrimary;
                Color cfo_c  = p.is_echo ? kTextMuted : kTextPrimary;
                Color rate_c = p.is_echo ? kTextMuted : kTextSecondary;
                Color len_c  = p.is_echo ? kTextMuted : kTextSecondary;
                Color id_c   = p.is_echo ? kTextMuted : kAccentBlue;

                drawString(ren, 30, y_row, p.time_str, time_c);

                std::string from_str = formatNodeId(p.from_node);
                if (p.is_echo) {
                    from_str += " [SELF]";
                }
                drawString(ren, 110, y_row, from_str, from_c);
                drawString(ren, 240, y_row, formatNodeId(p.to_node), to_c);

                char buf[64];
                snprintf(buf, sizeof(buf), "%+5.1f", p.snr_db);
                drawString(ren, 330, y_row, buf, snr_c);

                snprintf(buf, sizeof(buf), "%+5.1f", p.sir_db);
                drawString(ren, 412, y_row, buf, sir_c);

                snprintf(buf, sizeof(buf), "%+5.1f", p.lvl_dbfs);
                drawString(ren, 494, y_row, buf, lvl_c);

                snprintf(buf, sizeof(buf), "%+5d", p.cfo_hz);
                drawString(ren, 596, y_row, buf, cfo_c);

                snprintf(buf, sizeof(buf), "%+4.1f", p.rate_ppm);
                drawString(ren, 688, y_row, buf, rate_c);

                snprintf(buf, sizeof(buf), "%3u", p.payload_len);
                drawString(ren, 790, y_row, buf, len_c);

                snprintf(buf, sizeof(buf), "0x%08x", p.packet_id);
                drawString(ren, 838, y_row, buf, id_c);

                y_row += 20;
            }
        }

        SDL_RenderPresent(ren);
        SDL_Delay(30); // ~33 FPS cap, yields CPU
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
