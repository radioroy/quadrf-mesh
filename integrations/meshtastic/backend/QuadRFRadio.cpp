// SPDX-FileCopyrightText: 2026 Roy C. Gross
// SPDX-License-Identifier: GPL-3.0-only
#include "QuadRFRadio.h"
#include "QuadRFParrotModule.h"
#include "QuadRFPingModule.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "gps/RTC.h"
#include "quadrf/air_ipc.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

QuadRFRadio *QuadRFRadio::instance;
QuadRFRadio *quadrfRadio;

QuadRFRadio::QuadRFRadio(std::string socket_path)
    : NotifiedWorkerThread("QuadRFRadio"), socket_path_(std::move(socket_path))
{
    instance = this;
    quadrfRadio = this;
}

QuadRFRadio::~QuadRFRadio()
{
    stopControlServer();
    rx_running_ = false;
    closeSocket();
    if (rx_thread_.joinable())
        rx_thread_.join();
    if (instance == this)
        instance = nullptr;
    if (quadrfRadio == this)
        quadrfRadio = nullptr;
}

bool QuadRFRadio::init()
{
    RadioInterface::init();
    if (!connectSocket()) {
        LOG_ERROR("QuadRFRadio: connect %s failed: %s", socket_path_.c_str(), strerror(errno));
        return false;
    }
    rx_running_ = true;
    rx_thread_ = std::thread(&QuadRFRadio::rxThreadMain, this);
    LOG_INFO("QuadRFRadio: connected to %s", socket_path_.c_str());

    if (!QuadRFParrotModule::instance) {
        new QuadRFParrotModule();
    }
    if (!QuadRFPingModule::instance) {
        auto *ping = new QuadRFPingModule();
        ping->start();
    }
    startControlServer();

    return true;
}

bool QuadRFRadio::reconfigure()
{
    RadioInterface::reconfigure();
    // PHY owns RF/DSP; frequency is carried only as a fail-safe assertion.
    return true;
}

bool QuadRFRadio::connectSocket()
{
    closeSocket();
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0)
        return false;

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (socket_path_.size() >= sizeof(addr.sun_path)) {
        ::close(fd);
        errno = ENAMETOOLONG;
        return false;
    }
    std::memcpy(addr.sun_path, socket_path_.c_str(), socket_path_.size() + 1);

    if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        ::close(fd);
        return false;
    }

    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0)
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    sock_fd_ = fd;
    return true;
}

void QuadRFRadio::closeSocket()
{
    if (sock_fd_ >= 0) {
        ::shutdown(sock_fd_, SHUT_RDWR);
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

bool QuadRFRadio::writeFrame(const std::vector<uint8_t> &frame)
{
    if (sock_fd_ < 0 || frame.empty())
        return false;
    size_t off = 0;
    while (off < frame.size()) {
        const ssize_t w = ::write(sock_fd_, frame.data() + off, frame.size() - off);
        if (w < 0) {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                pollfd pfd{sock_fd_, POLLOUT, 0};
                if (poll(&pfd, 1, 200) <= 0)
                    return false;
                continue;
            }
            return false;
        }
        off += static_cast<size_t>(w);
    }
    return true;
}

void QuadRFRadio::rxThreadMain()
{
    quadrf::air_ipc::Deframer deframer;
    uint8_t buf[512];
    std::vector<quadrf::air_ipc::Deframer::Frame> frames;

    while (rx_running_) {
        if (sock_fd_ < 0) {
            delay(50);
            continue;
        }
        pollfd pfd{sock_fd_, POLLIN, 0};
        const int pr = poll(&pfd, 1, 100);
        if (pr < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        if (pr == 0)
            continue;

        const ssize_t n = ::read(sock_fd_, buf, sizeof(buf));
        if (n == 0) {
            LOG_WARN("QuadRFRadio: PHY socket closed");
            break;
        }
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            LOG_WARN("QuadRFRadio: read error: %s", strerror(errno));
            break;
        }

        frames.clear();
        deframer.feed(buf, static_cast<size_t>(n), frames);
        for (auto &fr : frames) {
            if (fr.type != quadrf::air_ipc::MsgType::kRxIndicate)
                continue;
            quadrf::air_ipc::RxIndicate rx;
            if (!quadrf::air_ipc::decodeRx(fr.body.data(), fr.body.size(), rx)) {
                LOG_WARN("QuadRFRadio: bad RxIndicate body=%zu", fr.body.size());
                rxBad++;
                continue;
            }
            enqueueRx(std::move(rx.air), rx.snr_db, rx.rssi_dbm, rx.cfo_hz, rx.rate_ppm);
        }
    }
}

void QuadRFRadio::enqueueRx(std::vector<uint8_t> air, float snr_db, int16_t rssi_dbm,
                            int32_t cfo_hz, float rate_ppm)
{
    {
        std::lock_guard<std::mutex> lk(rx_mu_);
        rx_q_.push_back(RxFrame{std::move(air), snr_db, rssi_dbm, cfo_hz, rate_ppm});
    }
    notify(ISR_RX, true);
}

ErrorCode QuadRFRadio::send(meshtastic_MeshPacket *p)
{
    printPacket("enqueuing for send", p);

    bool dropped = false;
    ErrorCode res = txQueue.enqueue(p, &dropped) ? ERRNO_OK : ERRNO_UNKNOWN;
    if (dropped)
        txDrop++;
    if (res != ERRNO_OK) {
        packetPool.release(p);
        return res;
    }

    setTransmitDelay();
    return res;
}

void QuadRFRadio::setTransmitDelay()
{
    meshtastic_MeshPacket *p = txQueue.getFront();
    if (!p)
        return;
    if (p->rx_snr == 0 && p->rx_rssi == 0)
        startTransmitTimer(true);
    else
        startTransmitTimerRebroadcast(p);
}

void QuadRFRadio::startTransmitTimer(bool withDelay)
{
    if (!txQueue.empty()) {
        uint32_t delayMsec = !withDelay ? 1 : getTxDelayMsec();
        if (sendingPacket && tx_end_time_ms_ > millis()) {
            uint32_t remaining = tx_end_time_ms_ - millis();
            if (remaining > delayMsec)
                delayMsec = remaining + 1;
        }
        notifyLater(delayMsec, TRANSMIT_DELAY_COMPLETED, false);
    }
}

void QuadRFRadio::startTransmitTimerRebroadcast(meshtastic_MeshPacket *p)
{
    if (!txQueue.empty()) {
        uint32_t delayMsec = getTxDelayMsecWeighted(p);
        notifyLater(delayMsec, TRANSMIT_DELAY_COMPLETED, false);
    }
}

void QuadRFRadio::handleTransmitInterrupt()
{
    if (sendingPacket)
        completeSending();
    isReceiving_ = true;
}

void QuadRFRadio::checkTxComplete()
{
    if (sendingPacket && millis() >= tx_end_time_ms_)
        handleTransmitInterrupt();
}

void QuadRFRadio::completeSending()
{
    auto p = sendingPacket;
    sendingPacket = NULL;
    if (!p)
        return;
    txGood++;
    if (!isFromUs(p))
        txRelay++;
    printPacket("Completed sending", p);
    packetPool.release(p);
}

bool QuadRFRadio::canSendImmediately()
{
    checkTxComplete();
    return sendingPacket == NULL && !isActivelyReceiving();
}

bool QuadRFRadio::isActivelyReceiving()
{
    std::lock_guard<std::mutex> lk(rx_mu_);
    return !rx_q_.empty();
}

bool QuadRFRadio::isChannelActive()
{
    return isActivelyReceiving();
}

bool QuadRFRadio::cancelSending(NodeNum from, PacketId id)
{
    auto p = txQueue.remove(from, id);
    if (p)
        packetPool.release(p);
    return p != NULL;
}

bool QuadRFRadio::findInTxQueue(NodeNum from, PacketId id)
{
    return txQueue.find(from, id);
}

meshtastic_QueueStatus QuadRFRadio::getQueueStatus()
{
    meshtastic_QueueStatus qs;
    qs.res = qs.mesh_packet_id = 0;
    qs.free = txQueue.getFree();
    qs.maxlen = txQueue.getMaxLen();
    return qs;
}

void QuadRFRadio::onNotify(uint32_t notification)
{
    switch (notification) {
    case ISR_TX:
        handleTransmitInterrupt();
        startTransmitTimer();
        break;
    case ISR_RX:
        checkTxComplete();
        handleReceiveInterrupt();
        startTransmitTimer();
        break;
    case TRANSMIT_DELAY_COMPLETED:
        checkTxComplete();
        handleReceiveInterrupt();
        if (!txQueue.empty()) {
            if (!canSendImmediately() || isChannelActive()) {
                setTransmitDelay();
            } else {
                meshtastic_MeshPacket *txp = txQueue.dequeue();
                assert(txp);
                startSend(txp);
                uint32_t xmitMsec = RadioInterface::getPacketTime(txp);
                airTime->logAirtime(TX_LOG, xmitMsec);
                tx_end_time_ms_ = millis() + xmitMsec;
                notifyLater(xmitMsec, ISR_TX, false);
            }
        }
        break;
    default:
        assert(0);
    }
}

void QuadRFRadio::startSend(meshtastic_MeshPacket *txp)
{
    printPacket("Start QuadRF send", txp);
    isReceiving_ = false;
    size_t numbytes = beginSending(txp);

    quadrf::air_ipc::TxEnqueue msg;
    // getFreq() is MHz; PHY wants Hz. 0 → PHY default.
    const float freq_mhz = getFreq();
    msg.freq_hz = quadrf::air_ipc::frequencyMHzToQuantizedHz(static_cast<double>(freq_mhz));
    msg.air.assign(reinterpret_cast<uint8_t *>(&radioBuffer),
                   reinterpret_cast<uint8_t *>(&radioBuffer) + numbytes);

    std::vector<uint8_t> frame;
    if (!quadrf::air_ipc::encodeTx(msg, frame) || !writeFrame(frame)) {
        LOG_ERROR("QuadRFRadio: TX enqueue failed (%zu air bytes)", numbytes);
        // Still complete so the queue does not stall.
    } else {
        LOG_INFO("QuadRF TX air=%zu freq_hz=%llu", numbytes, (unsigned long long)msg.freq_hz);
    }

    service->sendQueueStatusToPhone(getQueueStatus(), 0, txp->id);
}

void QuadRFRadio::handleReceiveInterrupt()
{
    checkTxComplete();
    for (;;) {
        RxFrame rx;
        {
            std::lock_guard<std::mutex> lk(rx_mu_);
            if (rx_q_.empty())
                return;
            rx = std::move(rx_q_.front());
            rx_q_.pop_front();
        }

        if (rx.air.size() < sizeof(PacketHeader)) {
            rxBad++;
            continue;
        }

        const size_t payloadLen = rx.air.size() - sizeof(PacketHeader);
        if (payloadLen > sizeof(radioBuffer.payload)) {
            rxBad++;
            continue;
        }

        std::memcpy(&radioBuffer, rx.air.data(), rx.air.size());
        if (radioBuffer.header.from == 0) {
            LOG_WARN("QuadRFRadio: ignore packet without sender");
            continue;
        }

        // Full-duplex SDR can demod our own TX. Half-duplex LoRa HALs never
        // see that; drop self-echoes here to match half-duplex transceiver semantics.
        if (nodeDB && radioBuffer.header.from == nodeDB->getNodeNum()) {
            LOG_INFO("QuadRFRadio: drop self-echo air=%zu snr=%.2f from=0x%x", rx.air.size(),
                     rx.snr_db, (unsigned)radioBuffer.header.from);
            continue;
        }

        meshtastic_MeshPacket *mp = packetPool.allocZeroed();
        mp->from = radioBuffer.header.from;
        mp->to = radioBuffer.header.to;
        mp->id = radioBuffer.header.id;
        mp->channel = radioBuffer.header.channel;
        mp->hop_limit = radioBuffer.header.flags & PACKET_FLAGS_HOP_LIMIT_MASK;
        mp->hop_start = (radioBuffer.header.flags & PACKET_FLAGS_HOP_START_MASK) >> PACKET_FLAGS_HOP_START_SHIFT;
        mp->want_ack = !!(radioBuffer.header.flags & PACKET_FLAGS_WANT_ACK_MASK);
        mp->via_mqtt = !!(radioBuffer.header.flags & PACKET_FLAGS_VIA_MQTT_MASK);
        mp->next_hop = mp->hop_start == 0 ? NO_NEXT_HOP_PREFERENCE : radioBuffer.header.next_hop;
        mp->relay_node = mp->hop_start == 0 ? NO_RELAY_NODE : radioBuffer.header.relay_node;
        mp->rx_snr = rx.snr_db;
        mp->rx_rssi = rx.rssi_dbm;
        mp->rx_time = getValidTime(RTCQualityDevice);
        last_rssi_ = rx.rssi_dbm;

        mp->which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
        std::memcpy(mp->encrypted.bytes, radioBuffer.payload, payloadLen);
        mp->encrypted.size = payloadLen;

        rxGood++;
        {
            std::lock_guard<std::mutex> lk(metrics_mu_);
            recent_metrics_.push_back({radioBuffer.header.id,
                RxMetrics{rx.snr_db, rx.rssi_dbm, rx.cfo_hz, rx.rate_ppm, millis()}});
            if (recent_metrics_.size() > 64) {
                recent_metrics_.pop_front();
            }
        }
        LOG_INFO("QuadRF RX air=%zu snr=%.2f rssi=%d", rx.air.size(), rx.snr_db, (int)rx.rssi_dbm);
        printPacket("Lora RX", mp);
        airTime->logAirtime(RX_LOG, RadioInterface::getPacketTime(mp, true));
        deliverToReceiver(mp);
    }
}

bool QuadRFRadio::getRxMetrics(uint32_t packet_id, RxMetrics& out)
{
    std::lock_guard<std::mutex> lk(metrics_mu_);
    for (auto it = recent_metrics_.rbegin(); it != recent_metrics_.rend(); ++it) {
        if (it->first == packet_id) {
            out = it->second;
            return true;
        }
    }
    return false;
}

void QuadRFRadio::startControlServer()
{
    const char *sock_path = "/run/quadrf/mesh_control.sock";
    ctl_sock_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (ctl_sock_fd_ < 0) {
        LOG_WARN("QuadRFRadio: failed to create mesh_control socket: %s", strerror(errno));
        return;
    }

    struct sockaddr_un addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, sock_path, sizeof(addr.sun_path) - 1);

    unlink(sock_path);
    if (bind(ctl_sock_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        LOG_WARN("QuadRFRadio: failed to bind %s: %s", sock_path, strerror(errno));
        close(ctl_sock_fd_);
        ctl_sock_fd_ = -1;
        return;
    }

    chmod(sock_path, 0666);

    if (listen(ctl_sock_fd_, 4) < 0) {
        LOG_WARN("QuadRFRadio: failed to listen on %s: %s", sock_path, strerror(errno));
        close(ctl_sock_fd_);
        ctl_sock_fd_ = -1;
        unlink(sock_path);
        return;
    }

    control_running_ = true;
    control_thread_ = std::thread(&QuadRFRadio::controlThreadMain, this);
    LOG_INFO("QuadRFRadio: control listener active on %s", sock_path);
}

void QuadRFRadio::stopControlServer()
{
    control_running_ = false;
    if (ctl_sock_fd_ >= 0) {
        close(ctl_sock_fd_);
        ctl_sock_fd_ = -1;
    }
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    unlink("/run/quadrf/mesh_control.sock");
}

void QuadRFRadio::controlThreadMain()
{
    while (control_running_) {
        struct pollfd pfd;
        pfd.fd = ctl_sock_fd_;
        pfd.events = POLLIN;
        pfd.revents = 0;
        int ret = poll(&pfd, 1, 500);
        if (ret <= 0) {
            continue;
        }

        int client_fd = accept(ctl_sock_fd_, nullptr, nullptr);
        if (client_fd < 0) {
            continue;
        }

        char buf[256];
        ssize_t n = read(client_fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            std::string cmd(buf);
            while (!cmd.empty() && (cmd.back() == '\r' || cmd.back() == '\n' || cmd.back() == ' ')) {
                cmd.pop_back();
            }

            std::string reply;
            if (cmd == "GET STATUS") {
                uint32_t r = QuadRFPingModule::instance ? QuadRFPingModule::instance->getIntervalSec() : 0;
                int p = (QuadRFParrotModule::instance && QuadRFParrotModule::instance->isEnabled()) ? 1 : 0;
                reply = "RANGE=" + std::to_string(r) + " PARROT=" + std::to_string(p) + "\n";
            } else if (cmd.rfind("SET RANGE ", 0) == 0) {
                uint32_t sec = static_cast<uint32_t>(std::strtoul(cmd.c_str() + 10, nullptr, 10));
                if (QuadRFPingModule::instance) {
                    QuadRFPingModule::instance->setIntervalSec(sec);
                }
                reply = "OK RANGE=" + std::to_string(sec) + "\n";
            } else if (cmd.rfind("SET PARROT ", 0) == 0) {
                int en = std::atoi(cmd.c_str() + 11);
                if (QuadRFParrotModule::instance) {
                    QuadRFParrotModule::instance->setEnabled(en != 0);
                }
                reply = "OK PARROT=" + std::to_string(en ? 1 : 0) + "\n";
            } else {
                reply = "ERR unknown command\n";
            }

            ssize_t w = write(client_fd, reply.data(), reply.size());
            (void)w;
        }
        close(client_fd);
    }
}

uint32_t QuadRFRadio::getPacketTime(uint32_t pl, bool /*received*/)
{
    float bandwidthHz = bw * 1000.0f;
    float tSym = (1 << sf) / bandwidthHz;
    bool lowDataOptEn = tSym > 16e-3;
    float tPreamble = (preambleLength + 4.25f) * tSym;
    float numPayloadSym =
        8 + max(ceilf(((8.0f * pl - 4 * sf + 28 + 16) / (4 * (sf - 2 * lowDataOptEn))) * cr), 0.0f);
    return static_cast<uint32_t>((tPreamble + numPayloadSym * tSym) * 1000.0f);
}

int16_t QuadRFRadio::getCurrentRSSI()
{
    return last_rssi_;
}
