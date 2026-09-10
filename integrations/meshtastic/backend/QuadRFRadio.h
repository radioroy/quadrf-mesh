// SPDX-FileCopyrightText: 2026 Roy C. Gross
// SPDX-License-Identifier: GPL-3.0-only
#pragma once

// Portduino RadioInterface bridging Meshtastic air frames to quadrf-lora-phy
// over the Air-IPC unix socket. Same role as SimRadio / SX1262 HAL:
// mesh/crypto stay in meshtasticd; QuadRF only modulates/demodulates.

#include "MeshPacketQueue.h"
#include "RadioInterface.h"
#include "concurrency/NotifiedWorkerThread.h"

#include <atomic>
#include <deque>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

class QuadRFRadio : public RadioInterface, protected concurrency::NotifiedWorkerThread
{
    enum PendingISR { ISR_NONE = 0, ISR_RX, ISR_TX, TRANSMIT_DELAY_COMPLETED };

    MeshPacketQueue txQueue = MeshPacketQueue(MAX_TX_QUEUE);

  public:
    explicit QuadRFRadio(std::string socket_path);
    ~QuadRFRadio() override;

    static QuadRFRadio *instance;

    bool init() override;
    bool reconfigure() override;

    ErrorCode send(meshtastic_MeshPacket *p) override;
    bool cancelSending(NodeNum from, PacketId id) override;
    bool findInTxQueue(NodeNum from, PacketId id) override;
    meshtastic_QueueStatus getQueueStatus() override;

    bool isChannelActive();
    bool isActivelyReceiving();
    int16_t getCurrentRSSI() override;

    uint32_t rxBad = 0, rxGood = 0, txGood = 0, txRelay = 0;
    uint16_t txDrop = 0;

    struct RxMetrics {
        float snr_db = 0;
        int16_t rssi_dbm = 0;
        int32_t cfo_hz = 0;
        float rate_ppm = 0;
        float sir_db = 0;
        float lvl_dbfs = 0;
        uint32_t timestamp_ms = 0;
    };

    bool getRxMetrics(uint32_t packet_id, RxMetrics& out);

    std::string getCallsign();
    void setCallsign(const std::string &call);

  protected:
    bool canSendImmediately();
    void completeSending();
    void checkTxComplete();
    uint32_t getPacketTime(uint32_t pl, bool received = false) override;

  private:
    void setTransmitDelay();
    void startTransmitTimer(bool withDelay = true);
    void startTransmitTimerRebroadcast(meshtastic_MeshPacket *p);
    void handleTransmitInterrupt();
    void handleReceiveInterrupt();
    void onNotify(uint32_t notification) override;
    void startSend(meshtastic_MeshPacket *txp);
    void pushModemToPhy();

    bool connectSocket();
    void closeSocket();
    void rxThreadMain();
    bool writeFrame(const std::vector<uint8_t> &frame);
    void enqueueRx(std::vector<uint8_t> air, float snr_db, int16_t rssi_dbm,
                   int32_t cfo_hz, float rate_ppm, float sir_db, float lvl_dbfs);

    void startControlServer();
    void stopControlServer();
    void controlThreadMain();

    void loadCallsign();
    void startBeaconThread();
    void stopBeaconThread();
    void beaconThreadMain();

    struct RxFrame {
        std::vector<uint8_t> air;
        float snr_db = 0;
        int16_t rssi_dbm = 0;
        int32_t cfo_hz = 0;
        float rate_ppm = 0;
        float sir_db = 0;
        float lvl_dbfs = 0;
    };

    std::string socket_path_;
    int sock_fd_ = -1;
    std::atomic<bool> rx_running_{false};
    std::thread rx_thread_;

    std::mutex rx_mu_;
    std::deque<RxFrame> rx_q_;

    int ctl_sock_fd_ = -1;
    std::atomic<bool> control_running_{false};
    std::thread control_thread_;

    mutable std::mutex callsign_mu_;
    std::string callsign_ = "NOCALL";
    time_t last_conf_mtime_ = 0;

    std::atomic<bool> beacon_running_{false};
    std::thread beacon_thread_;

    std::mutex peers_mu_;
    std::set<NodeNum> greeted_peers_;

    std::mutex metrics_mu_;
    std::deque<std::pair<uint32_t, RxMetrics>> recent_metrics_;

    bool isReceiving_ = true;
    int16_t last_rssi_ = -120;
    uint32_t tx_end_time_ms_ = 0;
    uint32_t last_queue_status_ms_ = 0;
};

extern QuadRFRadio *quadrfRadio;
