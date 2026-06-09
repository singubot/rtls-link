#pragma once

#include "config/features.hpp"

#ifdef USE_WIFI_UART_BRIDGE

#include <Arduino.h>
#include <IPAddress.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

#include "wifi_backend.hpp"

/**
 * @brief It basically needs a Serial port and a UDP port and IP address to forward the communication between the two.
 *
 */

class WifiUartBridge : public WifiBackend {
private:
public:
    WifiUartBridge(HardwareSerial &serial, IPAddress gsc_ip, uint16_t local_udp_port);

    void Update() override;

private:
    static constexpr uint16_t kMaxPacketSize = 4096;
    static constexpr uint16_t kUdpDriverBufferSize = 1460;
    static constexpr uint16_t kUdpPacketPayloadMax = 1024;
    static constexpr size_t kUartRxBufferSize = 4096;
    static constexpr size_t kUartTxBufferSize = 1024;
    static constexpr uint32_t kStatsLogIntervalMs = 5000;

    struct BridgeStats {
        uint32_t udpRxPackets = 0;
        uint32_t udpRxBytes = 0;
        uint32_t udpRxAtDriverLimitPackets = 0;
        uint32_t uartTxBytes = 0;
        uint32_t uartTxShortWrites = 0;
        uint32_t uartRxBytes = 0;
        uint32_t udpTxPackets = 0;
        uint32_t udpTxBytes = 0;
        uint32_t udpTxFailures = 0;
        uint32_t serialBufferFullEvents = 0;
        uint32_t noTargetDropBytes = 0;
        uint32_t updateCount = 0;
        uint32_t updateDurationSumUs = 0;
        uint32_t updateDurationMaxUs = 0;
        uint16_t serialAvailableMax = 0;
        uint16_t udpTxPayloadMax = 0;
    };

    void FlushSerialBufferToUdp();
    void LogStatsIfDue(uint32_t now_ms);
    void ResetStats();
    void RecordUpdateDuration(uint32_t duration_us);

private:
    HardwareSerial& m_Serial;
    IPAddress gsc_ip;
    uint16_t m_UdpPort;
    WiFiUDP m_Udp;
    uint8_t incomingPacket[kMaxPacketSize];
    uint8_t incomingSerialPacket[kMaxPacketSize];
    const IPAddress GSCIp;

    // Auto-discovery and buffering
    IPAddress m_TargetIp;
    uint32_t m_LastSendTime = 0;
    uint32_t m_LastStatsLogMs = 0;
    uint16_t m_BufferIndex = 0;
    static constexpr uint16_t kBufferThreshold = kUdpPacketPayloadMax; // Send if buffer exceeds this
    static constexpr uint32_t kTimeThresholdMs = 5;  // Send if older than this
    BridgeStats m_Stats;

};

#endif // USE_WIFI_UART_BRIDGE
