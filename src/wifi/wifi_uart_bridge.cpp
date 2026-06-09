#include "config/features.hpp"

#ifdef USE_WIFI_UART_BRIDGE

#include <Arduino.h>
#include <WiFi.h>

#include "wifi_uart_bridge.hpp"
#include "logging/logging.hpp"

#include "bsp/board.hpp"


WifiUartBridge::WifiUartBridge(HardwareSerial &serial, IPAddress gsc_ip, uint16_t local_udp_port)
    : m_Serial(serial), gsc_ip(gsc_ip), m_UdpPort(local_udp_port), GSCIp(gsc_ip)
{
    const auto& uart_pins = bsp::kBoardConfig.mavlink_uart;

    const size_t rxBufferSize = m_Serial.setRxBufferSize(kUartRxBufferSize);
    const size_t txBufferSize = m_Serial.setTxBufferSize(kUartTxBufferSize);
    if (rxBufferSize < kUartRxBufferSize || txBufferSize < kUartTxBufferSize) {
        LOG_WARN("UART bridge buffer setup requested rx=%u tx=%u, got rx=%u tx=%u",
                 static_cast<unsigned int>(kUartRxBufferSize),
                 static_cast<unsigned int>(kUartTxBufferSize),
                 static_cast<unsigned int>(rxBufferSize),
                 static_cast<unsigned int>(txBufferSize));
    }

    m_Serial.begin(921600, SERIAL_8N1, uart_pins.rx_pin, uart_pins.tx_pin);
    m_Udp.begin(m_UdpPort);
    m_TargetIp = gsc_ip;

    LOG_INFO("UART bridge initialized - Target: %s:%d rxBuf=%u txBuf=%u",
             gsc_ip.toString().c_str(),
             m_UdpPort,
             static_cast<unsigned int>(rxBufferSize),
             static_cast<unsigned int>(txBufferSize));
}

/**
 * @brief Maybe around 50 to 100hz update rate could be ok?
 *
 */
void WifiUartBridge::Update()
{
    const uint32_t updateStartUs = micros();
    const uint32_t nowMs = millis();

    // Only process packets if WiFi is connected in STATION mode or if in AP mode
    if (WiFi.status() == WL_CONNECTED || WiFi.getMode() == WIFI_AP) {
        int packetSize = m_Udp.parsePacket();
        if (packetSize) {
            if (packetSize >= kUdpDriverBufferSize) {
                m_Stats.udpRxAtDriverLimitPackets++;
            }

            // Auto-discovery: Update target IP to the sender of the packet
            IPAddress newTarget = m_Udp.remoteIP();
            if (newTarget != m_TargetIp) {
                m_TargetIp = newTarget;
            }

            // Receive incoming UDP packets
            int len = m_Udp.read(incomingPacket, kMaxPacketSize);
            if (len > 0) {
                m_Stats.udpRxPackets++;
                m_Stats.udpRxBytes += static_cast<uint32_t>(len);

                const size_t bytesWritten = m_Serial.write(incomingPacket, len);
                m_Stats.uartTxBytes += static_cast<uint32_t>(bytesWritten);
                if (bytesWritten != static_cast<size_t>(len)) {
                    m_Stats.uartTxShortWrites++;
                }
            }
        }

        // Read data from Serial into buffer
        int available = m_Serial.available();
        if (available > 0) {
            if (available > m_Stats.serialAvailableMax) {
                m_Stats.serialAvailableMax = static_cast<uint16_t>(min(available, static_cast<int>(UINT16_MAX)));
            }

            int spaceLeft = kMaxPacketSize - m_BufferIndex;
            if (available > spaceLeft) {
                m_Stats.serialBufferFullEvents++;
            }

            int toRead = min(available, spaceLeft);
            if (toRead > 0) {
                int bytesRead = m_Serial.readBytes(incomingSerialPacket + m_BufferIndex, toRead);
                m_BufferIndex += bytesRead;
                m_Stats.uartRxBytes += static_cast<uint32_t>(bytesRead);
            }
        }

        // Send conditions:
        // 1. Buffer has data AND (Buffer is large enough OR Time threshold exceeded)
        // 2. Target IP is valid (not 0.0.0.0)
        bool timeThresholdExceeded = (nowMs - m_LastSendTime) > kTimeThresholdMs;
        bool bufferThresholdExceeded = m_BufferIndex >= kBufferThreshold;

        if (m_BufferIndex > 0 && (timeThresholdExceeded || bufferThresholdExceeded)) {
            if (m_TargetIp != IPAddress(0,0,0,0)) {
                FlushSerialBufferToUdp();
                m_LastSendTime = nowMs;
                m_BufferIndex = 0;
            } else {
                // No target IP - discard when buffer is full
                if (m_BufferIndex >= kMaxPacketSize) {
                    m_Stats.noTargetDropBytes += m_BufferIndex;
                    m_BufferIndex = 0;
                }
            }
        }

    } else {
        // WiFi not connected - discard serial data
        while (m_Serial.available()) {
            m_Serial.read();
        }
    }

    RecordUpdateDuration(micros() - updateStartUs);
    LogStatsIfDue(nowMs);
}

void WifiUartBridge::FlushSerialBufferToUdp()
{
    uint16_t offset = 0;

    while (offset < m_BufferIndex) {
        const uint16_t remaining = m_BufferIndex - offset;
        const uint16_t sendLength = remaining > kUdpPacketPayloadMax ? kUdpPacketPayloadMax : remaining;
        const int beginOk = m_Udp.beginPacket(m_TargetIp, m_UdpPort);
        const size_t bytesWritten = beginOk == 1 ? m_Udp.write(incomingSerialPacket + offset, sendLength) : 0;
        const int endOk = beginOk == 1 ? m_Udp.endPacket() : 0;

        if (beginOk == 1 && bytesWritten == sendLength && endOk == 1) {
            m_Stats.udpTxPackets++;
            m_Stats.udpTxBytes += static_cast<uint32_t>(sendLength);
            if (sendLength > m_Stats.udpTxPayloadMax) {
                m_Stats.udpTxPayloadMax = sendLength;
            }
        } else {
            m_Stats.udpTxFailures++;
        }

        offset += sendLength;
    }
}

void WifiUartBridge::LogStatsIfDue(uint32_t now_ms)
{
    if (m_LastStatsLogMs == 0) {
        m_LastStatsLogMs = now_ms;
        return;
    }

    if ((now_ms - m_LastStatsLogMs) < kStatsLogIntervalMs) {
        return;
    }

    const bool hasActivity =
        m_Stats.udpRxPackets > 0
        || m_Stats.udpTxPackets > 0
        || m_Stats.uartRxBytes > 0
        || m_Stats.uartTxBytes > 0
        || m_Stats.udpTxFailures > 0
        || m_Stats.uartTxShortWrites > 0
        || m_Stats.udpRxAtDriverLimitPackets > 0
        || m_Stats.serialBufferFullEvents > 0
        || m_Stats.noTargetDropBytes > 0;

    if (hasActivity) {
        const uint32_t avgUpdateUs = m_Stats.updateCount > 0
            ? m_Stats.updateDurationSumUs / m_Stats.updateCount
            : 0;

        LOG_INFO("UART bridge I/O: udpRx=%lu/%luB uartTx=%luB uartRx=%luB udpTx=%lu/%luB",
                 static_cast<unsigned long>(m_Stats.udpRxPackets),
                 static_cast<unsigned long>(m_Stats.udpRxBytes),
                 static_cast<unsigned long>(m_Stats.uartTxBytes),
                 static_cast<unsigned long>(m_Stats.uartRxBytes),
                 static_cast<unsigned long>(m_Stats.udpTxPackets),
                 static_cast<unsigned long>(m_Stats.udpTxBytes));

        LOG_INFO("UART bridge diag: fail=%lu short=%lu rxLimit=%lu full=%lu noTgt=%lu maxAvail=%u maxPkt=%u upd=%lu avgMaxUs=%lu/%lu",
                 static_cast<unsigned long>(m_Stats.udpTxFailures),
                 static_cast<unsigned long>(m_Stats.uartTxShortWrites),
                 static_cast<unsigned long>(m_Stats.udpRxAtDriverLimitPackets),
                 static_cast<unsigned long>(m_Stats.serialBufferFullEvents),
                 static_cast<unsigned long>(m_Stats.noTargetDropBytes),
                 static_cast<unsigned int>(m_Stats.serialAvailableMax),
                 static_cast<unsigned int>(m_Stats.udpTxPayloadMax),
                 static_cast<unsigned long>(m_Stats.updateCount),
                 static_cast<unsigned long>(avgUpdateUs),
                 static_cast<unsigned long>(m_Stats.updateDurationMaxUs));
    }

    ResetStats();
    m_LastStatsLogMs = now_ms;
}

void WifiUartBridge::ResetStats()
{
    m_Stats = {};
}

void WifiUartBridge::RecordUpdateDuration(uint32_t duration_us)
{
    m_Stats.updateCount++;
    m_Stats.updateDurationSumUs += duration_us;
    if (duration_us > m_Stats.updateDurationMaxUs) {
        m_Stats.updateDurationMaxUs = duration_us;
    }
}

#endif // USE_WIFI_UART_BRIDGE
