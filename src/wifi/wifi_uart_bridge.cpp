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

    m_Serial.begin(921600, SERIAL_8N1, uart_pins.rx_pin, uart_pins.tx_pin);
    m_Udp.begin(m_UdpPort);
    m_TargetIp = gsc_ip;

    LOG_INFO("UART bridge initialized - Target: %s:%d", gsc_ip.toString().c_str(), m_UdpPort);
}

/**
 * @brief Maybe around 50 to 100hz update rate could be ok?
 *
 */
void WifiUartBridge::Update()
{
    // Only process packets if WiFi is connected in STATION mode or if in AP mode
    if (WiFi.status() == WL_CONNECTED || WiFi.getMode() == WIFI_AP) {
        int packetSize = m_Udp.parsePacket();
        if (packetSize) {
            // Auto-discovery: Update target IP to the sender of the packet
            IPAddress newTarget = m_Udp.remoteIP();
            if (newTarget != m_TargetIp) {
                m_TargetIp = newTarget;
            }

            // Receive incoming UDP packets
            int len = m_Udp.read(incomingPacket, max_packet_size);
            if (len > 0) {
                m_Serial.write(incomingPacket, len);
            }
        }

        // Read data from Serial into buffer
        // Read data from Serial into buffer
        int available = m_Serial.available();
        if (available > 0) {
            int spaceLeft = max_packet_size - m_BufferIndex;
            int toRead = min(available, spaceLeft);
            if (toRead > 0) {
                int bytesRead = m_Serial.readBytes(incomingSerialPacket + m_BufferIndex, toRead);
                m_BufferIndex += bytesRead;
            }
        }

        // Send conditions:
        // 1. Buffer has data AND (Buffer is large enough OR Time threshold exceeded)
        // 2. Target IP is valid (not 0.0.0.0)
        bool timeThresholdExceeded = (millis() - m_LastSendTime) > kTimeThresholdMs;
        bool bufferThresholdExceeded = m_BufferIndex >= kBufferThreshold;

        if (m_BufferIndex > 0 && (timeThresholdExceeded || bufferThresholdExceeded)) {
            if (m_TargetIp != IPAddress(0,0,0,0)) {
                m_Udp.beginPacket(m_TargetIp, m_UdpPort);
                m_Udp.write(incomingSerialPacket, m_BufferIndex);
                m_Udp.endPacket();

                m_LastSendTime = millis();
                m_BufferIndex = 0;
            } else {
                // No target IP - discard when buffer is full
                if (m_BufferIndex >= max_packet_size) {
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
}

#endif // USE_WIFI_UART_BRIDGE
