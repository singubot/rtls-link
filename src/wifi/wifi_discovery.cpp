#include "config/features.hpp"

#ifdef USE_WIFI_DISCOVERY

#include "wifi_discovery.hpp"
#include "uwb/uwb_frontend_littlefs.hpp"
#include "version.hpp"
#include "logging/logging.hpp"
#include "protocol/rtls_binary_protocol.hpp"

#if defined(USE_UWB_ANCHOR_TELEMETRY) && defined(USE_UWB_MODE_TDOA_ANCHOR)
#include "anchor/tdoa_anchor_api.h"
#include "protocol/tdoa_anchor_stats_frame.hpp"
#endif

#include <esp_mac.h>

WifiDiscovery::WifiDiscovery(uint16_t port, const WifiParams& wifiParams)
    : m_Port(port)
    , m_WifiParams(wifiParams)
{
    LOG_INFO("WifiDiscovery: Heartbeat mode on port %d", m_Port);
}

void WifiDiscovery::SetTelemetryCallback(TelemetryCallback callback) {
    m_TelemetryCallback = callback;
}

void WifiDiscovery::Update() {
#if defined(USE_UWB_ANCHOR_TELEMETRY) && defined(USE_UWB_MODE_TDOA_ANCHOR)
    UpdateAnchorTelemetry();
#endif

    // Send heartbeat at interval
    uint32_t now = millis();
    if (now - m_LastHeartbeat < kHeartbeatIntervalMs) {
        return;
    }
    m_LastHeartbeat = now;

    SendHeartbeat();
}

void WifiDiscovery::SendHeartbeat() {
    // Get GCS IP from wifi params
    IPAddress gcsIp;
    if (!gcsIp.fromString(m_WifiParams.gcsIp.data())) {
        return; // No valid GCS IP configured
    }

    rtls::protocol::BinaryFrameBuilder<512> frame;

    // Get IP and MAC based on WiFi mode
    bool isAP = (WiFi.getMode() == WIFI_AP);
    IPAddress deviceIp = isAP ? WiFi.softAPIP() : WiFi.localIP();

    // Get MAC without heap allocation (use correct interface type)
    uint8_t mac[6];
    esp_read_mac(mac, isAP ? ESP_MAC_WIFI_SOFTAP : ESP_MAC_WIFI_STA);

    // Get UWB params
    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();

    // devShortAddr is a 2-byte array; send printable digits when available
    char shortAddrStr[3] = {};
    size_t shortAddrLen = 0;
    if (uwbParams.devShortAddr[0] != '\0') {
        shortAddrStr[shortAddrLen++] = uwbParams.devShortAddr[0];
    }
    if (uwbParams.devShortAddr[1] != '\0') {
        shortAddrStr[shortAddrLen++] = uwbParams.devShortAddr[1];
    }
    if (shortAddrLen == 0) {
        shortAddrStr[shortAddrLen++] = '0';
    }
    shortAddrStr[shortAddrLen] = '\0';

    // Get telemetry data via callback (or use defaults)
    DeviceTelemetry telemetry = {};
    if (m_TelemetryCallback.is_valid()) {
        telemetry = m_TelemetryCallback();
    }

    // Get log level from compiled setting
#ifdef USE_LOGGING
    uint8_t logLevel = rtls::log::Logger::getCompiledLogLevel();
#else
    uint8_t logLevel = 0; // Disabled
#endif

    uint16_t flags = 0;
    if (telemetry.sending_pos) flags |= (1u << 0);
    if (telemetry.origin_sent) flags |= (1u << 1);
    if (telemetry.rf_enabled) flags |= (1u << 2);
    if (telemetry.rf_healthy) flags |= (1u << 3);
    if (telemetry.uwb_enabled) flags |= (1u << 4);
    if (telemetry.rf_forward_enabled) flags |= (1u << 5);
    if (m_WifiParams.logSerialEnabled) flags |= (1u << 6);
    if (m_WifiParams.logUdpEnabled) flags |= (1u << 7);

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    if (telemetry.dynamic_anchors_enabled) flags |= (1u << 8);
#endif

    frame.Begin(rtls::protocol::FrameType::Heartbeat);
    frame.AppendU8(ModeToRoleId(static_cast<uint8_t>(uwbParams.mode)));
    frame.AppendU16(flags);
    frame.AppendU8(telemetry.anchors_seen);
    frame.AppendU8(static_cast<uint8_t>(uwbParams.mavlinkTargetSystemId));
    frame.AppendU16(telemetry.avg_rate_cHz);
    frame.AppendU16(telemetry.min_rate_cHz);
    frame.AppendU16(telemetry.max_rate_cHz);
    frame.AppendU8(logLevel);
    frame.AppendU16(m_WifiParams.logUdpPort);
    frame.AppendBytes(mac, sizeof(mac));
    frame.AppendU8(deviceIp[0]);
    frame.AppendU8(deviceIp[1]);
    frame.AppendU8(deviceIp[2]);
    frame.AppendU8(deviceIp[3]);
    frame.AppendString(DEVICE_TYPE);
    frame.AppendString(shortAddrStr);
    frame.AppendString(shortAddrStr);
    frame.AppendString(FIRMWARE_VERSION);

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    const uint8_t dynamicCount = telemetry.dynamic_anchors_enabled
        ? telemetry.dynamic_anchor_count
        : 0;
    frame.AppendU8(dynamicCount);
    for (uint8_t i = 0; i < dynamicCount; i++) {
        frame.AppendU8(telemetry.dynamic_anchors[i].id);
        frame.AppendI32(rtls::protocol::MetersToMillimeters(telemetry.dynamic_anchors[i].x));
        frame.AppendI32(rtls::protocol::MetersToMillimeters(telemetry.dynamic_anchors[i].y));
        frame.AppendI32(rtls::protocol::MetersToMillimeters(telemetry.dynamic_anchors[i].z));
    }
#else
    frame.AppendU8(0);
#endif
    frame.Finish();

    static bool truncation_warned = false;
    if (frame.Truncated() && !truncation_warned) {
        LOG_WARN("WifiDiscovery: binary heartbeat truncated");
        truncation_warned = true;
    }

    m_Udp.beginPacket(gcsIp, m_Port);
    m_Udp.write(frame.Data(), frame.Size());
    m_Udp.endPacket();
}

#if defined(USE_UWB_ANCHOR_TELEMETRY) && defined(USE_UWB_MODE_TDOA_ANCHOR)
void WifiDiscovery::UpdateAnchorTelemetry() {
    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
    if (uwbParams.mode != UWBMode::ANCHOR_TDOA ||
        uwbParams.uwbEnable == 0 ||
        uwbParams.tdoaAnchorTelemetryEnable == 0 ||
        uwbParams.tdoaAnchorTelemetryPort == 0) {
        return;
    }

    const uint32_t now = millis();
    if (now - m_LastAnchorTelemetryMs < GetAnchorTelemetryIntervalMs()) {
        return;
    }
    m_LastAnchorTelemetryMs = now;

    SendAnchorTelemetry();
}

uint16_t WifiDiscovery::GetAnchorTelemetryIntervalMs() const {
    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
    if (uwbParams.tdoaAnchorTelemetryIntervalMs < kAnchorTelemetryMinIntervalMs) {
        return kAnchorTelemetryMinIntervalMs;
    }
    if (uwbParams.tdoaAnchorTelemetryIntervalMs > kAnchorTelemetryMaxIntervalMs) {
        return kAnchorTelemetryMaxIntervalMs;
    }
    return uwbParams.tdoaAnchorTelemetryIntervalMs;
}

void WifiDiscovery::SendAnchorTelemetry() {
    IPAddress gcsIp;
    if (!gcsIp.fromString(m_WifiParams.gcsIp.data())) {
        return;
    }

    uwbTdoa2AnchorStats_t stats = {};
    if (!uwbTdoa2AnchorGetStats(&stats)) {
        return;
    }

    rtls::protocol::BinaryFrameBuilder<512> frame;
    rtls::protocol::AppendTdoaAnchorStatsFrame(frame, stats);

    static bool truncationWarned = false;
    if (frame.Truncated() && !truncationWarned) {
        LOG_WARN("WifiDiscovery: anchor telemetry frame truncated");
        truncationWarned = true;
    }

    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
    const int beginResult = m_Udp.beginPacket(gcsIp, uwbParams.tdoaAnchorTelemetryPort);
    const size_t bytesWritten = beginResult ? m_Udp.write(frame.Data(), frame.Size()) : 0;
    const int endResult = beginResult ? m_Udp.endPacket() : 0;
    if ((!beginResult || bytesWritten != frame.Size() || !endResult) && !m_AnchorTelemetrySendWarned) {
        LOG_WARN("WifiDiscovery: anchor telemetry UDP send failed begin=%d written=%u/%u end=%d",
                 beginResult,
                 static_cast<unsigned int>(bytesWritten),
                 static_cast<unsigned int>(frame.Size()),
                 endResult);
        m_AnchorTelemetrySendWarned = true;
    }
}
#endif

uint8_t WifiDiscovery::ModeToRoleId(uint8_t mode) {
    switch (mode) {
        case 3: return 3; // anchor_tdoa
        case 4: return 4; // tag_tdoa
        default: return 0;
    }
}

#endif // USE_WIFI_DISCOVERY
