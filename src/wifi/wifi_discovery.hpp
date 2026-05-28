#pragma once

#include "config/features.hpp"

#ifdef USE_WIFI_DISCOVERY

#include <Arduino.h>
#include <IPAddress.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <etl/delegate.h>

#include "wifi_backend.hpp"
#include "wifi_params.hpp"

/**
 * @brief Dynamic anchor position telemetry (for discovery heartbeat)
 */
struct DynamicAnchorTelemetry {
    uint8_t id;
    float x, y, z;
};

/**
 * @brief Telemetry data for device status reporting.
 */
struct DeviceTelemetry {
    bool sending_pos = false;   // True if sent position to ArduPilot in last 2s
    uint8_t anchors_seen = 0;   // Unique anchor IDs in measurement set
    bool origin_sent = false;   // True if GPS origin sent to ArduPilot
    bool uwb_enabled = true;    // True if runtime UWB backend is enabled
    bool rf_forward_enabled = false; // True if rangefinder forwarding is enabled
    bool rf_enabled = false;    // True if rangefinder functionality is active
    bool rf_healthy = false;    // True if receiving non-stale rangefinder data
    // Update rate statistics (centi-Hz for 0.01 Hz precision without floats)
    uint16_t avg_rate_cHz = 0;  // Average update rate in centi-Hz (e.g., 1000 = 10.0 Hz)
    uint16_t min_rate_cHz = 0;  // Min rate in last 5s window
    uint16_t max_rate_cHz = 0;  // Max rate in last 5s window

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    bool dynamic_anchors_enabled = false;
    DynamicAnchorTelemetry dynamic_anchors[4];
    uint8_t dynamic_anchor_count = 0;
#endif
};

using TelemetryCallback = etl::delegate<DeviceTelemetry()>;

/**
 * @brief UDP Heartbeat Backend for RTLS-Link devices.
 *
 * Sends periodic binary heartbeat packets to the configured GCS IP address.
 *
 * Protocol:
 * - Send heartbeat every 2 seconds to GCS IP on configured port
 * - Payload is an RTLS binary protocol Heartbeat frame.
 */
class WifiDiscovery : public WifiBackend {
public:
    WifiDiscovery(uint16_t port, const WifiParams& wifiParams);

    void Update() override;

    /**
     * @brief Set the callback to retrieve telemetry data for heartbeat.
     * @param callback Delegate that returns DeviceTelemetry struct
     */
    void SetTelemetryCallback(TelemetryCallback callback);

private:
    void SendHeartbeat();
    uint8_t ModeToRoleId(uint8_t mode);
#if defined(USE_UWB_ANCHOR_TELEMETRY) && defined(USE_UWB_MODE_TDOA_ANCHOR)
    void UpdateAnchorTelemetry();
    uint16_t GetAnchorTelemetryIntervalMs() const;
    void SendAnchorTelemetry();
#endif

private:
    static constexpr uint32_t kHeartbeatIntervalMs = 2000; // 2 seconds
#if defined(USE_UWB_ANCHOR_TELEMETRY) && defined(USE_UWB_MODE_TDOA_ANCHOR)
    static constexpr uint16_t kAnchorTelemetryMinIntervalMs = 250;
    static constexpr uint16_t kAnchorTelemetryMaxIntervalMs = 60000;
#endif

    WiFiUDP m_Udp;
    uint16_t m_Port;
    const WifiParams& m_WifiParams;
    uint32_t m_LastHeartbeat = 0;
    TelemetryCallback m_TelemetryCallback;
#if defined(USE_UWB_ANCHOR_TELEMETRY) && defined(USE_UWB_MODE_TDOA_ANCHOR)
    uint32_t m_LastAnchorTelemetryMs = 0;
    bool m_AnchorTelemetrySendWarned = false;
#endif
};

#endif // USE_WIFI_DISCOVERY
