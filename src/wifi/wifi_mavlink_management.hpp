#pragma once

#include "config/features.hpp"

#ifdef USE_WIFI_MAVLINK_MANAGEMENT

#include <Arduino.h>
#include <IPAddress.h>
#include <WiFiUdp.h>

#include "wifi_backend.hpp"
#include "wifi_device_telemetry.hpp"
#include "wifi_params.hpp"

typedef struct __mavlink_message mavlink_message_t;
typedef struct __mavlink_status mavlink_status_t;

class WifiMavlinkManagement : public WifiBackend {
public:
    WifiMavlinkManagement(uint16_t port, const WifiParams& wifiParams);
    ~WifiMavlinkManagement() override;

    void Update() override;
    void SetTelemetryCallback(TelemetryCallback callback);

private:
    void ProcessPacket();
    void HandleMessage(const mavlink_message_t& message);
    void HandleParamExtRequestList(const mavlink_message_t& message);
    void HandleParamExtRequestRead(const mavlink_message_t& message);
    void HandleParamExtSet(const mavlink_message_t& message);
    void HandleRtlsCommand(const mavlink_message_t& message);

    void SendHeartbeat();
    void SendDeviceStatus();
    void SendParamExtValue(size_t index);
    void SendParamExtAck(const char* id, const char* value, ParamType type, uint8_t result);
    void SendCommandResponse(uint32_t requestId, uint16_t command, uint8_t result, uint8_t payloadType, const uint8_t* payload, size_t payloadLen);
    void SendTextResponse(uint32_t requestId, uint16_t command, uint8_t result, const char* text);
    void SendMessage(const mavlink_message_t& message);
    void SendMessageToGcs(const mavlink_message_t& message);
    void SendMessageTo(const mavlink_message_t& message, IPAddress targetIp, uint16_t targetPort);

    bool IsTarget(uint8_t targetSystem, uint8_t targetComponent) const;
    uint8_t SystemId() const;
    uint8_t ComponentId() const;

private:
    static constexpr uint32_t kStatusIntervalMs = 1000;
    static constexpr size_t kRxBufferSize = 512;

    WiFiUDP m_Udp;
    uint16_t m_Port;
    const WifiParams& m_WifiParams;
    IPAddress m_RemoteIp;
    uint16_t m_RemotePort = 0;
    uint32_t m_LastStatusMs = 0;
    mavlink_status_t* m_ParseStatus = nullptr;
    TelemetryCallback m_TelemetryCallback;
};

#endif // USE_WIFI_MAVLINK_MANAGEMENT
