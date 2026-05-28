#pragma once

#include "wifi_params.hpp"
#include "../littlefs_frontend.hpp"
#include "../scheduler.hpp"
#include "wifi_backend.hpp"
#include <freertos/semphr.h>

class WifiLittleFSFrontend : public LittleFSFrontend<WifiParams> {
public:
    WifiLittleFSFrontend() : LittleFSFrontend<WifiParams>("wifi") {}

    virtual void Init() override;
    virtual void Update() override;
    virtual ErrorParam SetParam(const char* name, const void* data, uint32_t len) override;

    virtual etl::span<const ParamDef> GetParamLayout() const override {
        return etl::span<const ParamDef>(s_ParamDefs, sizeof(s_ParamDefs)/sizeof(ParamDef));
    }

    virtual const etl::string_view GetParamGroup() const override {
        return etl::string_view("wifi");
    }

    WifiParams& GetParams() {
        return m_Params;
    }

    void StationConnectionThread();

private:
    bool SetupAP();
    void SetupStation();
    void SetupNetworkServices();
    void UpdateMode(WifiMode mode);
    void ClearBackends();
    void ClearBackendsUnlocked();
    void ApplyLoggingSettings();
    bool NetworkServicesActive() const;
    void RequestModeUpdate(WifiMode mode);
    void RequestNetworkServicesSetup();

    static constexpr uint32_t maxClients = 10;

    etl::vector<WifiBackend*, maxClients> m_Backends;
    SemaphoreHandle_t m_backendsMutex = nullptr;
    WifiMode m_currentMode = WifiMode::UNDEFINED;
    bool m_stationConnected = false;
    bool m_updatingBackends = false;
    bool m_pendingNetworkServicesSetup = false;
    bool m_pendingModeUpdate = false;
    WifiMode m_pendingMode = WifiMode::UNDEFINED;

public:
    static constexpr ParamDef s_ParamDefs[] = {
        PARAM_DEF(WifiParams, mode),
        PARAM_DEF(WifiParams, ssidAP),
        PARAM_DEF(WifiParams, pswdAP),
        PARAM_DEF(WifiParams, ssidST),
        PARAM_DEF(WifiParams, pswdST),
        PARAM_DEF(WifiParams, gcsIp),
        PARAM_DEF(WifiParams, udpPort),
        PARAM_DEF(WifiParams, enableWebServer),
        PARAM_DEF(WifiParams, enableUartBridge),
        PARAM_DEF(WifiParams, logUdpPort),
        PARAM_DEF(WifiParams, logSerialEnabled),
        PARAM_DEF(WifiParams, logUdpEnabled)
    };
};

namespace Front {
    extern WifiLittleFSFrontend wifiLittleFSFront;
}
