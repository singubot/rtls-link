#include "config/features.hpp"  // MUST be first project include

#include "wifi_frontend_littlefs.hpp"

#ifdef USE_OTA_WEB
#include "wifi_ota_server.hpp"
#endif

#ifdef USE_WIFI_UART_BRIDGE
#include "wifi_uart_bridge.hpp"
#endif

#ifdef USE_WIFI_MAVLINK_MANAGEMENT
#include "wifi_mavlink_management.hpp"
#endif

#include "logging/logging.hpp"

#include "command_handler/command_handler.hpp"
#include <utils/utils.hpp>
#include <etl/vector.h>

#include "app.hpp"
#include "uwb/uwb_tdoa_tag.hpp"
#include "uwb/uwb_frontend_littlefs.hpp"

namespace {
class BackendsLockGuard {
public:
    explicit BackendsLockGuard(SemaphoreHandle_t mutex)
        : m_mutex(mutex) {
        if (m_mutex == nullptr) {
            m_locked = true;
            return;
        }
        m_locked = (xSemaphoreTake(m_mutex, portMAX_DELAY) == pdTRUE);
    }

    ~BackendsLockGuard() {
        if (m_mutex != nullptr && m_locked) {
            xSemaphoreGive(m_mutex);
        }
    }

    bool IsLocked() const {
        return m_locked;
    }

private:
    SemaphoreHandle_t m_mutex = nullptr;
    bool m_locked = false;
};
} // namespace

// Free function for telemetry callback (ETL delegates require free function or static method)
#ifdef USE_WIFI_MAVLINK_MANAGEMENT
static DeviceTelemetry GetDeviceTelemetry() {
    DeviceTelemetry t;
    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();

#ifdef USE_MAVLINK
    t.sending_pos = App::IsSendingPositions();
    t.origin_sent = App::IsOriginSent();
#else
    t.sending_pos = false;
    t.origin_sent = false;
#endif

    // Only report anchors_seen for TDoA tag mode
#ifdef USE_UWB_MODE_TDOA_TAG
    if (uwbParams.mode == UWBMode::TAG_TDOA) {
        t.anchors_seen = UWBTagTDoA::GetAnchorsSeenCount();
    } else {
        t.anchors_seen = 0;  // Not applicable for non-TDoA-tag modes
    }
#else
    t.anchors_seen = 0;
#endif

    // Runtime subsystem state
#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
    t.uwb_enabled = (uwbParams.uwbEnable != 0);
#else
    t.uwb_enabled = true;
#endif
    t.rf_forward_enabled = (uwbParams.rfForwardEnable != 0);

    // Rangefinder status
#ifdef HAS_RANGEFINDER
    t.rf_enabled = App::IsRangefinderEnabled();
    t.rf_healthy = App::IsRangefinderHealthy();
#else
    t.rf_enabled = false;
    t.rf_healthy = false;
#endif

    // Update rate statistics (for tags)
#ifdef USE_RATE_STATISTICS
    t.avg_rate_cHz = App::GetAvgUpdateRateCHz();
    t.min_rate_cHz = App::GetMinRateCHz();
    t.max_rate_cHz = App::GetMaxRateCHz();
#else
    t.avg_rate_cHz = 0;
    t.min_rate_cHz = 0;
    t.max_rate_cHz = 0;
#endif

    // Dynamic anchor positions (for TDoA tag mode with dynamic positioning enabled)
#if defined(USE_DYNAMIC_ANCHOR_POSITIONS) && defined(USE_UWB_MODE_TDOA_TAG)
    if (uwbParams.mode == UWBMode::TAG_TDOA) {
        t.dynamic_anchors_enabled = UWBTagTDoA::AreDynamicAnchorPositionsReady();
        if (t.dynamic_anchors_enabled) {
            t.dynamic_anchor_count = UWBTagTDoA::GetDynamicAnchorPositions(
                t.dynamic_anchors, 8);
        } else {
            t.dynamic_anchor_count = 0;
        }
    } else {
        t.dynamic_anchors_enabled = false;
        t.dynamic_anchor_count = 0;
    }
#endif

    return t;
}
#endif // USE_WIFI_MAVLINK_MANAGEMENT

// I will define a static FreeRTOS task holder for station connection checks
static StaticTaskHolder<etl::delegate<void()>, 4096> wifi_connection_task = {
    "WifiConnTask", // Task name
    1,              // Frequency: 1Hz
    1,              // Priority
    etl::delegate<void()>(),  // Will be set in Init()
    {},
    {}
};

void WifiLittleFSFrontend::Init() {
    LittleFSFrontend<WifiParams>::Init();

    LOG_INFO("WifiLittleFSFrontend initialized");

    if (m_backendsMutex == nullptr) {
        m_backendsMutex = xSemaphoreCreateMutex();
        if (m_backendsMutex == nullptr) {
            LOG_ERROR("Failed to create WiFi backend mutex");
        }
    }

    UpdateMode(m_Params.mode);

    // Apply logging settings from stored params
    ApplyLoggingSettings();

    // Set the delegate after initialization
    wifi_connection_task.taskFunction = etl::delegate<void()>::create<WifiLittleFSFrontend, Front::wifiLittleFSFront, &WifiLittleFSFrontend::StationConnectionThread>();

    // Schedule the connection task
    Scheduler::scheduler.CreateStaticTask(wifi_connection_task);
}

void WifiLittleFSFrontend::Update() {
    bool applyModeUpdate = false;
    WifiMode pendingMode = WifiMode::UNDEFINED;
    bool applyNetworkServicesSetup = false;
    {
        BackendsLockGuard lock(m_backendsMutex);
        if (!lock.IsLocked()) {
            LOG_WARN("WiFi backend lock failed in Update()");
            return;
        }

        m_updatingBackends = true;
        for (WifiBackend* backend : m_Backends) {
            backend->Update();
        }
        m_updatingBackends = false;

        applyModeUpdate = m_pendingModeUpdate;
        pendingMode = m_pendingMode;
        applyNetworkServicesSetup = m_pendingNetworkServicesSetup;
        m_pendingModeUpdate = false;
        m_pendingMode = WifiMode::UNDEFINED;
        m_pendingNetworkServicesSetup = false;
    }

    if (applyModeUpdate) {
        UpdateMode(pendingMode);
    } else if (applyNetworkServicesSetup && NetworkServicesActive()) {
        SetupNetworkServices();
    }
}

bool WifiLittleFSFrontend::SetupAP() {
    LOG_INFO("WiFi AP mode - SSID: %s", m_Params.ssidAP.data());

    WiFi.mode(WIFI_AP);
    bool success = WiFi.softAP(m_Params.ssidAP.data(), m_Params.pswdAP.data());

    if (success) {
        LOG_INFO("AP IP: %s", WiFi.softAPIP().toString().c_str());
        SetupNetworkServices();
        return true;
    } else {
        LOG_WARN("AP setup failed");
        return false;
    }
}

void WifiLittleFSFrontend::SetupStation() {
    LOG_INFO("WiFi Station mode - SSID: %s", m_Params.ssidST.data());

    // Station backends are configured only after a confirmed connection event.
    m_stationConnected = false;
    WiFi.mode(WIFI_STA);
    WiFi.begin(m_Params.ssidST.data(), m_Params.pswdST.data());
}

void WifiLittleFSFrontend::SetupNetworkServices() {
    BackendsLockGuard lock(m_backendsMutex);
    if (!lock.IsLocked()) {
        LOG_WARN("WiFi backend lock failed in SetupNetworkServices()");
        return;
    }

    ClearBackendsUnlocked();

    if (m_Params.enableWebServer) {
#ifdef USE_OTA_WEB
        LOG_INFO("HTTP OTA server enabled");
        WifiOtaServer* otaServer = new WifiOtaServer(80);
        m_Backends.push_back(otaServer);
#else
        LOG_WARN("HTTP OTA server requested but USE_OTA_WEB not compiled");
#endif
    }

#ifdef USE_WIFI_MAVLINK_MANAGEMENT
    WifiMavlinkManagement* management = new WifiMavlinkManagement(m_Params);
    management->SetTelemetryCallback(TelemetryCallback::create<&GetDeviceTelemetry>());
    m_Backends.push_back(management);
    LOG_INFO("MAVLink management enabled on UDP port %u",
             static_cast<unsigned int>(WifiMavlinkManagement::kManagementPort));
#else
    LOG_WARN("MAVLink management requested but USE_WIFI_MAVLINK_MANAGEMENT not compiled");
#endif

    if (m_Params.enableUartBridge) {
#ifdef USE_WIFI_UART_BRIDGE
        LOG_INFO("UART bridge setup on port %d", m_Params.udpPort);
        IPAddress ip;
        if(ip.fromString(m_Params.gcsIp.data())) {
            WifiUartBridge* uartBridge = new WifiUartBridge(Serial2, ip, m_Params.udpPort);
            m_Backends.push_back(uartBridge);
            LOG_INFO("UART bridge enabled");
        } else {
            LOG_WARN("Invalid GCS IP address");
        }
#else
        LOG_WARN("UART bridge requested but USE_WIFI_UART_BRIDGE not compiled");
#endif
    }

}

void WifiLittleFSFrontend::UpdateMode(WifiMode mode) {
    // Rebuild runtime WiFi services on every explicit mode update.
    ClearBackends();
    m_stationConnected = false;

    m_currentMode = mode;
    
    switch (mode) {
        case WifiMode::AP:
            SetupAP();
            break;
        case WifiMode::STATION:
            SetupStation();
            break;
        default:
            LOG_ERROR("Undefined WiFi mode");
            break;
    }
}

void WifiLittleFSFrontend::StationConnectionThread() {
    if (m_currentMode == WifiMode::STATION) {
        if (WiFi.status() == WL_CONNECTED && !m_stationConnected) {
            LOG_INFO("WiFi connected - IP: %s", WiFi.localIP().toString().c_str());
            SetupNetworkServices();
            m_stationConnected = true;

            // Re-apply logging settings now that WiFi is connected
            // (UDP backend requires WiFi to be ready)
            ApplyLoggingSettings();
        } else if (WiFi.status() != WL_CONNECTED && m_stationConnected) {
            LOG_WARN("WiFi disconnected");
            m_stationConnected = false;
        }
    }
}

void WifiLittleFSFrontend::ClearBackends() {
    BackendsLockGuard lock(m_backendsMutex);
    if (!lock.IsLocked()) {
        LOG_WARN("WiFi backend lock failed in ClearBackends()");
        return;
    }

    ClearBackendsUnlocked();
}

void WifiLittleFSFrontend::ClearBackendsUnlocked() {
    for (auto* backend : m_Backends) {
        delete backend;
    }
    m_Backends.clear();
}

void WifiLittleFSFrontend::ApplyLoggingSettings() {
#ifdef USE_LOGGING
    // Ensure Logger is initialized before configuring
    rtls::log::Logger::init();

    // Apply serial and UDP enabled settings
    rtls::log::Logger::setSerialEnabled(m_Params.logSerialEnabled != 0);
    rtls::log::Logger::setUdpEnabled(m_Params.logUdpEnabled != 0);

    // Set UDP target IP (use GCS IP) and port for log streaming
    if (m_Params.gcsIp[0] != '\0') {
        rtls::log::Logger::setUdpTarget(m_Params.gcsIp.data(), m_Params.logUdpPort);
    }
#endif
}

bool WifiLittleFSFrontend::NetworkServicesActive() const {
    return m_currentMode == WifiMode::AP ||
           (m_currentMode == WifiMode::STATION && m_stationConnected);
}

void WifiLittleFSFrontend::RequestModeUpdate(WifiMode mode) {
    if (m_updatingBackends) {
        m_pendingModeUpdate = true;
        m_pendingMode = mode;
        return;
    }

    UpdateMode(mode);
}

void WifiLittleFSFrontend::RequestNetworkServicesSetup() {
    if (!NetworkServicesActive()) {
        return;
    }

    if (m_updatingBackends) {
        m_pendingNetworkServicesSetup = true;
        return;
    }

    SetupNetworkServices();
}

ErrorParam WifiLittleFSFrontend::SetParam(const char* name, const void* data, uint32_t len) {
    // Call base class implementation first
    ErrorParam result = LittleFSFrontend<WifiParams>::SetParam(name, data, len);

    if (result != ErrorParam::OK) {
        return result;
    }

    // Apply mode changes immediately when requested.
    if (strcmp(name, "mode") == 0) {
        RequestModeUpdate(m_Params.mode);
    }

    // Reconfigure runtime WiFi services when bridge/server parameters are updated
    // and networking is currently active.
    // gcsIp is intentionally excluded: MAVLink management reads it live every
    // heartbeat, and tearing down AsyncWebServer + rebinding port 80 occasionally
    // fails to re-bind, leaving HTTP OTA dead until reboot. UART bridge keeps its old gcsIp
    // until explicit toggle or reboot, which is the safer trade-off.
    if (strcmp(name, "enableWebServer") == 0 ||
        strcmp(name, "enableUartBridge") == 0 ||
        strcmp(name, "udpPort") == 0) {
        RequestNetworkServicesSetup();
    }

    // Check if this was a logging-related parameter and apply changes.
    if (strcmp(name, "logSerialEnabled") == 0 ||
        strcmp(name, "logUdpEnabled") == 0 ||
        strcmp(name, "logUdpPort") == 0 ||
        strcmp(name, "gcsIp") == 0) {
        ApplyLoggingSettings();
    }

    return result;
}

namespace Front {
    WifiLittleFSFrontend wifiLittleFSFront;
}
