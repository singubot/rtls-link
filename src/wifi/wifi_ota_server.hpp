#pragma once

#include "config/features.hpp"

#ifdef USE_OTA_WEB

#include <ESPAsyncWebServer.h>

#include "wifi_backend.hpp"

class WifiOtaServer : public WifiBackend {
public:
    explicit WifiOtaServer(uint16_t port);

    void Update() override {}

private:
    AsyncWebServer m_Server;
};

#endif // USE_OTA_WEB
