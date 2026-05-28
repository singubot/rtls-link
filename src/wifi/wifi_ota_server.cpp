#include "config/features.hpp"

#ifdef USE_OTA_WEB

#include "wifi_ota_server.hpp"

#include "logging/logging.hpp"
#include "ota/ota_handler.hpp"

WifiOtaServer::WifiOtaServer(uint16_t port)
    : m_Server(port)
{
    ota::initOtaRoutes(m_Server);
    m_Server.begin();
    LOG_INFO("HTTP OTA server started on port %u", static_cast<unsigned int>(port));
}

#endif // USE_OTA_WEB
