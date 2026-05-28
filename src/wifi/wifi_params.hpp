#pragma once

#include <Arduino.h>

#include <etl/array.h>
#include <etl/string.h>

#include "utils/utils.hpp"

static constexpr uint32_t MAX_SSID_LENGTH = 32;
static constexpr uint32_t MAX_PSWD_LENGTH = 64;
static constexpr uint32_t MAX_IP_LENGTH = 16;
static_assert(MAX_IP_LENGTH >= 16, "IPv4 buffers must hold 15 characters plus NUL");

enum class WifiMode : uint8_t {
    AP,
    STATION,
    UNDEFINED
};

/**
 * The parameter struct can't have indirect memory structures. We will be coping pointers to memory in that case. 
 * Instead of String, etc use const char[].
*/
struct WifiParams {
    WifiMode mode;                  // True if access point mode, false if station mode
    etl::array<char,MAX_SSID_LENGTH> ssidAP;   // Used when in AP mode
    etl::array<char,MAX_PSWD_LENGTH> pswdAP;   // Used when in AP mode
    etl::array<char,MAX_SSID_LENGTH> ssidST;   // Used when in Station mode
    etl::array<char,MAX_PSWD_LENGTH> pswdST;   // Used when in Station mode
    etl::array<char,MAX_IP_LENGTH>   gcsIp;         // IP address of the device
    uint16_t udpPort;               // Port of the device UDP server (Used for the UartBridge)
    uint8_t enableWebServer = 1;    // Enable temporary HTTP OTA server
    uint8_t enableUartBridge = 1;   // Enable the UART bridge (Bridge between serial port and UDP port)
    // Logging parameters
    uint16_t logUdpPort = 3334;     // UDP port for debug log streaming
    uint8_t logSerialEnabled = 1;   // Runtime: enable Serial log output (default: on)
    uint8_t logUdpEnabled = 0;      // Runtime: enable UDP log streaming (default: off)
}ULS_PACKED;
