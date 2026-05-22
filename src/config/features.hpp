/**
 * @file features.hpp
 * @brief Master feature flag definitions for modular firmware compilation
 *
 * This file defines all available compile-time feature flags and their defaults.
 * Features can be enabled/disabled via user_defines.txt or platformio.ini build_flags.
 *
 * Naming Convention (following ArduPilot/Betaflight patterns):
 *   USE_<SUBSYSTEM>           - Main subsystem toggle
 *   USE_<SUBSYSTEM>_<FEATURE> - Sub-feature within a subsystem
 *   USE_UWB_MODE_<MODE>       - UWB operational mode
 *   USE_RUNTIME_<FEATURE>     - Runtime behavior toggles
 *
 * @note This file MUST be included first in any source file that uses feature guards.
 */

#pragma once

// =============================================================================
// DEFAULT FEATURE DETECTION
// =============================================================================
// When no features are explicitly defined, enable all features for backward
// compatibility. This ensures existing builds work without modification.

#if !defined(USE_WIFI) && \
    !defined(USE_MAVLINK) && \
    !defined(USE_RTLSLINK_BEACON_BACKEND) && \
    !defined(USE_CONSOLE) && \
    !defined(USE_UWB_MODE_TDOA_ANCHOR) && \
    !defined(USE_UWB_MODE_TDOA_TAG) && \
    !defined(USE_OTA)
    #define RTLS_USE_DEFAULT_FEATURES
#endif

// =============================================================================
// DEFAULT FEATURE SET (all features enabled)
// =============================================================================

#ifdef RTLS_USE_DEFAULT_FEATURES

// --- WiFi Subsystem ---
#define USE_WIFI
#define USE_WIFI_WEBSERVER
#define USE_WIFI_TCP_LOGGING
#define USE_WIFI_UART_BRIDGE
#define USE_WIFI_DISCOVERY
#define USE_WIFI_MDNS

// --- MAVLink Output ---
#define USE_MAVLINK
#define USE_MAVLINK_POSITION
#define USE_MAVLINK_HEARTBEAT
#define USE_MAVLINK_ORIGIN
#define USE_MAVLINK_COVARIANCE
#define USE_RTLSLINK_BEACON_BACKEND

// Note: Rangefinder is board-specific, enabled below based on board
// #define USE_MAVLINK_RANGEFINDER

// --- Console/CLI ---
#define USE_CONSOLE
#define USE_CONSOLE_PARAM_RW
#define USE_CONSOLE_CONFIG_MGMT
#define USE_CONSOLE_LED_CONTROL

// --- UWB Modes ---
#define USE_UWB_MODE_TDOA_ANCHOR
#define USE_UWB_MODE_TDOA_TAG

// --- Dynamic anchor position calculation (TDoA tags) ---
// Enables automatic calculation of anchor positions from inter-anchor distances
// #define USE_DYNAMIC_ANCHOR_POSITIONS

// --- OTA Update Subsystem ---
#define USE_OTA                       // Master OTA toggle
#define USE_OTA_WEB                   // HTTP OTA upload via webserver

// --- Miscellaneous ---
#define USE_STATUS_LED_TASK
#define USE_RATE_STATISTICS
#define USE_FAST_CODE                 // IRAM placement for hot-path functions
#define USE_RUNTIME_SUBSYSTEM_TOGGLES // Runtime UWB on/off and service-level toggles

// --- Logging Subsystem ---
#define USE_LOGGING
#define USE_LOGGING_SERIAL
#define USE_LOGGING_UDP

#endif // RTLS_USE_DEFAULT_FEATURES

// =============================================================================
// LOGGING CONFIGURATION DEFAULTS
// =============================================================================
// These are applied whether or not RTLS_USE_DEFAULT_FEATURES is set

// Default global log level (INFO = 3)
// Can be overridden in platformio.ini or user_defines.txt
#ifndef LOG_GLOBAL_LEVEL
    #define LOG_GLOBAL_LEVEL 3
#endif

// =============================================================================
// BOARD-SPECIFIC DEFAULT OVERRIDES
// =============================================================================

// Rangefinder support only on ESP32S3 board
#if defined(ESP32S3_UWB_BOARD)
    // Enable rangefinder by default on ESP32S3 if using default features
    #if defined(RTLS_USE_DEFAULT_FEATURES) && defined(USE_MAVLINK)
        #ifndef USE_MAVLINK_RANGEFINDER
            #define USE_MAVLINK_RANGEFINDER
        #endif
    #endif

    // Board has LED support
    #define BOARD_HAS_LED 1
    #define BOARD_HAS_LED2 1
#endif

#if defined(MAKERFABS_ESP32_BOARD)
    // Makerfabs board has no LED (led_pin = -1 in config)
    #undef BOARD_HAS_LED
    #undef BOARD_HAS_LED2
#endif

// =============================================================================
// DERIVED FEATURE FLAGS
// =============================================================================
// These are automatically set based on other flags for convenience

// Any tag mode enabled?
#if defined(USE_UWB_MODE_TDOA_TAG)
    #define USE_UWB_TAG_MODES 1
#endif

// Any anchor mode enabled?
#if defined(USE_UWB_MODE_TDOA_ANCHOR)
    #define USE_UWB_ANCHOR_MODES 1
#endif

// Any TDoA mode enabled?
#if defined(USE_UWB_MODE_TDOA_ANCHOR) || defined(USE_UWB_MODE_TDOA_TAG)
    #define USE_UWB_TDOA_MODES 1
#endif

// Rangefinder hardware available?
#if defined(USE_MAVLINK_RANGEFINDER) && defined(ESP32S3_UWB_BOARD)
    #define HAS_RANGEFINDER 1
#endif

// Position output available (needed by tags)?
#if defined(USE_MAVLINK) || defined(USE_RTLSLINK_BEACON_BACKEND)
    #define HAS_POSITION_OUTPUT 1
#endif

// =============================================================================
// INCLUDE VALIDATION (after all flags are set)
// =============================================================================

#include "feature_validation.hpp"

// IRAM placement macro (must be after feature flags are resolved)
#include "fast_code.hpp"
