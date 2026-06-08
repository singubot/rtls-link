/**
 * @file logging.hpp
 * @brief Zero-overhead logging macros and Logger class declaration
 *
 * This file provides compile-time configurable logging with:
 * - Automatic tag extraction from __FILE__ (zero runtime overhead)
 * - Per-module log level overrides via RTLS_LOG_LOCAL_LEVEL
 * - Printf-style formatting
 * - Dual output: Serial and UDP (when enabled)
 *
 * Usage:
 *   // In your source file (optional: set per-file log level)
 *   #define RTLS_LOG_LOCAL_LEVEL 5  // VERBOSE for this module
 *   #include "logging/logging.hpp"
 *
 *   void myFunction() {
 *       LOG_INFO("Position: x=%.2f, y=%.2f", x, y);
 *       LOG_ERROR("Connection failed: %s", errorStr);
 *   }
 */

#pragma once

#include "config/features.hpp"
#include "log_levels.hpp"

#include <cstdarg>
#include <cstdint>
#include <cstdio>

// =============================================================================
// Compile-Time Tag Extraction
// =============================================================================

namespace rtls {
namespace log {
namespace detail {

/**
 * @brief Extract filename from path at compile time
 *
 * Strips directory path, returning only the filename.
 * This is constexpr so it has zero runtime overhead.
 *
 * @param path Full path string (__FILE__)
 * @return Pointer to filename portion
 */
constexpr const char* extractFilename(const char* path) {
    const char* file = path;
    while (*path) {
        if (*path == '/' || *path == '\\') {
            file = path + 1;
        }
        path++;
    }
    return file;
}

/**
 * @brief Get length of filename without extension
 * @param filename Filename to measure
 * @return Length up to (but not including) the last '.'
 */
constexpr size_t filenameBaseLength(const char* filename) {
    size_t len = 0;
    size_t lastDot = 0;
    const char* p = filename;
    while (*p) {
        if (*p == '.') {
            lastDot = len;
        }
        len++;
        p++;
    }
    return lastDot > 0 ? lastDot : len;
}

} // namespace detail
} // namespace log
} // namespace rtls

// =============================================================================
// Log Level Configuration
// =============================================================================

// Default global log level if not specified.
// Use RTLS-owned macro names: ESP-IDF/Arduino define LOG_LOCAL_LEVEL for
// their own logger, and sharing that namespace can compile RTLS logs out.
#ifndef RTLS_LOG_GLOBAL_LEVEL
    #ifdef USE_LOGGING
        #define RTLS_LOG_GLOBAL_LEVEL 3  // INFO by default
    #else
        #define RTLS_LOG_GLOBAL_LEVEL 0  // Disabled if USE_LOGGING not defined
    #endif
#endif

// Per-file override - defaults to global level
#ifndef RTLS_LOG_LOCAL_LEVEL
    #define RTLS_LOG_LOCAL_LEVEL RTLS_LOG_GLOBAL_LEVEL
#endif

// =============================================================================
// Logger Class Declaration
// =============================================================================

#ifdef USE_LOGGING

namespace rtls {
namespace log {

/**
 * @brief Thread-safe logger with Serial and UDP output support
 *
 * The Logger class provides:
 * - Printf-style log formatting
 * - Millisecond timestamps
 * - Thread-safe output using FreeRTOS mutex
 * - Runtime enable/disable for Serial and UDP outputs
 */
class Logger {
public:
    /**
     * @brief Initialize the logger
     *
     * Must be called before first log call. Sets up mutex and output backends.
     * Safe to call multiple times.
     */
    static void init();

    /**
     * @brief Main log function called by macros
     *
     * @param level Log level for this message
     * @param tag Module/file tag
     * @param format Printf-style format string
     * @param ... Format arguments
     */
    static void log(LogLevel level, const char* tag, const char* format, ...);

    /**
     * @brief Log with va_list (for wrapper functions)
     */
    static void logv(LogLevel level, const char* tag, const char* format, va_list args);

    // Runtime output control
    static void setSerialEnabled(bool enabled);
    static void setUdpEnabled(bool enabled);
    static bool isSerialEnabled();
    static bool isUdpEnabled();

    // UDP configuration
    static void setUdpTarget(const char* ip, uint16_t port);

    // Get current compiled log level (for heartbeat reporting)
    static constexpr uint8_t getCompiledLogLevel() { return RTLS_LOG_GLOBAL_LEVEL; }

private:
    static bool s_initialized;
    static bool s_serialEnabled;
    static bool s_udpEnabled;

    // Internal formatting and output
    static void formatAndOutput(LogLevel level, const char* tag,
                                 const char* format, va_list args);
};

} // namespace log
} // namespace rtls

#endif // USE_LOGGING

// =============================================================================
// Logging Macros
// =============================================================================

// Automatic tag from filename
#define LOG_TAG rtls::log::detail::extractFilename(__FILE__)

#ifdef USE_LOGGING

// Core implementation macro - compiles to nothing when level disabled
#define LOG_IMPL(level, tag, fmt, ...)                                         \
    do {                                                                        \
        if constexpr (static_cast<uint8_t>(level) <= RTLS_LOG_LOCAL_LEVEL) {   \
            rtls::log::Logger::log(level, tag, fmt, ##__VA_ARGS__);            \
        }                                                                       \
    } while (0)

// Main logging macros with automatic tag
#define LOG_ERROR(fmt, ...)   LOG_IMPL(rtls::log::LogLevel::ERROR, LOG_TAG, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)    LOG_IMPL(rtls::log::LogLevel::WARN, LOG_TAG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)    LOG_IMPL(rtls::log::LogLevel::INFO, LOG_TAG, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)   LOG_IMPL(rtls::log::LogLevel::DBG, LOG_TAG, fmt, ##__VA_ARGS__)
#define LOG_VERBOSE(fmt, ...) LOG_IMPL(rtls::log::LogLevel::VERBOSE, LOG_TAG, fmt, ##__VA_ARGS__)

// Variants with explicit tag (for special cases)
#define LOG_ERROR_TAG(tag, fmt, ...)   LOG_IMPL(rtls::log::LogLevel::ERROR, tag, fmt, ##__VA_ARGS__)
#define LOG_WARN_TAG(tag, fmt, ...)    LOG_IMPL(rtls::log::LogLevel::WARN, tag, fmt, ##__VA_ARGS__)
#define LOG_INFO_TAG(tag, fmt, ...)    LOG_IMPL(rtls::log::LogLevel::INFO, tag, fmt, ##__VA_ARGS__)
#define LOG_DEBUG_TAG(tag, fmt, ...)   LOG_IMPL(rtls::log::LogLevel::DBG, tag, fmt, ##__VA_ARGS__)
#define LOG_VERBOSE_TAG(tag, fmt, ...) LOG_IMPL(rtls::log::LogLevel::VERBOSE, tag, fmt, ##__VA_ARGS__)

#else // USE_LOGGING not defined

// No-op implementations when logging is disabled
#define LOG_IMPL(level, tag, fmt, ...) do {} while (0)
#define LOG_ERROR(fmt, ...)            do {} while (0)
#define LOG_WARN(fmt, ...)             do {} while (0)
#define LOG_INFO(fmt, ...)             do {} while (0)
#define LOG_DEBUG(fmt, ...)            do {} while (0)
#define LOG_VERBOSE(fmt, ...)          do {} while (0)
#define LOG_ERROR_TAG(tag, fmt, ...)   do {} while (0)
#define LOG_WARN_TAG(tag, fmt, ...)    do {} while (0)
#define LOG_INFO_TAG(tag, fmt, ...)    do {} while (0)
#define LOG_DEBUG_TAG(tag, fmt, ...)   do {} while (0)
#define LOG_VERBOSE_TAG(tag, fmt, ...) do {} while (0)

#endif // USE_LOGGING
