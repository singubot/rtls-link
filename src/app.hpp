#pragma once

/**
 * @file app.hpp
 * @brief Main application class for RTLS-Link
 */

#include "config/features.hpp"  // MUST be first project include

#include "front.hpp"
#include "scheduler.hpp"
#include "uwb/uwb_params.hpp"

#ifdef USE_MAVLINK
#include "mavlink/local_position_sensor.hpp"
#include "mavlink/uart_comm.hpp"
#endif

#ifdef USE_RTLSLINK_BEACON_BACKEND
#include "beacon/rtlslink_beacon_backend.hpp"
#endif

#include <etl/delegate.h>
#include <etl/span.h>
#include <cmath> // For sin and cos functions
#include <optional>
#include <array>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#ifdef HAS_RANGEFINDER
// Forward declaration for rangefinder sensor
class RangefinderSensor;
#endif

// Define a Vector3f struct for 3D coordinates
struct Vector3f {
    float x;
    float y;
    float z;
};

class App {
public:

    App();
    void Init();

    void Update();

#if defined(USE_STATUS_LED_TASK) && defined(BOARD_HAS_LED)
    // Task that blinks LED based on current status. Called at 5Hz rate
    void StatusLedTask();
#endif

#ifdef HAS_POSITION_OUTPUT
    static void SendSample(float x_m, float y_m, float z_m,
                           std::optional<std::array<float, 6>> positionCovariance = std::nullopt);
#endif

#ifdef USE_RTLSLINK_BEACON_BACKEND
    static void ConfigureRtlslinkBeaconAnchors(etl::span<const UWBAnchorParam> anchors);
    static void SendTdoaMeasurement(uint8_t anchor_a,
                                    uint8_t anchor_b,
                                    float distance_diff_m,
                                    float sigma_m,
                                    uint64_t solved_timestamp_us);
#endif

    static void Start();
    static void Stop();

#if defined(USE_MAVLINK) || defined(USE_RTLSLINK_BEACON_BACKEND)
    static HardwareSerial& GetArdupilotSerial();
#endif
    static App& GetInstance();

    // Helper function to correct for yaw orientation
    static Vector3f correct_for_orient_yaw(float x, float y, float z);

#if defined(USE_MAVLINK) && defined(USE_MAVLINK_POSITION)
    // Telemetry getter for position broadcast
    static bool IsSendingPositions();  // True if sent to ArduPilot in last 2s
#endif

#if defined(USE_MAVLINK) && defined(USE_MAVLINK_ORIGIN)
    // Telemetry getter for origin broadcast
    static bool IsOriginSent();        // True if GPS origin sent to ArduPilot
#endif

#ifdef HAS_RANGEFINDER
    static bool IsRangefinderEnabled(); // True if zCalcMode == RANGEFINDER
    static bool IsRangefinderHealthy(); // True if receiving non-stale rangefinder data
    // Get Z coordinate from rangefinder (returns NAN if not using rangefinder mode)
    static float GetRangefinderZ();
#endif

#ifdef USE_RATE_STATISTICS
    // Update rate statistics (centi-Hz for 0.01 Hz precision)
    static uint16_t GetAvgUpdateRateCHz();  // Average rate over recent samples
    static uint16_t GetMinRateCHz();        // Min rate in 5s window
    static uint16_t GetMaxRateCHz();        // Max rate in 5s window
#endif

private:
#ifdef USE_MAVLINK
    UartComm uart_comm_;
    LocalPositionSensor local_position_sensor_;
#ifdef USE_MAVLINK_HEARTBEAT
    uint64_t last_heartbeat_timestamp_ms_ = 0;
    uint64_t last_heartbeat_received_timestamp_ms_ = 0;
    uint8_t last_heartbeat_system_id_ = 0;
#endif // USE_MAVLINK_HEARTBEAT
#ifdef USE_MAVLINK_ORIGIN
    bool is_origin_position_sent_ = false;
    bool is_origin_set_ = false;
    uint64_t last_origin_attempt_timestamp_ms_ = 0;
    uint64_t last_origin_sent_timestamp_ms_ = 0;
    bool origin_target_fallback_logged_ = false;
    bool origin_target_missing_logged_ = false;
#endif // USE_MAVLINK_ORIGIN
#endif // USE_MAVLINK

#ifdef USE_RTLSLINK_BEACON_BACKEND
    RTLSLinkBeaconBackend rtlslink_beacon_backend_;
#endif

    uint64_t last_sample_timestamp_ms_ = 0;
    uint64_t device_unhealthy_timestamp_ms_ = 0;

#ifdef HAS_RANGEFINDER
    // Rangefinder sensor (ESP32S3 only)
    RangefinderSensor* rangefinder_sensor_ = nullptr;
    uint16_t last_rangefinder_distance_cm_ = 0;
    uint64_t last_rangefinder_timestamp_ms_ = 0;
    bool rangefinder_ever_received_ = false;
    uint64_t last_rangefinder_log_ms_ = 0;
    uint32_t rf_forward_fail_count_ = 0;  // Consecutive forwarding failures

    // Cached full DISTANCE_SENSOR message and source IDs, used by the
    // UWB-dropout fallback path so it can re-emit the last reading even
    // when SendSample stops being called.
    mavlink_distance_sensor_t last_distance_sensor_msg_ = {};
    uint8_t last_distance_sensor_sysid_ = 0;
    uint8_t last_distance_sensor_compid_ = 0;
    // Dropout fallback state — true once we've started auto-forwarding
    // because UWB has gone stale, false otherwise. Used to suppress
    // repeated state-transition logs.
    bool rangefinder_dropout_forward_active_ = false;
#endif

#ifdef USE_RATE_STATISTICS
    // Update rate tracking (microsecond-precision timestamps via esp_timer)
    static constexpr size_t kRateWindowSize = 50;  // Store last N timestamps for rate calculation
    static constexpr uint64_t kRateWindowDurationUs = 5000000;  // 5 second window (microseconds)
    static constexpr uint16_t kMaxRateCHz = 65535;  // Max value for uint16_t centi-Hz
    uint64_t sample_timestamps_[kRateWindowSize] = {0};
    size_t sample_timestamps_index_ = 0;
    size_t sample_timestamps_count_ = 0;
    uint16_t cached_avg_rate_cHz_ = 0;
    uint16_t cached_min_rate_cHz_ = 0;
    uint16_t cached_max_rate_cHz_ = 0;
    uint64_t last_rate_calc_ms_ = 0;
    SemaphoreHandle_t rate_stats_mutex_ = nullptr;  // Mutex for thread-safe rate statistics access

    void RecordSampleTimestamp();
    void CalculateRateStatistics();
#endif

private:
    // App::Update runs at 10 Hz, so a 100 ms freshness window races normal
    // scheduler jitter and can keep the 16 s origin timer from ever maturing.
    static constexpr uint64_t kDeviceHealtyMinDurationMs = 500;
    static constexpr uint32_t kArdupilotSerialBaudrate = 921600;
    static constexpr size_t kArdupilotSerialTxBufferSize = 1024;

#ifdef USE_MAVLINK
    static constexpr uint8_t kSystemId = 199;
    static constexpr uint8_t kComponentId = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
#ifdef USE_MAVLINK_HEARTBEAT
    static constexpr uint64_t kHeartbeatIntervalMs = 1000;
    static constexpr uint64_t kHeartbeatRcvTimeoutMs = 3000;
#endif // USE_MAVLINK_HEARTBEAT
#ifdef USE_MAVLINK_ORIGIN
    static constexpr uint64_t kSendOriginPositionAfterMs = 16000;    // After 16 seconds healthy device, send global origin position
    static constexpr uint64_t kOriginRetryIntervalMs = 2000;          // Retry origin TX every 2 seconds until first successful send
    static constexpr uint64_t kOriginResendIntervalMs = 10000;        // Re-send origin every 10 seconds to recover AP reboots/missed packets
#endif // USE_MAVLINK_ORIGIN
#endif // USE_MAVLINK

#ifdef HAS_RANGEFINDER
    static constexpr uint64_t kRangefinderStaleTimeoutMs = 500;  // Rangefinder data older than this is considered stale
    static constexpr uint64_t kRangefinderLogIntervalMs = 1000;  // Log rangefinder data once per second
    static constexpr uint32_t kRfForwardFailLogThreshold = 10;   // Log after this many consecutive failures
    static constexpr uint64_t kUwbDropoutForwardMs = 500;        // UWB silent for this long → start forwarding rangefinder ourselves
#endif

    static bool IsMavlinkOutputSelected();
    static bool IsRtlslinkBeaconOutputSelected();
};

extern App app;
