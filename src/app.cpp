#include "config/features.hpp"  // MUST be first project include

#include "app.hpp"

#include <Arduino.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <etl/delegate.h>
#include <etl/queue.h>

#include <utils/utils.hpp>
#include "logging/logging.hpp"

#include "bsp/board.hpp"
#include "uwb/uwb_frontend_littlefs.hpp"
#include "uwb/uwb_params.hpp"

#ifdef USE_MAVLINK
#include "mavlink/local_position_sensor.hpp"
#include "mavlink/uart_comm.hpp"
#endif

#ifdef HAS_RANGEFINDER
#include "mavlink/rangefinder_sensor.hpp"
#endif

#if defined(USE_MAVLINK) && defined(USE_RTLSLINK_BEACON_BACKEND)
App::App()
  : uart_comm_(App::GetArdupilotSerial())
  , local_position_sensor_(uart_comm_, kSystemId, kComponentId)
  , rtlslink_beacon_backend_(App::GetArdupilotSerial())
{}
#elif defined(USE_MAVLINK)
App::App()
  : uart_comm_(App::GetArdupilotSerial())
  , local_position_sensor_(uart_comm_, kSystemId, kComponentId)
{}
#elif defined(USE_RTLSLINK_BEACON_BACKEND)
App::App()
  : rtlslink_beacon_backend_(App::GetArdupilotSerial())
{}
#else
App::App()
{}
#endif

// For the future maybe we should wait for reception of heartbeats before starting sending samples
// to avoid transmition before the copter is ready
void App::Init()
{
  LOG_INFO("------ Initializing the application ------");

#if defined(USE_MAVLINK) || defined(USE_RTLSLINK_BEACON_BACKEND)
  Serial1.setTxBufferSize(kArdupilotSerialTxBufferSize);
  Serial1.begin(kArdupilotSerialBaudrate, SERIAL_8N1, bsp::kBoardConfig.uwb_data_uart.rx_pin, bsp::kBoardConfig.uwb_data_uart.tx_pin);
#endif

#ifdef USE_RTLSLINK_BEACON_BACKEND
  rtlslink_beacon_backend_.Init(kArdupilotSerialBaudrate, kArdupilotSerialTxBufferSize);
#endif

#ifdef USE_MAVLINK
#ifdef USE_MAVLINK_HEARTBEAT
  local_position_sensor_.set_heartbeat_callback([this](uint8_t system_id, uint8_t component_id) {
    (void)component_id;
    last_heartbeat_received_timestamp_ms_ = millis();
    last_heartbeat_system_id_ = system_id;
  });
#endif // USE_MAVLINK_HEARTBEAT
#endif // USE_MAVLINK

  // Initialize timestamp
  device_unhealthy_timestamp_ms_ = millis();

#ifdef USE_RATE_STATISTICS
  // Initialize mutex for rate statistics thread safety
  rate_stats_mutex_ = xSemaphoreCreateMutex();
#endif

#if defined(USE_STATUS_LED_TASK) && defined(BOARD_HAS_LED)
  LOG_INFO("Status task enabled");
  pinMode(bsp::kBoardConfig.led_pin, OUTPUT);
  digitalWrite(bsp::kBoardConfig.led_pin, LOW);
#endif

#ifdef HAS_RANGEFINDER
  // Initialize rangefinder sensor (ESP32S3 only)
  if (bsp::kBoardConfig.rangefinder_uart.rx_pin < 64) {
    // Use Serial0 for UART0 (Serial is USB CDC on ESP32S3)
    rangefinder_sensor_ = new RangefinderSensor(Serial0);
    rangefinder_sensor_->init(
        115200,
        bsp::kBoardConfig.rangefinder_uart.rx_pin,
        bsp::kBoardConfig.rangefinder_uart.tx_pin
    );

    rangefinder_sensor_->set_distance_callback(
        [this](mavlink_distance_sensor_t distance_msg, uint64_t timestamp_ms,
               uint8_t src_sysid, uint8_t src_compid) {
            last_rangefinder_distance_cm_ = distance_msg.current_distance;
            last_rangefinder_timestamp_ms_ = timestamp_ms;
            rangefinder_ever_received_ = true;
            // Cache the full message and source IDs so the UWB-dropout
            // fallback path in App::Update() can re-emit them.
            last_distance_sensor_msg_ = distance_msg;
            last_distance_sensor_sysid_ = src_sysid;
            last_distance_sensor_compid_ = src_compid;

            // Forward to ArduPilot if enabled
            const auto& params = Front::uwbLittleFSFront.GetParams();
            if (params.rfForwardEnable && IsMavlinkOutputSelected()) {
                bool success = local_position_sensor_.send_distance_sensor(
                    distance_msg,
                    params.rfForwardSensorId,
                    params.rfForwardOrientation,
                    src_sysid,
                    src_compid,
                    params.rfForwardPreserveSrcIds != 0);

                // Track and log failures
                if (!success) {
                    rf_forward_fail_count_++;
                    if (rf_forward_fail_count_ == kRfForwardFailLogThreshold) {
                        LOG_WARN("Rangefinder forward failed %lu consecutive times",
                               static_cast<unsigned long>(rf_forward_fail_count_));
                    }
                } else {
                    rf_forward_fail_count_ = 0;
                }
            }

            // Time-limited logging (once per second max)
            if (timestamp_ms - last_rangefinder_log_ms_ >= kRangefinderLogIntervalMs) {
                LOG_DEBUG("Rangefinder: %u cm (%.2f m) [fwd=%d]",
                       distance_msg.current_distance,
                       static_cast<float>(distance_msg.current_distance) / 100.0f,
                       params.rfForwardEnable ? 1 : 0);
                last_rangefinder_log_ms_ = timestamp_ms;
            }
        }
    );

    LOG_INFO("Rangefinder sensor initialized");
  }
#endif // HAS_RANGEFINDER

  LOG_INFO("------ Application initialized ------");
}


void App::Update()
{
#ifdef USE_RTLSLINK_BEACON_BACKEND
  if (IsRtlslinkBeaconOutputSelected()) {
    rtlslink_beacon_backend_.Update();
  }
#endif

#ifdef USE_MAVLINK
  const bool mavlink_output_selected = IsMavlinkOutputSelected();
  uint64_t now_ms = millis();

#ifdef HAS_RANGEFINDER
  // Keep rangefinder state fresh even when RTLSLink Beacon is the selected
  // output; SendSample may still use it for zCalcMode=RANGEFINDER.
  if (rangefinder_sensor_) {
    rangefinder_sensor_->process_received_bytes();
  }
#endif

  if (!mavlink_output_selected) {
    return;
  }

  // App health stats (static to persist across calls)
  static uint32_t app_stats_healthy_cycles = 0;
  static uint32_t app_stats_unhealthy_cycles = 0;
  static uint64_t app_stats_last_log_ms = 0;
  static constexpr uint64_t APP_STATS_LOG_INTERVAL_MS = 1000;

  uint8_t buffer[1024];
  uint32_t buffer_index = 0;

  // ********** SENDING **********

#ifdef USE_MAVLINK_HEARTBEAT
  // Listen and send heartbeat
  if (now_ms - last_heartbeat_timestamp_ms_ > kHeartbeatIntervalMs) {
    local_position_sensor_.send_heartbeat();
    last_heartbeat_timestamp_ms_ = now_ms;
  }
#endif // USE_MAVLINK_HEARTBEAT

  // Check if device is healthy
  if (now_ms - last_sample_timestamp_ms_ > kDeviceHealtyMinDurationMs) {
    // Device is unhealthy
    device_unhealthy_timestamp_ms_ = now_ms;
    app_stats_unhealthy_cycles++;
  } else {
    // Device is healthy
    app_stats_healthy_cycles++;
#ifdef USE_MAVLINK_ORIGIN
    uint64_t time_since_unhealthy = now_ms - device_unhealthy_timestamp_ms_;
#ifdef USE_MAVLINK_HEARTBEAT
    uint64_t time_since_rcv_heartbeat = now_ms - last_heartbeat_received_timestamp_ms_;
    bool heartbeat_recent = time_since_rcv_heartbeat < kHeartbeatRcvTimeoutMs;
#else
    bool heartbeat_recent = true;
#endif
    bool origin_retry_due = (now_ms - last_origin_attempt_timestamp_ms_) >= kOriginRetryIntervalMs;
    bool origin_resend_due = is_origin_position_sent_
                          && (now_ms - last_origin_sent_timestamp_ms_) >= kOriginResendIntervalMs;
    bool should_try_send_origin = (time_since_unhealthy > kSendOriginPositionAfterMs)
                               && heartbeat_recent
                               && origin_retry_due
                               && (!is_origin_position_sent_ || origin_resend_due);

    if (should_try_send_origin) {
      const auto& params = Front::uwbLittleFSFront.GetParams();
      uint8_t target_system_id = params.mavlinkTargetSystemId;

#ifdef USE_MAVLINK_HEARTBEAT
      if (target_system_id == 0 && last_heartbeat_system_id_ != 0) {
        target_system_id = last_heartbeat_system_id_;
        if (!origin_target_fallback_logged_) {
          LOG_WARN("mavlinkTargetSystemId is 0, falling back to heartbeat system id %u",
                   static_cast<unsigned int>(target_system_id));
          origin_target_fallback_logged_ = true;
        }
      }
#endif

      last_origin_attempt_timestamp_ms_ = now_ms;

      if (target_system_id == 0) {
        if (!origin_target_missing_logged_) {
          LOG_WARN("Skipping origin TX: target system id is 0");
          origin_target_missing_logged_ = true;
        }
      } else {
        origin_target_missing_logged_ = false;
        bool sent = local_position_sensor_.send_set_gps_global_origin(
            params.originLat,
            params.originLon,
            params.originAlt,
            target_system_id, micros());

        if (sent) {
          bool first_success = !is_origin_position_sent_;
          is_origin_position_sent_ = true;
          last_origin_sent_timestamp_ms_ = now_ms;
          if (first_success) {
            LOG_INFO("Origin sent to system %u", static_cast<unsigned int>(target_system_id));
          } else {
            LOG_DEBUG("Origin re-sent to system %u", static_cast<unsigned int>(target_system_id));
          }
        } else {
          LOG_WARN("Origin TX failed (system %u) - retrying",
                   static_cast<unsigned int>(target_system_id));
        }
      }
    }
#endif // USE_MAVLINK_ORIGIN
  }

  // Periodic App health log (every 1 second)
  if (now_ms - app_stats_last_log_ms >= APP_STATS_LOG_INTERVAL_MS) {
    uint64_t time_since_unhealthy = now_ms - device_unhealthy_timestamp_ms_;
#ifdef USE_MAVLINK_HEARTBEAT
    uint64_t time_since_heartbeat = now_ms - last_heartbeat_received_timestamp_ms_;
#else
    uint64_t time_since_heartbeat = 0;
#endif
    LOG_DEBUG("H:%u U:%u | OriginSent:%d UnhealthyAge:%llums HB_Age:%llums",
           app_stats_healthy_cycles, app_stats_unhealthy_cycles,
           is_origin_position_sent_ ? 1 : 0,
           time_since_unhealthy, time_since_heartbeat);
    app_stats_healthy_cycles = 0;
    app_stats_unhealthy_cycles = 0;
    app_stats_last_log_ms = now_ms;
  }

  // ********** RECEIVING **********

#ifdef HAS_RANGEFINDER
  // UWB-dropout rangefinder fallback: when UWB stops producing samples (so
  // SendSample's vision_position_estimate path no longer carries the height)
  // but the rangefinder is still healthy, auto-forward the cached
  // DISTANCE_SENSOR so ArduPilot's altitude EKF keeps getting corrections.
  // Only kicks in when explicit forwarding is OFF — otherwise the regular
  // callback path already handles it.
  {
    const auto& params = Front::uwbLittleFSFront.GetParams();
    bool explicit_forwarding = (params.rfForwardEnable != 0);
    bool uwb_silent = (last_sample_timestamp_ms_ == 0)
                    || ((now_ms - last_sample_timestamp_ms_) > kUwbDropoutForwardMs);

    if (!explicit_forwarding && uwb_silent && IsRangefinderHealthy()) {
      // Re-emit the last received DISTANCE_SENSOR using the same override/
      // preserve-source-id semantics the user configured.
      bool sent = local_position_sensor_.send_distance_sensor(
          last_distance_sensor_msg_,
          params.rfForwardSensorId,
          params.rfForwardOrientation,
          last_distance_sensor_sysid_,
          last_distance_sensor_compid_,
          params.rfForwardPreserveSrcIds != 0);
      if (sent && !rangefinder_dropout_forward_active_) {
        LOG_WARN("UWB dropout — forwarding rangefinder distance directly");
        rangefinder_dropout_forward_active_ = true;
      }
    } else if (rangefinder_dropout_forward_active_ && !uwb_silent) {
      LOG_INFO("UWB recovered — rangefinder dropout forward stopped");
      rangefinder_dropout_forward_active_ = false;
    }
  }
#endif

  // For now we are only receiving heartbeat messages
  // Read the buffer
  while (Serial1.available() && buffer_index < sizeof(buffer)) {
    uint8_t c = Serial1.read();
    buffer[buffer_index++] = c;
  }

  // Process the buffer
  local_position_sensor_.process_received_bytes(buffer, buffer_index);

#endif // USE_MAVLINK
}

#if defined(USE_STATUS_LED_TASK) && defined(BOARD_HAS_LED)
void App::StatusLedTask()
{
  static uint32_t i = 0;

  // It will blink to the number of connected anchors
  if (Front::uwbLittleFSFront.GetConnectedDevices() > i) {
    digitalWrite(bsp::kBoardConfig.led_pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));  // Use FreeRTOS delay instead of Arduino delay()
    digitalWrite(bsp::kBoardConfig.led_pin, LOW);
  }

  // Reset i every 10 cycles
  i++;
  i = i % 10;
}
#endif

// Helper function to correct for yaw orientation
// Note: This function works correctly with negative values of kRotationDegrees
// because the standard 2D rotation matrix is used:
// [cos(θ) -sin(θ)]
// [sin(θ)  cos(θ)]
// When θ is negative, the rotation is clockwise instead of counterclockwise,
// which is mathematically valid and works as expected.
Vector3f App::correct_for_orient_yaw(float x, float y, float z) {
  Vector3f result = {x, y, z};
  
  // Exit immediately if no correction needed (optimization)
  float rotationDegrees = Front::uwbLittleFSFront.GetParams().rotationDegrees;
  if (rotationDegrees == 0.0f) {
    return result;
  }

  // Calculate rotation constants
  float orient_yaw_rad = rotationDegrees * M_PI / 180.0f; // Convert degrees to radians
  float orient_cos_yaw = cosf(orient_yaw_rad);
  float orient_sin_yaw = sinf(orient_yaw_rad);

  // Rotate x,y by -orient_yaw (apply 2D rotation matrix)
  result.x = x * orient_cos_yaw - y * orient_sin_yaw;
  result.y = x * orient_sin_yaw + y * orient_cos_yaw;
  // z stays the same

  return result;
}

#ifdef HAS_RANGEFINDER
float App::GetRangefinderZ() {
  ZCalcMode mode = Front::uwbLittleFSFront.GetParams().zCalcMode;

  if (mode == ZCalcMode::RANGEFINDER) {
    // Check if we have ever received data and it's not stale
    if (app.rangefinder_ever_received_) {
      uint64_t age_ms = millis() - app.last_rangefinder_timestamp_ms_;
      if (age_ms <= kRangefinderStaleTimeoutMs) {
        // Convert cm to meters and negate for NED frame
        // Rangefinder measures positive distance to ground, but in NED
        // Z is positive downward, so altitude above ground is negative
        return -static_cast<float>(app.last_rangefinder_distance_cm_) / 100.0f;
      }
    }
    // Never received or stale - return NAN to indicate unavailable
    return NAN;
  }

  // Return NAN to indicate "use original Z from TDoA"
  return NAN;
}

bool App::IsRangefinderEnabled() {
  const auto& params = Front::uwbLittleFSFront.GetParams();
  return (params.zCalcMode == ZCalcMode::RANGEFINDER) || (params.rfForwardEnable != 0);
}

bool App::IsRangefinderHealthy() {
  if (!app.rangefinder_ever_received_) {
    return false;
  }
  return (millis() - app.last_rangefinder_timestamp_ms_) <= kRangefinderStaleTimeoutMs;
}
#endif // HAS_RANGEFINDER

#if defined(USE_MAVLINK) && defined(USE_MAVLINK_POSITION)
bool App::IsSendingPositions() {
  // Guard for initial boot (last_sample_timestamp_ms_ starts at 0)
  if (app.last_sample_timestamp_ms_ == 0) {
    return false;
  }
  // Use 2s window to match heartbeat interval (avoids flapping)
  return (millis() - app.last_sample_timestamp_ms_) < 2000;
}
#endif // USE_MAVLINK && USE_MAVLINK_POSITION

#if defined(USE_MAVLINK) && defined(USE_MAVLINK_ORIGIN)
bool App::IsOriginSent() {
  return app.is_origin_position_sent_;
}
#endif // USE_MAVLINK && USE_MAVLINK_ORIGIN

#ifdef USE_RATE_STATISTICS
// --- Update rate tracking (microsecond-precision via esp_timer) ---
void App::RecordSampleTimestamp() {
  if (rate_stats_mutex_ == nullptr) return;

  if (xSemaphoreTake(rate_stats_mutex_, pdMS_TO_TICKS(1)) == pdTRUE) {
    uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    sample_timestamps_[sample_timestamps_index_] = now_us;
    sample_timestamps_index_ = (sample_timestamps_index_ + 1) % kRateWindowSize;
    if (sample_timestamps_count_ < kRateWindowSize) {
      sample_timestamps_count_++;
    }
    xSemaphoreGive(rate_stats_mutex_);
  }
}

void App::CalculateRateStatistics() {
  if (rate_stats_mutex_ == nullptr) return;

  uint64_t now_ms = millis();

  // Only recalculate every 500ms to reduce overhead
  if (now_ms - last_rate_calc_ms_ < 500) {
    return;
  }

  // Acquire mutex for thread-safe access to sample_timestamps_
  if (xSemaphoreTake(rate_stats_mutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;  // Could not acquire mutex, skip this calculation (don't update throttle)
  }

  // Update throttle timestamp only after successful mutex acquisition
  last_rate_calc_ms_ = now_ms;

  // Need at least 2 samples to calculate rate
  if (sample_timestamps_count_ < 2) {
    cached_avg_rate_cHz_ = 0;
    cached_min_rate_cHz_ = 0;
    cached_max_rate_cHz_ = 0;
    xSemaphoreGive(rate_stats_mutex_);
    return;
  }

  uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());

  // Collect timestamps within the 5-second window (microseconds)
  uint64_t window_start = (now_us > kRateWindowDurationUs) ? (now_us - kRateWindowDurationUs) : 0;

  // Count samples in window and find time range
  uint32_t samples_in_window = 0;
  uint64_t oldest_in_window = now_us;
  uint64_t newest_in_window = 0;

  // Iterate through all stored timestamps
  for (size_t i = 0; i < sample_timestamps_count_; i++) {
    uint64_t ts = sample_timestamps_[i];
    if (ts >= window_start && ts <= now_us) {
      samples_in_window++;
      if (ts < oldest_in_window) oldest_in_window = ts;
      if (ts > newest_in_window) newest_in_window = ts;
    }
  }

  // Use local variables to calculate all rates, then update cached values atomically
  uint16_t new_avg_rate = 0;
  uint16_t new_min_rate = 0;
  uint16_t new_max_rate = 0;

  // Calculate average rate with overflow protection
  // rate_cHz = (samples - 1) * 100_000_000 / duration_us
  if (samples_in_window >= 2 && newest_in_window > oldest_in_window) {
    uint64_t duration_us = newest_in_window - oldest_in_window;
    uint64_t rate = ((samples_in_window - 1) * 100000000ULL) / duration_us;
    new_avg_rate = (rate > kMaxRateCHz) ? kMaxRateCHz : static_cast<uint16_t>(rate);
  }

  // For min/max, collect sorted timestamps and compute intervals
  uint64_t sorted_ts[kRateWindowSize];
  size_t sorted_count = 0;
  for (size_t i = 0; i < sample_timestamps_count_; i++) {
    uint64_t ts = sample_timestamps_[i];
    if (ts >= window_start && ts <= now_us) {
      sorted_ts[sorted_count++] = ts;
    }
  }

  // Release mutex after copying data - sorting and rate calculation can proceed without it
  xSemaphoreGive(rate_stats_mutex_);

  // Simple insertion sort (small array)
  for (size_t i = 1; i < sorted_count; i++) {
    uint64_t key = sorted_ts[i];
    int j = i - 1;
    while (j >= 0 && sorted_ts[j] > key) {
      sorted_ts[j + 1] = sorted_ts[j];
      j--;
    }
    sorted_ts[j + 1] = key;
  }

  // Compute min/max intervals (microseconds)
  uint64_t min_interval_us = UINT64_MAX;
  uint64_t max_interval_us = 0;
  if (sorted_count >= 2) {
    for (size_t i = 1; i < sorted_count; i++) {
      uint64_t interval = sorted_ts[i] - sorted_ts[i - 1];
      if (interval > 0) {
        if (interval < min_interval_us) min_interval_us = interval;
        if (interval > max_interval_us) max_interval_us = interval;
      }
    }

    // Convert intervals to rates (rate = 1/interval) with overflow protection
    // rate_cHz = 100_000_000 / interval_us
    // min_interval -> max_rate, max_interval -> min_rate
    if (min_interval_us > 0 && min_interval_us < UINT64_MAX) {
      uint64_t rate = 100000000ULL / min_interval_us;
      new_max_rate = (rate > kMaxRateCHz) ? kMaxRateCHz : static_cast<uint16_t>(rate);
    }
    if (max_interval_us > 0) {
      uint64_t rate = 100000000ULL / max_interval_us;
      new_min_rate = (rate > kMaxRateCHz) ? kMaxRateCHz : static_cast<uint16_t>(rate);
    }
  }

  // Update all cached values together to ensure consistency for readers
  cached_avg_rate_cHz_ = new_avg_rate;
  cached_min_rate_cHz_ = new_min_rate;
  cached_max_rate_cHz_ = new_max_rate;
}

uint16_t App::GetAvgUpdateRateCHz() {
  app.CalculateRateStatistics();
  return app.cached_avg_rate_cHz_;
}

uint16_t App::GetMinRateCHz() {
  app.CalculateRateStatistics();
  return app.cached_min_rate_cHz_;
}

uint16_t App::GetMaxRateCHz() {
  app.CalculateRateStatistics();
  return app.cached_max_rate_cHz_;
}
#endif // USE_RATE_STATISTICS

#if defined(USE_MAVLINK) && defined(USE_MAVLINK_COVARIANCE)
/**
 * @brief Rotates a 3x3 position covariance matrix by yaw angle
 *
 * Applies the transformation P' = R * P * R^T where R is a 2D rotation matrix.
 * The Z variance is unchanged, but cross-correlations with Z are rotated.
 *
 * @param cov Input covariance [var_x, cov_xy, cov_xz, var_y, cov_yz, var_z]
 * @param yaw_rad Yaw rotation angle in radians
 * @return Rotated covariance array
 */
static std::array<float, 6> rotateCovarianceByYaw(
    const std::array<float, 6>& cov, float yaw_rad)
{
  if (yaw_rad == 0.0f) {
    return cov;
  }

  float c = cosf(yaw_rad);
  float s = sinf(yaw_rad);

  // Input: [var_x, cov_xy, cov_xz, var_y, cov_yz, var_z]
  float var_x = cov[0];
  float cov_xy = cov[1];
  float cov_xz = cov[2];
  float var_y = cov[3];
  float cov_yz = cov[4];
  float var_z = cov[5];

  // 2D rotation for XY plane, Z unchanged
  // R = [c -s 0; s c 0; 0 0 1]
  // P_rot = R * P * R^T

  std::array<float, 6> rotated;
  rotated[0] = c*c*var_x + 2*c*s*cov_xy + s*s*var_y;           // var_x'
  rotated[1] = c*c*cov_xy - s*s*cov_xy + c*s*(var_y - var_x);  // cov_xy'
  rotated[2] = c*cov_xz + s*cov_yz;                             // cov_xz'
  rotated[3] = s*s*var_x - 2*c*s*cov_xy + c*c*var_y;           // var_y'
  rotated[4] = -s*cov_xz + c*cov_yz;                            // cov_yz'
  rotated[5] = var_z;                                           // var_z' (unchanged)

  return rotated;
}

/**
 * @brief Maps 6-element position covariance to MAVLink 21-element array
 *
 * MAVLink uses row-major upper triangular packing for 6x6 covariance.
 * We only provide the position block. Unprovided cross/orientation terms are
 * set to zero so consumers that treat a finite first element as a complete
 * covariance do not propagate NaN through their pose-noise handling.
 */
static std::array<float, VISION_POSITION_COVARIANCE_SIZE> mapToMAVLinkCovariance(
    const std::array<float, 6>& posCovariance)
{
  std::array<float, VISION_POSITION_COVARIANCE_SIZE> mavCov;

  // Once covariance[0] is finite, ArduPilot consumes pose covariance as a
  // complete matrix. Keep unprovided terms finite and uncorrelated.
  for (size_t i = 0; i < mavCov.size(); ++i) {
    mavCov[i] = 0.0f;
  }

  // Position variances and covariances (3x3 block)
  // Row 0: [var_x, cov_xy, cov_xz, cov_x_roll, cov_x_pitch, cov_x_yaw]
  mavCov[0] = posCovariance[0];   // var(x)
  mavCov[1] = posCovariance[1];   // cov(x,y)
  mavCov[2] = posCovariance[2];   // cov(x,z)
  // indices 3,4,5 = position-orientation cross-covariances -> 0 (no correlation)
  mavCov[3] = 0.0f;
  mavCov[4] = 0.0f;
  mavCov[5] = 0.0f;

  // Row 1: [var_y, cov_yz, cov_y_roll, cov_y_pitch, cov_y_yaw]
  mavCov[6] = posCovariance[3];   // var(y)
  mavCov[7] = posCovariance[4];   // cov(y,z)
  // indices 8,9,10 = position-orientation cross-covariances -> 0
  mavCov[8] = 0.0f;
  mavCov[9] = 0.0f;
  mavCov[10] = 0.0f;

  // Row 2: [var_z, cov_z_roll, cov_z_pitch, cov_z_yaw]
  mavCov[11] = posCovariance[5];  // var(z)
  // indices 12,13,14 = position-orientation cross-covariances -> 0
  mavCov[12] = 0.0f;
  mavCov[13] = 0.0f;
  mavCov[14] = 0.0f;

  // Orientation covariance block: indices 15..20 remain 0.0f. UWB provides no
  // orientation covariance, and ArduPilot will clamp the resulting angular
  // error to its configured VISO_YAW_M_NSE.

  return mavCov;
}
#endif // USE_MAVLINK && USE_MAVLINK_COVARIANCE

#ifdef HAS_POSITION_OUTPUT
void App::SendSample(float x_m, float y_m, float z_m,
                     std::optional<std::array<float, 6>> positionCovariance)
{
  // Determine Z coordinate based on zCalcMode parameter
  ZCalcMode mode = Front::uwbLittleFSFront.GetParams().zCalcMode;
  float final_z;
#ifdef HAS_RANGEFINDER
  if (mode == ZCalcMode::RANGEFINDER) {
    // Rangefinder mode: use rangefinder Z directly (NAN if unavailable)
    final_z = GetRangefinderZ();
  } else
#endif
  {
    // NONE or UWB mode: use TDoA Z
    final_z = z_m;
  }

  // Apply coordinate system rotation to correct for beacon system orientation
  Vector3f rotated_vector = correct_for_orient_yaw(x_m, y_m, final_z);

#ifdef USE_RTLSLINK_BEACON_BACKEND
  if (IsRtlslinkBeaconOutputSelected()) {
    app.rtlslink_beacon_backend_.SendPosition(rotated_vector.x, rotated_vector.y, rotated_vector.z);
    app.last_sample_timestamp_ms_ = millis();
#ifdef USE_RATE_STATISTICS
    app.RecordSampleTimestamp();
#endif
    return;
  }
#endif

#if defined(USE_MAVLINK) && defined(USE_MAVLINK_POSITION)
  if (!IsMavlinkOutputSelected()) {
    return;
  }

  // Check if we have received a heartbeat recently
#ifdef USE_MAVLINK_HEARTBEAT
  bool heartbeatOk = (millis() - app.last_heartbeat_received_timestamp_ms_ < kHeartbeatRcvTimeoutMs);
#else
  bool heartbeatOk = true;  // Always send if no heartbeat checking
#endif

  if (heartbeatOk) {
#ifdef USE_MAVLINK_COVARIANCE
    // Defense in depth: also check enableCovMatrix parameter here
    bool sendCovMatrix = positionCovariance.has_value() &&
                         Front::uwbLittleFSFront.GetParams().enableCovMatrix != 0;

    if (sendCovMatrix) {
      // Rotate covariance to match rotated position
      float rotationDegrees = Front::uwbLittleFSFront.GetParams().rotationDegrees;
      float yaw_rad = rotationDegrees * M_PI / 180.0f;

      std::array<float, 6> rotatedCov = rotateCovarianceByYaw(*positionCovariance, yaw_rad);
      std::array<float, VISION_POSITION_COVARIANCE_SIZE> mavCov = mapToMAVLinkCovariance(rotatedCov);

      app.local_position_sensor_.send_vision_position_estimate(
          rotated_vector.x, rotated_vector.y, rotated_vector.z,
          0, 0, 0,  // No orientation from UWB
          &mavCov);
    } else
#endif // USE_MAVLINK_COVARIANCE
    {
      // No covariance - send with nullptr (will use NaN)
      app.local_position_sensor_.send_vision_position_estimate(
          rotated_vector.x, rotated_vector.y, rotated_vector.z,
          0, 0, 0);
    }
    app.last_sample_timestamp_ms_ = millis();
#ifdef USE_RATE_STATISTICS
    app.RecordSampleTimestamp();  // Track for rate statistics
#endif
  }
#endif // USE_MAVLINK && USE_MAVLINK_POSITION
}
#endif // HAS_POSITION_OUTPUT

#ifdef USE_RTLSLINK_BEACON_BACKEND
void App::ConfigureRtlslinkBeaconAnchors(etl::span<const UWBAnchorParam> anchors)
{
  app.rtlslink_beacon_backend_.ConfigureAnchors(anchors, Front::uwbLittleFSFront.GetParams().rotationDegrees);
}

void App::SendTdoaMeasurement(uint8_t anchor_a,
                              uint8_t anchor_b,
                              float distance_diff_m,
                              float sigma_m,
                              uint64_t solved_timestamp_us)
{
  if (!IsRtlslinkBeaconOutputSelected()) {
    return;
  }
  app.rtlslink_beacon_backend_.EnqueueTdoa(anchor_a, anchor_b, distance_diff_m, sigma_m, solved_timestamp_us);
}
#endif

void App::Start()
{
  // For now the start will do nothing
}

#if defined(USE_MAVLINK) || defined(USE_RTLSLINK_BEACON_BACKEND)
HardwareSerial& App::GetArdupilotSerial()
{
    return Serial1;
}
#endif

bool App::IsMavlinkOutputSelected()
{
#ifdef USE_MAVLINK
#ifdef USE_RTLSLINK_BEACON_BACKEND
  return Front::uwbLittleFSFront.GetParams().outputBackend == OutputBackend::MAVLINK;
#else
  return true;
#endif
#else
  return false;
#endif
}

bool App::IsRtlslinkBeaconOutputSelected()
{
#ifdef USE_RTLSLINK_BEACON_BACKEND
#ifdef USE_MAVLINK
  return Front::uwbLittleFSFront.GetParams().outputBackend == OutputBackend::RTLSLINK_BEACON;
#else
  return true;
#endif
#else
  return false;
#endif
}
