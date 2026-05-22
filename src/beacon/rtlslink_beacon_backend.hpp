#pragma once

#include "config/features.hpp"

#ifdef USE_RTLSLINK_BEACON_BACKEND

#include <Arduino.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <etl/array.h>
#include <etl/span.h>

#include "uwb/uwb_params.hpp"

class RTLSLinkBeaconBackend {
public:
    explicit RTLSLinkBeaconBackend(HardwareSerial& serial);

    void Init(uint32_t baudrate, size_t tx_buffer_size);
    void Update();

    void ConfigureAnchors(etl::span<const UWBAnchorParam> anchors, float rotation_degrees);
    void SendPosition(float x_m, float y_m, float z_m);
    bool EnqueueTdoa(uint8_t anchor_a,
                     uint8_t anchor_b,
                     float distance_diff_m,
                     float sigma_m,
                     uint64_t solved_timestamp_us);

    bool IsConfigured() const { return config_accepted_; }
    uint32_t DroppedStale() const { return dropped_stale_; }
    uint32_t DroppedTxBusy() const { return dropped_tx_busy_; }
    uint32_t DroppedTdoaTxBusy() const { return dropped_tdoa_tx_busy_; }
    uint32_t DroppedTdoaSerialFull() const { return dropped_tdoa_serial_full_; }
    uint32_t DroppedTdoaPhysicalGuard() const { return dropped_tdoa_physical_guard_; }

private:
    static constexpr uint8_t kFrameMagic1 = 0x52; // 'R'
    static constexpr uint8_t kFrameMagic2 = 0x42; // 'B'
    static constexpr uint8_t kProtocolVersion = 1;
    static constexpr uint8_t kPayloadLenMax = 32;
    static constexpr uint8_t kMaxAnchors = 6;
    static constexpr uint32_t kConfigRetryIntervalMs = 500;
    static constexpr uint16_t kMaxTdoaAgeMs = 200;
    static constexpr uint16_t kDefaultPositionErrorMm = 500;

    enum class MsgId : uint8_t {
        HELLO = 1,
        ANCHOR_CONFIG = 2,
        POSITION = 3,
        TDOA = 4,
        CONFIG_END = 5,
        ORIGIN_CONFIG = 6,
        ACK = 0x80,
    };

    enum class AckStatus : uint8_t {
        OK = 0,
        UNSUPPORTED_VERSION = 1,
        BAD_CONFIG = 2,
        BAD_FRAME = 3,
    };

    enum class ParseState : uint8_t {
        MAGIC_1,
        MAGIC_2,
        MSG_ID,
        LEN,
        SEQ,
        PAYLOAD,
        CRC_LOW,
        CRC_HIGH,
    };

    struct Anchor {
        uint8_t id = 0;
        int32_t x_mm = 0;
        int32_t y_mm = 0;
        int32_t z_mm = 0;
        bool valid = false;
    };

    struct PendingTdoa {
        uint64_t solved_us = 0;
        int32_t distance_diff_mm = 0;
        uint16_t sigma_mm = 0;
        uint8_t anchor_a = 0;
        uint8_t anchor_b = 0;
    };

    static bool ParseAnchorId(const UWBShortAddr& short_addr, uint8_t& out_anchor_id);
    static int32_t MetersToMm(float meters);
    static uint16_t MetersToU16Mm(float meters);
    static int32_t DegToDegE7(double degrees);
    static int32_t MetersToCm(float meters);
    static uint16_t Crc16Update(uint16_t crc, uint8_t b);
    static void WriteI32Le(uint8_t* p, int32_t v);
    static void WriteU16Le(uint8_t* p, uint16_t v);
    static void Rotate(float rotation_degrees, float& x, float& y);

    void ResetParser();
    void ParseByte(uint8_t b);
    void HandleFrame();
    void SendConfig();
    bool SendTdoa(const PendingTdoa& tdoa);
    void SendFrame(MsgId msg_id, const uint8_t* payload, uint8_t payload_len);
    bool TakeTxMutex(TickType_t timeout_ticks);
    void GiveTxMutex();
    void WriteFrameLocked(MsgId msg_id, const uint8_t* payload, uint8_t payload_len);
    bool TdoaPassesPhysicalGuard(uint8_t anchor_a, uint8_t anchor_b, float distance_diff_m);
    bool AnchorBaselineM(uint8_t anchor_a, uint8_t anchor_b, float& baseline_m) const;
    uint16_t ComputeAgeMs(uint64_t solved_us, size_t frame_len) const;
    uint32_t EstimateSerialDrainUs(size_t frame_len) const;

    HardwareSerial& serial_;
    uint32_t baudrate_ = 921600;
    size_t tx_buffer_size_ = 0;
    bool initialized_ = false;
    bool config_accepted_ = false;
    uint32_t last_config_ms_ = 0;
    uint8_t seq_ = 0;
    uint8_t anchor_count_ = 0;
    etl::array<Anchor, kMaxAnchors> anchors_ = {};

    SemaphoreHandle_t tx_mutex_ = nullptr;
    uint32_t dropped_stale_ = 0;
    uint32_t dropped_tx_busy_ = 0;
    uint32_t dropped_tdoa_tx_busy_ = 0;
    uint32_t dropped_tdoa_serial_full_ = 0;
    uint32_t dropped_tdoa_physical_guard_ = 0;

    ParseState parse_state_ = ParseState::MAGIC_1;
    uint8_t rx_msg_id_ = 0;
    uint8_t rx_payload_len_ = 0;
    uint8_t rx_seq_ = 0;
    uint8_t rx_payload_[kPayloadLenMax] = {};
    uint8_t rx_payload_idx_ = 0;
    uint16_t rx_crc_ = 0xffff;
    uint16_t rx_frame_crc_ = 0;
};

#endif // USE_RTLSLINK_BEACON_BACKEND
