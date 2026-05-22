#include "config/features.hpp"

#ifdef USE_RTLSLINK_BEACON_BACKEND

#include "rtlslink_beacon_backend.hpp"

#include <algorithm>
#include <cmath>
#include <esp_timer.h>

#include "logging/logging.hpp"
#include "uwb/uwb_frontend_littlefs.hpp"

RTLSLinkBeaconBackend::RTLSLinkBeaconBackend(HardwareSerial& serial)
    : serial_(serial)
{}

void RTLSLinkBeaconBackend::Init(uint32_t baudrate, size_t tx_buffer_size)
{
    if (initialized_) {
        return;
    }

    baudrate_ = baudrate;
    tx_buffer_size_ = tx_buffer_size;
    if (tx_mutex_ == nullptr) {
        tx_mutex_ = xSemaphoreCreateMutex();
    }
    if (tx_mutex_ == nullptr) {
        LOG_ERROR("RTLSLink beacon backend mutex allocation failed");
        return;
    }

    initialized_ = true;
    LOG_INFO("RTLSLink beacon backend ready at %lu baud", static_cast<unsigned long>(baudrate));
}

void RTLSLinkBeaconBackend::ConfigureAnchors(etl::span<const UWBAnchorParam> anchors, float rotation_degrees)
{
    anchors_.fill({});
    uint8_t max_anchor_id = 0;
    uint8_t configured = 0;

    for (const auto& anchor_param : anchors) {
        uint8_t anchor_id = 0;
        if (!ParseAnchorId(anchor_param.shortAddr, anchor_id) || anchor_id >= kMaxAnchors) {
            LOG_WARN("RTLSLink beacon ignoring anchor '%c%c'",
                     anchor_param.shortAddr[0], anchor_param.shortAddr[1]);
            continue;
        }

        float x = anchor_param.x;
        float y = anchor_param.y;
        Rotate(rotation_degrees, x, y);

        anchors_[anchor_id] = {
            .id = anchor_id,
            .x_mm = MetersToMm(x),
            .y_mm = MetersToMm(y),
            .z_mm = MetersToMm(anchor_param.z),
            .valid = true,
        };
        max_anchor_id = std::max(max_anchor_id, anchor_id);
        configured++;
    }

    anchor_count_ = configured == 0 ? 0 : static_cast<uint8_t>(max_anchor_id + 1);
    config_accepted_ = false;
    last_config_ms_ = 0;

    LOG_INFO("RTLSLink beacon configured %u anchors (count=%u)",
             static_cast<unsigned int>(configured),
             static_cast<unsigned int>(anchor_count_));
    if (configured != anchor_count_) {
        LOG_WARN("RTLSLink beacon requires contiguous anchor ids 0..%u; configured=%u",
                 static_cast<unsigned int>(anchor_count_ - 1),
                 static_cast<unsigned int>(configured));
    }
}

void RTLSLinkBeaconBackend::Update()
{
    if (!initialized_) {
        return;
    }

    uint32_t nbytes = std::min(static_cast<uint32_t>(serial_.available()), 128U);
    while (nbytes-- > 0) {
        const int c = serial_.read();
        if (c >= 0) {
            ParseByte(static_cast<uint8_t>(c));
        }
    }

    if (!config_accepted_) {
        const uint32_t now_ms = millis();
        if ((now_ms - last_config_ms_) >= kConfigRetryIntervalMs) {
            SendConfig();
            last_config_ms_ = now_ms;
        }
        return;
    }
}

void RTLSLinkBeaconBackend::SendPosition(float x_m, float y_m, float z_m)
{
    if (!initialized_ || !config_accepted_) {
        return;
    }

    uint8_t payload[14];
    WriteI32Le(&payload[0], MetersToMm(x_m));
    WriteI32Le(&payload[4], MetersToMm(y_m));
    WriteI32Le(&payload[8], MetersToMm(z_m));
    WriteU16Le(&payload[12], kDefaultPositionErrorMm);
    SendFrame(MsgId::POSITION, payload, sizeof(payload));
}

bool RTLSLinkBeaconBackend::EnqueueTdoa(uint8_t anchor_a,
                                        uint8_t anchor_b,
                                        float distance_diff_m,
                                        float sigma_m,
                                        uint64_t solved_timestamp_us)
{
    if (!initialized_ || !config_accepted_) {
        return false;
    }
    if (anchor_a >= anchor_count_ || anchor_b >= anchor_count_ || anchor_a == anchor_b) {
        return false;
    }
    if (!anchors_[anchor_a].valid || !anchors_[anchor_b].valid) {
        return false;
    }
    if (!std::isfinite(distance_diff_m) || !std::isfinite(sigma_m)) {
        return false;
    }

    if (!TdoaPassesPhysicalGuard(anchor_a, anchor_b, distance_diff_m)) {
        return false;
    }

    const auto& params = Front::uwbLittleFSFront.GetParams();
    const float sigma_floor_m = std::isfinite(params.rtlsBeaconTdoaSigmaFloorM)
        ? std::max(params.rtlsBeaconTdoaSigmaFloorM, 0.0f)
        : 0.0f;

    PendingTdoa pending = {
        .solved_us = solved_timestamp_us,
        .distance_diff_mm = MetersToMm(distance_diff_m),
        .sigma_mm = MetersToU16Mm(std::max(std::max(sigma_m, sigma_floor_m), 0.001f)),
        .anchor_a = anchor_a,
        .anchor_b = anchor_b,
    };

    return SendTdoa(pending);
}

bool RTLSLinkBeaconBackend::ParseAnchorId(const UWBShortAddr& short_addr, uint8_t& out_anchor_id)
{
    if (short_addr[0] < '0' || short_addr[0] > '9') {
        return false;
    }

    uint8_t value = static_cast<uint8_t>(short_addr[0] - '0');
    if (short_addr[1] != '\0') {
        if (short_addr[1] < '0' || short_addr[1] > '9') {
            return false;
        }
        value = static_cast<uint8_t>(value * 10 + static_cast<uint8_t>(short_addr[1] - '0'));
    }

    out_anchor_id = value;
    return true;
}

int32_t RTLSLinkBeaconBackend::MetersToMm(float meters)
{
    if (!std::isfinite(meters)) {
        return 0;
    }
    const float scaled = meters * 1000.0f;
    return static_cast<int32_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

uint16_t RTLSLinkBeaconBackend::MetersToU16Mm(float meters)
{
    const int32_t mm = MetersToMm(meters);
    if (mm <= 0) {
        return 1;
    }
    if (mm > UINT16_MAX) {
        return UINT16_MAX;
    }
    return static_cast<uint16_t>(mm);
}

int32_t RTLSLinkBeaconBackend::DegToDegE7(double degrees)
{
    if (!std::isfinite(degrees)) {
        return 0;
    }
    const double scaled = degrees * 1.0e7;
    return static_cast<int32_t>(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
}

int32_t RTLSLinkBeaconBackend::MetersToCm(float meters)
{
    if (!std::isfinite(meters)) {
        return 0;
    }
    const float scaled = meters * 100.0f;
    return static_cast<int32_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

uint16_t RTLSLinkBeaconBackend::Crc16Update(uint16_t crc, uint8_t b)
{
    crc ^= static_cast<uint16_t>(b) << 8;
    for (uint8_t i = 0; i < 8; i++) {
        crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                             : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

void RTLSLinkBeaconBackend::WriteI32Le(uint8_t* p, int32_t v)
{
    const uint32_t u = static_cast<uint32_t>(v);
    p[0] = static_cast<uint8_t>(u);
    p[1] = static_cast<uint8_t>(u >> 8);
    p[2] = static_cast<uint8_t>(u >> 16);
    p[3] = static_cast<uint8_t>(u >> 24);
}

void RTLSLinkBeaconBackend::WriteU16Le(uint8_t* p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

void RTLSLinkBeaconBackend::Rotate(float rotation_degrees, float& x, float& y)
{
    if (rotation_degrees == 0.0f) {
        return;
    }

    const float yaw_rad = rotation_degrees * static_cast<float>(M_PI) / 180.0f;
    const float c = cosf(yaw_rad);
    const float s = sinf(yaw_rad);
    const float rx = x * c - y * s;
    const float ry = x * s + y * c;
    x = rx;
    y = ry;
}

void RTLSLinkBeaconBackend::ResetParser()
{
    parse_state_ = ParseState::MAGIC_1;
    rx_msg_id_ = 0;
    rx_payload_len_ = 0;
    rx_seq_ = 0;
    rx_payload_idx_ = 0;
    rx_crc_ = 0xffff;
    rx_frame_crc_ = 0;
}

void RTLSLinkBeaconBackend::ParseByte(uint8_t b)
{
    switch (parse_state_) {
    case ParseState::MAGIC_1:
        if (b == kFrameMagic1) {
            rx_crc_ = Crc16Update(0xffff, b);
            parse_state_ = ParseState::MAGIC_2;
        }
        break;
    case ParseState::MAGIC_2:
        if (b != kFrameMagic2) {
            ResetParser();
            if (b == kFrameMagic1) {
                rx_crc_ = Crc16Update(0xffff, b);
                parse_state_ = ParseState::MAGIC_2;
            }
            break;
        }
        rx_crc_ = Crc16Update(rx_crc_, b);
        parse_state_ = ParseState::MSG_ID;
        break;
    case ParseState::MSG_ID:
        rx_msg_id_ = b;
        rx_crc_ = Crc16Update(rx_crc_, b);
        parse_state_ = ParseState::LEN;
        break;
    case ParseState::LEN:
        rx_payload_len_ = b;
        if (rx_payload_len_ > kPayloadLenMax) {
            ResetParser();
            break;
        }
        rx_crc_ = Crc16Update(rx_crc_, b);
        parse_state_ = ParseState::SEQ;
        break;
    case ParseState::SEQ:
        rx_seq_ = b;
        rx_crc_ = Crc16Update(rx_crc_, b);
        rx_payload_idx_ = 0;
        parse_state_ = rx_payload_len_ == 0 ? ParseState::CRC_LOW : ParseState::PAYLOAD;
        break;
    case ParseState::PAYLOAD:
        rx_payload_[rx_payload_idx_++] = b;
        rx_crc_ = Crc16Update(rx_crc_, b);
        if (rx_payload_idx_ >= rx_payload_len_) {
            parse_state_ = ParseState::CRC_LOW;
        }
        break;
    case ParseState::CRC_LOW:
        rx_frame_crc_ = b;
        parse_state_ = ParseState::CRC_HIGH;
        break;
    case ParseState::CRC_HIGH:
        rx_frame_crc_ |= static_cast<uint16_t>(b) << 8;
        if (rx_frame_crc_ == rx_crc_) {
            HandleFrame();
        }
        ResetParser();
        break;
    }
}

void RTLSLinkBeaconBackend::HandleFrame()
{
    if (rx_msg_id_ != static_cast<uint8_t>(MsgId::ACK) || rx_payload_len_ < 3) {
        return;
    }

    const MsgId acked = static_cast<MsgId>(rx_payload_[0]);
    const AckStatus status = static_cast<AckStatus>(rx_payload_[1]);
    const uint8_t version = rx_payload_[2];

    if (acked == MsgId::CONFIG_END && status == AckStatus::OK && version == kProtocolVersion) {
        if (!config_accepted_) {
            LOG_INFO("RTLSLink beacon config accepted");
        }
        config_accepted_ = true;
    } else if (status != AckStatus::OK) {
        config_accepted_ = false;
    }
}

void RTLSLinkBeaconBackend::SendConfig()
{
    if (anchor_count_ == 0) {
        return;
    }

    uint8_t hello[3] = {kProtocolVersion, 0, anchor_count_};
    SendFrame(MsgId::HELLO, hello, sizeof(hello));

    for (uint8_t id = 0; id < anchor_count_; id++) {
        const Anchor& anchor = anchors_[id];
        if (!anchor.valid) {
            continue;
        }

        uint8_t payload[13];
        payload[0] = id;
        WriteI32Le(&payload[1], anchor.x_mm);
        WriteI32Le(&payload[5], anchor.y_mm);
        WriteI32Le(&payload[9], anchor.z_mm);
        SendFrame(MsgId::ANCHOR_CONFIG, payload, sizeof(payload));
    }

    const auto& params = Front::uwbLittleFSFront.GetParams();
    const int32_t origin_lat_e7 = DegToDegE7(params.originLat);
    const int32_t origin_lon_e7 = DegToDegE7(params.originLon);
    const int32_t origin_alt_cm = MetersToCm(params.originAlt);
    if (origin_lat_e7 != 0 || origin_lon_e7 != 0 || origin_alt_cm != 0) {
        uint8_t origin_payload[12];
        WriteI32Le(&origin_payload[0], origin_lat_e7);
        WriteI32Le(&origin_payload[4], origin_lon_e7);
        WriteI32Le(&origin_payload[8], origin_alt_cm);
        SendFrame(MsgId::ORIGIN_CONFIG, origin_payload, sizeof(origin_payload));
    }

    uint8_t config_end[1] = {anchor_count_};
    SendFrame(MsgId::CONFIG_END, config_end, sizeof(config_end));
}

bool RTLSLinkBeaconBackend::SendTdoa(const PendingTdoa& tdoa)
{
    uint8_t payload[10];
    payload[0] = tdoa.anchor_a;
    payload[1] = tdoa.anchor_b;
    WriteI32Le(&payload[2], tdoa.distance_diff_mm);
    WriteU16Le(&payload[6], tdoa.sigma_mm);

    if (tx_mutex_ == nullptr || xSemaphoreTake(tx_mutex_, 0) != pdTRUE) {
        dropped_tdoa_tx_busy_++;
        return false;
    }

    constexpr size_t frame_len = 2 + 3 + sizeof(payload) + 2;
    const int available = serial_.availableForWrite();
    if (available >= 0 && static_cast<size_t>(available) < frame_len) {
        dropped_tdoa_serial_full_++;
        GiveTxMutex();
        return false;
    }

    const uint16_t age_ms = ComputeAgeMs(tdoa.solved_us, frame_len);
    if (age_ms > kMaxTdoaAgeMs) {
        dropped_stale_++;
        GiveTxMutex();
        return false;
    }

    WriteU16Le(&payload[8], age_ms);
    WriteFrameLocked(MsgId::TDOA, payload, sizeof(payload));
    GiveTxMutex();
    return true;
}

void RTLSLinkBeaconBackend::SendFrame(MsgId msg_id, const uint8_t* payload, uint8_t payload_len)
{
    if (!TakeTxMutex(pdMS_TO_TICKS(5))) {
        return;
    }

    WriteFrameLocked(msg_id, payload, payload_len);
    GiveTxMutex();
}

bool RTLSLinkBeaconBackend::TakeTxMutex(TickType_t timeout_ticks)
{
    if (tx_mutex_ == nullptr || xSemaphoreTake(tx_mutex_, timeout_ticks) != pdTRUE) {
        dropped_tx_busy_++;
        return false;
    }
    return true;
}

void RTLSLinkBeaconBackend::GiveTxMutex()
{
    xSemaphoreGive(tx_mutex_);
}

void RTLSLinkBeaconBackend::WriteFrameLocked(MsgId msg_id, const uint8_t* payload, uint8_t payload_len)
{
    uint8_t frame[2 + 3 + kPayloadLenMax + 2];
    uint8_t len = 0;
    frame[len++] = kFrameMagic1;
    frame[len++] = kFrameMagic2;
    frame[len++] = static_cast<uint8_t>(msg_id);
    frame[len++] = payload_len;
    frame[len++] = seq_++;

    for (uint8_t i = 0; i < payload_len; i++) {
        frame[len++] = payload[i];
    }

    uint16_t crc = 0xffff;
    for (uint8_t i = 0; i < len; i++) {
        crc = Crc16Update(crc, frame[i]);
    }
    WriteU16Le(&frame[len], crc);
    len += 2;

    serial_.write(frame, len);
}

bool RTLSLinkBeaconBackend::TdoaPassesPhysicalGuard(uint8_t anchor_a,
                                                    uint8_t anchor_b,
                                                    float distance_diff_m)
{
    const auto& params = Front::uwbLittleFSFront.GetParams();
    if (params.rtlsBeaconTdoaPhysicalGuardEnable == 0) {
        return true;
    }

    float baseline_m = 0.0f;
    if (!AnchorBaselineM(anchor_a, anchor_b, baseline_m)) {
        dropped_tdoa_physical_guard_++;
        return false;
    }

    const float margin_m = std::isfinite(params.rtlsBeaconTdoaPhysicalGuardMarginM)
        ? std::max(params.rtlsBeaconTdoaPhysicalGuardMarginM, 0.0f)
        : 0.0f;
    if (std::fabs(distance_diff_m) > (baseline_m + margin_m)) {
        dropped_tdoa_physical_guard_++;
        return false;
    }
    return true;
}

bool RTLSLinkBeaconBackend::AnchorBaselineM(uint8_t anchor_a, uint8_t anchor_b, float& baseline_m) const
{
    if (anchor_a >= anchor_count_ || anchor_b >= anchor_count_) {
        return false;
    }
    const Anchor& a = anchors_[anchor_a];
    const Anchor& b = anchors_[anchor_b];
    if (!a.valid || !b.valid) {
        return false;
    }

    const float dx_m = static_cast<float>(a.x_mm - b.x_mm) * 0.001f;
    const float dy_m = static_cast<float>(a.y_mm - b.y_mm) * 0.001f;
    const float dz_m = static_cast<float>(a.z_mm - b.z_mm) * 0.001f;
    baseline_m = std::sqrt(dx_m * dx_m + dy_m * dy_m + dz_m * dz_m);
    return std::isfinite(baseline_m);
}

uint16_t RTLSLinkBeaconBackend::ComputeAgeMs(uint64_t solved_us, size_t frame_len) const
{
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    const uint64_t elapsed_us = now_us > solved_us ? now_us - solved_us : 0;
    if (elapsed_us > (static_cast<uint64_t>(kMaxTdoaAgeMs) * 1000ULL)) {
        return kMaxTdoaAgeMs + 1;
    }

    const uint32_t age_us = static_cast<uint32_t>(elapsed_us) + EstimateSerialDrainUs(frame_len);
    const uint32_t age_ms = ((age_us + 999U) / 1000U)
                          + Front::uwbLittleFSFront.GetParams().rtlsBeaconAgeBiasMs;
    return age_ms > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(age_ms);
}

uint32_t RTLSLinkBeaconBackend::EstimateSerialDrainUs(size_t frame_len) const
{
    size_t pending_bytes = 0;
    if (tx_buffer_size_ > 0) {
        const int available = serial_.availableForWrite();
        if (available >= 0 && static_cast<size_t>(available) < tx_buffer_size_) {
            pending_bytes = tx_buffer_size_ - static_cast<size_t>(available);
        }
    }

    const uint64_t bytes = pending_bytes + frame_len;
    return static_cast<uint32_t>((bytes * 10ULL * 1000000ULL + baudrate_ - 1ULL) / baudrate_);
}

#endif // USE_RTLSLINK_BEACON_BACKEND
