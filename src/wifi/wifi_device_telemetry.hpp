#pragma once

#include <Arduino.h>
#include <etl/delegate.h>

#include "config/features.hpp"

struct DynamicAnchorTelemetry {
    uint8_t id;
    float x;
    float y;
    float z;
};

struct DeviceTelemetry {
    bool sending_pos = false;
    uint8_t anchors_seen = 0;
    bool origin_sent = false;
    bool uwb_enabled = true;
    bool rf_forward_enabled = false;
    bool rf_enabled = false;
    bool rf_healthy = false;
    uint16_t avg_rate_cHz = 0;
    uint16_t min_rate_cHz = 0;
    uint16_t max_rate_cHz = 0;

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    bool dynamic_anchors_enabled = false;
    DynamicAnchorTelemetry dynamic_anchors[4];
    uint8_t dynamic_anchor_count = 0;
#endif
};

using TelemetryCallback = etl::delegate<DeviceTelemetry()>;
