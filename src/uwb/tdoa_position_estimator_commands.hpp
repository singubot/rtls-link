#pragma once

#include "config/features.hpp"

#ifdef USE_UWB_MODE_TDOA_TAG

#include <Arduino.h>

#include "protocol/rtls_binary_protocol.hpp"

namespace TDoAPositionEstimatorCommands {
    String StatusJson();
    void AppendBinaryStatus(rtls::protocol::BinaryFrameBuilder<2048>& outFrame);
    void ResetStats();
}

#endif // USE_UWB_MODE_TDOA_TAG
