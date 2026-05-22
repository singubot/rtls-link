#pragma once

#include <Arduino.h>

#include <etl/array.h>

#include "config/features.hpp"
#include "utils/utils.hpp"

enum class UWBMode : uint8_t {
    ANCHOR_TDOA = 3,
    TAG_TDOA = 4,
    UNKNOWN = 255
};

enum class ZCalcMode : uint8_t {
    NONE = 0,        // Use Z from UWB TDoA estimator (default)
    RANGEFINDER = 1, // Use Z from MAVLink DISTANCE_SENSOR
    UWB = 2          // Reserved for future UWB-based Z calculation
};

enum class OutputBackend : uint8_t {
    MAVLINK = 0,          // Current/default ArduPilot integration path
    RTLSLINK_BEACON = 1   // ArduPilot AP_Beacon_RTLSLink serial backend
};

// Anchor layout configurations for dynamic position calculation
// A0 is always at origin. The layout number determines which anchors
// define the +X and +Y axes.
enum class AnchorLayout : uint8_t {
    RECTANGULAR_A1X_A3Y = 0,  // +X=A1, +Y=A3 (default)
    RECTANGULAR_A1X_A2Y = 1,  // +X=A1, +Y=A2
    RECTANGULAR_A3X_A1Y = 2,  // +X=A3, +Y=A1
    RECTANGULAR_A2X_A3Y = 3,  // +X=A2, +Y=A3
    CUSTOM = 255              // Reserved for future custom layouts
};

using UWBShortAddr = etl::array<char, 2>;

/**
 * @brief Used as a mirror for anchor data on the main UWBParams structure.
 * 
 */
struct UWBAnchorParam {
    UWBShortAddr shortAddr;
    float x;
    float y;
    float z;
}ULS_PACKED;

struct UWBParams {
    UWBMode mode;                                       // Device UWB mode
    uint8_t uwbEnable = 1;                              // Runtime UWB backend enable (0=disabled, 1=enabled)
    UWBShortAddr devShortAddr;                          // Device short address
    uint8_t anchorCount;                                // Number of anchors that might be available to the device
    // Anchor devices and it's relative locations
    UWBShortAddr devId1; // Anchor devices short addresses
    float x1;
    float y1;
    float z1;
    UWBShortAddr devId2;
    float x2;
    float y2;
    float z2;
    UWBShortAddr devId3;
    float x3;
    float y3;
    float z3;
    UWBShortAddr devId4;
    float x4;
    float y4;
    float z4;
    UWBShortAddr devId5;
    float x5;
    float y5;
    float z5;
    UWBShortAddr devId6;
    float x6;
    float y6;
    float z6;
    uint16_t ADelay;            // Antenna delay    
    double originLat;           // Origin Latitude
    double originLon;           // Origin Longitude
    float originAlt;            // Origin Altitude
    uint8_t mavlinkTargetSystemId; // MAVLink Target System ID
    // Runtime output transport. Keep MAVLink as the default when it is compiled,
    // otherwise select the compiled RTLSLink beacon backend.
#ifdef USE_MAVLINK
    OutputBackend outputBackend = OutputBackend::MAVLINK;
#elif defined(USE_RTLSLINK_BEACON_BACKEND)
    OutputBackend outputBackend = OutputBackend::RTLSLINK_BEACON;
#else
    OutputBackend outputBackend = OutputBackend::MAVLINK;
#endif
    float rotationDegrees;      // Rotation degrees to NED frame
    ZCalcMode zCalcMode;        // Z calculation mode (0=None/TDoA, 1=Rangefinder, 2=UWB-reserved)
    uint8_t rtlsBeaconAgeBiasMs = 2; // Additional TDoA age bias for AP-side UART/scheduler latency
    float rtlsBeaconTdoaSigmaFloorM = 0.25f; // Minimum TDoA one-sigma error reported to AP_Beacon_RTLSLink
    uint8_t rtlsBeaconTdoaPhysicalGuardEnable = 1; // 0=disabled, 1=drop physically impossible TDoA samples
    float rtlsBeaconTdoaPhysicalGuardMarginM = 1.0f; // Extra allowed range-difference beyond anchor baseline
    // Rangefinder forwarding parameters
    uint8_t rfForwardEnable = 0;        // 0=disabled, 1=enabled (forward DISTANCE_SENSOR to ArduPilot)
    uint8_t rfForwardSensorId = 0xFF;   // Sensor ID override (0xFF = preserve source value)
    uint8_t rfForwardOrientation = 0xFF; // Orientation override (0xFF = preserve source)
    uint8_t rfForwardPreserveSrcIds = 0; // 0=use UWB device IDs (default), 1=preserve source IDs
    // Position estimation parameters
    uint8_t enableCovMatrix = 0;    // 0=disabled, 1=enabled (send covariance to ArduPilot)
    float rmseThreshold = 0.8f;     // RMSE threshold for position validity (meters)
    uint8_t use2DEstimator = 1;     // 0=3D Newton-Raphson, 1=2D (XY-only with fixed Z, default)
    // UWB Radio settings (TDoA mode only)
    uint8_t channel = 2;            // UWB channel (1-7), default 2
    uint8_t dwMode = 0;             // DW1000 mode index (0=SHORTDATA_FAST_ACCURACY, see getModeByIndex)
    uint8_t txPowerLevel = 3;       // TX power level (0=low, 1=med-low, 2=med-high, 3=high/large)
    uint8_t smartPowerEnable = 0;   // Smart power (0=disabled, 1=enabled)

    // TDoA TDMA schedule (anchors)
    // NOTE: 0 values keep legacy behavior (8 slots, ~2ms slot length).
    uint8_t tdoaSlotCount = 0;          // Active TDMA slots per frame (2-8), 0=legacy (8)
    uint16_t tdoaSlotDurationUs = 0;    // Slot duration in microseconds, 0=legacy (~2ms)
#ifdef ESP32S3_UWB_BOARD
    uint8_t tdoaMatcherPolicy = 0;      // 0=YOUNGEST, 1=RANDOM/rotating eligible candidate
#endif

    // Dynamic anchor positioning (TDoA tags)
    uint8_t dynamicAnchorPosEnabled = 0;  // 0=static (use configured positions), 1=dynamic (calculate from inter-anchor distances)
    uint8_t anchorLayout = 0;             // AnchorLayout enum value (0=RECTANGULAR_0_ORIGIN)
    float anchorHeight = 0.0f;            // Height for Z calculation (NED: Z = -height)
    uint8_t anchorPosLocked = 0;          // Bitmask: bit N = anchor N position locked
    uint16_t distanceAvgSamples = 50;     // Number of samples to average before calculating (default: 50)

    // TDoA anchor model correction (TDoA tags)
    uint8_t tdoaAnchorModelMode = 0;              // 0=OFF, 1=MONITOR, 2=LOCKED_ANCHOR_MODEL
    uint8_t tdoaAnchorModelStartupCollect = 0;    // 0=manual, 1=collect/lock after startup ranging begins
    uint32_t tdoaAnchorModelCollectWindowMs = 10000;
    uint16_t tdoaAnchorModelMinSamplesPerPair = 20;
    uint8_t tdoaAnchorModelDomain = 0;            // 0=RAW_EFFECTIVE, 1=PROPAGATION
    uint16_t tdoaAnchorModelHealthThresholdTicks = 250;
    uint16_t tdoaAnchorModelHealthWindow = 50;
    uint8_t tdoaAnchorModelHealthQuorum = 5;

    // static constant values that will be useful for parameter reading & writing
    static constexpr uint8_t maxAnchorCount = 6;
}ULS_PACKED;

/**
 * @todo We need to add array and custom structs to parameter system in the future to avoid this mess.
 * 
 */
