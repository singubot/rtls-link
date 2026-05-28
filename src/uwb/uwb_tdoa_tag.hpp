#pragma once

#include "config/features.hpp"

#ifdef USE_UWB_MODE_TDOA_TAG

#include <freertos/FreeRTOS.h>

#include <etl/span.h>

#include "uwb_ops.hpp"

extern "C" {
    #include "libdw1000.h"
}

#include "uwb_backend.hpp"
#include "uwb_params.hpp"

#include "utils/dispatcher.hpp"

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
#include "tag/dynamicAnchorPositions.hpp"
// Forward declaration for telemetry struct (defined in wifi_discovery.hpp)
struct DynamicAnchorTelemetry;
#endif

class UWBTagTDoA : public UWBBackend {
public:
    UWBTagTDoA(const bsp::UWBConfig& uwb_config, etl::span<const UWBAnchorParam> anchors);

    template <libDw1000::IsrFlags TFlags>
    void OnEvent();             // Called outside ISR context

    void Update() override;

    virtual uint32_t GetNumberOfConnectedDevices() override;

    /**
     * @brief Get the number of unique anchors seen in the current measurement set.
     * Thread-safe with caching - returns cached value if mutex is unavailable.
     * @return Number of unique anchor IDs (0-16)
     */
    static uint8_t GetAnchorsSeenCount();

    static void ResetAnchorModel();
    static bool StartAnchorModelCollection();
    static bool LockAnchorModel();
    static String GetAnchorModelStatusJson();
    static String GetAnchorModelCollectStatusJson();
    static String ExportAnchorModelJson();
    static String GetEstimatorStatsJson();
    static void ResetEstimatorStats();
    static bool ValidateStaticAnchors(etl::span<const UWBAnchorParam> anchors);
    static bool ApplyStaticAnchors(etl::span<const UWBAnchorParam> anchors);
#ifdef ESP32S3_UWB_BOARD
    static void ApplyMatcherPolicy(uint8_t policy);
#endif

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    /**
     * @brief Check if dynamic anchor positioning is enabled.
     * @return true if enabled
     */
    static bool IsDynamicPositioningEnabled();

    /**
     * @brief Get the calculated dynamic anchor positions.
     * Thread-safe - uses mutex with short timeout to avoid blocking discovery.
     * @param out Output array for anchor positions
     * @param maxCount Maximum number of positions to retrieve
     * @return Number of positions written to out array
     */
    static uint8_t GetDynamicAnchorPositions(DynamicAnchorTelemetry* out, uint8_t maxCount);
    static void ApplyDynamicAnchorPositioningEnabled(uint8_t enabled);
    static bool AreDynamicPositionsReadyForEstimator();
#ifdef USE_RTLSLINK_BEACON_BACKEND
    static bool ConfigureRtlslinkBeaconFromCurrentAnchors();
#endif
#endif

private:
    // Libdw1000 device
    dwDevice_t m_Device;
    dwOps_t m_Ops = {
        .spiRead = libDw1000::SpiRead,
        .spiWrite = libDw1000::SpiWrite,
        .spiSetSpeed = libDw1000::SpiSetSpeed,
        .delayms = libDw1000::DelayMs,
        .reset = libDw1000::Reset,
    };

    // User data for libdw1000
    libDw1000::DwData m_DwData = {
        .rst_pin = bsp::kBoardConfig.uwb.pins.reset_pin,
        .cs_pin = bsp::kBoardConfig.uwb.pins.spi_cs_pin,
        .interrupt_flags = 0,
    };

    struct TDoASample {
        float distance_diff;
        uint8_t anchor_a_id;
        uint8_t anchor_b_id;
    };

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    // Dynamic anchor position calculation
    static DynamicAnchorPositionCalculator s_dynamicCalc;
    static bool s_useDynamicPositions;
    static uint32_t s_lastPositionUpdate;

    // Callback for inter-anchor distance updates from tdoa_tag_algorithm
    static void onInterAnchorDistance(uint8_t fromAnchor, uint8_t toAnchor, uint16_t distanceTimestampUnits, uint16_t fromAntennaDelay);

    // Check and apply dynamic position updates
    void maybeUpdateDynamicPositions();
#endif
};


using TagTDoADispatcher = Dispatcher<libDw1000::IsrFlags, UWBTagTDoA,
        DispatchEntry<libDw1000::IsrFlags, libDw1000::IsrFlags::RX_DONE, UWBTagTDoA, &UWBTagTDoA::OnEvent<libDw1000::IsrFlags::RX_DONE>>,
        DispatchEntry<libDw1000::IsrFlags, libDw1000::IsrFlags::TX_DONE, UWBTagTDoA, &UWBTagTDoA::OnEvent<libDw1000::IsrFlags::TX_DONE>>,
        DispatchEntry<libDw1000::IsrFlags, libDw1000::IsrFlags::RX_TIMEOUT, UWBTagTDoA, &UWBTagTDoA::OnEvent<libDw1000::IsrFlags::RX_TIMEOUT>>,
        DispatchEntry<libDw1000::IsrFlags, libDw1000::IsrFlags::RX_FAILED, UWBTagTDoA, &UWBTagTDoA::OnEvent<libDw1000::IsrFlags::RX_FAILED>>>;

#endif // USE_UWB_MODE_TDOA_TAG
