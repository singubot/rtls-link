#include "config/features.hpp"  // MUST be first project include

#ifdef USE_UWB_MODE_TDOA_TAG

#include "logging/logging.hpp"

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
#include "tag/dynamicAnchorPositions.hpp"
#endif

#include <Eigen.h>

#include <esp_timer.h>

#include <atomic>
#include <algorithm>
#include <cmath>
#include <optional>
#include <array>

#include "tdoa_newton_raphson.hpp"

#include "tag/tdoa_tag_algorithm.hpp"

#include "uwb_tdoa_tag.hpp"

#include "scheduler.hpp"
#include "app.hpp"
#include "bsp/board.hpp"
#include "dw1000_radio_config.hpp"
#include "uwb_frontend_littlefs.hpp"
#include "tdoa_anchor_model.hpp"
#include "tdoa_anchor_model_commands.hpp"
#include "tdoa_common.hpp"
#include "tdoa_measurement_buffer.hpp"
#include "tdoa_pairs.hpp"

namespace {

using PackedPositionCovariance = std::array<float, 6>; // [var_x, cov_xy, cov_xz, var_y, cov_yz, var_z]

static constexpr float kRangefinderAssisted2DVerticalStdDevM = 0.01f;
static constexpr float kRangefinderAssisted2DVerticalVarianceM2 =
    kRangefinderAssisted2DVerticalStdDevM * kRangefinderAssisted2DVerticalStdDevM;

static bool usesRangefinderZ(const UWBParams& params)
{
#ifdef HAS_RANGEFINDER
    return params.zCalcMode == ZCalcMode::RANGEFINDER;
#else
    (void)params;
    return false;
#endif
}

static PackedPositionCovariance packRangefinderAssisted2DCovariance(
    const tdoa_estimator::CovMatrix2D& xy_covariance)
{
    return PackedPositionCovariance{
        static_cast<float>(xy_covariance(0, 0)),          // var_x from 2D TDoA solver
        static_cast<float>(xy_covariance(0, 1)),          // cov_xy from 2D TDoA solver
        0.0f,                                             // cov_xz unavailable in 2D mode
        static_cast<float>(xy_covariance(1, 1)),          // var_y from 2D TDoA solver
        0.0f,                                             // cov_yz unavailable in 2D mode
        kRangefinderAssisted2DVerticalVarianceM2          // var_z from rangefinder-assisted output path
    };
}

static PackedPositionCovariance pack3DCovariance(
    const tdoa_estimator::CovMatrix3D& covariance)
{
    return PackedPositionCovariance{
        static_cast<float>(covariance(0, 0)),  // var_x
        static_cast<float>(covariance(0, 1)),  // cov_xy
        static_cast<float>(covariance(0, 2)),  // cov_xz
        static_cast<float>(covariance(1, 1)),  // var_y
        static_cast<float>(covariance(1, 2)),  // cov_yz
        static_cast<float>(covariance(2, 2))   // var_z
    };
}

} // namespace

static FAST_CODE void tagInterruptISR();
static FAST_CODE void txCallback(dwDevice_t *dev);
static FAST_CODE void rxCallback(dwDevice_t *dev);
static FAST_CODE void rxTimeoutCallback(dwDevice_t *dev);
static FAST_CODE void rxFailedCallback(dwDevice_t *dev);

static FAST_CODE void estimatorCallback(tdoaMeasurement_t* tdoa);
static bool anchorModelTofCallback(uint8_t fromAnchor, uint8_t toAnchor, uint16_t rawDistanceTimestampUnits, uint16_t fromAntennaDelay, uint16_t toAntennaDelay, uint16_t* outDistanceTimestampUnits);
static void estimatorProcess();

// 8 anchors → 8C2 = 28 unique unordered pairs. Each pair has its own slot,
// so the producer never needs to scan, and stale removal becomes a per-slot
// timestamp check at consume time.
static constexpr uint8_t kNumAnchors = 8;
static constexpr uint8_t kNumPairs = (kNumAnchors * (kNumAnchors - 1)) / 2;

using PairSlot = tdoa::MeasurementSlot;

static etl::array<tdoa::MeasurementSlot, kNumPairs> pair_slots = {};
static SemaphoreHandle_t measurements_mtx = xSemaphoreCreateMutex();
static etl::array<UWBAnchorParam, 8> anchor_positions;
static etl::array<bool, 8> configured_anchor_ids = {};

#if defined(USE_DYNAMIC_ANCHOR_POSITIONS) && defined(USE_RTLSLINK_BEACON_BACKEND)
static void configureRtlslinkBeaconFromAnchorPositions()
{
    etl::array<UWBAnchorParam, 8> dynamic_anchors = {};
    uint8_t dynamic_anchor_count = 0;

    if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(50)) != pdTRUE) {
        LOG_WARN("RTLSLink beacon dynamic anchor config skipped - mutex busy");
        return;
    }

    for (uint8_t id = 0; id < dynamic_anchors.size(); id++) {
        if (!configured_anchor_ids[id]) {
            continue;
        }
        dynamic_anchors[dynamic_anchor_count++] = anchor_positions[id];
    }

    xSemaphoreGive(measurements_mtx);

    if (dynamic_anchor_count == 0) {
        LOG_WARN("RTLSLink beacon dynamic anchor config skipped - no anchors");
        return;
    }

    App::ConfigureRtlslinkBeaconAnchors(
        etl::span<const UWBAnchorParam>(dynamic_anchors.data(), dynamic_anchor_count));
    LOG_INFO("RTLSLink beacon configured from dynamic anchor positions (%u anchors)",
             static_cast<unsigned int>(dynamic_anchor_count));
}
#endif

// Stale threshold for TDoA pair measurements (350ms — one frame at min TDMA rate).
static constexpr uint64_t kStaleThresholdUs = 350000;

// Estimator task handle for direct notification from producer.
static TaskHandle_t s_estimator_task_handle = nullptr;

// Lock-free counter of fresh-pair count maintained by producer and reader.
// Producer increments only when it transitions a stale slot to fresh. Reader
// decrements as it consumes. Used to gate task notifications without holding
// the mutex.
static std::atomic<uint32_t> fresh_pair_count{0};

// Last-notify timestamp for debouncing producer-side wakeups.
static std::atomic<uint32_t> last_notify_ms{0};

static volatile bool isr_flag = false;

// Cache for anchors_seen (used when mutex is unavailable)
static uint8_t cached_anchors_seen = 0;
static TDoAAnchorModel s_anchorModel;

struct EstimatorStats {
    uint32_t samplesSent = 0;
    uint32_t samplesRejected = 0;
    uint32_t rejectRmse = 0;
    uint32_t rejectNan = 0;
    uint32_t rejectInsufficient = 0;
    uint32_t staleRemoved = 0;
    uint32_t lastRmseMm = 0;
    uint8_t lastMeasurementCount = 0;
    uint8_t minMeasurementCount = UINT8_MAX;
    uint8_t maxMeasurementCount = 0;
};

static EstimatorStats* s_estimatorStats = nullptr;
static SemaphoreHandle_t estimator_stats_mtx = nullptr;

static tdoaEngineMatchingAlgorithm_t matcherPolicyFromParam(uint8_t policy)
{
#ifdef ESP32S3_UWB_BOARD
    return policy == 1
        ? TdoaEngineMatchingAlgorithmRandom
        : TdoaEngineMatchingAlgorithmYoungest;
#else
    (void)policy;
    return TdoaEngineMatchingAlgorithmYoungest;
#endif
}

static const char* matcherPolicyName(tdoaEngineMatchingAlgorithm_t policy)
{
    switch (policy) {
        case TdoaEngineMatchingAlgorithmRandom:
            return "RANDOM";
        case TdoaEngineMatchingAlgorithmYoungest:
        default:
            return "YOUNGEST";
    }
}

static uint8_t configuredMatcherPolicy()
{
#ifdef ESP32S3_UWB_BOARD
    return Front::uwbLittleFSFront.GetParams().tdoaMatcherPolicy;
#else
    return 0;
#endif
}

static void ensureEstimatorStatsMutex()
{
    if (s_estimatorStats == nullptr) {
        s_estimatorStats = new EstimatorStats();
    }
    if (s_estimatorStats == nullptr) {
        return;
    }
    if (estimator_stats_mtx == nullptr) {
        estimator_stats_mtx = xSemaphoreCreateMutex();
    }
}

static void recordEstimatorInputTdoa(uint8_t anchorA, uint8_t anchorB, float distanceDiffMeters)
{
    (void)anchorA;
    (void)anchorB;
    (void)distanceDiffMeters;
}

static void recordMeasurementCountLocked(size_t measurementCount)
{
    const uint8_t clampedCount = measurementCount > UINT8_MAX
        ? UINT8_MAX
        : static_cast<uint8_t>(measurementCount);
    if (clampedCount < s_estimatorStats->minMeasurementCount) {
        s_estimatorStats->minMeasurementCount = clampedCount;
    }
    if (clampedCount > s_estimatorStats->maxMeasurementCount) {
        s_estimatorStats->maxMeasurementCount = clampedCount;
    }
}

static void recordEstimatorAccepted(float x, float y, float z, double rmse, size_t measurementCount)
{
    ensureEstimatorStatsMutex();
    if (s_estimatorStats == nullptr || estimator_stats_mtx == nullptr || xSemaphoreTake(estimator_stats_mtx, pdMS_TO_TICKS(2)) != pdTRUE) {
        return;
    }

    s_estimatorStats->samplesSent++;
    s_estimatorStats->lastRmseMm = rmse > 0.0
        ? static_cast<uint32_t>((rmse * 1000.0) + 0.5)
        : 0;
    s_estimatorStats->lastMeasurementCount = measurementCount > UINT8_MAX
        ? UINT8_MAX
        : static_cast<uint8_t>(measurementCount);
    recordMeasurementCountLocked(measurementCount);
    (void)x;
    (void)y;
    (void)z;

    xSemaphoreGive(estimator_stats_mtx);
}

static void recordEstimatorRejected(bool rmse, bool nan, bool insufficient, size_t measurementCount)
{
    ensureEstimatorStatsMutex();
    if (s_estimatorStats == nullptr || estimator_stats_mtx == nullptr || xSemaphoreTake(estimator_stats_mtx, pdMS_TO_TICKS(2)) != pdTRUE) {
        return;
    }

    s_estimatorStats->samplesRejected++;
    if (rmse) {
        s_estimatorStats->rejectRmse++;
    }
    if (nan) {
        s_estimatorStats->rejectNan++;
    }
    if (insufficient) {
        s_estimatorStats->rejectInsufficient++;
    }
    s_estimatorStats->lastMeasurementCount = measurementCount > UINT8_MAX
        ? UINT8_MAX
        : static_cast<uint8_t>(measurementCount);
    recordMeasurementCountLocked(measurementCount);

    xSemaphoreGive(estimator_stats_mtx);
}

static void recordStaleRemoved(size_t count)
{
    if (count == 0) {
        return;
    }

    ensureEstimatorStatsMutex();
    if (s_estimatorStats == nullptr || estimator_stats_mtx == nullptr || xSemaphoreTake(estimator_stats_mtx, pdMS_TO_TICKS(2)) != pdTRUE) {
        return;
    }
    s_estimatorStats->staleRemoved += count;
    xSemaphoreGive(estimator_stats_mtx);
}

static void resetEstimatorStats()
{
    ensureEstimatorStatsMutex();
    if (s_estimatorStats == nullptr || estimator_stats_mtx == nullptr || xSemaphoreTake(estimator_stats_mtx, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    *s_estimatorStats = EstimatorStats();

    xSemaphoreGive(estimator_stats_mtx);
}

static String anchorModelStatus()
{
    return s_anchorModel.StatusJson();
}

#if TDOA_STATS_LOGGING == ENABLE
static uint32_t stats_samples_sent = 0;
static uint32_t stats_samples_rejected = 0;
static uint32_t stats_reject_rmse = 0;          // Rejected due to RMSE > threshold
static uint32_t stats_reject_nan = 0;           // Rejected due to NaN in estimate
static uint32_t stats_reject_insufficient = 0;  // Rejected due to < MIN_MEASUREMENTS
static uint32_t stats_stale_removed = 0;        // Number of stale slot expirations
static uint32_t stats_min_meas_count = UINT32_MAX;
static uint32_t stats_max_meas_count = 0;
static std::atomic<uint32_t> stats_producer_dropped{0};  // producer mutex timeouts
// Solve-duration tracking (microseconds; bracketed around newtonRaphson call only).
static uint32_t stats_solve_count = 0;
static uint64_t stats_solve_sum_us = 0;
static uint32_t stats_solve_min_us = UINT32_MAX;
static uint32_t stats_solve_max_us = 0;
static uint32_t stats_iter_sum = 0;             // accumulated iteration count
static uint64_t stats_last_log_time_ms = 0;
static constexpr uint64_t STATS_LOG_INTERVAL_MS = 1000;
#endif

// Position logging with timed interval
static uint64_t position_last_log_time_ms = 0;
static constexpr uint64_t POSITION_LOG_INTERVAL_MS = 500;  // Log position every 500ms

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
// Static member definitions for dynamic anchor positioning
DynamicAnchorPositionCalculator UWBTagTDoA::s_dynamicCalc;
bool UWBTagTDoA::s_useDynamicPositions = false;
uint32_t UWBTagTDoA::s_lastPositionUpdate = 0;
// Coordination flags between dynamic-anchor updates and estimator task
static std::atomic<bool> s_dynamicPositionsReadyForEstimator{false};
static std::atomic<bool> s_dynamicEstimatorReinitRequested{false};
static std::atomic<uint32_t> s_dynamicTransitionHoldoffUntilMs{0};

// Interval for dynamic position updates (ms)
static constexpr uint32_t DYNAMIC_POS_UPDATE_INTERVAL_MS = 200;
static constexpr uint32_t DYNAMIC_REINIT_HOLDOFF_MS = 300;
#endif

// Event-driven (continuous) task — body blocks on xTaskNotifyTake with a
// watchdog timeout, so cadence is driven by measurement arrivals not polling.
// Priority 2 sits strictly above the App/WiFi/Console tasks (all priority 1)
// that produce its input, so a solve isn't preempted by lower-priority work.
static StaticTaskHolder<etl::delegate<void()>, 16384, TaskType::CONTINUOUS> pos_estimator_task = {
  "PosEstimatorTask",
  0,                // unused for continuous tasks
  2,                // Priority (above App task at 1)
  etl::delegate<void()>::create<&estimatorProcess>(),
    {},
    {}
};


UWBTagTDoA::UWBTagTDoA(const bsp::UWBConfig& uwb_config, etl::span<const UWBAnchorParam> anchors)
    : UWBBackend(uwb_config)
{
    // NOTE: Look into short data fast accuracy...
    // Using a lambda to attach the class method as an interrupt handler
    LOG_INFO("--- UWB Tag TDOA Mode ---");
    vTaskDelay(pdMS_TO_TICKS(300));
    ensureEstimatorStatsMutex();

    configured_anchor_ids.fill(false);

    // Fill in anchor positions lookup table
    for (uint32_t i = 0; i < anchors.size(); i++) {
        uint8_t anchorId = 0;
        if (!tdoa::ParseAnchorId(anchors[i].shortAddr, anchorId)) {
            LOG_WARN("Ignoring invalid configured anchor short address '%c%c' (expected 0-7)",
                     anchors[i].shortAddr[0], anchors[i].shortAddr[1]);
            continue;
        }
        anchor_positions[anchorId] = anchors[i];
        configured_anchor_ids[anchorId] = true;
    }

    // Get UWB radio/settings parameters before deciding which anchor geometry to publish.
    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    s_useDynamicPositions = (uwbParams.dynamicAnchorPosEnabled != 0);
#endif

#ifdef USE_RTLSLINK_BEACON_BACKEND
#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    if (s_useDynamicPositions) {
        LOG_INFO("RTLSLink beacon waiting for dynamic anchor positions");
    } else
#endif
    {
        App::ConfigureRtlslinkBeaconAnchors(anchors);
    }
#endif

    // Spi pins already setup on uwb_backend
    dwInit(&m_Device, &m_Ops);          // Initialize the driver. Init resets user data!
    m_Device.userdata = &m_DwData;

    int result = dwConfigure(&m_Device);      // Configure the DW1000
    if (result != 0) {
        LOG_WARN("DW1000 configuration failed, devid: %u", static_cast<uint32_t>(result));
    }
    LOG_INFO("DW1000 Configured");
    vTaskDelay(pdMS_TO_TICKS(300));

    dwEnableAllLeds(&m_Device);

    dwTime_t delay = {.full = 0};       
    dwSetAntenaDelay(&m_Device, delay); // hmmm but why?
    dwAttachSentHandler(&m_Device, txCallback);
    dwAttachReceivedHandler(&m_Device, rxCallback);
    dwAttachReceiveTimeoutHandler(&m_Device, rxTimeoutCallback);
    dwAttachReceiveFailedHandler(&m_Device, rxFailedCallback);
    dwNewConfiguration(&m_Device);
    dwSetDefaults(&m_Device);

    dw1000_radio::ApplyTdoaRadioParams(&m_Device, uwbParams);

    dwSetReceiveWaitTimeout(&m_Device, 10000);

    dwCommitConfiguration(&m_Device);

    uint32_t dev_id = dwGetDeviceId(&m_Device);
    LOG_INFO("Initialized TDoA Tag: 0x%08X", dev_id);

#ifdef ESP32S3_UWB_BOARD
    ApplyMatcherPolicy(uwbParams.tdoaMatcherPolicy);
#endif

    // Init the tdoa anchor algorithm
    uwbTdoa2TagAlgorithm.init(&m_Device, estimatorCallback);
    s_anchorModel.Configure(uwbParams);
    uwbTdoa2TagSetTofCallback(anchorModelTofCallback);

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    // Initialize dynamic anchor positioning if enabled
    s_dynamicPositionsReadyForEstimator.store(false, std::memory_order_relaxed);
    s_dynamicEstimatorReinitRequested.store(false, std::memory_order_relaxed);
    s_dynamicTransitionHoldoffUntilMs.store(0, std::memory_order_relaxed);
    if (s_useDynamicPositions) {
        DynamicAnchorConfig dynamicConfig = {
            .layout = uwbParams.anchorLayout,
            .anchorCount = 4,  // Rectangular layout uses 4 anchors
            .anchorHeight = uwbParams.anchorHeight,
            .avgSampleCount = uwbParams.distanceAvgSamples,
            .lockedMask = uwbParams.anchorPosLocked
        };
        s_dynamicCalc.init(dynamicConfig);
        s_lastPositionUpdate = 0;

        // Register callback to receive inter-anchor distances
        uwbTdoa2TagSetDistanceCallback(&UWBTagTDoA::onInterAnchorDistance);

        LOG_INFO("Dynamic anchor positioning enabled (layout=%d, height=%.2f, samples=%d)",
                 uwbParams.anchorLayout, uwbParams.anchorHeight, uwbParams.distanceAvgSamples);
    }
#endif

    attachInterrupt(digitalPinToInterrupt(uwb_config.pins.int_pin),
        tagInterruptISR, RISING);

    // Any on event needed?
    uwbTdoa2TagAlgorithm.onEvent(&m_Device, uwbEvent_t::eventTimeout);

    // Start the pos estimator task and capture its handle for direct
    // notifications from the producer (estimatorCallback).
    Scheduler::scheduler.CreateStaticTask(pos_estimator_task);
    s_estimator_task_handle = pos_estimator_task.handle;
}


void UWBTagTDoA::Update()
{
    while (isr_flag) {
        // Clear the flag before handling the interrupt to avoid losing a new
        // interrupt that arrives during dwHandleInterrupt().
        isr_flag = false;
        dwHandleInterrupt(&m_Device);
    }

    // Call libdw1000 loop
    if (m_DwData.interrupt_flags != 0) {
        TagTDoADispatcher dispatcher(this);
        dispatcher.Dispatch(static_cast<libDw1000::IsrFlags>(m_DwData.interrupt_flags));
    }

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    // Periodically update anchor positions from dynamic calculation
    maybeUpdateDynamicPositions();
#endif
}

uint8_t UWBTagTDoA::GetAnchorsSeenCount()
{
    // Filter slots by timestamp recency, not the transient `fresh` flag —
    // `fresh` is cleared by the consumer between solves so it would flap to
    // zero. A slot with timestamp_us != 0 and age <= kStaleThresholdUs has
    // a valid measurement regardless of whether the estimator has consumed
    // it yet.
    if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(10)) == pdTRUE) {
        const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
        uint8_t seen_mask = 0;
        for (const auto& slot : pair_slots) {
            if (slot.timestamp_us == 0) continue;  // never written
            if ((now_us - slot.timestamp_us) > kStaleThresholdUs) continue;
            seen_mask |= (1u << slot.anchor_a);
            seen_mask |= (1u << slot.anchor_b);
        }
        cached_anchors_seen = static_cast<uint8_t>(__builtin_popcount(seen_mask));
        xSemaphoreGive(measurements_mtx);
    }
    return cached_anchors_seen;
}

uint32_t UWBTagTDoA::GetNumberOfConnectedDevices()
{
    // Use the thread-safe anchors_seen count for consistency
    return GetAnchorsSeenCount();
}

template<libDw1000::IsrFlags TFlags>
void UWBTagTDoA::OnEvent()
{
    if constexpr (TFlags == libDw1000::RX_DONE) {
        uwbTdoa2TagAlgorithm.onEvent(&m_Device, uwbEvent_t::eventPacketReceived);
    } else if constexpr (TFlags == libDw1000::TX_DONE) {
        uwbTdoa2TagAlgorithm.onEvent(&m_Device, uwbEvent_t::eventPacketSent);
    } else if constexpr (TFlags == libDw1000::RX_TIMEOUT) {
        uwbTdoa2TagAlgorithm.onEvent(&m_Device, uwbEvent_t::eventReceiveTimeout);
    } else if constexpr (TFlags == libDw1000::RX_FAILED) {
        uwbTdoa2TagAlgorithm.onEvent(&m_Device, uwbEvent_t::eventReceiveFailed);
    }
    m_DwData.interrupt_flags &= ~TFlags;  // Clear the specific flag at the end
}

static FAST_CODE void tagInterruptISR() {
    isr_flag = true;
}

/* TODO: Move to FreeRTOS notifications */
static FAST_CODE void txCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::TX_DONE;
}

static FAST_CODE void rxCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::RX_DONE;
}

static FAST_CODE void rxTimeoutCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::RX_TIMEOUT;
}

static FAST_CODE void rxFailedCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::RX_FAILED;
}

// Constants for producer notification debouncing and consumer wake.
static constexpr uint32_t kProducerMutexTimeoutMs = 2;
static constexpr uint32_t kNotifyDebounceMs = 5;
static constexpr size_t kMinMeasurementsForNotify = 4;

static FAST_CODE void estimatorCallback(tdoaMeasurement_t* tdoa)
{
    if (tdoa == nullptr) {
        return;
    }
    if (tdoa->anchorIdA >= kNumAnchors
        || tdoa->anchorIdB >= kNumAnchors
        || tdoa->anchorIdA == tdoa->anchorIdB
        || !configured_anchor_ids[tdoa->anchorIdA]
        || !configured_anchor_ids[tdoa->anchorIdB]) {
        return;
    }

    tdoa::AnchorPair pair;
    bool reversed = false;
    if (!tdoa::CanonicalizePair(tdoa->anchorIdA, tdoa->anchorIdB, kNumAnchors, pair, reversed)) {
        return;
    }

    float canonical_tdoa = tdoa->distanceDiff;
    if (reversed) {
        canonical_tdoa = -canonical_tdoa;
    }
    const uint8_t idx = tdoa::PairIndexCanonical(pair, kNumAnchors);
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());

    // Bounded mutex wait — if the consumer is mid-solve we drop this sample
    // rather than backpressure the UWB stack. One TDoA frame missing is
    // invisible compared to the cost of stalling the producer.
    if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(kProducerMutexTimeoutMs)) != pdTRUE) {
#if TDOA_STATS_LOGGING == ENABLE
        stats_producer_dropped.fetch_add(1, std::memory_order_relaxed);
#endif
        return;
    }

    PairSlot& slot = pair_slots[idx];
    const bool was_fresh = slot.fresh;
    slot.tdoa = canonical_tdoa;
    slot.timestamp_us = now_us;
    slot.anchor_a = pair.a;
    slot.anchor_b = pair.b;
    slot.fresh = true;
    // Update fresh-pair count INSIDE the mutex so it stays serialized with
    // the slot.fresh transition. Doing this outside the mutex races against
    // the consumer's fetch_sub and can underflow the counter.
    uint32_t fresh;
    if (!was_fresh) {
        fresh = fresh_pair_count.fetch_add(1, std::memory_order_relaxed) + 1;
    } else {
        fresh = fresh_pair_count.load(std::memory_order_relaxed);
    }
    xSemaphoreGive(measurements_mtx);

#ifdef USE_RTLSLINK_BEACON_BACKEND
    App::SendTdoaMeasurement(pair.a, pair.b, canonical_tdoa, tdoa->stdDev, tdoa->solvedTimestampUs);
#endif

    recordEstimatorInputTdoa(tdoa->anchorIdA, tdoa->anchorIdB, tdoa->distanceDiff);

    // Notify the estimator task once we have enough fresh pairs and the
    // debounce window has elapsed. Prevents per-callback wake storms when
    // UWB frames arrive in tight bursts.
    if (fresh >= kMinMeasurementsForNotify && s_estimator_task_handle != nullptr) {
        uint32_t now_ms = millis();
        uint32_t last = last_notify_ms.load(std::memory_order_relaxed);
        if ((now_ms - last) >= kNotifyDebounceMs
            && last_notify_ms.compare_exchange_strong(last, now_ms,
                                                     std::memory_order_relaxed)) {
            xTaskNotifyGive(s_estimator_task_handle);
        }
    }
}

// Watchdog wake interval — keeps the task responsive when measurements stop
// arriving (so dynamic-anchor reinit checks and stats dumps still run).
static constexpr uint32_t kEstimatorWatchdogMs = 50;

static bool anchorModelTofCallback(uint8_t fromAnchor,
                                   uint8_t toAnchor,
                                   uint16_t rawDistanceTimestampUnits,
                                   uint16_t fromAntennaDelay,
                                   uint16_t toAntennaDelay,
                                   uint16_t* outDistanceTimestampUnits)
{
    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
    return s_anchorModel.ProcessInterAnchorTof(fromAnchor,
                                               toAnchor,
                                               rawDistanceTimestampUnits,
                                               fromAntennaDelay,
                                               toAntennaDelay,
                                               outDistanceTimestampUnits,
                                               uwbParams);
}

void UWBTagTDoA::ResetAnchorModel()
{
    s_anchorModel.Reset();
}

bool UWBTagTDoA::StartAnchorModelCollection()
{
    return s_anchorModel.StartCollection(Front::uwbLittleFSFront.GetParams());
}

bool UWBTagTDoA::LockAnchorModel()
{
    return s_anchorModel.Lock(Front::uwbLittleFSFront.GetParams());
}

String UWBTagTDoA::GetAnchorModelStatusJson()
{
    return anchorModelStatus();
}

String UWBTagTDoA::GetAnchorModelCollectStatusJson()
{
    return s_anchorModel.CollectStatusJson();
}

String UWBTagTDoA::ExportAnchorModelJson()
{
    return s_anchorModel.ExportJson();
}

String UWBTagTDoA::GetEstimatorStatsJson()
{
    return String();
}

void UWBTagTDoA::ResetEstimatorStats()
{
    resetEstimatorStats();
}

#ifdef ESP32S3_UWB_BOARD
void UWBTagTDoA::ApplyMatcherPolicy(uint8_t policy)
{
    const tdoaEngineMatchingAlgorithm_t matcherPolicy = matcherPolicyFromParam(policy);
    uwbTdoa2TagSetMatchingAlgorithm(matcherPolicy);
    LOG_INFO("TDoA matcher policy: %s", matcherPolicyName(matcherPolicy));
}
#endif

namespace TDoAAnchorModelCommands {
void Reset()
{
    s_anchorModel.Reset();
}

bool StartCollection()
{
    return s_anchorModel.StartCollection(Front::uwbLittleFSFront.GetParams());
}

bool Lock()
{
    return s_anchorModel.Lock(Front::uwbLittleFSFront.GetParams());
}

String StatusJson()
{
    return anchorModelStatus();
}

String CollectStatusJson()
{
    return s_anchorModel.CollectStatusJson();
}

String ExportJson()
{
    return s_anchorModel.ExportJson();
}

String EstimatorStatsJson()
{
    return String();
}

void AppendBinaryStatus(rtls::protocol::BinaryFrameBuilder<2048>& outFrame, uint8_t view)
{
    s_anchorModel.AppendBinaryStatus(outFrame, view);
}

void ResetEstimatorStats()
{
    resetEstimatorStats();
}
}

static void estimatorProcess() {
    static tdoa_estimator::PosMatrix anchors_left;
    static tdoa_estimator::PosMatrix anchors_right;
    static tdoa_estimator::DynVector tdoas;
    static tdoa_estimator::PosVector3D last_position = tdoa_estimator::PosVector3D::Zero();
    static bool first_estimation = true;
    static bool last_use_2d = true;
    static constexpr tdoa_estimator::Scalar ASSUMED_TAG_Z = 0.0f;
    static constexpr int NUM_ITERATIONS = 5;
    static constexpr size_t MIN_MEASUREMENTS = kMinMeasurementsForNotify;

    // Continuous task — block on notification until producer wakes us, or
    // until the watchdog timeout elapses (so dynamic-anchor / stats
    // bookkeeping still runs during quiet periods).
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kEstimatorWatchdogMs));

    const bool USE_2D_ESTIMATOR = (Front::uwbLittleFSFront.GetParams().use2DEstimator != 0);
    if (USE_2D_ESTIMATOR != last_use_2d) {
        first_estimation = true;
        last_use_2d = USE_2D_ESTIMATOR;
        LOG_INFO("Estimator mode changed to %s - resetting warm-start",
                 USE_2D_ESTIMATOR ? "2D" : "3D");
    }

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    // Brief holdoff right after dynamic-anchor takeover to avoid mixing old/new geometry.
    uint32_t holdoff_until_ms = s_dynamicTransitionHoldoffUntilMs.load(std::memory_order_relaxed);
    if (holdoff_until_ms != 0) {
        uint32_t now_ms = millis();
        if (static_cast<int32_t>(holdoff_until_ms - now_ms) > 0) {
            return;
        }
        s_dynamicTransitionHoldoffUntilMs.store(0, std::memory_order_relaxed);
    }

    if (UWBTagTDoA::IsDynamicPositioningEnabled()
        && s_dynamicEstimatorReinitRequested.exchange(false, std::memory_order_relaxed)) {
        first_estimation = true;
        LOG_INFO("Reinitializing estimator from dynamic anchor positions");
    }
#endif

    // Two-pass snapshot under the mutex:
    //   Pass 1: expire stale or unconfigured fresh slots (drop them) and count
    //           the remaining usable fresh slots.
    //   Pass 2: only if the count meets MIN_MEASUREMENTS, copy them out and
    //           clear their fresh flag. Otherwise leave them fresh so the
    //           next wake can accumulate to a workable batch.
    // This prevents the watchdog from silently discarding partial batches at
    // low TDMA rates.
    bool have_enough = false;
    size_t copy_count = 0;
    size_t measurement_count_for_stats = 0;
    PairSlot snapshot[kNumPairs];
    etl::array<UWBAnchorParam, kNumAnchors> anchor_snapshot = {};

    if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(20)) == pdTRUE) {
        const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
        const tdoa::MeasurementSnapshotResult snapshotResult =
            tdoa::SnapshotFreshMeasurements(pair_slots,
                                             configured_anchor_ids,
                                             now_us,
                                             kStaleThresholdUs,
                                             MIN_MEASUREMENTS,
                                             snapshot,
                                             kNumPairs);

        have_enough = snapshotResult.haveEnough;
        copy_count = snapshotResult.copied;
        measurement_count_for_stats = snapshotResult.measurementCountForStats;
        recordStaleRemoved(snapshotResult.expired);
#if TDOA_STATS_LOGGING == ENABLE
        stats_stale_removed += snapshotResult.expired;
#endif

        const uint32_t fresh_delta = snapshotResult.consumed + snapshotResult.expired;
        if (fresh_delta > 0) {
            fresh_pair_count.fetch_sub(fresh_delta, std::memory_order_relaxed);
        }
        if (have_enough) {
            anchor_snapshot = anchor_positions;
        }

        xSemaphoreGive(measurements_mtx);
    } else {
        // Couldn't grab the mutex — try again on next wake.
        return;
    }

#if TDOA_STATS_LOGGING == ENABLE
    if (copy_count < stats_min_meas_count) stats_min_meas_count = copy_count;
    if (copy_count > stats_max_meas_count) stats_max_meas_count = copy_count;
#endif

    if (have_enough) {
        anchors_left.resize(copy_count, 3);
        anchors_right.resize(copy_count, 3);
        tdoas.resize(copy_count);

        for (size_t i = 0; i < copy_count; ++i) {
            const PairSlot& s = snapshot[i];
            anchors_left.row(i) << static_cast<tdoa_estimator::Scalar>(anchor_snapshot[s.anchor_a].x),
                                   static_cast<tdoa_estimator::Scalar>(anchor_snapshot[s.anchor_a].y),
                                   static_cast<tdoa_estimator::Scalar>(anchor_snapshot[s.anchor_a].z);
            anchors_right.row(i) << static_cast<tdoa_estimator::Scalar>(anchor_snapshot[s.anchor_b].x),
                                    static_cast<tdoa_estimator::Scalar>(anchor_snapshot[s.anchor_b].y),
                                    static_cast<tdoa_estimator::Scalar>(anchor_snapshot[s.anchor_b].z);
            // Slot stores canonical tdoa = d(b) - d(a) (matching tdoaMeasurement_t's
            // "right - left" convention with right=b, left=a). The solver's
            // residual is d(L) - d(R) - tdoas(i) with L=a, R=b, so flip sign.
            tdoas(i) = -static_cast<tdoa_estimator::Scalar>(s.tdoa);
        }

        if (first_estimation) {
            tdoa_estimator::PosVector3D avg_pos = tdoa_estimator::PosVector3D::Zero();
            for (size_t i = 0; i < copy_count; ++i) {
                avg_pos += anchors_left.row(i).transpose();
                avg_pos += anchors_right.row(i).transpose();
            }
            avg_pos /= static_cast<tdoa_estimator::Scalar>(2 * copy_count);
            last_position = avg_pos;
            if (USE_2D_ESTIMATOR) {
                last_position(2) = ASSUMED_TAG_Z;
            }
            first_estimation = false;
            const char* anchor_source = "configured";
#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
            if (UWBTagTDoA::IsDynamicPositioningEnabled()) {
                anchor_source = s_dynamicPositionsReadyForEstimator.load(std::memory_order_relaxed)
                              ? "dynamic" : "configured (dynamic pending)";
            }
#endif
            LOG_INFO("Position estimator initialized at [%.2f, %.2f, %.2f] using %s anchors",
                     last_position(0), last_position(1), last_position(2), anchor_source);
        }
    }

    // === NO MUTEX: Computation and output use only local data ===
    // NR solver operates on local matrices (anchors_left, anchors_right, tdoas).
    // SendSample writes to Serial via MAVLink — decoupled from measurement writes.

    if (!have_enough) {
        recordEstimatorRejected(false, false, true, measurement_count_for_stats);
#if TDOA_STATS_LOGGING == ENABLE
        stats_samples_rejected++;
        stats_reject_insufficient++;
#endif
    } else {
        // --- Run Estimator ---
        tdoa_estimator::PosVector3D current_estimate_3d = last_position; // Use last state as starting point
        bool is_valid_estimate = false;
        float solution_rmse = 0.0f;
        int solver_iterations = 0;

        // Covariance to pass to App layer
        std::optional<std::array<float, 6>> position_covariance = std::nullopt;

        // Get configurable parameters
        const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
        const tdoa_estimator::Scalar rmseThreshold = static_cast<tdoa_estimator::Scalar>(uwbParams.rmseThreshold);
        const bool enableCovMatrix = uwbParams.enableCovMatrix != 0;

        if (USE_2D_ESTIMATOR) {
            // Prepare inputs for 2D estimator
            tdoa_estimator::PosVector2D initial_guess_2d = current_estimate_3d.head<2>();
            // Use the Z component from the current 3D state as the fixed Z for this iteration
            tdoa_estimator::Scalar fixed_z_for_estimation = current_estimate_3d(2);

            // Run 2D Newton-Raphson — bracketed for stats. Brackets are no-ops
            // when stats logging is disabled.
#if TDOA_STATS_LOGGING == ENABLE
            uint64_t solve_start_us = static_cast<uint64_t>(esp_timer_get_time());
#endif
            tdoa_estimator::SolverResult2D result = tdoa_estimator::newtonRaphson2D(
                anchors_left,
                anchors_right,
                tdoas,
                initial_guess_2d,
                fixed_z_for_estimation,
                NUM_ITERATIONS,
                1e-3f,  // convergenceThreshold
                rmseThreshold
            );
#if TDOA_STATS_LOGGING == ENABLE
            uint32_t solve_us = static_cast<uint32_t>(
                static_cast<uint64_t>(esp_timer_get_time()) - solve_start_us);
            stats_solve_count++;
            stats_solve_sum_us += solve_us;
            stats_iter_sum += static_cast<uint32_t>(result.iterations);
            if (solve_us < stats_solve_min_us) stats_solve_min_us = solve_us;
            if (solve_us > stats_solve_max_us) stats_solve_max_us = solve_us;
#endif
            solver_iterations = result.iterations;

            if (result.valid) {
                // Update only X and Y components of the 3D state vector
                current_estimate_3d.head<2>() = result.position;
                // Z component remains unchanged from the previous state in 2D mode
                is_valid_estimate = true;
                solution_rmse = result.rmse;

                // Extract covariance if valid and enabled.
                // In 2D mode, the solver only owns XY uncertainty. ArduPilot collapses
                // the MAVLink position covariance to one scalar, so we only send this
                // 2D covariance when the outgoing Z is rangefinder-assisted.
                if (enableCovMatrix && result.covarianceValid) {
                    if (usesRangefinderZ(uwbParams)) {
                        position_covariance = packRangefinderAssisted2DCovariance(result.positionCovariance);
                    }
                }
            }

        } else { // Use 3D Estimator
            // Use the full 3D vector as the initial guess
            tdoa_estimator::PosVector3D initial_guess_3d = current_estimate_3d;

#if TDOA_STATS_LOGGING == ENABLE
            uint64_t solve_start_us = static_cast<uint64_t>(esp_timer_get_time());
#endif
            tdoa_estimator::SolverResult result = tdoa_estimator::newtonRaphson(
                anchors_left,
                anchors_right,
                tdoas,
                initial_guess_3d,
                NUM_ITERATIONS,
                1e-3f,  // convergenceThreshold
                rmseThreshold
            );
#if TDOA_STATS_LOGGING == ENABLE
            uint32_t solve_us = static_cast<uint32_t>(
                static_cast<uint64_t>(esp_timer_get_time()) - solve_start_us);
            stats_solve_count++;
            stats_solve_sum_us += solve_us;
            stats_iter_sum += static_cast<uint32_t>(result.iterations);
            if (solve_us < stats_solve_min_us) stats_solve_min_us = solve_us;
            if (solve_us > stats_solve_max_us) stats_solve_max_us = solve_us;
#endif
            solver_iterations = result.iterations;

            if (result.valid) {
                // Update the full 3D state vector
                current_estimate_3d = result.position;
                is_valid_estimate = true;
                solution_rmse = result.rmse;

                // Extract covariance if valid and enabled
                if (enableCovMatrix && result.covarianceValid) {
                    position_covariance = pack3DCovariance(result.positionCovariance);
                }
            }
        }

        // --- Send Data to Application ---
        bool has_nan = current_estimate_3d.hasNaN();
        if (is_valid_estimate && !has_nan) { // Only send if the estimate is valid
            recordEstimatorAccepted(current_estimate_3d(0),
                                    current_estimate_3d(1),
                                    current_estimate_3d(2),
                                    solution_rmse,
                                    copy_count);
            App::SendSample(current_estimate_3d(0), current_estimate_3d(1), current_estimate_3d(2), position_covariance);

            // Update the persistent state for the next iteration (Warm Start)
            last_position = current_estimate_3d;

#if TDOA_STATS_LOGGING == ENABLE
            stats_samples_sent++;
#endif
        } else {
            recordEstimatorRejected(!is_valid_estimate && !has_nan, has_nan, false, copy_count);
#if TDOA_STATS_LOGGING == ENABLE
            stats_samples_rejected++;
            // Track rejection reason
            if (has_nan) {
                stats_reject_nan++;
            } else if (!is_valid_estimate) {
                stats_reject_rmse++;  // RMSE was too high in solver
            }
#endif
        }

        // --- Position Logging (timed interval) - logs regardless of validity ---
        uint64_t now_position_log = millis();
        if (now_position_log - position_last_log_time_ms >= POSITION_LOG_INTERVAL_MS) {
            const char* valid_str = (is_valid_estimate && !current_estimate_3d.hasNaN()) ? "OK" : "INVALID";
            LOG_DEBUG("Position: X=%.2f Y=%.2f Z=%.2f RMSE=%.3fm [%s]",
                      current_estimate_3d(0), current_estimate_3d(1), current_estimate_3d(2), solution_rmse, valid_str);
            position_last_log_time_ms = now_position_log;
        }
    }

    // Stats dump runs regardless of have_enough (single-writer, no mutex needed)
#if TDOA_STATS_LOGGING == ENABLE
    uint64_t now_ms = millis();
    if (now_ms - stats_last_log_time_ms >= STATS_LOG_INTERVAL_MS) {
        uint32_t prod_dropped = stats_producer_dropped.exchange(0, std::memory_order_relaxed);
        uint32_t solve_avg_us = (stats_solve_count > 0)
            ? static_cast<uint32_t>(stats_solve_sum_us / stats_solve_count) : 0;
        uint32_t iter_avg_x10 = (stats_solve_count > 0)
            ? (stats_iter_sum * 10u) / stats_solve_count : 0;  // tenths of an iter
        LOG_DEBUG("Stats: Sent=%u Rej=%u (RMSE=%u NaN=%u Insuff=%u) Meas=[%u-%u] Stale=%u ProdDrop=%u "
                  "Solve=[%u/%u/%u]us Iter=%u.%u",
                  stats_samples_sent, stats_samples_rejected,
                  stats_reject_rmse, stats_reject_nan, stats_reject_insufficient,
                  (stats_min_meas_count == UINT32_MAX) ? 0 : stats_min_meas_count,
                  stats_max_meas_count,
                  stats_stale_removed,
                  prod_dropped,
                  (stats_solve_min_us == UINT32_MAX) ? 0 : stats_solve_min_us,
                  solve_avg_us,
                  stats_solve_max_us,
                  iter_avg_x10 / 10, iter_avg_x10 % 10);
        stats_samples_sent = 0;
        stats_samples_rejected = 0;
        stats_reject_rmse = 0;
        stats_reject_nan = 0;
        stats_reject_insufficient = 0;
        stats_stale_removed = 0;
        stats_min_meas_count = UINT32_MAX;
        stats_max_meas_count = 0;
        stats_solve_count = 0;
        stats_solve_sum_us = 0;
        stats_solve_min_us = UINT32_MAX;
        stats_solve_max_us = 0;
        stats_iter_sum = 0;
        stats_last_log_time_ms = now_ms;
    }
#endif
}

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
// Include for DynamicAnchorTelemetry definition
#include "wifi/wifi_discovery.hpp"

bool UWBTagTDoA::IsDynamicPositioningEnabled() {
    return s_useDynamicPositions;
}

uint8_t UWBTagTDoA::GetDynamicAnchorPositions(DynamicAnchorTelemetry* out, uint8_t maxCount) {
    if (!s_useDynamicPositions || out == nullptr || maxCount == 0) {
        return 0;
    }

    // Try to acquire mutex with short timeout (10ms) to avoid blocking discovery
    if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(10)) != pdTRUE) {
        return 0;  // Mutex unavailable, skip this update
    }

    // Copy positions from anchor_positions array
    uint8_t count = 0;
    for (uint8_t i = 0; i < 4 && count < maxCount; i++) {
        out[count].id = i;
        out[count].x = anchor_positions[i].x;
        out[count].y = anchor_positions[i].y;
        out[count].z = anchor_positions[i].z;
        count++;
    }

    xSemaphoreGive(measurements_mtx);
    return count;
}

void UWBTagTDoA::onInterAnchorDistance(uint8_t fromAnchor, uint8_t toAnchor, uint16_t distanceTimestampUnits, uint16_t fromAntennaDelay) {
    if (!s_useDynamicPositions) {
        return;
    }

    // Look up the "to" anchor's antenna delay from previously received packets
    uint16_t toAntennaDelay = uwbTdoa2TagGetAnchorAntennaDelay(toAnchor);

    // Correct inter-anchor ToF distance: subtract both endpoints' antenna delays.
    // The raw distance includes uncorrected antenna delays from both anchors.
    // because TDoA anchors set dwSetAntenaDelay(0) in the DW1000
    int32_t corrected = static_cast<int32_t>(distanceTimestampUnits)
                      - static_cast<int32_t>(fromAntennaDelay)
                      - static_cast<int32_t>(toAntennaDelay);
    if (corrected < 0) corrected = 0;

    // Convert corrected DW1000 timestamp units to meters
    float distanceMeters = static_cast<float>(corrected) * DW1000_TIME_TO_METERS;

    // Update the calculator with the corrected distance measurement
    s_dynamicCalc.updateDistance(fromAnchor, toAnchor, distanceMeters);
}

void UWBTagTDoA::maybeUpdateDynamicPositions() {
    if (!s_useDynamicPositions) {
        return;
    }

    // Throttle updates to avoid excessive computation
    uint32_t now = millis();
    if ((now - s_lastPositionUpdate) < DYNAMIC_POS_UPDATE_INTERVAL_MS) {
        return;
    }

    // Check if we have enough data to calculate positions
    if (!s_dynamicCalc.canCalculate()) {
        return;
    }

    // Calculate new positions
    point_t newPositions[4];
    if (s_dynamicCalc.calculatePositions(newPositions, 4)) {
        bool updated_positions = false;
        bool first_dynamic_update = false;

        // CRITICAL: Lock mutex before updating shared anchor_positions
        // This prevents race conditions with estimatorProcess() which reads these values
        if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
            for (int i = 0; i < 4; i++) {
                anchor_positions[i].x = newPositions[i].x;
                anchor_positions[i].y = newPositions[i].y;
                anchor_positions[i].z = newPositions[i].z;
            }

            if (!s_dynamicPositionsReadyForEstimator.load(std::memory_order_relaxed)) {
                // Drop pre-transition measurements so the next solve uses only
                // measurements gathered after dynamic geometry is active.
                for (auto& slot : pair_slots) {
                    slot.fresh = false;
                }
                fresh_pair_count.store(0, std::memory_order_relaxed);
                first_dynamic_update = true;
            }

            xSemaphoreGive(measurements_mtx);
            updated_positions = true;
        } else {
            // Avoid tight retry loops when the estimator holds the mutex.
            s_lastPositionUpdate = now;
            return;
        }

        // First successful dynamic update should reset the estimator warm-start
        // so initialization matches measured anchor positions.
        if (updated_positions && first_dynamic_update) {
            s_dynamicPositionsReadyForEstimator.store(true, std::memory_order_relaxed);
            s_dynamicEstimatorReinitRequested.store(true, std::memory_order_relaxed);
            s_dynamicTransitionHoldoffUntilMs.store(now + DYNAMIC_REINIT_HOLDOFF_MS, std::memory_order_relaxed);
#ifdef USE_RTLSLINK_BEACON_BACKEND
            configureRtlslinkBeaconFromAnchorPositions();
#endif
            LOG_INFO("Dynamic anchor positions ready, estimator reinit scheduled");
        }

        s_lastPositionUpdate = now;

        // Log the updated positions periodically
        static uint32_t lastLogTime = 0;
        if ((now - lastLogTime) >= 2000) {  // Log every 2 seconds
            LOG_DEBUG("Dynamic Anchors: A0(%.2f,%.2f) A1(%.2f,%.2f) A2(%.2f,%.2f) A3(%.2f,%.2f)",
                      anchor_positions[0].x, anchor_positions[0].y,
                      anchor_positions[1].x, anchor_positions[1].y,
                      anchor_positions[2].x, anchor_positions[2].y,
                      anchor_positions[3].x, anchor_positions[3].y);
            lastLogTime = now;
        }
    }
}
#endif // USE_DYNAMIC_ANCHOR_POSITIONS

#endif // USE_UWB_MODE_TDOA_TAG
