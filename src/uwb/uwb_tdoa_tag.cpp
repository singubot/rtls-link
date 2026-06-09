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
#include <limits>
#include <optional>
#include <array>

#include "tdoa_newton_raphson.hpp"
#include "tdoa_robust_estimator.hpp"

#include "tag/tdoa_tag_algorithm.hpp"

#include "uwb_tdoa_tag.hpp"

#include "scheduler.hpp"
#include "app.hpp"
#include "bsp/board.hpp"
#include "dw1000_radio_config.hpp"
#include "uwb_frontend_littlefs.hpp"
#include "tdoa_anchor_model.hpp"
#include "tdoa_anchor_model_commands.hpp"
#include "tdoa_position_estimator_commands.hpp"
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
static constexpr uint8_t kDynamicAnchorCount2D = 4;
static constexpr uint8_t kDynamicAnchorCount3D = 8;

using PairSlot = tdoa::MeasurementSlot;

static constexpr size_t kMin3DMeasurementsForSolve = 7;
static constexpr uint8_t kMin3DUniqueAnchorsForSolve = 6;
static constexpr uint8_t kMin3DPlaneAnchorsPerSide = 2;
static constexpr uint64_t kMax3DBatchSpanUs = 120000;
static constexpr tdoa_estimator::Scalar kMin3DPlaneSeparationM = 0.5f;
static constexpr tdoa_estimator::Scalar kMin3DAnchorAxisSeparationM = 0.5f;
static constexpr tdoa_estimator::Scalar kMin3DGeometryHorizontalInformation = 0.25f;
static constexpr tdoa_estimator::Scalar kMin3DGeometryZInformation = 0.05f;
static constexpr tdoa_estimator::Scalar kMin3DGeometryDeterminantRatio = 3.0e-4f;
static constexpr uint8_t kMin3DAxisSpanningPairs = 1;
static constexpr tdoa_estimator::Scalar kMax3DPositionVarianceM2 = 9.0f;
static constexpr uint8_t kEstimatorModeLegacy = 0;
static constexpr uint8_t kEstimatorModeRobust = 1;
static constexpr uint8_t kEstimatorModeCompare = 2;
static constexpr uint8_t kEstimatorMode2D = 255;
static constexpr uint8_t kEstimatorDiagOff = 0;
static constexpr uint8_t kEstimatorDiagSummary = 1;
static constexpr uint8_t kEstimatorDiagRows = 2;
static constexpr uint8_t kEstimatorMaxSelectedRows = 20;
static constexpr uint8_t kEstimatorSelectedDiagCapacity = 12;
#ifdef TDOA_ESTIMATOR_JUMP_DIAG
static constexpr uint8_t kEstimatorJumpEventCapacity = 8;
static constexpr uint8_t kEstimatorJumpEventRowCapacity = 6;
static constexpr tdoa_estimator::Scalar kEstimatorJumpDeltaM = 1.0f;
static constexpr tdoa_estimator::Scalar kEstimatorJumpHorizontalDeltaM = 0.8f;
static constexpr tdoa_estimator::Scalar kEstimatorJumpVerticalDeltaM = 0.7f;
static constexpr tdoa_estimator::Scalar kEstimatorJumpVelocityMps = 8.0f;
static constexpr tdoa_estimator::Scalar kEstimatorJumpAccelMps2 = 35.0f;
static constexpr uint32_t kEstimatorJumpMinIntervalUs = 1000000;
static constexpr uint32_t kEstimatorJumpSlowSolveUs = 8000;
static constexpr uint32_t kEstimatorJumpRmseMm = 300;
static constexpr uint32_t kEstimatorJumpResidualScaleMm = 350;
#endif

enum EstimatorDiagnosticFlags : uint8_t {
    kEstimatorDiagFlagAccepted = 1u << 0,
    kEstimatorDiagFlagRobustPass = 1u << 1,
    kEstimatorDiagFlagPairSelection = 1u << 2,
    kEstimatorDiagFlagCompareMode = 1u << 3,
    kEstimatorDiagFlagFallbackLegacy = 1u << 4,
    kEstimatorDiagFlagCovarianceSent = 1u << 5,
    kEstimatorDiagFlagCovarianceInvalid = 1u << 6,
    kEstimatorDiagFlagRobustInvalid = 1u << 7,
};

#ifdef TDOA_ESTIMATOR_JUMP_DIAG
enum EstimatorJumpFlags : uint16_t {
    kEstimatorJumpFlagDelta = 1u << 0,
    kEstimatorJumpFlagVelocity = 1u << 1,
    kEstimatorJumpFlagAcceleration = 1u << 2,
    kEstimatorJumpFlagSlowSolve = 1u << 3,
    kEstimatorJumpFlagHighRmse = 1u << 4,
    kEstimatorJumpFlagHighResidualScale = 1u << 5,
    kEstimatorJumpFlagLowRows = 1u << 6,
};
#endif

static uint8_t sanitizeEstimatorMode(uint8_t mode)
{
    return mode <= kEstimatorModeCompare ? mode : kEstimatorModeRobust;
}

static uint8_t sanitizeEstimatorDiag(uint8_t diag)
{
    return diag <= kEstimatorDiagRows ? diag : kEstimatorDiagOff;
}

static uint32_t elapsedUs(uint64_t start_us)
{
    const uint64_t elapsed = static_cast<uint64_t>(esp_timer_get_time()) - start_us;
    return elapsed > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(elapsed);
}

static uint32_t metersToMillimetersUnsigned(double meters)
{
    if (!std::isfinite(meters) || meters <= 0.0) {
        return 0;
    }
    const double scaled = meters * 1000.0;
    return scaled >= static_cast<double>(UINT32_MAX)
        ? UINT32_MAX
        : static_cast<uint32_t>(scaled + 0.5);
}

static int32_t metersToMillimetersSigned(float meters)
{
    if (!std::isfinite(meters)) {
        return 0;
    }
    return rtls::protocol::MetersToMillimeters(meters);
}

static uint8_t weightToQ8(tdoa_estimator::Scalar weight)
{
    if (!std::isfinite(static_cast<double>(weight)) || weight <= 0.0f) {
        return 0;
    }
    if (weight >= 1.0f) {
        return UINT8_MAX;
    }
    return static_cast<uint8_t>((weight * 255.0f) + 0.5f);
}

static bool is3DCovarianceAcceptable(const tdoa_estimator::CovMatrix3D& covariance)
{
    for (uint8_t axis = 0; axis < 3; axis++) {
        const tdoa_estimator::Scalar variance = covariance(axis, axis);
        if (!std::isfinite(static_cast<double>(variance))
            || variance < 0.0f
            || variance > kMax3DPositionVarianceM2) {
            return false;
        }
    }
    return true;
}

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
static uint8_t dynamicAnchorCountForParams(const UWBParams& params)
{
    return params.use2DEstimator != 0
        ? kDynamicAnchorCount2D
        : kDynamicAnchorCount3D;
}
#endif

static uint8_t configuredDynamicAnchorCount()
{
#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    return dynamicAnchorCountForParams(Front::uwbLittleFSFront.GetParams());
#else
    return 0;
#endif
}

static bool anchorsAreNonCoplanar3D(etl::span<const UWBAnchorParam> anchors)
{
    if (anchors.size() < 4) {
        return false;
    }

    const UWBAnchorParam& p0 = anchors[0];
    auto dist2 = [](const UWBAnchorParam& a, const UWBAnchorParam& b) {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        const float dz = a.z - b.z;
        return dx * dx + dy * dy + dz * dz;
    };

    size_t i1 = anchors.size();
    float scale2 = 0.0f;
    for (size_t i = 1; i < anchors.size(); i++) {
        scale2 = std::max(scale2, dist2(p0, anchors[i]));
        if (dist2(p0, anchors[i]) > 1e-6f) {
            i1 = i;
            break;
        }
    }
    if (i1 == anchors.size()) {
        return false;
    }

    auto vecFromP0 = [&p0](const UWBAnchorParam& p, float& x, float& y, float& z) {
        x = p.x - p0.x;
        y = p.y - p0.y;
        z = p.z - p0.z;
    };

    float v1x = 0.0f;
    float v1y = 0.0f;
    float v1z = 0.0f;
    vecFromP0(anchors[i1], v1x, v1y, v1z);

    float nx = 0.0f;
    float ny = 0.0f;
    float nz = 0.0f;
    size_t i2 = anchors.size();
    for (size_t i = 1; i < anchors.size(); i++) {
        if (i == i1) {
            continue;
        }
        float v2x = 0.0f;
        float v2y = 0.0f;
        float v2z = 0.0f;
        vecFromP0(anchors[i], v2x, v2y, v2z);
        const float cx = v1y * v2z - v1z * v2y;
        const float cy = v1z * v2x - v1x * v2z;
        const float cz = v1x * v2y - v1y * v2x;
        const float normal2 = cx * cx + cy * cy + cz * cz;
        scale2 = std::max(scale2, dist2(p0, anchors[i]));
        if (normal2 > 1e-8f) {
            nx = cx;
            ny = cy;
            nz = cz;
            i2 = i;
            break;
        }
    }
    if (i2 == anchors.size()) {
        return false;
    }

    const float normalNorm = std::sqrt(nx * nx + ny * ny + nz * nz);
    const float scale = std::sqrt(std::max(scale2, 1.0f));
    const float tolerance = std::max(0.01f, scale * 0.001f);
    for (size_t i = 1; i < anchors.size(); i++) {
        float vx = 0.0f;
        float vy = 0.0f;
        float vz = 0.0f;
        vecFromP0(anchors[i], vx, vy, vz);
        const float distanceFromPlane = std::abs(nx * vx + ny * vy + nz * vz) / normalNorm;
        if (distanceFromPlane > tolerance) {
            return true;
        }
    }

    return false;
}

static etl::array<tdoa::MeasurementSlot, kNumPairs> pair_slots = {};
static SemaphoreHandle_t measurements_mtx = xSemaphoreCreateMutex();
static etl::array<UWBAnchorParam, kNumAnchors> anchor_positions;
static etl::array<bool, kNumAnchors> configured_anchor_ids = {};
static std::atomic<bool> s_estimatorReinitRequested{false};

#if defined(USE_DYNAMIC_ANCHOR_POSITIONS) && defined(USE_RTLSLINK_BEACON_BACKEND)
static bool configureRtlslinkBeaconFromAnchorPositions()
{
    etl::array<UWBAnchorParam, kDynamicAnchorCount3D> dynamic_anchors = {};
    uint8_t dynamic_anchor_count = 0;
    const uint8_t expected_dynamic_count = configuredDynamicAnchorCount();

    if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(50)) != pdTRUE) {
        LOG_WARN("RTLSLink beacon dynamic anchor config skipped - mutex busy");
        return false;
    }

    for (uint8_t id = 0; id < expected_dynamic_count; id++) {
        if (!configured_anchor_ids[id]) {
            continue;
        }
        dynamic_anchors[dynamic_anchor_count++] = anchor_positions[id];
    }

    xSemaphoreGive(measurements_mtx);

    if (dynamic_anchor_count == 0) {
        LOG_WARN("RTLSLink beacon dynamic anchor config skipped - no anchors");
        return false;
    }

    const bool configured = App::ConfigureRtlslinkBeaconAnchors(
        etl::span<const UWBAnchorParam>(dynamic_anchors.data(), dynamic_anchor_count));
    if (configured) {
        LOG_INFO("RTLSLink beacon configured from dynamic anchor positions (%u anchors)",
                 static_cast<unsigned int>(dynamic_anchor_count));
    }
    return configured;
}

static void clearRtlslinkBeaconDynamicAnchors()
{
    etl::array<UWBAnchorParam, 1> emptyAnchors = {};
    App::ConfigureRtlslinkBeaconAnchors(etl::span<const UWBAnchorParam>(emptyAnchors.data(), 0));
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

static void clearFreshMeasurementsLocked()
{
    for (auto& slot : pair_slots) {
        slot.fresh = false;
    }
    fresh_pair_count.store(0, std::memory_order_relaxed);
}

static bool buildStaticAnchorConfig(etl::span<const UWBAnchorParam> anchors,
                                    etl::array<UWBAnchorParam, kNumAnchors>& next_anchor_positions,
                                    etl::array<bool, kNumAnchors>& next_configured_anchor_ids,
                                    uint8_t& configuredOut)
{
    uint8_t configured = 0;
    uint8_t max_anchor_id = 0;
    for (const auto& anchor : anchors) {
        uint8_t anchorId = 0;
        if (!tdoa::ParseAnchorId(anchor.shortAddr, anchorId) || anchorId >= kNumAnchors) {
            LOG_ERROR("Rejected static anchor config: invalid short address '%c%c' (expected 0-7)",
                     anchor.shortAddr[0], anchor.shortAddr[1]);
            return false;
        }
        if (next_configured_anchor_ids[anchorId]) {
            LOG_ERROR("Rejected static anchor config: duplicate anchor id %u",
                     static_cast<unsigned int>(anchorId));
            return false;
        }
        if (!std::isfinite(anchor.x) || !std::isfinite(anchor.y) || !std::isfinite(anchor.z)) {
            LOG_ERROR("Rejected static anchor config: non-finite coordinates for anchor id %u",
                     static_cast<unsigned int>(anchorId));
            return false;
        }
        next_anchor_positions[anchorId] = anchor;
        next_configured_anchor_ids[anchorId] = true;
        max_anchor_id = std::max(max_anchor_id, anchorId);
        configured++;
    }

    if (configured == 0) {
        LOG_ERROR("Rejected static anchor config: no anchors configured");
        return false;
    }

    const uint8_t required_count = static_cast<uint8_t>(max_anchor_id + 1);
    bool contiguous = configured == required_count;
    for (uint8_t id = 0; contiguous && id < required_count; id++) {
        contiguous = next_configured_anchor_ids[id];
    }
    if (!contiguous) {
        LOG_ERROR("Rejected non-contiguous static anchor ids (configured=%u max=%u)",
                  static_cast<unsigned int>(configured),
                  static_cast<unsigned int>(max_anchor_id));
        return false;
    }

    configuredOut = configured;
    return true;
}

static bool applyStaticAnchorsLocked(etl::span<const UWBAnchorParam> anchors, uint8_t& configuredOut)
{
    etl::array<UWBAnchorParam, kNumAnchors> next_anchor_positions = {};
    etl::array<bool, kNumAnchors> next_configured_anchor_ids = {};
    if (!buildStaticAnchorConfig(anchors,
                                 next_anchor_positions,
                                 next_configured_anchor_ids,
                                 configuredOut)) {
        return false;
    }

    anchor_positions = next_anchor_positions;
    configured_anchor_ids = next_configured_anchor_ids;
    clearFreshMeasurementsLocked();
    return true;
}

// Last-notify timestamp for debouncing producer-side wakeups.
static std::atomic<uint32_t> last_notify_ms{0};

static volatile bool isr_flag = false;

// Cache for anchors_seen (used when mutex is unavailable)
static uint8_t cached_anchors_seen = 0;
static TDoAAnchorModel s_anchorModel;

struct EstimatorSelectedRowStats {
    uint8_t anchorA = 0;
    uint8_t anchorB = 0;
    uint32_t ageUs = 0;
    int32_t tdoaMm = 0;
    int32_t residualMm = 0;
    uint8_t baseWeightQ8 = 0;
    uint8_t finalWeightQ8 = 0;
};

#ifdef TDOA_ESTIMATOR_JUMP_DIAG
struct EstimatorJumpEventStats {
    uint32_t sequence = 0;
    uint32_t timestampMs = 0;
    uint16_t jumpFlags = 0;
    uint8_t mode = kEstimatorModeRobust;
    uint8_t inputRows = 0;
    uint8_t selectedRows = 0;
    uint8_t uniqueAnchors = 0;
    uint8_t iterations = 0;
    uint8_t rowCount = 0;
    uint32_t dtUs = 0;
    uint32_t solveUs = 0;
    uint32_t rmseMm = 0;
    uint32_t residualScaleMm = 0;
    uint32_t deltaMm = 0;
    uint32_t horizontalDeltaMm = 0;
    int32_t verticalDeltaMm = 0;
    uint32_t speedMmps = 0;
    uint32_t accelMmps2 = 0;
    int32_t prevXMm = 0;
    int32_t prevYMm = 0;
    int32_t prevZMm = 0;
    int32_t candidateXMm = 0;
    int32_t candidateYMm = 0;
    int32_t candidateZMm = 0;
    EstimatorSelectedRowStats rows[kEstimatorJumpEventRowCapacity] = {};
};
#endif

struct EstimatorSolveStats {
    uint8_t mode = kEstimatorModeRobust;
    uint8_t diagLevel = kEstimatorDiagOff;
    uint8_t flags = 0;
    uint8_t inputRows = 0;
    uint8_t selectedRows = 0;
    uint8_t uniqueAnchors = 0;
    uint8_t iterations = 0;
    uint32_t solveUs = 0;
    uint32_t legacySolveUs = 0;
    uint32_t robustSolveUs = 0;
    uint32_t rmseMm = 0;
    uint32_t residualScaleMm = 0;
    int32_t xMm = 0;
    int32_t yMm = 0;
    int32_t zMm = 0;
    uint32_t compareDeltaMm = 0;
    uint32_t legacyRmseMm = 0;
    uint32_t robustRmseMm = 0;
    uint8_t selectedDiagCount = 0;
    EstimatorSelectedRowStats selected[kEstimatorSelectedDiagCapacity] = {};
};

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
    uint32_t solveCount = 0;
    uint64_t solveSumUs = 0;
    uint32_t solveMinUs = UINT32_MAX;
    uint32_t solveMaxUs = 0;
    uint32_t compareRuns = 0;
    uint32_t compareFallbackLegacy = 0;
    uint32_t compareRobustInvalid = 0;
    EstimatorSolveStats lastSolve = {};
#ifdef TDOA_ESTIMATOR_JUMP_DIAG
    uint32_t jumpEventsTotal = 0;
    uint8_t jumpEventHead = 0;
    uint8_t jumpEventCount = 0;
    EstimatorJumpEventStats jumpEvents[kEstimatorJumpEventCapacity] = {};
#endif
};

static EstimatorStats* s_estimatorStats = nullptr;
static SemaphoreHandle_t estimator_stats_mtx = nullptr;
static std::atomic<uint32_t> s_estimatorProducerDroppedTotal{0};
static bool copyEstimatorStats(EstimatorStats& outStats);

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

static uint8_t clampToU8(size_t value)
{
    return value > UINT8_MAX ? UINT8_MAX : static_cast<uint8_t>(value);
}

static void recordEstimatorSolveStats(const EstimatorSolveStats& solveStats)
{
    ensureEstimatorStatsMutex();
    if (s_estimatorStats == nullptr || estimator_stats_mtx == nullptr || xSemaphoreTake(estimator_stats_mtx, pdMS_TO_TICKS(2)) != pdTRUE) {
        return;
    }

    s_estimatorStats->lastSolve = solveStats;
    if (solveStats.solveUs > 0) {
        s_estimatorStats->solveCount++;
        s_estimatorStats->solveSumUs += solveStats.solveUs;
        if (solveStats.solveUs < s_estimatorStats->solveMinUs) {
            s_estimatorStats->solveMinUs = solveStats.solveUs;
        }
        if (solveStats.solveUs > s_estimatorStats->solveMaxUs) {
            s_estimatorStats->solveMaxUs = solveStats.solveUs;
        }
    }
    if ((solveStats.flags & kEstimatorDiagFlagCompareMode) != 0) {
        s_estimatorStats->compareRuns++;
    }
    if ((solveStats.flags & kEstimatorDiagFlagFallbackLegacy) != 0) {
        s_estimatorStats->compareFallbackLegacy++;
    }
    if ((solveStats.flags & kEstimatorDiagFlagRobustInvalid) != 0) {
        s_estimatorStats->compareRobustInvalid++;
    }

    xSemaphoreGive(estimator_stats_mtx);
}

#ifdef TDOA_ESTIMATOR_JUMP_DIAG
static void recordEstimatorJumpEvent(const EstimatorJumpEventStats& event)
{
    ensureEstimatorStatsMutex();
    if (s_estimatorStats == nullptr || estimator_stats_mtx == nullptr || xSemaphoreTake(estimator_stats_mtx, pdMS_TO_TICKS(2)) != pdTRUE) {
        return;
    }

    EstimatorJumpEventStats next = event;
    next.sequence = ++s_estimatorStats->jumpEventsTotal;
    s_estimatorStats->jumpEvents[s_estimatorStats->jumpEventHead] = next;
    s_estimatorStats->jumpEventHead =
        static_cast<uint8_t>((s_estimatorStats->jumpEventHead + 1) % kEstimatorJumpEventCapacity);
    if (s_estimatorStats->jumpEventCount < kEstimatorJumpEventCapacity) {
        s_estimatorStats->jumpEventCount++;
    }

    xSemaphoreGive(estimator_stats_mtx);
}

static uint8_t eventIndexFromOldest(const EstimatorStats& stats, uint8_t offset)
{
    const uint8_t oldest = stats.jumpEventCount < kEstimatorJumpEventCapacity
        ? 0
        : stats.jumpEventHead;
    return static_cast<uint8_t>((oldest + offset) % kEstimatorJumpEventCapacity);
}

static uint32_t timerMs32()
{
    const uint64_t ms = static_cast<uint64_t>(esp_timer_get_time()) / 1000u;
    return ms > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(ms);
}

static void appendEstimatorJumpEvents(rtls::protocol::BinaryFrameBuilder<2048>& outFrame)
{
    EstimatorStats stats;
    if (!copyEstimatorStats(stats)) {
        outFrame.Begin(rtls::protocol::FrameType::TdoaPositionEstimatorEvents,
                       rtls::protocol::StatusCode::Error);
        outFrame.AppendString("estimator stats unavailable");
        outFrame.Finish();
        return;
    }

    outFrame.Begin(rtls::protocol::FrameType::TdoaPositionEstimatorEvents);
    outFrame.AppendU8(1); // payload version
    outFrame.AppendU8(kEstimatorJumpEventCapacity);
    outFrame.AppendU8(kEstimatorJumpEventRowCapacity);
    outFrame.AppendU8(stats.jumpEventCount);
    outFrame.AppendU32(stats.jumpEventsTotal);

    for (uint8_t i = 0; i < stats.jumpEventCount; i++) {
        const EstimatorJumpEventStats& event = stats.jumpEvents[eventIndexFromOldest(stats, i)];
        outFrame.AppendU32(event.sequence);
        outFrame.AppendU32(event.timestampMs);
        outFrame.AppendU16(event.jumpFlags);
        outFrame.AppendU8(event.mode);
        outFrame.AppendU8(event.inputRows);
        outFrame.AppendU8(event.selectedRows);
        outFrame.AppendU8(event.uniqueAnchors);
        outFrame.AppendU8(event.iterations);
        outFrame.AppendU8(event.rowCount);
        outFrame.AppendU32(event.dtUs);
        outFrame.AppendU32(event.solveUs);
        outFrame.AppendU32(event.rmseMm);
        outFrame.AppendU32(event.residualScaleMm);
        outFrame.AppendU32(event.deltaMm);
        outFrame.AppendU32(event.horizontalDeltaMm);
        outFrame.AppendI32(event.verticalDeltaMm);
        outFrame.AppendU32(event.speedMmps);
        outFrame.AppendU32(event.accelMmps2);
        outFrame.AppendI32(event.prevXMm);
        outFrame.AppendI32(event.prevYMm);
        outFrame.AppendI32(event.prevZMm);
        outFrame.AppendI32(event.candidateXMm);
        outFrame.AppendI32(event.candidateYMm);
        outFrame.AppendI32(event.candidateZMm);
        const uint8_t rowCount = std::min<uint8_t>(event.rowCount, kEstimatorJumpEventRowCapacity);
        for (uint8_t row = 0; row < rowCount; row++) {
            const EstimatorSelectedRowStats& src = event.rows[row];
            outFrame.AppendU8(src.anchorA);
            outFrame.AppendU8(src.anchorB);
            outFrame.AppendU32(src.ageUs);
            outFrame.AppendI32(src.tdoaMm);
            outFrame.AppendI32(src.residualMm);
            outFrame.AppendU8(src.baseWeightQ8);
            outFrame.AppendU8(src.finalWeightQ8);
        }
    }

    outFrame.Finish();
}
#else
static void appendEstimatorJumpEvents(rtls::protocol::BinaryFrameBuilder<2048>& outFrame)
{
    outFrame.Begin(rtls::protocol::FrameType::TdoaPositionEstimatorEvents,
                   rtls::protocol::StatusCode::Error);
    outFrame.AppendString("TDoA estimator jump diagnostics disabled");
    outFrame.Finish();
}
#endif

static void recordMeasurementCountLocked(size_t measurementCount)
{
    const uint8_t clampedCount = clampToU8(measurementCount);
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
    s_estimatorStats->lastMeasurementCount = clampToU8(measurementCount);
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
    s_estimatorStats->lastMeasurementCount = clampToU8(measurementCount);
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
    s_estimatorProducerDroppedTotal.store(0, std::memory_order_relaxed);

    xSemaphoreGive(estimator_stats_mtx);
}

static bool copyEstimatorStats(EstimatorStats& outStats)
{
    ensureEstimatorStatsMutex();
    if (s_estimatorStats == nullptr || estimator_stats_mtx == nullptr || xSemaphoreTake(estimator_stats_mtx, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    outStats = *s_estimatorStats;
    xSemaphoreGive(estimator_stats_mtx);
    return true;
}

static String positionEstimatorStatsJson()
{
    EstimatorStats stats;
    if (!copyEstimatorStats(stats)) {
        return String("{\"success\":false,\"error\":\"estimator stats unavailable\"}");
    }

    const uint32_t dropped = s_estimatorProducerDroppedTotal.load(std::memory_order_relaxed);
    const uint32_t solveAvgUs = stats.solveCount > 0
        ? static_cast<uint32_t>(stats.solveSumUs / stats.solveCount)
        : 0;

    String result;
    result.reserve(384);
    result += "{\"success\":true";
    result += ",\"mode\":";
    result += static_cast<unsigned int>(stats.lastSolve.mode);
    result += ",\"diag\":";
    result += static_cast<unsigned int>(stats.lastSolve.diagLevel);
    result += ",\"flags\":";
    result += static_cast<unsigned int>(stats.lastSolve.flags);
    result += ",\"sent\":";
    result += static_cast<unsigned long>(stats.samplesSent);
    result += ",\"rejected\":";
    result += static_cast<unsigned long>(stats.samplesRejected);
    result += ",\"producerDropped\":";
    result += static_cast<unsigned long>(dropped);
    result += ",\"lastRows\":";
    result += static_cast<unsigned int>(stats.lastSolve.inputRows);
    result += ",\"selectedRows\":";
    result += static_cast<unsigned int>(stats.lastSolve.selectedRows);
    result += ",\"lastRmseMm\":";
    result += static_cast<unsigned long>(stats.lastSolve.rmseMm);
    result += ",\"lastSolveUs\":";
    result += static_cast<unsigned long>(stats.lastSolve.solveUs);
    result += ",\"solveAvgUs\":";
    result += static_cast<unsigned long>(solveAvgUs);
    result += ",\"compareRuns\":";
    result += static_cast<unsigned long>(stats.compareRuns);
    result += ",\"compareFallbackLegacy\":";
    result += static_cast<unsigned long>(stats.compareFallbackLegacy);
    result += ",\"compareRobustInvalid\":";
    result += static_cast<unsigned long>(stats.compareRobustInvalid);
    result += "}";
    return result;
}

static void appendPositionEstimatorStatus(rtls::protocol::BinaryFrameBuilder<2048>& outFrame)
{
    EstimatorStats stats;
    if (!copyEstimatorStats(stats)) {
        outFrame.Begin(rtls::protocol::FrameType::TdoaPositionEstimatorStatus,
                       rtls::protocol::StatusCode::Error);
        outFrame.AppendString("estimator stats unavailable");
        outFrame.Finish();
        return;
    }

    const EstimatorSolveStats& solve = stats.lastSolve;
    const uint32_t dropped = s_estimatorProducerDroppedTotal.load(std::memory_order_relaxed);
    const uint32_t solveAvgUs = stats.solveCount > 0
        ? static_cast<uint32_t>(stats.solveSumUs / stats.solveCount)
        : 0;

    outFrame.Begin(rtls::protocol::FrameType::TdoaPositionEstimatorStatus);
    outFrame.AppendU8(solve.mode);
    outFrame.AppendU8(solve.diagLevel);
    outFrame.AppendU8(solve.flags);
    outFrame.AppendU8(solve.inputRows);
    outFrame.AppendU8(solve.selectedRows);
    outFrame.AppendU8(solve.uniqueAnchors);
    outFrame.AppendU8(solve.iterations);
    outFrame.AppendU8(0); // reserved
    outFrame.AppendU32(stats.samplesSent);
    outFrame.AppendU32(stats.samplesRejected);
    outFrame.AppendU32(stats.rejectRmse);
    outFrame.AppendU32(stats.rejectNan);
    outFrame.AppendU32(stats.rejectInsufficient);
    outFrame.AppendU32(stats.staleRemoved);
    outFrame.AppendU32(dropped);
    outFrame.AppendU32(stats.solveCount);
    outFrame.AppendU32(stats.solveMinUs == UINT32_MAX ? 0 : stats.solveMinUs);
    outFrame.AppendU32(solveAvgUs);
    outFrame.AppendU32(stats.solveMaxUs);
    outFrame.AppendU32(solve.solveUs);
    outFrame.AppendU32(solve.legacySolveUs);
    outFrame.AppendU32(solve.robustSolveUs);
    outFrame.AppendU32(solve.rmseMm);
    outFrame.AppendU32(solve.residualScaleMm);
    outFrame.AppendI32(solve.xMm);
    outFrame.AppendI32(solve.yMm);
    outFrame.AppendI32(solve.zMm);
    outFrame.AppendU32(stats.compareRuns);
    outFrame.AppendU32(stats.compareFallbackLegacy);
    outFrame.AppendU32(stats.compareRobustInvalid);
    outFrame.AppendU32(solve.legacyRmseMm);
    outFrame.AppendU32(solve.robustRmseMm);
    outFrame.AppendU32(solve.compareDeltaMm);
    const uint8_t statusDiagCount = solve.diagLevel >= kEstimatorDiagRows
        ? solve.selectedDiagCount
        : 0;
    outFrame.AppendU8(statusDiagCount);
    for (uint8_t i = 0; i < statusDiagCount; i++) {
        const EstimatorSelectedRowStats& row = solve.selected[i];
        outFrame.AppendU8(row.anchorA);
        outFrame.AppendU8(row.anchorB);
        outFrame.AppendI32(row.residualMm);
        outFrame.AppendU8(row.baseWeightQ8);
        outFrame.AppendU8(row.finalWeightQ8);
    }
    outFrame.Finish();
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
static SemaphoreHandle_t s_dynamicCalcMutex = nullptr;
// Coordination flags between dynamic-anchor updates and estimator task
static std::atomic<bool> s_dynamicPositionsReadyForEstimator{false};
static std::atomic<bool> s_dynamicEstimatorReinitRequested{false};
static std::atomic<uint32_t> s_dynamicTransitionHoldoffUntilMs{0};

// Interval for dynamic position updates (ms)
static constexpr uint32_t DYNAMIC_POS_UPDATE_INTERVAL_MS = 200;
static constexpr uint32_t DYNAMIC_REINIT_HOLDOFF_MS = 300;

static bool takeDynamicCalcMutex(TickType_t timeout)
{
    if (s_dynamicCalcMutex == nullptr) {
        s_dynamicCalcMutex = xSemaphoreCreateMutex();
    }
    return s_dynamicCalcMutex != nullptr
        && xSemaphoreTake(s_dynamicCalcMutex, timeout) == pdTRUE;
}

static void giveDynamicCalcMutex()
{
    if (s_dynamicCalcMutex != nullptr) {
        xSemaphoreGive(s_dynamicCalcMutex);
    }
}

void UWBTagTDoA::ClearDynamicAnchorRuntimeState(bool resetCalculator, bool clearLiveAnchors)
{
    s_dynamicPositionsReadyForEstimator.store(false, std::memory_order_relaxed);
    s_dynamicEstimatorReinitRequested.store(false, std::memory_order_relaxed);
    s_dynamicTransitionHoldoffUntilMs.store(0, std::memory_order_relaxed);
    s_lastPositionUpdate = 0;

    if (resetCalculator && takeDynamicCalcMutex(pdMS_TO_TICKS(50))) {
        s_dynamicCalc.reset();
        giveDynamicCalcMutex();
    }

    if (clearLiveAnchors
        && measurements_mtx != nullptr
        && xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (uint8_t i = 0; i < kNumAnchors; i++) {
            configured_anchor_ids[i] = false;
            anchor_positions[i] = {};
        }
        clearFreshMeasurementsLocked();
        xSemaphoreGive(measurements_mtx);
    }

#ifdef USE_RTLSLINK_BEACON_BACKEND
    if (clearLiveAnchors) {
        clearRtlslinkBeaconDynamicAnchors();
    }
#endif
}
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

    // Get UWB radio/settings parameters before deciding which anchor geometry to publish.
    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    s_useDynamicPositions = (uwbParams.dynamicAnchorPosEnabled != 0);
#endif

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    if (!s_useDynamicPositions) {
        ApplyStaticAnchors(anchors);
    } else {
        LOG_INFO("Dynamic anchor positioning enabled; waiting for calculated geometry");
    }
#else
    ApplyStaticAnchors(anchors);
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
            .anchorCount = dynamicAnchorCountForParams(uwbParams),
            .anchorHeight = uwbParams.anchorHeight,
            .anchorPlaneSeparation = uwbParams.anchorPlaneSeparation,
            .avgSampleCount = uwbParams.distanceAvgSamples,
            .lockedMask = uwbParams.anchorPosLocked
        };
        if (takeDynamicCalcMutex(pdMS_TO_TICKS(50))) {
            s_dynamicCalc.init(dynamicConfig);
            giveDynamicCalcMutex();
        } else {
            LOG_ERROR("Dynamic anchor calculator init skipped - mutex unavailable");
        }
        s_lastPositionUpdate = 0;

        // Register callback to receive inter-anchor distances
        uwbTdoa2TagSetDistanceCallback(&UWBTagTDoA::onInterAnchorDistance);

        LOG_INFO("Dynamic anchor positioning enabled (layout=%d, anchors=%u, height=%.2f, planeSep=%.2f, samples=%d)",
                 uwbParams.anchorLayout,
                 static_cast<unsigned int>(dynamicConfig.anchorCount),
                 uwbParams.anchorHeight,
                 uwbParams.anchorPlaneSeparation,
                 uwbParams.distanceAvgSamples);
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

void UWBTagTDoA::SetEnabled(bool enabled)
{
    if (enabled) {
        return;
    }

    isr_flag = false;
    m_DwData.interrupt_flags = 0;

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    ClearDynamicAnchorRuntimeState(true, true);
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
        || tdoa->anchorIdA == tdoa->anchorIdB) {
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
    const uint64_t measurement_time_us =
        (tdoa->solvedTimestampUs != 0 && tdoa->solvedTimestampUs <= now_us)
            ? tdoa->solvedTimestampUs
            : now_us;

    // Bounded mutex wait — if the consumer is mid-solve we drop this sample
    // rather than backpressure the UWB stack. One TDoA frame missing is
    // invisible compared to the cost of stalling the producer.
    if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(kProducerMutexTimeoutMs)) != pdTRUE) {
        s_estimatorProducerDroppedTotal.fetch_add(1, std::memory_order_relaxed);
#if TDOA_STATS_LOGGING == ENABLE
        stats_producer_dropped.fetch_add(1, std::memory_order_relaxed);
#endif
        return;
    }

    if (!configured_anchor_ids[pair.a] || !configured_anchor_ids[pair.b]) {
        xSemaphoreGive(measurements_mtx);
        return;
    }

    PairSlot& slot = pair_slots[idx];
    const bool was_fresh = slot.fresh;
    slot.tdoa = canonical_tdoa;
    slot.timestamp_us = measurement_time_us;
    slot.anchor_a = pair.a;
    slot.anchor_b = pair.b;
    slot.fresh = true;
    slot.sigma_m = (std::isfinite(tdoa->stdDev) && tdoa->stdDev > 0.0f)
        ? tdoa->stdDev
        : 0.15f;
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

bool UWBTagTDoA::ApplyStaticAnchors(etl::span<const UWBAnchorParam> anchors)
{
    if (measurements_mtx == nullptr) {
        return false;
    }

    if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(50)) != pdTRUE) {
        LOG_WARN("Static anchor config skipped - measurement mutex busy");
        return false;
    }

    uint8_t configured = 0;
    const bool applied = applyStaticAnchorsLocked(anchors, configured);
    xSemaphoreGive(measurements_mtx);

    if (!applied) {
        return false;
    }

    s_estimatorReinitRequested.store(true, std::memory_order_relaxed);
    LOG_INFO("Static anchor config applied to estimator (%u anchors)",
             static_cast<unsigned int>(configured));
    return true;
}

bool UWBTagTDoA::ValidateStaticAnchors(etl::span<const UWBAnchorParam> anchors)
{
    etl::array<UWBAnchorParam, kNumAnchors> next_anchor_positions = {};
    etl::array<bool, kNumAnchors> next_configured_anchor_ids = {};
    uint8_t configured = 0;
    return buildStaticAnchorConfig(anchors,
                                   next_anchor_positions,
                                   next_configured_anchor_ids,
                                   configured);
}

bool UWBTagTDoA::ValidateStaticAnchorsForEstimator(etl::span<const UWBAnchorParam> anchors,
                                                   bool use2DEstimator)
{
    if (!ValidateStaticAnchors(anchors)) {
        return false;
    }
    if (use2DEstimator && anchors.size() < 4) {
        LOG_ERROR("Rejected static anchor config: 2D estimator requires at least 4 anchors");
        return false;
    }
    if (!use2DEstimator && anchors.size() < kMin3DUniqueAnchorsForSolve) {
        LOG_ERROR("Rejected static anchor config: 3D estimator requires at least %u anchors",
                  static_cast<unsigned int>(kMin3DUniqueAnchorsForSolve));
        return false;
    }
    if (!use2DEstimator && !anchorsAreNonCoplanar3D(anchors)) {
        LOG_ERROR("Rejected static anchor config: 3D estimator requires non-coplanar anchors");
        return false;
    }
    return true;
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
    return positionEstimatorStatsJson();
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

namespace TDoAPositionEstimatorCommands {

String StatusJson()
{
    return positionEstimatorStatsJson();
}

void AppendBinaryStatus(rtls::protocol::BinaryFrameBuilder<2048>& outFrame)
{
    appendPositionEstimatorStatus(outFrame);
}

void AppendBinaryEvents(rtls::protocol::BinaryFrameBuilder<2048>& outFrame)
{
    appendEstimatorJumpEvents(outFrame);
}

void ResetStats()
{
    resetEstimatorStats();
}

}

static bool is3DCovarianceUsable(const tdoa_estimator::SolverResult& result)
{
    return result.covarianceValid && is3DCovarianceAcceptable(result.positionCovariance);
}

static bool is3DResultAcceptable(const tdoa_estimator::SolverResult& result, bool requireCovariance)
{
    return result.valid
        && result.converged
        && (!requireCovariance || is3DCovarianceUsable(result));
}

static tdoa_estimator::RobustEstimatorOptions makeRobustOptions(
    tdoa_estimator::Scalar rmseThreshold,
    uint8_t maxIterations)
{
    tdoa_estimator::RobustEstimatorOptions options;
    options.min_rows = static_cast<uint8_t>(kMin3DMeasurementsForSolve);
    options.min_unique_anchors = kMin3DUniqueAnchorsForSolve;
    options.max_selected_rows = kEstimatorMaxSelectedRows;
    options.max_iterations = maxIterations;
    options.convergence_threshold = 1e-3f;
    options.rmse_threshold = rmseThreshold;
    options.enable_pair_selection = true;
    options.enable_robust_pass = true;
    options.reference_sigma_m = 0.15f;
    return options;
}

static uint32_t positionDeltaMm(const tdoa_estimator::PosVector3D& a,
                                const tdoa_estimator::PosVector3D& b)
{
    return metersToMillimetersUnsigned((a - b).norm());
}

struct EstimatorGeometryStats {
    uint8_t uniqueAnchors = 0;
    uint8_t lowPlaneAnchors = 0;
    uint8_t highPlaneAnchors = 0;
    uint8_t xSpanningPairs = 0;
    uint8_t ySpanningPairs = 0;
    tdoa_estimator::Scalar xInformation = 0.0f;
    tdoa_estimator::Scalar yInformation = 0.0f;
    tdoa_estimator::Scalar zInformation = 0.0f;
    tdoa_estimator::Scalar determinantRatio = 0.0f;
};

static tdoa_estimator::PosVector3D anchorPositionVector(const UWBAnchorParam& anchor)
{
    tdoa_estimator::PosVector3D position;
    position << static_cast<tdoa_estimator::Scalar>(anchor.x),
                static_cast<tdoa_estimator::Scalar>(anchor.y),
                static_cast<tdoa_estimator::Scalar>(anchor.z);
    return position;
}

static EstimatorGeometryStats evaluate3DGeometry(const PairSlot* rows,
                                                 size_t rowCount,
                                                 const etl::array<UWBAnchorParam, kNumAnchors>& anchors,
                                                 const tdoa_estimator::PosVector3D& referencePosition)
{
    EstimatorGeometryStats stats;
    if (rows == nullptr || rowCount == 0) {
        return stats;
    }

    bool anchorSeen[kNumAnchors] = {};
    tdoa_estimator::Scalar minZ = std::numeric_limits<tdoa_estimator::Scalar>::max();
    tdoa_estimator::Scalar maxZ = -std::numeric_limits<tdoa_estimator::Scalar>::max();
    Eigen::Matrix<tdoa_estimator::Scalar, 3, 3> information =
        Eigen::Matrix<tdoa_estimator::Scalar, 3, 3>::Zero();

    for (size_t i = 0; i < rowCount; i++) {
        const PairSlot& row = rows[i];
        if (row.anchor_a >= kNumAnchors || row.anchor_b >= kNumAnchors) {
            continue;
        }

        const UWBAnchorParam& anchorParamsA = anchors[row.anchor_a];
        const UWBAnchorParam& anchorParamsB = anchors[row.anchor_b];
        if (std::fabs(static_cast<tdoa_estimator::Scalar>(anchorParamsA.x - anchorParamsB.x))
            >= kMin3DAnchorAxisSeparationM) {
            stats.xSpanningPairs++;
        }
        if (std::fabs(static_cast<tdoa_estimator::Scalar>(anchorParamsA.y - anchorParamsB.y))
            >= kMin3DAnchorAxisSeparationM) {
            stats.ySpanningPairs++;
        }

        if (!anchorSeen[row.anchor_a]) {
            anchorSeen[row.anchor_a] = true;
            stats.uniqueAnchors++;
            minZ = std::min(minZ, static_cast<tdoa_estimator::Scalar>(anchorParamsA.z));
            maxZ = std::max(maxZ, static_cast<tdoa_estimator::Scalar>(anchorParamsA.z));
        }
        if (!anchorSeen[row.anchor_b]) {
            anchorSeen[row.anchor_b] = true;
            stats.uniqueAnchors++;
            minZ = std::min(minZ, static_cast<tdoa_estimator::Scalar>(anchorParamsB.z));
            maxZ = std::max(maxZ, static_cast<tdoa_estimator::Scalar>(anchorParamsB.z));
        }

        const tdoa_estimator::PosVector3D anchorA = anchorPositionVector(anchorParamsA);
        const tdoa_estimator::PosVector3D anchorB = anchorPositionVector(anchorParamsB);
        tdoa_estimator::Scalar distanceA = (referencePosition - anchorA).norm();
        tdoa_estimator::Scalar distanceB = (referencePosition - anchorB).norm();
        if (distanceA < tdoa_estimator::Scalar(1.0e-4f)) {
            distanceA = tdoa_estimator::Scalar(1.0e-4f);
        }
        if (distanceB < tdoa_estimator::Scalar(1.0e-4f)) {
            distanceB = tdoa_estimator::Scalar(1.0e-4f);
        }

        const tdoa_estimator::PosVector3D gradient =
            ((referencePosition - anchorA) / distanceA)
            - ((referencePosition - anchorB) / distanceB);
        information += gradient * gradient.transpose();
    }

    if (stats.uniqueAnchors == 0
        || !std::isfinite(static_cast<double>(minZ))
        || !std::isfinite(static_cast<double>(maxZ))) {
        return stats;
    }

    const tdoa_estimator::Scalar planeSeparation = maxZ - minZ;
    if (planeSeparation >= kMin3DPlaneSeparationM) {
        const tdoa_estimator::Scalar midZ = (minZ + maxZ) * tdoa_estimator::Scalar(0.5f);
        for (uint8_t i = 0; i < kNumAnchors; i++) {
            if (!anchorSeen[i]) {
                continue;
            }
            if (static_cast<tdoa_estimator::Scalar>(anchors[i].z) < midZ) {
                stats.lowPlaneAnchors++;
            } else {
                stats.highPlaneAnchors++;
            }
        }
    }

    const tdoa_estimator::Scalar trace = information.trace();
    if (trace > tdoa_estimator::Scalar(1.0e-6f)) {
        const tdoa_estimator::Scalar determinant =
            information(0, 0) * ((information(1, 1) * information(2, 2)) - (information(1, 2) * information(2, 1)))
            - information(0, 1) * ((information(1, 0) * information(2, 2)) - (information(1, 2) * information(2, 0)))
            + information(0, 2) * ((information(1, 0) * information(2, 1)) - (information(1, 1) * information(2, 0)));
        if (std::isfinite(static_cast<double>(determinant)) && determinant > tdoa_estimator::Scalar(0)) {
            stats.determinantRatio = determinant / (trace * trace * trace);
        }
    }
    stats.xInformation = information(0, 0);
    stats.yInformation = information(1, 1);
    stats.zInformation = information(2, 2);
    return stats;
}

static bool is3DGeometryAcceptable(const EstimatorGeometryStats& stats)
{
    return stats.uniqueAnchors >= kMin3DUniqueAnchorsForSolve
        && stats.lowPlaneAnchors >= kMin3DPlaneAnchorsPerSide
        && stats.highPlaneAnchors >= kMin3DPlaneAnchorsPerSide
        && stats.xSpanningPairs >= kMin3DAxisSpanningPairs
        && stats.ySpanningPairs >= kMin3DAxisSpanningPairs
        && stats.xInformation >= kMin3DGeometryHorizontalInformation
        && stats.yInformation >= kMin3DGeometryHorizontalInformation
        && stats.zInformation >= kMin3DGeometryZInformation
        && stats.determinantRatio >= kMin3DGeometryDeterminantRatio;
}

#ifdef TDOA_ESTIMATOR_JUMP_DIAG
static void maybeRecordEstimatorJumpEvent(const EstimatorSolveStats& solveStats,
                                          const tdoa_estimator::PosVector3D& previousPosition,
                                          const tdoa_estimator::PosVector3D& candidatePosition,
                                          uint64_t previousTimeUs,
                                          tdoa_estimator::Scalar previousSpeedMps,
                                          tdoa_estimator::Scalar& outCurrentSpeedMps)
{
    static uint64_t lastCaptureUs = 0;
    outCurrentSpeedMps = previousSpeedMps;
    if (previousTimeUs == 0) {
        return;
    }

    const uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());
    const uint64_t dtUs64 = nowUs > previousTimeUs ? nowUs - previousTimeUs : 0;
    if (dtUs64 == 0) {
        return;
    }

    const tdoa_estimator::PosVector3D delta = candidatePosition - previousPosition;
    const tdoa_estimator::Scalar deltaNorm = delta.norm();
    const tdoa_estimator::Scalar horizontalDelta =
        std::sqrt((delta(0) * delta(0)) + (delta(1) * delta(1)));
    const tdoa_estimator::Scalar dtSeconds =
        static_cast<tdoa_estimator::Scalar>(dtUs64) / static_cast<tdoa_estimator::Scalar>(1000000);
    const tdoa_estimator::Scalar speedMps = dtSeconds > tdoa_estimator::Scalar(0)
        ? deltaNorm / dtSeconds
        : tdoa_estimator::Scalar(0);
    const tdoa_estimator::Scalar accelMps2 = dtSeconds > tdoa_estimator::Scalar(0)
        ? std::fabs(speedMps - previousSpeedMps) / dtSeconds
        : tdoa_estimator::Scalar(0);
    outCurrentSpeedMps = speedMps;

    uint16_t flags = 0;
    if (deltaNorm >= kEstimatorJumpDeltaM
        || horizontalDelta >= kEstimatorJumpHorizontalDeltaM
        || std::fabs(delta(2)) >= kEstimatorJumpVerticalDeltaM) {
        flags |= kEstimatorJumpFlagDelta;
    }
    if (speedMps >= kEstimatorJumpVelocityMps) {
        flags |= kEstimatorJumpFlagVelocity;
    }
    if (accelMps2 >= kEstimatorJumpAccelMps2) {
        flags |= kEstimatorJumpFlagAcceleration;
    }
    if (solveStats.solveUs >= kEstimatorJumpSlowSolveUs) {
        flags |= kEstimatorJumpFlagSlowSolve;
    }
    if (solveStats.rmseMm >= kEstimatorJumpRmseMm) {
        flags |= kEstimatorJumpFlagHighRmse;
    }
    if (solveStats.residualScaleMm >= kEstimatorJumpResidualScaleMm) {
        flags |= kEstimatorJumpFlagHighResidualScale;
    }
    if (solveStats.selectedRows <= kMin3DMeasurementsForSolve) {
        flags |= kEstimatorJumpFlagLowRows;
    }

    const uint16_t captureFlags = flags & static_cast<uint16_t>(
        kEstimatorJumpFlagDelta
        | kEstimatorJumpFlagSlowSolve
        | kEstimatorJumpFlagHighRmse);
    if (captureFlags == 0) {
        return;
    }
    if (lastCaptureUs != 0 && nowUs > lastCaptureUs && (nowUs - lastCaptureUs) < kEstimatorJumpMinIntervalUs) {
        return;
    }
    lastCaptureUs = nowUs;

    EstimatorJumpEventStats event;
    event.timestampMs = timerMs32();
    event.jumpFlags = flags;
    event.mode = solveStats.mode;
    event.inputRows = solveStats.inputRows;
    event.selectedRows = solveStats.selectedRows;
    event.uniqueAnchors = solveStats.uniqueAnchors;
    event.iterations = solveStats.iterations;
    event.rowCount = std::min<uint8_t>(solveStats.selectedDiagCount, kEstimatorJumpEventRowCapacity);
    event.dtUs = dtUs64 > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(dtUs64);
    event.solveUs = solveStats.solveUs;
    event.rmseMm = solveStats.rmseMm;
    event.residualScaleMm = solveStats.residualScaleMm;
    event.deltaMm = metersToMillimetersUnsigned(deltaNorm);
    event.horizontalDeltaMm = metersToMillimetersUnsigned(horizontalDelta);
    event.verticalDeltaMm = metersToMillimetersSigned(static_cast<float>(delta(2)));
    event.speedMmps = metersToMillimetersUnsigned(speedMps);
    event.accelMmps2 = metersToMillimetersUnsigned(accelMps2);
    event.prevXMm = metersToMillimetersSigned(static_cast<float>(previousPosition(0)));
    event.prevYMm = metersToMillimetersSigned(static_cast<float>(previousPosition(1)));
    event.prevZMm = metersToMillimetersSigned(static_cast<float>(previousPosition(2)));
    event.candidateXMm = metersToMillimetersSigned(static_cast<float>(candidatePosition(0)));
    event.candidateYMm = metersToMillimetersSigned(static_cast<float>(candidatePosition(1)));
    event.candidateZMm = metersToMillimetersSigned(static_cast<float>(candidatePosition(2)));
    for (uint8_t i = 0; i < event.rowCount; i++) {
        event.rows[i] = solveStats.selected[i];
    }
    recordEstimatorJumpEvent(event);
}
#endif

static void copyRobustDiagnostics(EstimatorSolveStats& outStats,
                                  const tdoa_estimator::RobustEstimatorResult& result,
                                  const tdoa_estimator::RobustTdoaRow* rows)
{
    outStats.inputRows = result.input_rows;
    outStats.selectedRows = result.selected_rows;
    outStats.uniqueAnchors = result.unique_anchors;
    outStats.residualScaleMm = metersToMillimetersUnsigned(result.residual_scale_m);
    if (result.robust_pass_used) {
        outStats.flags |= kEstimatorDiagFlagRobustPass;
    }
    if (result.pair_selection_used) {
        outStats.flags |= kEstimatorDiagFlagPairSelection;
    }
    if (rows == nullptr) {
        return;
    }
#ifdef TDOA_ESTIMATOR_JUMP_DIAG
    const uint8_t diagCapacity = outStats.diagLevel >= kEstimatorDiagRows
        ? kEstimatorSelectedDiagCapacity
        : kEstimatorJumpEventRowCapacity;
#else
    if (outStats.diagLevel < kEstimatorDiagRows) {
        return;
    }
    const uint8_t diagCapacity = kEstimatorSelectedDiagCapacity;
#endif

    const uint8_t diagCount = std::min<uint8_t>(result.selected_rows, diagCapacity);
    outStats.selectedDiagCount = diagCount;
    for (uint8_t i = 0; i < diagCount; i++) {
        const uint8_t sourceIndex = result.selected_indices[i];
        const tdoa_estimator::RobustTdoaRow& row = rows[sourceIndex];
        EstimatorSelectedRowStats& dst = outStats.selected[i];
        dst.anchorA = row.anchor_a;
        dst.anchorB = row.anchor_b;
        dst.ageUs = row.age_us;
        dst.tdoaMm = rtls::protocol::MetersToMillimeters(
            static_cast<float>(row.tdoa));
        dst.residualMm = rtls::protocol::MetersToMillimeters(
            static_cast<float>(result.residuals[sourceIndex]));
        dst.baseWeightQ8 = weightToQ8(result.base_weights[sourceIndex]);
        dst.finalWeightQ8 = weightToQ8(result.final_weights[sourceIndex]);
    }
}

static void estimatorProcess() {
    static tdoa_estimator::PosMatrix anchors_left;
    static tdoa_estimator::PosMatrix anchors_right;
    static tdoa_estimator::DynVector tdoas;
    static tdoa_estimator::RobustTdoaRow robust_rows[kNumPairs];
    static tdoa_estimator::PosVector3D last_position = tdoa_estimator::PosVector3D::Zero();
    static bool first_estimation = true;
    static bool last_use_2d = true;
    static uint8_t last_3d_estimator_mode = kEstimatorModeRobust;
#ifdef TDOA_ESTIMATOR_JUMP_DIAG
    static uint64_t last_position_time_us = 0;
    static tdoa_estimator::Scalar last_speed_mps = 0.0f;
#endif
    static constexpr tdoa_estimator::Scalar ASSUMED_TAG_Z = 0.0f;
    static constexpr int NUM_ITERATIONS_2D = 5;
    static constexpr int NUM_ITERATIONS_3D = 10;

    // Continuous task — block on notification until producer wakes us, or
    // until the watchdog timeout elapses (so dynamic-anchor / stats
    // bookkeeping still runs during quiet periods).
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kEstimatorWatchdogMs));

    const auto& currentParams = Front::uwbLittleFSFront.GetParams();
    const bool USE_2D_ESTIMATOR = (currentParams.use2DEstimator != 0);
    const uint8_t runtimeEstimatorMode = sanitizeEstimatorMode(currentParams.tdoaEstimatorMode);
    const size_t min_measurements = USE_2D_ESTIMATOR
        ? kMinMeasurementsForNotify
        : kMin3DMeasurementsForSolve;
    if (USE_2D_ESTIMATOR != last_use_2d) {
        first_estimation = true;
        last_use_2d = USE_2D_ESTIMATOR;
        LOG_INFO("Estimator mode changed to %s - resetting warm-start",
                 USE_2D_ESTIMATOR ? "2D" : "3D");
    }
    if (!USE_2D_ESTIMATOR && runtimeEstimatorMode != last_3d_estimator_mode) {
        first_estimation = true;
        last_3d_estimator_mode = runtimeEstimatorMode;
        LOG_INFO("3D estimator runtime mode changed to %u - resetting warm-start",
                 static_cast<unsigned int>(runtimeEstimatorMode));
    }

    if (s_estimatorReinitRequested.exchange(false, std::memory_order_relaxed)) {
        first_estimation = true;
        LOG_INFO("Reinitializing estimator from static anchor config");
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
    uint64_t snapshot_now_us = 0;
    PairSlot snapshot[kNumPairs];
    etl::array<UWBAnchorParam, kNumAnchors> anchor_snapshot = {};

    if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(20)) == pdTRUE) {
        const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
        snapshot_now_us = now_us;
        const tdoa::MeasurementSnapshotResult snapshotResult =
            tdoa::SnapshotFreshMeasurements(pair_slots,
                                             configured_anchor_ids,
                                             now_us,
                                             kStaleThresholdUs,
                                             min_measurements,
                                             snapshot,
                                             kNumPairs,
                                             USE_2D_ESTIMATOR ? 0 : kMin3DUniqueAnchorsForSolve,
                                             USE_2D_ESTIMATOR ? 0 : kMax3DBatchSpanUs);

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

            if (!USE_2D_ESTIMATOR) {
                tdoa_estimator::RobustTdoaRow& row = robust_rows[i];
                row.anchor_a = s.anchor_a;
                row.anchor_b = s.anchor_b;
                row.anchor_a_pos = anchors_left.row(i).transpose();
                row.anchor_b_pos = anchors_right.row(i).transpose();
                row.tdoa = tdoas(i);
                const uint64_t age_us = snapshot_now_us >= s.timestamp_us
                    ? snapshot_now_us - s.timestamp_us
                    : 0;
                row.age_us = age_us > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(age_us);
                row.nominal_sigma_m = std::isfinite(s.sigma_m) && s.sigma_m > 0.0f
                    ? static_cast<tdoa_estimator::Scalar>(s.sigma_m)
                    : static_cast<tdoa_estimator::Scalar>(0.15f);
                row.health = static_cast<tdoa_estimator::Scalar>(1.0f);
            }
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
#ifdef TDOA_ESTIMATOR_JUMP_DIAG
            last_position_time_us = 0;
            last_speed_mps = 0.0f;
#endif
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
        EstimatorSolveStats solveStats;

        // Covariance to pass to App layer
        std::optional<std::array<float, 6>> position_covariance = std::nullopt;

        // Get configurable parameters
        const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
        const tdoa_estimator::Scalar rmseThreshold = static_cast<tdoa_estimator::Scalar>(uwbParams.rmseThreshold);
        const bool enableCovMatrix = uwbParams.enableCovMatrix != 0;
        const uint8_t estimatorDiagLevel = sanitizeEstimatorDiag(uwbParams.tdoaEstimatorDiag);
        solveStats.mode = USE_2D_ESTIMATOR ? kEstimatorMode2D : runtimeEstimatorMode;
        solveStats.diagLevel = estimatorDiagLevel;
        solveStats.inputRows = clampToU8(copy_count);
        solveStats.selectedRows = clampToU8(copy_count);
        if (!USE_2D_ESTIMATOR) {
            const EstimatorGeometryStats geometryStats =
                evaluate3DGeometry(snapshot, copy_count, anchor_snapshot, current_estimate_3d);
            solveStats.uniqueAnchors = geometryStats.uniqueAnchors;
            if (!is3DGeometryAcceptable(geometryStats)) {
                solveStats.xMm = metersToMillimetersSigned(static_cast<float>(current_estimate_3d(0)));
                solveStats.yMm = metersToMillimetersSigned(static_cast<float>(current_estimate_3d(1)));
                solveStats.zMm = metersToMillimetersSigned(static_cast<float>(current_estimate_3d(2)));
                recordEstimatorSolveStats(solveStats);
                recordEstimatorRejected(false, false, true, copy_count);
#if TDOA_STATS_LOGGING == ENABLE
                stats_samples_rejected++;
                stats_reject_insufficient++;
#endif
                return;
            }
        }

        if (USE_2D_ESTIMATOR) {
            // Prepare inputs for 2D estimator
            tdoa_estimator::PosVector2D initial_guess_2d = current_estimate_3d.head<2>();
            // Use the Z component from the current 3D state as the fixed Z for this iteration
            tdoa_estimator::Scalar fixed_z_for_estimation = current_estimate_3d(2);

            // Run 2D Newton-Raphson — bracketed for stats. Brackets are no-ops
            // when stats logging is disabled.
            uint64_t solve_start_us = static_cast<uint64_t>(esp_timer_get_time());
            tdoa_estimator::SolverResult2D result = tdoa_estimator::newtonRaphson2D(
                anchors_left,
                anchors_right,
                tdoas,
                initial_guess_2d,
                fixed_z_for_estimation,
                NUM_ITERATIONS_2D,
                1e-3f,  // convergenceThreshold
                rmseThreshold
            );
            uint32_t solve_us = elapsedUs(solve_start_us);
            solveStats.solveUs = solve_us;
            solveStats.iterations = clampToU8(static_cast<size_t>(result.iterations));
            solveStats.rmseMm = metersToMillimetersUnsigned(result.rmse);
#if TDOA_STATS_LOGGING == ENABLE
            stats_solve_count++;
            stats_solve_sum_us += solve_us;
            stats_iter_sum += static_cast<uint32_t>(result.iterations);
            if (solve_us < stats_solve_min_us) stats_solve_min_us = solve_us;
            if (solve_us > stats_solve_max_us) stats_solve_max_us = solve_us;
#endif
            if (result.valid && result.converged) {
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

            auto accept3DResult = [&](const tdoa_estimator::SolverResult& result) {
                current_estimate_3d = result.position;
                is_valid_estimate = true;
                solution_rmse = static_cast<float>(result.rmse);
                if (!is3DCovarianceUsable(result)) {
                    solveStats.flags |= kEstimatorDiagFlagCovarianceInvalid;
                }
                if (enableCovMatrix && is3DCovarianceUsable(result)) {
                    position_covariance = pack3DCovariance(result.positionCovariance);
                }
            };

            const auto runLegacy3D = [&]() {
                return tdoa_estimator::newtonRaphson(
                    anchors_left,
                    anchors_right,
                    tdoas,
                    initial_guess_3d,
                    NUM_ITERATIONS_3D,
                    1e-3f,
                    rmseThreshold);
            };

            const tdoa_estimator::RobustEstimatorOptions robustOptions =
                makeRobustOptions(rmseThreshold, NUM_ITERATIONS_3D);

            if (runtimeEstimatorMode == kEstimatorModeLegacy) {
                const uint64_t solve_start_us = static_cast<uint64_t>(esp_timer_get_time());
                const tdoa_estimator::SolverResult result = runLegacy3D();
                const uint32_t solve_us = elapsedUs(solve_start_us);
                solveStats.solveUs = solve_us;
                solveStats.legacySolveUs = solve_us;
                solveStats.iterations = clampToU8(static_cast<size_t>(result.iterations));
                solveStats.rmseMm = metersToMillimetersUnsigned(result.rmse);

#if TDOA_STATS_LOGGING == ENABLE
                stats_solve_count++;
                stats_solve_sum_us += solve_us;
                stats_iter_sum += static_cast<uint32_t>(result.iterations);
                if (solve_us < stats_solve_min_us) stats_solve_min_us = solve_us;
                if (solve_us > stats_solve_max_us) stats_solve_max_us = solve_us;
#endif
                if (is3DResultAcceptable(result, enableCovMatrix)) {
                    accept3DResult(result);
                }
            } else if (runtimeEstimatorMode == kEstimatorModeCompare) {
                solveStats.flags |= kEstimatorDiagFlagCompareMode;
                const uint64_t compare_start_us = static_cast<uint64_t>(esp_timer_get_time());

                const uint64_t legacy_start_us = static_cast<uint64_t>(esp_timer_get_time());
                const tdoa_estimator::SolverResult legacyResult = runLegacy3D();
                const uint32_t legacy_us = elapsedUs(legacy_start_us);

                const uint64_t robust_start_us = static_cast<uint64_t>(esp_timer_get_time());
                const tdoa_estimator::RobustEstimatorResult robustResult =
                    tdoa_estimator::estimateRobust3D(
                        robust_rows,
                        copy_count,
                        initial_guess_3d,
                        robustOptions);
                const uint32_t robust_us = elapsedUs(robust_start_us);

                const uint32_t total_us = elapsedUs(compare_start_us);
                solveStats.solveUs = total_us;
                solveStats.legacySolveUs = legacy_us;
                solveStats.robustSolveUs = robust_us;
                solveStats.legacyRmseMm = metersToMillimetersUnsigned(legacyResult.rmse);
                solveStats.robustRmseMm = metersToMillimetersUnsigned(robustResult.solve.rmse);
                copyRobustDiagnostics(solveStats, robustResult, robust_rows);

                const bool legacyOk = is3DResultAcceptable(legacyResult, enableCovMatrix);
                const bool robustOk = is3DResultAcceptable(robustResult.solve, enableCovMatrix);
                if (!robustOk) {
                    solveStats.flags |= kEstimatorDiagFlagRobustInvalid;
                }
                if (legacyOk && robustOk) {
                    solveStats.compareDeltaMm = positionDeltaMm(legacyResult.position, robustResult.solve.position);
                }

                if (robustOk) {
                    solveStats.iterations = clampToU8(static_cast<size_t>(robustResult.solve.iterations));
                    solveStats.rmseMm = metersToMillimetersUnsigned(robustResult.solve.rmse);
                    accept3DResult(robustResult.solve);
                } else if (legacyOk) {
                    solveStats.flags |= kEstimatorDiagFlagFallbackLegacy;
                    solveStats.iterations = clampToU8(static_cast<size_t>(legacyResult.iterations));
                    solveStats.rmseMm = metersToMillimetersUnsigned(legacyResult.rmse);
                    accept3DResult(legacyResult);
                } else {
                    solveStats.iterations = clampToU8(static_cast<size_t>(robustResult.solve.iterations));
                    solveStats.rmseMm = metersToMillimetersUnsigned(robustResult.solve.rmse);
                }

#if TDOA_STATS_LOGGING == ENABLE
                stats_solve_count++;
                stats_solve_sum_us += total_us;
                stats_iter_sum += static_cast<uint32_t>(solveStats.iterations);
                if (total_us < stats_solve_min_us) stats_solve_min_us = total_us;
                if (total_us > stats_solve_max_us) stats_solve_max_us = total_us;
#endif
            } else {
                const uint64_t solve_start_us = static_cast<uint64_t>(esp_timer_get_time());
                const tdoa_estimator::RobustEstimatorResult result =
                    tdoa_estimator::estimateRobust3D(
                        robust_rows,
                        copy_count,
                        initial_guess_3d,
                        robustOptions);
                const uint32_t solve_us = elapsedUs(solve_start_us);
                solveStats.solveUs = solve_us;
                solveStats.robustSolveUs = solve_us;
                solveStats.iterations = clampToU8(static_cast<size_t>(result.solve.iterations));
                solveStats.rmseMm = metersToMillimetersUnsigned(result.solve.rmse);
                copyRobustDiagnostics(solveStats, result, robust_rows);

#if TDOA_STATS_LOGGING == ENABLE
                stats_solve_count++;
                stats_solve_sum_us += solve_us;
                stats_iter_sum += static_cast<uint32_t>(result.solve.iterations);
                if (solve_us < stats_solve_min_us) stats_solve_min_us = solve_us;
                if (solve_us > stats_solve_max_us) stats_solve_max_us = solve_us;
#endif
                if (is3DResultAcceptable(result.solve, enableCovMatrix)) {
                    accept3DResult(result.solve);
                }
            }
        }

        // --- Send Data to Application ---
        bool has_nan = current_estimate_3d.hasNaN();
        if (is_valid_estimate && !has_nan) {
            solveStats.flags |= kEstimatorDiagFlagAccepted;
        }
        if (position_covariance.has_value()) {
            solveStats.flags |= kEstimatorDiagFlagCovarianceSent;
        }
        solveStats.xMm = metersToMillimetersSigned(static_cast<float>(current_estimate_3d(0)));
        solveStats.yMm = metersToMillimetersSigned(static_cast<float>(current_estimate_3d(1)));
        solveStats.zMm = metersToMillimetersSigned(static_cast<float>(current_estimate_3d(2)));
        recordEstimatorSolveStats(solveStats);
        if (is_valid_estimate && !has_nan) { // Only send if the estimate is valid
#ifdef TDOA_ESTIMATOR_JUMP_DIAG
            tdoa_estimator::Scalar current_speed_mps = last_speed_mps;
            maybeRecordEstimatorJumpEvent(solveStats,
                                          last_position,
                                          current_estimate_3d,
                                          last_position_time_us,
                                          last_speed_mps,
                                          current_speed_mps);
#endif
            recordEstimatorAccepted(current_estimate_3d(0),
                                    current_estimate_3d(1),
                                    current_estimate_3d(2),
                                    solution_rmse,
                                    copy_count);
            App::SendSample(current_estimate_3d(0), current_estimate_3d(1), current_estimate_3d(2), position_covariance);

            // Update the persistent state for the next iteration (Warm Start)
            last_position = current_estimate_3d;
#ifdef TDOA_ESTIMATOR_JUMP_DIAG
            last_position_time_us = static_cast<uint64_t>(esp_timer_get_time());
            last_speed_mps = current_speed_mps;
#endif

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
#include "wifi/wifi_device_telemetry.hpp"

bool UWBTagTDoA::IsDynamicPositioningEnabled() {
    return s_useDynamicPositions;
}

bool UWBTagTDoA::AreDynamicPositionsReadyForEstimator() {
    return s_useDynamicPositions
        && s_dynamicPositionsReadyForEstimator.load(std::memory_order_relaxed);
}

bool UWBTagTDoA::AreDynamicAnchorPositionsReady() {
    return AreDynamicPositionsReadyForEstimator();
}

void UWBTagTDoA::ApplyDynamicAnchorPositioningEnabled(uint8_t enabled) {
    const bool should_enable = enabled != 0;
    if (!should_enable) {
        if (s_useDynamicPositions) {
            LOG_INFO("Dynamic anchor positioning disabled at runtime");
        }
        s_useDynamicPositions = false;
        uwbTdoa2TagSetDistanceCallback(nullptr);
        ClearDynamicAnchorRuntimeState(true, false);
        return;
    }

    if (!s_useDynamicPositions) {
        s_useDynamicPositions = true;
        ClearDynamicAnchorRuntimeState(true, true);
        ApplyDynamicAnchorRuntimeConfig(Front::uwbLittleFSFront.GetParams());
        uwbTdoa2TagSetDistanceCallback(&UWBTagTDoA::onInterAnchorDistance);
        LOG_INFO("Dynamic anchor positioning enabled at runtime; waiting for calculated geometry");
    }
}

void UWBTagTDoA::ApplyDynamicAnchorRuntimeConfig(const UWBParams& params) {
    if (!s_useDynamicPositions) {
        if (params.dynamicAnchorPosEnabled != 0) {
            LOG_WARN("Dynamic anchor runtime config skipped because dynamic positioning is not active");
        }
        return;
    }

    DynamicAnchorConfig dynamicConfig = {
        .layout = params.anchorLayout,
        .anchorCount = dynamicAnchorCountForParams(params),
        .anchorHeight = params.anchorHeight,
        .anchorPlaneSeparation = params.anchorPlaneSeparation,
        .avgSampleCount = params.distanceAvgSamples,
        .lockedMask = params.anchorPosLocked
    };
    if (!takeDynamicCalcMutex(pdMS_TO_TICKS(50))) {
        LOG_ERROR("Dynamic anchor runtime config skipped - calculator mutex unavailable");
        return;
    }
    s_dynamicCalc.init(dynamicConfig);
    giveDynamicCalcMutex();
    s_lastPositionUpdate = 0;
    s_dynamicPositionsReadyForEstimator.store(false, std::memory_order_relaxed);
    s_dynamicEstimatorReinitRequested.store(false, std::memory_order_relaxed);
    s_dynamicTransitionHoldoffUntilMs.store(0, std::memory_order_relaxed);

    if (measurements_mtx != nullptr
        && xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (uint8_t i = 0; i < kNumAnchors; i++) {
            configured_anchor_ids[i] = false;
            anchor_positions[i] = {};
        }
        clearFreshMeasurementsLocked();
        xSemaphoreGive(measurements_mtx);
    }

    LOG_INFO("Dynamic anchor runtime config applied (layout=%d, anchors=%u, height=%.2f, planeSep=%.2f, samples=%d)",
             params.anchorLayout,
             static_cast<unsigned int>(dynamicConfig.anchorCount),
             params.anchorHeight,
             params.anchorPlaneSeparation,
             params.distanceAvgSamples);
}

void UWBTagTDoA::ApplyDynamicAnchorLockMask(uint8_t lockedMask) {
    if (!s_useDynamicPositions) {
        return;
    }

    if (!takeDynamicCalcMutex(pdMS_TO_TICKS(50))) {
        LOG_ERROR("Dynamic anchor lock mask skipped - calculator mutex unavailable");
        return;
    }
    s_dynamicCalc.setLockedMask(lockedMask);
    giveDynamicCalcMutex();

    LOG_INFO("Dynamic anchor lock mask applied (mask=0x%02X)", lockedMask);
}

#ifdef USE_RTLSLINK_BEACON_BACKEND
bool UWBTagTDoA::ConfigureRtlslinkBeaconFromCurrentAnchors() {
    return configureRtlslinkBeaconFromAnchorPositions();
}
#endif

uint8_t UWBTagTDoA::GetDynamicAnchorPositions(DynamicAnchorTelemetry* out, uint8_t maxCount) {
    if (!AreDynamicAnchorPositionsReady() || out == nullptr || maxCount == 0) {
        return 0;
    }

    // Try to acquire mutex with short timeout (10ms) to avoid blocking telemetry updates.
    if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(10)) != pdTRUE) {
        return 0;  // Mutex unavailable, skip this update
    }

    uint8_t count = 0;
    const uint8_t expected_dynamic_count = configuredDynamicAnchorCount();
    for (uint8_t i = 0; i < expected_dynamic_count && count < maxCount; i++) {
        if (!configured_anchor_ids[i]) {
            continue;
        }
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

    // Update the calculator with the corrected distance measurement.
    if (takeDynamicCalcMutex(pdMS_TO_TICKS(2))) {
        s_dynamicCalc.updateDistance(fromAnchor, toAnchor, distanceMeters);
        giveDynamicCalcMutex();
    }
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

    const uint8_t dynamic_anchor_count = configuredDynamicAnchorCount();

    point_t newPositions[kDynamicAnchorCount3D];
    bool calculated = false;
    if (takeDynamicCalcMutex(pdMS_TO_TICKS(10))) {
        if (s_dynamicCalc.canCalculate()) {
            calculated = s_dynamicCalc.calculatePositions(newPositions, dynamic_anchor_count);
        }
        giveDynamicCalcMutex();
    }
    if (!calculated) {
        if (s_dynamicPositionsReadyForEstimator.exchange(false, std::memory_order_relaxed)) {
            if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                for (uint8_t i = 0; i < kNumAnchors; i++) {
                    configured_anchor_ids[i] = false;
                    anchor_positions[i] = {};
                }
                clearFreshMeasurementsLocked();
                xSemaphoreGive(measurements_mtx);
            }
            s_dynamicEstimatorReinitRequested.store(false, std::memory_order_relaxed);
            s_dynamicTransitionHoldoffUntilMs.store(0, std::memory_order_relaxed);
#ifdef USE_RTLSLINK_BEACON_BACKEND
            clearRtlslinkBeaconDynamicAnchors();
#endif
            LOG_WARN("Dynamic anchor positions no longer valid; estimator waiting for fresh geometry");
        }
        return;
    }

    {
        bool updated_positions = false;
        bool first_dynamic_update = false;

        // CRITICAL: Lock mutex before updating shared anchor_positions
        // This prevents race conditions with estimatorProcess() which reads these values
        if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
            for (uint8_t i = 0; i < dynamic_anchor_count; i++) {
                anchor_positions[i].shortAddr[0] = static_cast<char>('0' + i);
                anchor_positions[i].shortAddr[1] = '\0';
                anchor_positions[i].x = newPositions[i].x;
                anchor_positions[i].y = newPositions[i].y;
                anchor_positions[i].z = newPositions[i].z;
                configured_anchor_ids[i] = true;
            }
            for (uint8_t i = dynamic_anchor_count; i < kNumAnchors; i++) {
                configured_anchor_ids[i] = false;
                anchor_positions[i] = {};
            }

            if (!s_dynamicPositionsReadyForEstimator.load(std::memory_order_relaxed)) {
                // Drop pre-transition measurements so the next solve uses only
                // measurements gathered after dynamic geometry is active.
                clearFreshMeasurementsLocked();
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
            LOG_DEBUG("Dynamic Anchors: count=%u A0(%.2f,%.2f,%.2f) A1(%.2f,%.2f,%.2f) A2(%.2f,%.2f,%.2f) A3(%.2f,%.2f,%.2f)",
                      static_cast<unsigned int>(dynamic_anchor_count),
                      anchor_positions[0].x, anchor_positions[0].y,
                      anchor_positions[0].z,
                      anchor_positions[1].x, anchor_positions[1].y,
                      anchor_positions[1].z,
                      anchor_positions[2].x, anchor_positions[2].y,
                      anchor_positions[2].z,
                      anchor_positions[3].x, anchor_positions[3].y,
                      anchor_positions[3].z);
            lastLogTime = now;
        }
    }
}
#endif // USE_DYNAMIC_ANCHOR_POSITIONS

#endif // USE_UWB_MODE_TDOA_TAG
