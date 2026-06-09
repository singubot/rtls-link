#include "config/features.hpp"

#include "uwb_frontend_littlefs.hpp"
#include "logging/logging.hpp"
#include <cmath>
#include <cstring>

#ifdef USE_RTLSLINK_BEACON_BACKEND
#include "app.hpp"
#endif

#ifdef USE_UWB_MODE_TDOA_TAG
#include "uwb_tdoa_tag.hpp"
#endif

#ifdef USE_UWB_MODE_TDOA_ANCHOR
#include "uwb_tdoa_anchor.hpp"
#endif

namespace {

UWBParams s_lastAcceptedStaticAnchors = {};
bool s_hasLastAcceptedStaticAnchors = false;

bool isStaticAnchorCommitParam(const char* name)
{
    return strcmp(name, "anchorCount") == 0;
}

bool isStaticAnchorGeometryParam(const char* name)
{
    if (name[0] == 'x' || name[0] == 'y' || name[0] == 'z') {
        return name[1] >= '1' && name[1] <= '8' && name[2] == '\0';
    }
    if (strncmp(name, "devId", 5) == 0) {
        return name[5] >= '1' && name[5] <= '8' && name[6] == '\0';
    }
    return false;
}

#ifdef USE_RTLSLINK_BEACON_BACKEND
bool isRtlslinkRuntimeConfigParam(const char* name)
{
    return strcmp(name, "outputBackend") == 0
        || strcmp(name, "rotationDegrees") == 0
        || strcmp(name, "originLat") == 0
        || strcmp(name, "originLon") == 0
        || strcmp(name, "originAlt") == 0;
}

bool isFiniteRtlslinkRuntimeConfigValue(const char* name, const void* data)
{
    if (data == nullptr) {
        return false;
    }

    if (strcmp(name, "rotationDegrees") == 0 || strcmp(name, "originAlt") == 0) {
        float parsed = 0.0f;
        return Utils::TransformStrToData(ParamType::FLOAT,
                                         static_cast<const char*>(data),
                                         &parsed) == Utils::ErrorTransform::OK
            && std::isfinite(parsed);
    }

    if (strcmp(name, "originLat") == 0 || strcmp(name, "originLon") == 0) {
        double parsed = 0.0;
        return Utils::TransformStrToData(ParamType::DOUBLE,
                                         static_cast<const char*>(data),
                                         &parsed) == Utils::ErrorTransform::OK
            && std::isfinite(parsed);
    }

    return true;
}

bool hasFiniteRtlslinkRuntimeConfig(const UWBParams& params)
{
    return std::isfinite(params.rotationDegrees)
        && std::isfinite(params.originLat)
        && std::isfinite(params.originLon)
        && std::isfinite(params.originAlt);
}
#endif

bool isValidAnchorCountValue(const void* data)
{
    if (data == nullptr) {
        return false;
    }

    uint32_t parsed = 0;
    if (Utils::TransformStrToData(ParamType::UINT8,
                                  static_cast<const char*>(data),
                                  &parsed) != Utils::ErrorTransform::OK) {
        return false;
    }
    if (parsed == 0 || parsed > UWBParams::maxAnchorCount) {
        return false;
    }

    return true;
}

bool parseUint8ParamValue(const void* data, uint8_t& outValue)
{
    if (data == nullptr) {
        return false;
    }

    uint32_t parsed = 0;
    if (Utils::TransformStrToData(ParamType::UINT8,
                                  static_cast<const char*>(data),
                                  &parsed) != Utils::ErrorTransform::OK) {
        return false;
    }
    if (parsed > UINT8_MAX) {
        return false;
    }

    outValue = static_cast<uint8_t>(parsed);
    return true;
}

bool isValidEstimatorModeValue(const void* data)
{
    uint8_t parsed = 0;
    return parseUint8ParamValue(data, parsed) && parsed <= 2;
}

bool isValidEstimatorDiagValue(const void* data)
{
    uint8_t parsed = 0;
    return parseUint8ParamValue(data, parsed) && parsed <= 2;
}

bool hasValidEstimatorRuntimeConfig(const UWBParams& params)
{
    return params.tdoaEstimatorMode <= 2
        && params.tdoaEstimatorDiag <= 2;
}

bool parseUwbModeValue(const void* data, UWBMode& outMode)
{
    if (data == nullptr) {
        return false;
    }

    uint32_t parsed = 0;
    if (Utils::TransformStrToData(ParamType::UINT8,
                                  static_cast<const char*>(data),
                                  &parsed) != Utils::ErrorTransform::OK) {
        return false;
    }
    if (parsed != static_cast<uint8_t>(UWBMode::ANCHOR_TDOA)
        && parsed != static_cast<uint8_t>(UWBMode::TAG_TDOA)) {
        return false;
    }

    outMode = static_cast<UWBMode>(parsed);
    return true;
}

bool dynamicTagGeometryEnabled(const UWBParams& params)
{
    return params.mode == UWBMode::TAG_TDOA && params.dynamicAnchorPosEnabled != 0;
}

bool staticTagGeometryRequired(const UWBParams& params)
{
    return params.mode == UWBMode::TAG_TDOA && !dynamicTagGeometryEnabled(params);
}

bool isDynamicAnchorRuntimeParam(const char* name)
{
    return strcmp(name, "dynamicAnchorPosEnabled") == 0
        || strcmp(name, "use2DEstimator") == 0
        || strcmp(name, "anchorLayout") == 0
        || strcmp(name, "anchorHeight") == 0
        || strcmp(name, "anchorPlaneSeparation") == 0
        || strcmp(name, "distanceAvgSamples") == 0
        || strcmp(name, "anchorPosLocked") == 0;
}

bool isDynamicAnchorReinitRuntimeParam(const char* name)
{
    return isDynamicAnchorRuntimeParam(name)
        && strcmp(name, "anchorPosLocked") != 0;
}

bool validateDynamicTagGeometryForParams(const UWBParams& params)
{
    if (!dynamicTagGeometryEnabled(params)) {
        return true;
    }
    if (params.anchorLayout > 3) {
        LOG_ERROR("Rejected dynamic anchor config: unsupported anchorLayout %u",
                  static_cast<unsigned int>(params.anchorLayout));
        return false;
    }
    if (!std::isfinite(params.anchorHeight)) {
        LOG_ERROR("Rejected dynamic anchor config: non-finite anchorHeight");
        return false;
    }
    if (params.use2DEstimator == 0) {
        if (!std::isfinite(params.anchorPlaneSeparation)
            || params.anchorPlaneSeparation <= 0.0f) {
            LOG_ERROR("Rejected dynamic 3D anchor config: positive anchorPlaneSeparation required");
            return false;
        }
    }
    return true;
}

#ifdef USE_UWB_MODE_TDOA_TAG
bool validateStaticTagGeometryForParams(const UWBParams& params,
                                        etl::span<const UWBAnchorParam> anchors)
{
    if (!staticTagGeometryRequired(params)) {
        return true;
    }
    return UWBTagTDoA::ValidateStaticAnchorsForEstimator(
        anchors,
        params.use2DEstimator != 0,
        params.tdoaEstimatorMode);
}
#endif

void copyStaticAnchorConfig(const UWBParams& from, UWBParams& to)
{
    to.anchorCount = from.anchorCount;
    to.devId1 = from.devId1;
    to.x1 = from.x1;
    to.y1 = from.y1;
    to.z1 = from.z1;
    to.devId2 = from.devId2;
    to.x2 = from.x2;
    to.y2 = from.y2;
    to.z2 = from.z2;
    to.devId3 = from.devId3;
    to.x3 = from.x3;
    to.y3 = from.y3;
    to.z3 = from.z3;
    to.devId4 = from.devId4;
    to.x4 = from.x4;
    to.y4 = from.y4;
    to.z4 = from.z4;
    to.devId5 = from.devId5;
    to.x5 = from.x5;
    to.y5 = from.y5;
    to.z5 = from.z5;
    to.devId6 = from.devId6;
    to.x6 = from.x6;
    to.y6 = from.y6;
    to.z6 = from.z6;
    to.devId7 = from.devId7;
    to.x7 = from.x7;
    to.y7 = from.y7;
    to.z7 = from.z7;
    to.devId8 = from.devId8;
    to.x8 = from.x8;
    to.y8 = from.y8;
    to.z8 = from.z8;
}

void rememberAcceptedStaticAnchorConfig(const UWBParams& params)
{
    copyStaticAnchorConfig(params, s_lastAcceptedStaticAnchors);
    s_hasLastAcceptedStaticAnchors = true;
}

void restoreAcceptedStaticAnchorConfig(UWBParams& params, const UWBParams& fallback)
{
    copyStaticAnchorConfig(s_hasLastAcceptedStaticAnchors ? s_lastAcceptedStaticAnchors : fallback,
                           params);
}

} // namespace

void UWBLittleFSFrontend::InitBackendForCurrentMode() {
    if (m_Backend != nullptr) {
        return;
    }
    auto anchors = GetAnchors();
    switch (m_Params.mode) {
    case UWBMode::ANCHOR_TDOA:
#ifdef USE_UWB_MODE_TDOA_ANCHOR
        m_Backend = new UWBAnchorTDoA(bsp::kBoardConfig.uwb, m_Params.devShortAddr, m_Params.ADelay);
#else
        LOG_WARN("TDoA Anchor mode requested but USE_UWB_MODE_TDOA_ANCHOR not compiled");
#endif
        break;
    case UWBMode::TAG_TDOA:
#ifdef USE_UWB_MODE_TDOA_TAG
        m_Backend = new UWBTagTDoA(bsp::kBoardConfig.uwb, anchors);
#else
        LOG_WARN("TDoA Tag mode requested but USE_UWB_MODE_TDOA_TAG not compiled");
#endif
        break;
    default:
        LOG_ERROR("Unknown UWB mode");
        break;
    }
    if (m_Backend != nullptr) {
        m_BackendMode = m_Params.mode;
    }
}

bool UWBLittleFSFrontend::HasBackendModeMismatch() const
{
    return m_Backend != nullptr && m_BackendMode != m_Params.mode;
}

void UWBLittleFSFrontend::Init() {
    const ErrorParam loadResult = LoadParams();
    if (loadResult != ErrorParam::OK && loadResult != ErrorParam::FILE_NOT_FOUND) {
        LOG_ERROR("UWB stored params rejected; continuing with previous/default runtime config");
    }
    LOG_INFO("UWB frontend initializing");

#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
    if (m_Params.uwbEnable == 0) {
        LOG_WARN("UWB backend disabled by parameter (uwb.uwbEnable=0)");
    } else {
        InitBackendForCurrentMode();
        if (!ApplyLoadedRuntimeConfig()) {
            LOG_ERROR("UWB stored runtime config rejected during initialization");
        }
    }
#else
    InitBackendForCurrentMode();
    if (!ApplyLoadedRuntimeConfig()) {
        LOG_ERROR("UWB stored runtime config rejected during initialization");
    }
#endif

    LOG_INFO("UWB frontend initialized");
}

ErrorParam UWBLittleFSFrontend::LoadParams() {
    const UWBParams previousParams = m_Params;
    const ErrorParam result = LittleFSFrontend<UWBParams>::LoadParams();
    if (result == ErrorParam::OK) {
        if (!hasValidEstimatorRuntimeConfig(m_Params)) {
            LOG_ERROR("Rejected stored UWB params with invalid TDoA estimator runtime config");
            m_Params = previousParams;
            return ErrorParam::INVALID_DATA;
        }
#ifdef USE_RTLSLINK_BEACON_BACKEND
        if (!hasFiniteRtlslinkRuntimeConfig(m_Params)) {
            LOG_ERROR("Rejected stored UWB params with non-finite RTLSLink runtime config");
            m_Params = previousParams;
            return ErrorParam::INVALID_DATA;
        }
#endif
#ifdef USE_UWB_MODE_TDOA_TAG
        auto anchors = GetAnchors();
        if (m_Params.mode == UWBMode::TAG_TDOA
            && !validateDynamicTagGeometryForParams(m_Params)) {
            LOG_ERROR("Rejected stored UWB params with invalid dynamic TAG_TDOA geometry");
            m_Params = previousParams;
            return ErrorParam::INVALID_DATA;
        }
        if (m_Params.mode == UWBMode::TAG_TDOA
            && !validateStaticTagGeometryForParams(m_Params, anchors)) {
            LOG_ERROR("Rejected stored UWB params with invalid TAG_TDOA anchor geometry");
            m_Params = previousParams;
            return ErrorParam::INVALID_DATA;
        }
#endif
#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
        if (m_Params.uwbEnable == 0) {
            SetRuntimeEnabled(false);
            return result;
        }
        if (!SetRuntimeEnabled(true)) {
            LOG_ERROR("Rejected stored UWB params; backend could not be enabled");
            m_Params = previousParams;
            SetRuntimeEnabled(previousParams.uwbEnable != 0);
            return ErrorParam::INVALID_DATA;
        }
#endif
        if (m_Backend != nullptr && !ApplyLoadedRuntimeConfig()) {
            RestoreTagRuntimeState(previousParams, true, true, true);
#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
            SetRuntimeEnabled(previousParams.uwbEnable != 0);
#endif
            return ErrorParam::INVALID_DATA;
        }
    }
    return result;
}

bool UWBLittleFSFrontend::ApplyLoadedRuntimeConfig()
{
    if (m_Params.mode != UWBMode::TAG_TDOA || m_Backend == nullptr) {
        return true;
    }

#if defined(USE_DYNAMIC_ANCHOR_POSITIONS) && defined(USE_UWB_MODE_TDOA_TAG)
    UWBTagTDoA::ApplyDynamicAnchorPositioningEnabled(m_Params.dynamicAnchorPosEnabled);
    if (dynamicTagGeometryEnabled(m_Params)) {
        UWBTagTDoA::ApplyDynamicAnchorRuntimeConfig(m_Params);
#ifdef USE_RTLSLINK_BEACON_BACKEND
        ClearRtlslinkBeaconAnchors();
#endif
        LOG_INFO("Dynamic anchor positioning active; static TAG_TDOA geometry not applied");
        return true;
    }
#endif

    if (!ApplyTagRuntimeAnchorsTransaction(true, true)) {
        LOG_ERROR("Rejected stored UWB runtime config; live estimator/beacon apply failed");
        return false;
    }
    rememberAcceptedStaticAnchorConfig(m_Params);
    return true;
}

bool UWBLittleFSFrontend::RestoreTagRuntimeState(const UWBParams& params,
                                                 bool restoreEstimator,
                                                 bool restoreRtlslinkBeacon,
                                                 bool clearPendingDynamicBeacon)
{
    m_Params = params;
    if (m_Params.mode != UWBMode::TAG_TDOA) {
        return true;
    }

    bool restored = true;
#if defined(USE_DYNAMIC_ANCHOR_POSITIONS) && defined(USE_UWB_MODE_TDOA_TAG)
    UWBTagTDoA::ApplyDynamicAnchorPositioningEnabled(m_Params.dynamicAnchorPosEnabled);
#endif
#if defined(USE_DYNAMIC_ANCHOR_POSITIONS) && defined(USE_UWB_MODE_TDOA_TAG) && defined(USE_RTLSLINK_BEACON_BACKEND)
    if (clearPendingDynamicBeacon
        && restoreRtlslinkBeacon
        && m_Params.dynamicAnchorPosEnabled != 0
        && UWBTagTDoA::IsDynamicPositioningEnabled()
        && !UWBTagTDoA::AreDynamicPositionsReadyForEstimator()) {
        restored = ClearRtlslinkBeaconAnchors();
        restoreRtlslinkBeacon = false;
    }
#else
    (void)clearPendingDynamicBeacon;
#endif

    restored = ApplyTagRuntimeAnchorsTransaction(restoreEstimator, restoreRtlslinkBeacon) && restored;
    if (!restored) {
        LOG_WARN("Failed to fully restore UWB runtime state");
    }
    return restored;
}

bool UWBLittleFSFrontend::ApplyTagRuntimeAnchorsTransaction(bool applyEstimator, bool applyRtlslinkBeacon)
{
#ifdef USE_UWB_MODE_TDOA_TAG
    auto anchors = GetAnchors();
    if (applyEstimator && !validateStaticTagGeometryForParams(m_Params, anchors)) {
        return false;
    }
#else
    (void)applyEstimator;
#endif

    if (applyRtlslinkBeacon && !ApplyStaticAnchorsToLiveBackends(false, true)) {
        return false;
    }

    if (applyEstimator && !ApplyStaticAnchorsToLiveBackends(true, false)) {
        return false;
    }

    return true;
}

bool UWBLittleFSFrontend::ClearRtlslinkBeaconAnchors()
{
#ifdef USE_RTLSLINK_BEACON_BACKEND
    etl::array<UWBAnchorParam, 1> emptyAnchors = {};
    return App::ConfigureRtlslinkBeaconAnchors(
        etl::span<const UWBAnchorParam>(emptyAnchors.data(), 0));
#else
    return true;
#endif
}

bool UWBLittleFSFrontend::ApplyStaticAnchorsToLiveBackends(bool applyEstimator, bool applyRtlslinkBeacon)
{
    if (m_Params.mode != UWBMode::TAG_TDOA) {
        return true;
    }

    auto anchors = GetAnchors();

#ifdef USE_UWB_MODE_TDOA_TAG
    if (applyEstimator && !validateStaticTagGeometryForParams(m_Params, anchors)) {
        return false;
    }
#endif

#if defined(USE_DYNAMIC_ANCHOR_POSITIONS) && defined(USE_UWB_MODE_TDOA_TAG)
    const bool dynamicParamEnabled = (m_Params.dynamicAnchorPosEnabled != 0);
    const bool dynamicRunning = UWBTagTDoA::IsDynamicPositioningEnabled();
    if (dynamicParamEnabled) {
        if (!dynamicRunning) {
#ifdef USE_RTLSLINK_BEACON_BACKEND
            if (applyRtlslinkBeacon) {
                ClearRtlslinkBeaconAnchors();
            }
#else
            (void)applyRtlslinkBeacon;
#endif
            if (applyEstimator) {
                LOG_INFO("Dynamic anchor positioning pending; dynamic calculations are not active");
            }
            return true;
        }

        if (UWBTagTDoA::AreDynamicPositionsReadyForEstimator()) {
            if (applyEstimator) {
                LOG_INFO("Static anchor config saved; dynamic anchor positioning is active");
            }
#ifdef USE_RTLSLINK_BEACON_BACKEND
            bool beaconConfigured = true;
            if (applyRtlslinkBeacon) {
                beaconConfigured = UWBTagTDoA::ConfigureRtlslinkBeaconFromCurrentAnchors();
            }
#else
            const bool beaconConfigured = true;
            (void)applyRtlslinkBeacon;
#endif
            return beaconConfigured;
        }

#ifdef USE_RTLSLINK_BEACON_BACKEND
        if (applyRtlslinkBeacon) {
            LOG_INFO("RTLSLink beacon waiting for dynamic anchor positions");
            ClearRtlslinkBeaconAnchors();
        }
#else
        (void)applyRtlslinkBeacon;
#endif
        if (applyEstimator) {
            LOG_INFO("Dynamic anchor positioning pending; static TAG_TDOA geometry not applied");
        }
        return true;
    }
#endif

    bool estimatorApplied = true;
#ifdef USE_UWB_MODE_TDOA_TAG
    if (applyEstimator) {
        estimatorApplied = UWBTagTDoA::ApplyStaticAnchors(anchors);
    }
#else
    (void)applyEstimator;
#endif

#ifdef USE_RTLSLINK_BEACON_BACKEND
    if (applyRtlslinkBeacon) {
        if (!estimatorApplied) {
            LOG_WARN("RTLSLink beacon static anchor config skipped - estimator apply failed");
            return false;
        }
        if (!App::ConfigureRtlslinkBeaconAnchors(anchors)) {
            LOG_WARN("RTLSLink beacon static anchor config skipped - backend busy or rejected config");
            return false;
        }
    }
#else
    (void)applyRtlslinkBeacon;
#endif

    return estimatorApplied;
}

bool UWBLittleFSFrontend::SetRuntimeEnabled(bool enabled) {
#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
    m_Params.uwbEnable = enabled ? 1 : 0;

    if (m_Backend != nullptr) {
        if (enabled && HasBackendModeMismatch()) {
            LOG_ERROR("UWB backend mode change requires reboot before runtime can be enabled");
            return false;
        }
        m_Backend->SetEnabled(enabled);
        return true;
    }

    if (enabled) {
        InitBackendForCurrentMode();
        return m_Backend != nullptr;
    }

    return true;
#else
    (void)enabled;
    return true;
#endif
}

ErrorParam UWBLittleFSFrontend::SetParam(const char* name, const void* data, uint32_t len) {
    const bool staticAnchorCommit = isStaticAnchorCommitParam(name);
    const UWBParams previousParams = m_Params;
    if (staticAnchorCommit) {
        if (!isValidAnchorCountValue(data)) {
            LOG_ERROR("Rejected invalid UWB anchorCount (valid range 1-%u)",
                      static_cast<unsigned int>(UWBParams::maxAnchorCount));
            return ErrorParam::INVALID_DATA;
        }
    }
    if (strcmp(name, "mode") == 0) {
        UWBMode requestedMode = UWBMode::UNKNOWN;
        if (!parseUwbModeValue(data, requestedMode)) {
            LOG_ERROR("Rejected invalid UWB mode");
            return ErrorParam::INVALID_DATA;
        }
        if (m_Backend != nullptr && requestedMode != m_BackendMode) {
            LOG_ERROR("Rejected UWB mode change; backend mode changes require reboot");
            return ErrorParam::INVALID_DATA;
        }
    }
    if (strcmp(name, "tdoaEstimatorMode") == 0 && !isValidEstimatorModeValue(data)) {
        LOG_ERROR("Rejected invalid TDoA estimator mode (valid range 0-2)");
        return ErrorParam::INVALID_DATA;
    }
    if (strcmp(name, "tdoaEstimatorDiag") == 0 && !isValidEstimatorDiagValue(data)) {
        LOG_ERROR("Rejected invalid TDoA estimator diagnostics level (valid range 0-2)");
        return ErrorParam::INVALID_DATA;
    }
#ifdef USE_RTLSLINK_BEACON_BACKEND
    if (isRtlslinkRuntimeConfigParam(name) && !isFiniteRtlslinkRuntimeConfigValue(name, data)) {
        LOG_ERROR("Rejected non-finite RTLSLink runtime UWB parameter '%s'", name);
        return ErrorParam::INVALID_DATA;
    }
#endif

    ErrorParam result = LittleFSFrontend<UWBParams>::SetParam(name, data, len);
    if (result != ErrorParam::OK) {
        return result;
    }

    auto rollbackParamsAndRuntime = [this](const UWBParams& rollbackParams,
                                           bool restoreEstimator,
                                           bool restoreRtlslinkBeacon,
                                           bool clearPendingDynamicBeacon = false) {
        RestoreTagRuntimeState(rollbackParams,
                               restoreEstimator,
                               restoreRtlslinkBeacon,
                               clearPendingDynamicBeacon);
#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
        SetRuntimeEnabled(rollbackParams.uwbEnable != 0);
#endif
        const ErrorParam rollbackResult = SaveParams();
        if (rollbackResult != ErrorParam::OK) {
            return rollbackResult;
        }
        return ErrorParam::INVALID_DATA;
    };

    if (m_Params.mode == UWBMode::TAG_TDOA
        && !validateDynamicTagGeometryForParams(m_Params)) {
        LOG_ERROR("Rejected invalid dynamic TAG_TDOA geometry; restoring previous UWB params");
        return rollbackParamsAndRuntime(previousParams, false, true, true);
    }

#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
    if (strcmp(name, "uwbEnable") == 0) {
        if (m_Params.uwbEnable == 0) {
            LOG_INFO("UWB runtime disabled");
            SetRuntimeEnabled(false);
        } else {
            LOG_INFO("UWB runtime enabled");
            if (!SetRuntimeEnabled(true) || !ApplyLoadedRuntimeConfig()) {
                LOG_ERROR("Rejected UWB runtime enable; restoring previous UWB params");
                return rollbackParamsAndRuntime(previousParams, true, true, true);
            }
        }
        return result;
    }
#endif

    if (strcmp(name, "mode") == 0) {
#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
        if (m_Params.uwbEnable != 0 && m_Backend == nullptr) {
            InitBackendForCurrentMode();
        } else if (m_Backend != nullptr) {
            LOG_WARN("UWB mode changed at runtime; reboot required to fully apply backend mode");
        }
#else
        if (m_Backend == nullptr) {
            InitBackendForCurrentMode();
        } else {
            LOG_WARN("UWB mode changed at runtime; reboot required to fully apply backend mode");
        }
#endif
    }

#if defined(USE_UWB_MODE_TDOA_TAG) && defined(ESP32S3_UWB_BOARD)
    if (strcmp(name, "tdoaMatcherPolicy") == 0 && m_Params.mode == UWBMode::TAG_TDOA) {
        UWBTagTDoA::ApplyMatcherPolicy(m_Params.tdoaMatcherPolicy);
    }
#endif

#if defined(USE_DYNAMIC_ANCHOR_POSITIONS) && defined(USE_UWB_MODE_TDOA_TAG)
    if (strcmp(name, "dynamicAnchorPosEnabled") == 0 && m_Params.mode == UWBMode::TAG_TDOA) {
        bool transitionApplied = true;
        if (m_Params.dynamicAnchorPosEnabled == 0) {
            transitionApplied = ApplyTagRuntimeAnchorsTransaction(true, true);
            if (transitionApplied) {
                UWBTagTDoA::ApplyDynamicAnchorPositioningEnabled(m_Params.dynamicAnchorPosEnabled);
                rememberAcceptedStaticAnchorConfig(m_Params);
            }
        } else {
            UWBTagTDoA::ApplyDynamicAnchorPositioningEnabled(m_Params.dynamicAnchorPosEnabled);
            if (UWBTagTDoA::AreDynamicPositionsReadyForEstimator()) {
                transitionApplied = ApplyStaticAnchorsToLiveBackends(false, true);
            } else {
                transitionApplied = ApplyTagRuntimeAnchorsTransaction(true, true);
                if (transitionApplied) {
                    rememberAcceptedStaticAnchorConfig(m_Params);
                }
            }
        }
        if (!transitionApplied) {
            LOG_ERROR("Rejected dynamic anchor positioning change; restoring previous UWB params");
            return rollbackParamsAndRuntime(previousParams, false, true, true);
        }
    }
#endif

#if defined(USE_DYNAMIC_ANCHOR_POSITIONS) && defined(USE_UWB_MODE_TDOA_TAG)
    if (m_Params.mode == UWBMode::TAG_TDOA
        && dynamicTagGeometryEnabled(m_Params)
        && isDynamicAnchorRuntimeParam(name)) {
        if (strcmp(name, "anchorPosLocked") == 0) {
            UWBTagTDoA::ApplyDynamicAnchorLockMask(m_Params.anchorPosLocked);
        } else if (isDynamicAnchorReinitRuntimeParam(name)) {
            UWBTagTDoA::ApplyDynamicAnchorRuntimeConfig(m_Params);
        }
#ifdef USE_RTLSLINK_BEACON_BACKEND
        if (isDynamicAnchorReinitRuntimeParam(name)) {
            ClearRtlslinkBeaconAnchors();
        }
#endif
    }
#endif

    if (m_Params.mode == UWBMode::TAG_TDOA) {
#ifdef USE_RTLSLINK_BEACON_BACKEND
        if (isRtlslinkRuntimeConfigParam(name)) {
            if (!ApplyStaticAnchorsToLiveBackends(false, true)) {
                LOG_ERROR("Rejected RTLSLink runtime config change; restoring previous UWB params");
                return rollbackParamsAndRuntime(previousParams, false, true);
            }
        }
#endif
        if (staticAnchorCommit) {
            if (!ApplyTagRuntimeAnchorsTransaction(true, true)) {
                LOG_ERROR("Rejected invalid UWB static anchor config; restoring previous accepted anchors");
                restoreAcceptedStaticAnchorConfig(m_Params, previousParams);
                return rollbackParamsAndRuntime(m_Params, true, true);
            }
            rememberAcceptedStaticAnchorConfig(m_Params);
        } else if (!dynamicTagGeometryEnabled(m_Params) && strcmp(name, "use2DEstimator") == 0) {
            if (!ApplyTagRuntimeAnchorsTransaction(true, true)) {
                LOG_ERROR("Rejected estimator mode change; static anchor geometry is incompatible");
                return rollbackParamsAndRuntime(previousParams, true, true);
            }
            rememberAcceptedStaticAnchorConfig(m_Params);
        } else if (isStaticAnchorGeometryParam(name)) {
#if defined(USE_DYNAMIC_ANCHOR_POSITIONS) && defined(USE_UWB_MODE_TDOA_TAG)
            if (m_Params.dynamicAnchorPosEnabled != 0 && UWBTagTDoA::AreDynamicPositionsReadyForEstimator()) {
                LOG_INFO("Static anchor parameter saved; dynamic anchor positioning is active");
            } else
#endif
            {
                LOG_INFO("Static anchor parameter saved; write anchorCount to apply live");
            }
        }
    }

    return result;
}

void UWBLittleFSFrontend::Update() {
#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
    if (m_Params.uwbEnable == 0) {
        return;
    }
#endif

    if (m_Backend) {
        m_Backend->Update();
    }
}

uint32_t UWBLittleFSFrontend::GetConnectedDevices() {
#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
    if (m_Params.uwbEnable == 0) {
        return 0;
    }
#endif

    if (m_Backend) {
        return m_Backend->GetNumberOfConnectedDevices();
    }
    return 0;
}

etl::vector<UWBAnchorParam, UWBParams::maxAnchorCount> UWBLittleFSFrontend::GetAnchors() {
    etl::vector<UWBAnchorParam, UWBParams::maxAnchorCount> anchors;
    anchors.reserve(UWBParams::maxAnchorCount);

    uint8_t anchorCount = min(m_Params.anchorCount, UWBParams::maxAnchorCount);

    if (anchorCount >= 1) anchors.push_back({m_Params.devId1, m_Params.x1, m_Params.y1, m_Params.z1});
    if (anchorCount >= 2) anchors.push_back({m_Params.devId2, m_Params.x2, m_Params.y2, m_Params.z2});
    if (anchorCount >= 3) anchors.push_back({m_Params.devId3, m_Params.x3, m_Params.y3, m_Params.z3});
    if (anchorCount >= 4) anchors.push_back({m_Params.devId4, m_Params.x4, m_Params.y4, m_Params.z4});
    if (anchorCount >= 5) anchors.push_back({m_Params.devId5, m_Params.x5, m_Params.y5, m_Params.z5});
    if (anchorCount >= 6) anchors.push_back({m_Params.devId6, m_Params.x6, m_Params.y6, m_Params.z6});
    if (anchorCount >= 7) anchors.push_back({m_Params.devId7, m_Params.x7, m_Params.y7, m_Params.z7});
    if (anchorCount >= 8) anchors.push_back({m_Params.devId8, m_Params.x8, m_Params.y8, m_Params.z8});
    
    return anchors;
}

namespace Front {
    UWBLittleFSFrontend uwbLittleFSFront;
}
