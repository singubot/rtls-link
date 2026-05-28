#include "config/features.hpp"

#include "uwb_frontend_littlefs.hpp"
#include "logging/logging.hpp"
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
}

void UWBLittleFSFrontend::Init() {
    LittleFSFrontend<UWBParams>::Init();
    LOG_INFO("UWB frontend initializing");

#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
    if (m_Params.uwbEnable == 0) {
        LOG_WARN("UWB backend disabled by parameter (uwb.uwbEnable=0)");
    } else {
        InitBackendForCurrentMode();
        ApplyLoadedRuntimeConfig();
    }
#else
    InitBackendForCurrentMode();
    ApplyLoadedRuntimeConfig();
#endif

    LOG_INFO("UWB frontend initialized");
}

ErrorParam UWBLittleFSFrontend::LoadParams() {
    const ErrorParam result = LittleFSFrontend<UWBParams>::LoadParams();
    if (result == ErrorParam::OK) {
        ApplyLoadedRuntimeConfig();
    }
    return result;
}

void UWBLittleFSFrontend::ApplyLoadedRuntimeConfig()
{
    if (m_Params.mode != UWBMode::TAG_TDOA || m_Backend == nullptr) {
        return;
    }

#if defined(USE_DYNAMIC_ANCHOR_POSITIONS) && defined(USE_UWB_MODE_TDOA_TAG)
    UWBTagTDoA::ApplyDynamicAnchorPositioningEnabled(m_Params.dynamicAnchorPosEnabled);
#endif

    if (ApplyStaticAnchorsToLiveBackends(true, true)) {
        rememberAcceptedStaticAnchorConfig(m_Params);
    }
}

bool UWBLittleFSFrontend::ApplyStaticAnchorsToLiveBackends(bool applyEstimator, bool applyRtlslinkBeacon)
{
    if (m_Params.mode != UWBMode::TAG_TDOA) {
        return true;
    }

    auto anchors = GetAnchors();

#ifdef USE_UWB_MODE_TDOA_TAG
    if (applyEstimator && !UWBTagTDoA::ValidateStaticAnchors(anchors)) {
        return false;
    }
#endif

#if defined(USE_DYNAMIC_ANCHOR_POSITIONS) && defined(USE_UWB_MODE_TDOA_TAG)
    const bool dynamicParamEnabled = (m_Params.dynamicAnchorPosEnabled != 0);
    const bool dynamicRunning = UWBTagTDoA::IsDynamicPositioningEnabled();
    if (dynamicParamEnabled && dynamicRunning) {
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
            applyRtlslinkBeacon = false;
        }
#endif
        if (applyEstimator) {
            LOG_INFO("Static anchor config saved; dynamic anchor positioning is pending");
        }
    }
    if (dynamicParamEnabled && !dynamicRunning && applyEstimator) {
        LOG_INFO("Dynamic anchor positioning requires reboot; applying static anchors live until reboot");
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

    ErrorParam result = LittleFSFrontend<UWBParams>::SetParam(name, data, len);
    if (result != ErrorParam::OK) {
        return result;
    }

#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
    if (strcmp(name, "uwbEnable") == 0) {
        if (m_Params.uwbEnable == 0) {
            LOG_INFO("UWB runtime disabled");
        } else {
            LOG_INFO("UWB runtime enabled");
            InitBackendForCurrentMode();
            ApplyLoadedRuntimeConfig();
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
            transitionApplied = ApplyStaticAnchorsToLiveBackends(true, true);
            if (transitionApplied) {
                UWBTagTDoA::ApplyDynamicAnchorPositioningEnabled(m_Params.dynamicAnchorPosEnabled);
                rememberAcceptedStaticAnchorConfig(m_Params);
            }
        } else {
            UWBTagTDoA::ApplyDynamicAnchorPositioningEnabled(m_Params.dynamicAnchorPosEnabled);
            if (UWBTagTDoA::AreDynamicPositionsReadyForEstimator()) {
                transitionApplied = ApplyStaticAnchorsToLiveBackends(false, true);
            } else {
                transitionApplied = ApplyStaticAnchorsToLiveBackends(true, true);
                if (transitionApplied) {
                    rememberAcceptedStaticAnchorConfig(m_Params);
                }
            }
        }
        if (!transitionApplied) {
            LOG_ERROR("Rejected dynamic anchor positioning change; restoring previous UWB params");
            m_Params = previousParams;
            const ErrorParam rollbackResult = SaveParams();
            if (rollbackResult != ErrorParam::OK) {
                return rollbackResult;
            }
            return ErrorParam::INVALID_DATA;
        }
    }
#endif

    if (m_Params.mode == UWBMode::TAG_TDOA) {
#ifdef USE_RTLSLINK_BEACON_BACKEND
        if (isRtlslinkRuntimeConfigParam(name)) {
            ApplyStaticAnchorsToLiveBackends(false, true);
        }
#endif
        if (staticAnchorCommit) {
            if (!ApplyStaticAnchorsToLiveBackends(true, true)) {
                LOG_ERROR("Rejected invalid UWB static anchor config; restoring previous accepted anchors");
                restoreAcceptedStaticAnchorConfig(m_Params, previousParams);
                const ErrorParam rollbackResult = SaveParams();
                ApplyStaticAnchorsToLiveBackends(true, true);
                if (rollbackResult != ErrorParam::OK) {
                    return rollbackResult;
                }
                return ErrorParam::INVALID_DATA;
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
