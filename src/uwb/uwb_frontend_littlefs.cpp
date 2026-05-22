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

#ifdef USE_RTLSLINK_BEACON_BACKEND
namespace {

bool isRtlslinkAnchorConfigParam(const char* name)
{
    if (strcmp(name, "outputBackend") == 0 || strcmp(name, "rotationDegrees") == 0 || strcmp(name, "anchorCount") == 0) {
        return true;
    }
    if (name[0] == 'x' || name[0] == 'y' || name[0] == 'z') {
        return name[1] >= '1' && name[1] <= '6' && name[2] == '\0';
    }
    if (strncmp(name, "devId", 5) == 0) {
        return name[5] >= '1' && name[5] <= '6' && name[6] == '\0';
    }
    return false;
}

} // namespace
#endif

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
    }
#else
    InitBackendForCurrentMode();
#endif

    LOG_INFO("UWB frontend initialized");
}

ErrorParam UWBLittleFSFrontend::SetParam(const char* name, const void* data, uint32_t len) {
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

#ifdef USE_RTLSLINK_BEACON_BACKEND
    if (m_Params.mode == UWBMode::TAG_TDOA && isRtlslinkAnchorConfigParam(name)) {
#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
        if (m_Params.dynamicAnchorPosEnabled != 0) {
            LOG_INFO("RTLSLink beacon static anchor config skipped while dynamic anchors are enabled");
            return result;
        }
#endif
        auto anchors = GetAnchors();
        App::ConfigureRtlslinkBeaconAnchors(anchors);
    }
#endif

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
    
    return anchors;
}

namespace Front {
    UWBLittleFSFrontend uwbLittleFSFront;
}
