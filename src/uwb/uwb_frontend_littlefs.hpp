#pragma once

#include "config/features.hpp"

#include "uwb_params.hpp"
#include "../littlefs_frontend.hpp"
#include "uwb_backend.hpp"

class UWBLittleFSFrontend : public LittleFSFrontend<UWBParams> {
public:
    UWBLittleFSFrontend() : LittleFSFrontend<UWBParams>("uwb") {}

    virtual void Init() override;
    virtual void Update() override;
    virtual ErrorParam SetParam(const char* name, const void* data, uint32_t len) override;

    virtual etl::span<const ParamDef> GetParamLayout() const override {
        return etl::span<const ParamDef>(s_ParamDefs, sizeof(s_ParamDefs)/sizeof(ParamDef));
    }

    virtual const etl::string_view GetParamGroup() const override {
        return etl::string_view("uwb");
    }

    UWBParams& GetParams() {
        return m_Params;
    }

    uint32_t GetConnectedDevices();

protected:
    void InitBackendForCurrentMode();
    etl::vector<UWBAnchorParam, UWBParams::maxAnchorCount> GetAnchors();

    UWBBackend* m_Backend = nullptr;

public:
    static constexpr ParamDef s_ParamDefs[] = {
        PARAM_DEF(UWBParams, mode),
#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
        PARAM_DEF(UWBParams, uwbEnable),
#endif
        PARAM_DEF(UWBParams, devShortAddr),
        PARAM_DEF(UWBParams, anchorCount),
        PARAM_DEF(UWBParams, devId1),
        PARAM_DEF(UWBParams, x1),
        PARAM_DEF(UWBParams, y1),
        PARAM_DEF(UWBParams, z1),
        PARAM_DEF(UWBParams, devId2),
        PARAM_DEF(UWBParams, x2),
        PARAM_DEF(UWBParams, y2),
        PARAM_DEF(UWBParams, z2),
        PARAM_DEF(UWBParams, devId3),
        PARAM_DEF(UWBParams, x3),
        PARAM_DEF(UWBParams, y3),
        PARAM_DEF(UWBParams, z3),
        PARAM_DEF(UWBParams, devId4),
        PARAM_DEF(UWBParams, x4),
        PARAM_DEF(UWBParams, y4),
        PARAM_DEF(UWBParams, z4),
        PARAM_DEF(UWBParams, devId5),
        PARAM_DEF(UWBParams, x5),
        PARAM_DEF(UWBParams, y5),
        PARAM_DEF(UWBParams, z5),
        PARAM_DEF(UWBParams, devId6),
        PARAM_DEF(UWBParams, x6),
        PARAM_DEF(UWBParams, y6),
        PARAM_DEF(UWBParams, z6),
        PARAM_DEF(UWBParams, ADelay),
        PARAM_DEF(UWBParams, originLat),
        PARAM_DEF(UWBParams, originLon),
        PARAM_DEF(UWBParams, originAlt),
        PARAM_DEF(UWBParams, mavlinkTargetSystemId),
        PARAM_DEF(UWBParams, outputBackend),
        PARAM_DEF(UWBParams, rotationDegrees),
        PARAM_DEF(UWBParams, zCalcMode),
        PARAM_DEF(UWBParams, rtlsBeaconAgeBiasMs),
        PARAM_DEF(UWBParams, rtlsBeaconTdoaSigmaFloorM),
        PARAM_DEF(UWBParams, rtlsBeaconTdoaPhysicalGuardEnable),
        PARAM_DEF(UWBParams, rtlsBeaconTdoaPhysicalGuardMarginM),
        PARAM_DEF(UWBParams, rfForwardEnable),
        PARAM_DEF(UWBParams, rfForwardSensorId),
        PARAM_DEF(UWBParams, rfForwardOrientation),
        PARAM_DEF(UWBParams, rfForwardPreserveSrcIds),
        PARAM_DEF(UWBParams, enableCovMatrix),
        PARAM_DEF(UWBParams, rmseThreshold),
        PARAM_DEF(UWBParams, use2DEstimator),
        PARAM_DEF(UWBParams, channel),
        PARAM_DEF(UWBParams, dwMode),
        PARAM_DEF(UWBParams, txPowerLevel),
        PARAM_DEF(UWBParams, smartPowerEnable),
        PARAM_DEF(UWBParams, tdoaSlotCount),
        PARAM_DEF(UWBParams, tdoaSlotDurationUs),
#ifdef ESP32S3_UWB_BOARD
        PARAM_DEF(UWBParams, tdoaMatcherPolicy),
#endif
        PARAM_DEF(UWBParams, dynamicAnchorPosEnabled),
        PARAM_DEF(UWBParams, anchorLayout),
        PARAM_DEF(UWBParams, anchorHeight),
        PARAM_DEF(UWBParams, anchorPosLocked),
        PARAM_DEF(UWBParams, distanceAvgSamples),
        PARAM_DEF(UWBParams, tdoaAnchorModelMode),
        PARAM_DEF(UWBParams, tdoaAnchorModelStartupCollect),
        PARAM_DEF(UWBParams, tdoaAnchorModelCollectWindowMs),
        PARAM_DEF(UWBParams, tdoaAnchorModelMinSamplesPerPair),
        PARAM_DEF(UWBParams, tdoaAnchorModelDomain),
        PARAM_DEF(UWBParams, tdoaAnchorModelHealthThresholdTicks),
        PARAM_DEF(UWBParams, tdoaAnchorModelHealthWindow),
        PARAM_DEF(UWBParams, tdoaAnchorModelHealthQuorum)
    };
};

namespace Front {
    extern UWBLittleFSFrontend uwbLittleFSFront;
}
