#include "param_registry.hpp"

#include <string.h>

#include "config/features.hpp"
#include "front.hpp"

namespace rtls::params {
namespace {

static constexpr RegistryEntry kEntries[] = {
    {"WIFI_MODE", "wifi", "mode"},
    {"WIFI_SSID_AP", "wifi", "ssidAP"},
    {"WIFI_PSWD_AP", "wifi", "pswdAP"},
    {"WIFI_SSID_ST", "wifi", "ssidST"},
    {"WIFI_PSWD_ST", "wifi", "pswdST"},
    {"WIFI_GCS_IP", "wifi", "gcsIp"},
    {"WIFI_UART_PORT", "wifi", "udpPort"},
    {"WIFI_OTA_EN", "wifi", "enableWebServer"},
    {"WIFI_UART_EN", "wifi", "enableUartBridge"},
    {"WIFI_MGMT_EN", "wifi", "enableMavlinkManagement"},
    {"WIFI_MGMT_PORT", "wifi", "mavlinkManagementPort"},
    {"WIFI_LOG_PORT", "wifi", "logUdpPort"},
    {"WIFI_LOG_SER", "wifi", "logSerialEnabled"},
    {"WIFI_LOG_UDP", "wifi", "logUdpEnabled"},

    {"APP_LED2_PIN", "app", "led2Pin"},
    {"APP_LED2_STATE", "app", "led2State"},

    {"UWB_MODE", "uwb", "mode"},
#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
    {"UWB_ENABLE", "uwb", "uwbEnable"},
#endif
    {"UWB_ADDR", "uwb", "devShortAddr"},
    {"UWB_ANCH_CNT", "uwb", "anchorCount"},
    {"UWB_A1_ID", "uwb", "devId1"},
    {"UWB_A1_X", "uwb", "x1"},
    {"UWB_A1_Y", "uwb", "y1"},
    {"UWB_A1_Z", "uwb", "z1"},
    {"UWB_A2_ID", "uwb", "devId2"},
    {"UWB_A2_X", "uwb", "x2"},
    {"UWB_A2_Y", "uwb", "y2"},
    {"UWB_A2_Z", "uwb", "z2"},
    {"UWB_A3_ID", "uwb", "devId3"},
    {"UWB_A3_X", "uwb", "x3"},
    {"UWB_A3_Y", "uwb", "y3"},
    {"UWB_A3_Z", "uwb", "z3"},
    {"UWB_A4_ID", "uwb", "devId4"},
    {"UWB_A4_X", "uwb", "x4"},
    {"UWB_A4_Y", "uwb", "y4"},
    {"UWB_A4_Z", "uwb", "z4"},
    {"UWB_A5_ID", "uwb", "devId5"},
    {"UWB_A5_X", "uwb", "x5"},
    {"UWB_A5_Y", "uwb", "y5"},
    {"UWB_A5_Z", "uwb", "z5"},
    {"UWB_A6_ID", "uwb", "devId6"},
    {"UWB_A6_X", "uwb", "x6"},
    {"UWB_A6_Y", "uwb", "y6"},
    {"UWB_A6_Z", "uwb", "z6"},
    {"UWB_ADELAY", "uwb", "ADelay"},
    {"UWB_ORG_LAT", "uwb", "originLat"},
    {"UWB_ORG_LON", "uwb", "originLon"},
    {"UWB_ORG_ALT", "uwb", "originAlt"},
    {"UWB_MAV_SYS", "uwb", "mavlinkTargetSystemId"},
    {"UWB_OUT", "uwb", "outputBackend"},
    {"UWB_ROT_DEG", "uwb", "rotationDegrees"},
    {"UWB_Z_MODE", "uwb", "zCalcMode"},
    {"UWB_BCN_BIAS", "uwb", "rtlsBeaconAgeBiasMs"},
    {"UWB_BCN_SIG", "uwb", "rtlsBeaconTdoaSigmaFloorM"},
    {"UWB_BCN_GUARD", "uwb", "rtlsBeaconTdoaPhysicalGuardEnable"},
    {"UWB_BCN_GMRGN", "uwb", "rtlsBeaconTdoaPhysicalGuardMarginM"},
    {"UWB_RF_EN", "uwb", "rfForwardEnable"},
    {"UWB_RF_ID", "uwb", "rfForwardSensorId"},
    {"UWB_RF_ORIENT", "uwb", "rfForwardOrientation"},
    {"UWB_RF_SRCID", "uwb", "rfForwardPreserveSrcIds"},
    {"UWB_COV_EN", "uwb", "enableCovMatrix"},
    {"UWB_RMSE", "uwb", "rmseThreshold"},
    {"UWB_EST_2D", "uwb", "use2DEstimator"},
    {"UWB_CHAN", "uwb", "channel"},
    {"UWB_DW_MODE", "uwb", "dwMode"},
    {"UWB_TX_PWR", "uwb", "txPowerLevel"},
    {"UWB_SMARTPWR", "uwb", "smartPowerEnable"},
    {"UWB_SLOT_CNT", "uwb", "tdoaSlotCount"},
    {"UWB_SLOT_US", "uwb", "tdoaSlotDurationUs"},
    {"UWB_ATLM_EN", "uwb", "tdoaAnchorTelemetryEnable"},
    {"UWB_ATLM_MS", "uwb", "tdoaAnchorTelemetryIntervalMs"},
    {"UWB_ATLM_PORT", "uwb", "tdoaAnchorTelemetryPort"},
#ifdef ESP32S3_UWB_BOARD
    {"UWB_MATCH_POL", "uwb", "tdoaMatcherPolicy"},
#endif
    {"UWB_DYN_EN", "uwb", "dynamicAnchorPosEnabled"},
    {"UWB_LAYOUT", "uwb", "anchorLayout"},
    {"UWB_HEIGHT", "uwb", "anchorHeight"},
    {"UWB_LOCK_MASK", "uwb", "anchorPosLocked"},
    {"UWB_AVG_SAMP", "uwb", "distanceAvgSamples"},
    {"UWB_AMOD_MODE", "uwb", "tdoaAnchorModelMode"},
    {"UWB_AMOD_START", "uwb", "tdoaAnchorModelStartupCollect"},
    {"UWB_AMOD_WINMS", "uwb", "tdoaAnchorModelCollectWindowMs"},
    {"UWB_AMOD_MIN", "uwb", "tdoaAnchorModelMinSamplesPerPair"},
    {"UWB_AMOD_DOM", "uwb", "tdoaAnchorModelDomain"},
    {"UWB_AMOD_HTHR", "uwb", "tdoaAnchorModelHealthThresholdTicks"},
    {"UWB_AMOD_HWIN", "uwb", "tdoaAnchorModelHealthWindow"},
    {"UWB_AMOD_HQ", "uwb", "tdoaAnchorModelHealthQuorum"},
};

} // namespace

size_t Count()
{
    return sizeof(kEntries) / sizeof(kEntries[0]);
}

const RegistryEntry* Get(size_t index)
{
    return index < Count() ? &kEntries[index] : nullptr;
}

const RegistryEntry* FindById(const char* id, size_t len)
{
    if (id == nullptr) {
        return nullptr;
    }

    for (const RegistryEntry& entry : kEntries) {
        const size_t entryLen = strlen(entry.id);
        if (entryLen == len && strncmp(entry.id, id, len) == 0) {
            return &entry;
        }
    }
    return nullptr;
}

ErrorParam Read(const RegistryEntry& entry, char* value, uint32_t& len, ParamType& type)
{
    return Front::ReadGlobalParam(entry.group, entry.name, value, len, type);
}

ErrorParam Write(const RegistryEntry& entry, const char* value, uint32_t len)
{
    return Front::WriteGlobalParam(entry.group, entry.name, value, len);
}

} // namespace rtls::params
