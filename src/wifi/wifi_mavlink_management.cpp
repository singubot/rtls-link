#include "config/features.hpp"

#ifdef USE_WIFI_MAVLINK_MANAGEMENT

#include "wifi_mavlink_management.hpp"

#include <esp_mac.h>
#include <string.h>
#include <WiFi.h>
#include <rtlslink/mavlink.h>

#include "command_handler/command_handler.hpp"
#include "logging/logging.hpp"
#include "param_registry.hpp"
#include "protocol/rtls_binary_protocol.hpp"
#include "uwb/uwb_frontend_littlefs.hpp"
#include "version.hpp"

namespace {

static constexpr uint8_t kDefaultSystemId = 1;
static constexpr uint8_t kManagementComponentId = MAV_COMP_ID_ONBOARD_COMPUTER;

size_t boundedStringLen(const char* value, size_t maxLen)
{
    size_t len = 0;
    while (len < maxLen && value[len] != '\0') {
        len++;
    }
    return len;
}

void copyBoundedString(char* dst, size_t dstLen, const char* src)
{
    if (dst == nullptr || dstLen == 0) {
        return;
    }
    memset(dst, 0, dstLen);
    if (src == nullptr) {
        return;
    }
    const size_t len = boundedStringLen(src, dstLen - 1);
    memcpy(dst, src, len);
}

uint8_t paramExtType(ParamType type)
{
    switch (type) {
        case ParamType::UINT8:
        case ParamType::BOOL:
        case ParamType::ENUM:
            return MAV_PARAM_EXT_TYPE_UINT8;
        case ParamType::UINT16:
            return MAV_PARAM_EXT_TYPE_UINT16;
        case ParamType::UINT32:
            return MAV_PARAM_EXT_TYPE_UINT32;
        case ParamType::INT8:
            return MAV_PARAM_EXT_TYPE_INT8;
        case ParamType::INT16:
            return MAV_PARAM_EXT_TYPE_INT16;
        case ParamType::INT32:
            return MAV_PARAM_EXT_TYPE_INT32;
        case ParamType::FLOAT:
            return MAV_PARAM_EXT_TYPE_REAL32;
        case ParamType::DOUBLE:
            return MAV_PARAM_EXT_TYPE_REAL64;
        case ParamType::STRING:
            return MAV_PARAM_EXT_TYPE_CUSTOM;
        default:
            return MAV_PARAM_EXT_TYPE_CUSTOM;
    }
}

const char* commandToString(uint16_t command, const char* name)
{
    switch (command) {
        case RTLS_COMMAND_FIRMWARE_INFO: return "firmware-info";
        case RTLS_COMMAND_SAVE_CONFIG: return "save-config";
        case RTLS_COMMAND_LOAD_CONFIG: return "load-config";
        case RTLS_COMMAND_BACKUP_CONFIG: return "backup-config";
        case RTLS_COMMAND_LIST_CONFIGS: return "list-configs";
        case RTLS_COMMAND_TOGGLE_LED2: return "toggle-led2";
        case RTLS_COMMAND_GET_LED2_STATE: return "get-led2-state";
        case RTLS_COMMAND_TDOA_DISTANCES: return "tdoa-distances";
        case RTLS_COMMAND_TDOA_ANCHOR_STATS: return "tdoa-anchor-stats";
        case RTLS_COMMAND_TDOA_ANCHOR_MODEL_RESET: return "tdoa-anchor-model-reset";
        case RTLS_COMMAND_TDOA_ANCHOR_MODEL_COLLECT_START: return "tdoa-anchor-model-collect-start";
        case RTLS_COMMAND_TDOA_ANCHOR_MODEL_COLLECT_STATUS: return "tdoa-anchor-model-collect-status";
        case RTLS_COMMAND_TDOA_ANCHOR_MODEL_LOCK: return "tdoa-anchor-model-lock";
        case RTLS_COMMAND_TDOA_ANCHOR_MODEL_STATUS: return "tdoa-anchor-model-status";
        case RTLS_COMMAND_TDOA_ANCHOR_MODEL_EXPORT: return "tdoa-anchor-model-export";
        case RTLS_COMMAND_TDOA_ESTIMATOR_STATS_RESET: return "tdoa-estimator-stats-reset";
        default:
            break;
    }

    static char namedCommand[64];
    const char* prefix = nullptr;
    switch (command) {
        case RTLS_COMMAND_SAVE_CONFIG_AS: prefix = "save-config-as -name "; break;
        case RTLS_COMMAND_LOAD_CONFIG_NAMED: prefix = "load-config-named -name "; break;
        case RTLS_COMMAND_READ_CONFIG_NAMED: prefix = "read-config-named -name "; break;
        case RTLS_COMMAND_DELETE_CONFIG: prefix = "delete-config -name "; break;
        default: return nullptr;
    }

    snprintf(namedCommand, sizeof(namedCommand), "%s%s", prefix, name == nullptr ? "" : name);
    return namedCommand;
}

uint8_t resultFromStatus(rtls::protocol::StatusCode status)
{
    switch (status) {
        case rtls::protocol::StatusCode::Ok:
            return RTLS_RESULT_ACCEPTED;
        case rtls::protocol::StatusCode::NotSupported:
            return RTLS_RESULT_UNSUPPORTED;
        case rtls::protocol::StatusCode::InvalidMode:
            return RTLS_RESULT_INVALID_MODE;
        case rtls::protocol::StatusCode::NotFound:
            return RTLS_RESULT_NOT_FOUND;
        case rtls::protocol::StatusCode::InvalidName:
            return RTLS_RESULT_INVALID_ARGUMENT;
        default:
            return RTLS_RESULT_FAILED;
    }
}

} // namespace

WifiMavlinkManagement::WifiMavlinkManagement(uint16_t port, const WifiParams& wifiParams)
    : m_Port(port)
    , m_WifiParams(wifiParams)
    , m_ParseStatus(new mavlink_status_t{})
{
    if (m_Udp.begin(m_Port) == 1) {
        LOG_INFO("MAVLink management UDP listening on port %u", static_cast<unsigned int>(m_Port));
    } else {
        LOG_ERROR("MAVLink management UDP bind failed on port %u", static_cast<unsigned int>(m_Port));
    }
}

WifiMavlinkManagement::~WifiMavlinkManagement()
{
    delete m_ParseStatus;
}

void WifiMavlinkManagement::SetTelemetryCallback(TelemetryCallback callback)
{
    m_TelemetryCallback = callback;
}

void WifiMavlinkManagement::Update()
{
    ProcessPacket();

    const uint32_t now = millis();
    if (now - m_LastStatusMs >= kStatusIntervalMs) {
        m_LastStatusMs = now;
        SendHeartbeat();
        SendDeviceStatus();
    }
}

void WifiMavlinkManagement::ProcessPacket()
{
    int packetSize = m_Udp.parsePacket();
    while (packetSize > 0) {
        m_RemoteIp = m_Udp.remoteIP();
        m_RemotePort = m_Udp.remotePort();

        uint8_t buffer[kRxBufferSize];
        const int readLen = m_Udp.read(buffer, sizeof(buffer));
        for (int i = 0; i < readLen; i++) {
            mavlink_message_t message;
            if (mavlink_parse_char(MAVLINK_COMM_2, buffer[i], &message, m_ParseStatus)) {
                HandleMessage(message);
            }
        }

        packetSize = m_Udp.parsePacket();
    }
}

void WifiMavlinkManagement::HandleMessage(const mavlink_message_t& message)
{
    switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            break;
        case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST:
            HandleParamExtRequestList(message);
            break;
        case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
            HandleParamExtRequestRead(message);
            break;
        case MAVLINK_MSG_ID_PARAM_EXT_SET:
            HandleParamExtSet(message);
            break;
        case MAVLINK_MSG_ID_RTLS_COMMAND:
            HandleRtlsCommand(message);
            break;
        default:
            break;
    }
}

void WifiMavlinkManagement::HandleParamExtRequestList(const mavlink_message_t& message)
{
    mavlink_param_ext_request_list_t request;
    mavlink_msg_param_ext_request_list_decode(&message, &request);
    if (!IsTarget(request.target_system, request.target_component)) {
        return;
    }

    for (size_t i = 0; i < rtls::params::Count(); i++) {
        SendParamExtValue(i);
        delay(2);
    }
}

void WifiMavlinkManagement::HandleParamExtRequestRead(const mavlink_message_t& message)
{
    mavlink_param_ext_request_read_t request;
    mavlink_msg_param_ext_request_read_decode(&message, &request);
    if (!IsTarget(request.target_system, request.target_component)) {
        return;
    }

    if (request.param_index >= 0) {
        SendParamExtValue(static_cast<size_t>(request.param_index));
        return;
    }

    const size_t idLen = boundedStringLen(request.param_id, MAVLINK_MSG_PARAM_EXT_REQUEST_READ_FIELD_PARAM_ID_LEN);
    const rtls::params::RegistryEntry* entry = rtls::params::FindById(request.param_id, idLen);
    if (entry == nullptr) {
        SendParamExtAck(request.param_id, "", ParamType::UNDEFINED, PARAM_ACK_VALUE_UNSUPPORTED);
        return;
    }

    const size_t index = static_cast<size_t>(entry - rtls::params::Get(0));
    SendParamExtValue(index);
}

void WifiMavlinkManagement::HandleParamExtSet(const mavlink_message_t& message)
{
    mavlink_param_ext_set_t request;
    mavlink_msg_param_ext_set_decode(&message, &request);
    if (!IsTarget(request.target_system, request.target_component)) {
        return;
    }

    const size_t idLen = boundedStringLen(request.param_id, MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_ID_LEN);
    const rtls::params::RegistryEntry* entry = rtls::params::FindById(request.param_id, idLen);
    if (entry == nullptr) {
        SendParamExtAck(request.param_id, "", ParamType::UNDEFINED, PARAM_ACK_VALUE_UNSUPPORTED);
        return;
    }

    const uint32_t valueLen = static_cast<uint32_t>(
        boundedStringLen(request.param_value, MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_LEN));
    const ErrorParam result = rtls::params::Write(*entry, request.param_value, valueLen);

    char value[128] = {};
    uint32_t readLen = sizeof(value) - 1;
    ParamType type = ParamType::UNDEFINED;
    if (result == ErrorParam::OK) {
        rtls::params::Read(*entry, value, readLen, type);
        value[readLen] = '\0';
    }

    SendParamExtAck(entry->id,
                    result == ErrorParam::OK ? value : "",
                    type,
                    result == ErrorParam::OK ? PARAM_ACK_ACCEPTED : PARAM_ACK_FAILED);
}

void WifiMavlinkManagement::HandleRtlsCommand(const mavlink_message_t& message)
{
    mavlink_rtls_command_t command;
    mavlink_msg_rtls_command_decode(&message, &command);

    char name[33] = {};
    const size_t nameLen = command.name_len > 32 ? 32 : command.name_len;
    memcpy(name, command.name, nameLen);

    if (command.command == RTLS_COMMAND_REBOOT) {
        SendTextResponse(command.request_id, command.command, RTLS_RESULT_ACCEPTED, "OK");
        delay(50);
        ESP.restart();
        return;
    }

    const char* commandString = commandToString(command.command, name);
    if (commandString == nullptr) {
        SendTextResponse(command.request_id, command.command, RTLS_RESULT_UNSUPPORTED, "Unsupported command");
        return;
    }

    CommandBinaryFrame binaryResponse;
    if (CommandHandler::TryExecuteBinaryCommand(commandString, binaryResponse)) {
        const uint8_t status = binaryResponse.Size() >= rtls::protocol::kFrameHeaderSize
            ? binaryResponse.Data()[8]
            : static_cast<uint8_t>(rtls::protocol::StatusCode::Error);
        SendCommandResponse(command.request_id,
                            command.command,
                            resultFromStatus(static_cast<rtls::protocol::StatusCode>(status)),
                            RTLS_PAYLOAD_TYPE_BINARY_FRAME,
                            binaryResponse.Data(),
                            binaryResponse.Size());
        return;
    }

    String result = CommandHandler::ExecuteCommand(commandString);
    SendTextResponse(command.request_id,
                     command.command,
                     result.startsWith("Error") ? RTLS_RESULT_FAILED : RTLS_RESULT_ACCEPTED,
                     result.c_str());
}

void WifiMavlinkManagement::SendHeartbeat()
{
    mavlink_message_t message;
    mavlink_msg_heartbeat_pack(SystemId(),
                               ComponentId(),
                               &message,
                               MAV_TYPE_GENERIC,
                               MAV_AUTOPILOT_INVALID,
                               0,
                               0,
                               MAV_STATE_ACTIVE);
    SendMessageToGcs(message);
}

void WifiMavlinkManagement::SendDeviceStatus()
{
    DeviceTelemetry telemetry = {};
    if (m_TelemetryCallback.is_valid()) {
        telemetry = m_TelemetryCallback();
    }

    uint32_t flags = 0;
    if (telemetry.sending_pos) flags |= RTLS_DEVICE_STATUS_FLAG_SENDING_POSITION;
    if (telemetry.origin_sent) flags |= RTLS_DEVICE_STATUS_FLAG_ORIGIN_SENT;
    if (telemetry.rf_enabled) flags |= RTLS_DEVICE_STATUS_FLAG_RANGEFINDER_ENABLED;
    if (telemetry.rf_healthy) flags |= RTLS_DEVICE_STATUS_FLAG_RANGEFINDER_HEALTHY;
    if (telemetry.uwb_enabled) flags |= RTLS_DEVICE_STATUS_FLAG_UWB_ENABLED;
    if (telemetry.rf_forward_enabled) flags |= RTLS_DEVICE_STATUS_FLAG_RF_FORWARD_ENABLED;
    if (m_WifiParams.logSerialEnabled) flags |= RTLS_DEVICE_STATUS_FLAG_LOG_SERIAL_ENABLED;
    if (m_WifiParams.logUdpEnabled) flags |= RTLS_DEVICE_STATUS_FLAG_LOG_UDP_ENABLED;
#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    if (telemetry.dynamic_anchors_enabled) flags |= RTLS_DEVICE_STATUS_FLAG_DYNAMIC_ANCHORS_ENABLED;
#endif

    const bool isAP = (WiFi.getMode() == WIFI_AP);
    const IPAddress deviceIp = isAP ? WiFi.softAPIP() : WiFi.localIP();
    uint8_t ip[4] = {deviceIp[0], deviceIp[1], deviceIp[2], deviceIp[3]};
    uint8_t mac[6] = {};
    esp_read_mac(mac, isAP ? ESP_MAC_WIFI_SOFTAP : ESP_MAC_WIFI_STA);

    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
    char shortAddr[8] = {};
    if (uwbParams.devShortAddr[0] != '\0') {
        shortAddr[0] = uwbParams.devShortAddr[0];
        shortAddr[1] = uwbParams.devShortAddr[1] != '\0' ? uwbParams.devShortAddr[1] : '\0';
    } else {
        shortAddr[0] = '0';
    }

    char deviceType[16] = {};
    char firmwareVersion[16] = {};
    copyBoundedString(deviceType, sizeof(deviceType), DEVICE_TYPE);
    copyBoundedString(firmwareVersion, sizeof(firmwareVersion), FIRMWARE_VERSION);

    uint8_t dynamicAnchorCount = 0;
    uint8_t dynamicAnchorIds[4] = {};
    int32_t dynamicAnchorXmm[4] = {};
    int32_t dynamicAnchorYmm[4] = {};
    int32_t dynamicAnchorZmm[4] = {};
#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    if (telemetry.dynamic_anchors_enabled) {
        dynamicAnchorCount = telemetry.dynamic_anchor_count > 4 ? 4 : telemetry.dynamic_anchor_count;
        for (uint8_t i = 0; i < dynamicAnchorCount; i++) {
            dynamicAnchorIds[i] = telemetry.dynamic_anchors[i].id;
            dynamicAnchorXmm[i] = rtls::protocol::MetersToMillimeters(telemetry.dynamic_anchors[i].x);
            dynamicAnchorYmm[i] = rtls::protocol::MetersToMillimeters(telemetry.dynamic_anchors[i].y);
            dynamicAnchorZmm[i] = rtls::protocol::MetersToMillimeters(telemetry.dynamic_anchors[i].z);
        }
    }
#endif

    mavlink_message_t message;
    mavlink_msg_rtls_device_status_pack(SystemId(),
                                        ComponentId(),
                                        &message,
                                        millis(),
                                        flags,
                                        telemetry.avg_rate_cHz,
                                        telemetry.min_rate_cHz,
                                        telemetry.max_rate_cHz,
                                        m_WifiParams.logUdpPort,
                                        static_cast<uint8_t>(uwbParams.mode),
                                        telemetry.anchors_seen,
                                        uwbParams.mavlinkTargetSystemId,
#ifdef USE_LOGGING
                                        rtls::log::Logger::getCompiledLogLevel(),
#else
                                        0,
#endif
                                        dynamicAnchorCount,
                                        dynamicAnchorIds,
                                        dynamicAnchorXmm,
                                        dynamicAnchorYmm,
                                        dynamicAnchorZmm,
                                        ip,
                                        mac,
                                        deviceType,
                                        shortAddr,
                                        firmwareVersion);
    SendMessageToGcs(message);
}

void WifiMavlinkManagement::SendParamExtValue(size_t index)
{
    const rtls::params::RegistryEntry* entry = rtls::params::Get(index);
    if (entry == nullptr) {
        return;
    }

    char value[128] = {};
    uint32_t len = sizeof(value) - 1;
    ParamType type = ParamType::UNDEFINED;
    if (rtls::params::Read(*entry, value, len, type) != ErrorParam::OK) {
        return;
    }
    value[len] = '\0';

    char id[16] = {};
    copyBoundedString(id, sizeof(id), entry->id);

    mavlink_message_t message;
    mavlink_msg_param_ext_value_pack(SystemId(),
                                     ComponentId(),
                                     &message,
                                     id,
                                     value,
                                     paramExtType(type),
                                     static_cast<uint16_t>(rtls::params::Count()),
                                     static_cast<uint16_t>(index));
    SendMessage(message);
}

void WifiMavlinkManagement::SendParamExtAck(const char* id, const char* value, ParamType type, uint8_t result)
{
    char paramId[16] = {};
    char paramValue[128] = {};
    copyBoundedString(paramId, sizeof(paramId), id);
    copyBoundedString(paramValue, sizeof(paramValue), value);

    mavlink_message_t message;
    mavlink_msg_param_ext_ack_pack(SystemId(),
                                   ComponentId(),
                                   &message,
                                   paramId,
                                   paramValue,
                                   paramExtType(type),
                                   result);
    SendMessage(message);
}

void WifiMavlinkManagement::SendCommandResponse(uint32_t requestId,
                                                uint16_t command,
                                                uint8_t result,
                                                uint8_t payloadType,
                                                const uint8_t* payload,
                                                size_t payloadLen)
{
    static constexpr size_t kChunkSize = 220;
    const uint8_t chunkCount = payloadLen == 0
        ? 1
        : static_cast<uint8_t>((payloadLen + kChunkSize - 1) / kChunkSize);

    for (uint8_t chunk = 0; chunk < chunkCount; chunk++) {
        const size_t offset = static_cast<size_t>(chunk) * kChunkSize;
        const size_t remaining = payloadLen > offset ? payloadLen - offset : 0;
        const uint8_t chunkLen = static_cast<uint8_t>(remaining > kChunkSize ? kChunkSize : remaining);
        uint8_t payloadBuffer[kChunkSize] = {};
        if (payload != nullptr && chunkLen > 0) {
            memcpy(payloadBuffer, payload + offset, chunkLen);
        }

        mavlink_message_t message;
        mavlink_msg_rtls_command_response_pack(SystemId(),
                                               ComponentId(),
                                               &message,
                                               requestId,
                                               command,
                                               result,
                                               payloadLen == 0 ? RTLS_PAYLOAD_TYPE_NONE : payloadType,
                                               chunk,
                                               chunkCount,
                                               chunkLen,
                                               payloadBuffer);
        SendMessage(message);
        delay(2);
    }
}

void WifiMavlinkManagement::SendTextResponse(uint32_t requestId, uint16_t command, uint8_t result, const char* text)
{
    uint8_t payload[220] = {};
    const size_t len = text == nullptr ? 0 : boundedStringLen(text, sizeof(payload) - 1);
    if (len > 0) {
        memcpy(payload, text, len);
    }

    mavlink_message_t message;
    mavlink_msg_rtls_command_response_pack(SystemId(),
                                           ComponentId(),
                                           &message,
                                           requestId,
                                           command,
                                           result,
                                           RTLS_PAYLOAD_TYPE_TEXT,
                                           0,
                                           1,
                                           static_cast<uint8_t>(len),
                                           payload);
    SendMessage(message);
}

void WifiMavlinkManagement::SendMessage(const mavlink_message_t& message)
{
    if (m_RemotePort == 0) {
        SendMessageToGcs(message);
        return;
    }

    SendMessageTo(message, m_RemoteIp, m_RemotePort);
}

void WifiMavlinkManagement::SendMessageToGcs(const mavlink_message_t& message)
{
    IPAddress targetIp;
    if (!targetIp.fromString(m_WifiParams.gcsIp.data())) {
        return;
    }
    SendMessageTo(message, targetIp, m_Port);
}

void WifiMavlinkManagement::SendMessageTo(const mavlink_message_t& message, IPAddress targetIp, uint16_t targetPort)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
    if (m_Udp.beginPacket(targetIp, targetPort) != 1) {
        return;
    }
    m_Udp.write(buffer, len);
    m_Udp.endPacket();
}

bool WifiMavlinkManagement::IsTarget(uint8_t targetSystem, uint8_t targetComponent) const
{
    const bool systemMatches = targetSystem == 0 || targetSystem == SystemId();
    const bool componentMatches = targetComponent == 0 || targetComponent == ComponentId();
    return systemMatches && componentMatches;
}

uint8_t WifiMavlinkManagement::SystemId() const
{
    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
    if (uwbParams.devShortAddr[0] >= '0' && uwbParams.devShortAddr[0] <= '9') {
        return static_cast<uint8_t>((uwbParams.devShortAddr[0] - '0') + 1);
    }
    return kDefaultSystemId;
}

uint8_t WifiMavlinkManagement::ComponentId() const
{
    return kManagementComponentId;
}

#endif // USE_WIFI_MAVLINK_MANAGEMENT
