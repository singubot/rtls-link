#include "config/features.hpp"

#include <Arduino.h>
#include <SimpleCLI.h>

#include "command_handler.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "front.hpp"
#include "scheduler.hpp"
#include "version.hpp"
#include "logging/logging.hpp"
#include "protocol/rtls_binary_protocol.hpp"

#include "uwb/uwb_frontend_littlefs.hpp"
#include "app/app_frontend_littlefs.hpp"
#ifdef USE_UWB_MODE_TDOA_ANCHOR
#include "anchor/tdoa_anchor_api.h"
#endif
#ifdef USE_UWB_MODE_TDOA_TAG
#include "uwb/tdoa_anchor_model_commands.hpp"
#endif
#ifdef USE_CONSOLE_CONFIG_MGMT
#include "config_manager/config_manager.hpp"
#endif

static constexpr int COMMAND_QUEUE_SIZE = 4;
static SimpleCLI simpleCLI(COMMAND_QUEUE_SIZE, COMMAND_QUEUE_SIZE);
static SemaphoreHandle_t commandQueueMutex;
static String commandResult;

#if defined(USE_CONSOLE_PARAM_RW) || defined(USE_CONSOLE_CONFIG_MGMT)
static bool IsUwbShortAddrName(const char* name);
static const UWBShortAddr* GetUwbShortAddrByName(const char* name);
static String UwbShortAddrToString(const UWBShortAddr& addr);
#endif

#ifdef USE_UWB_MODE_TDOA_TAG
static bool IsTagTdoaMode();
#endif

static bool commandStartsWith(const char* command, const char* prefix)
{
    return command != nullptr && strncmp(command, prefix, strlen(prefix)) == 0;
}

static String commandNameArg(const char* command)
{
    if (command == nullptr) {
        return "";
    }
    const char* nameFlag = strstr(command, "-name ");
    if (nameFlag == nullptr) {
        return "";
    }
    String name = String(nameFlag + 6);
    name.trim();
    return name;
}

static void appendAck(CommandBinaryFrame& outFrame,
                      rtls::protocol::StatusCode status,
                      const char* message)
{
    outFrame.Begin(rtls::protocol::FrameType::CommandAck, status);
    outFrame.AppendString(message == nullptr ? "" : message);
    outFrame.Finish();
}

#if defined(USE_CONSOLE_PARAM_RW) || defined(USE_CONSOLE_CONFIG_MGMT)
static bool isValueNumeric(const char* value)
{
    if (value == nullptr || value[0] == '\0') {
        return false;
    }
    bool hasDot = false;
    for (size_t i = 0; value[i] != '\0'; i++) {
        const char c = value[i];
        if (c == '.') {
            if (hasDot) {
                return false;
            }
            hasDot = true;
        } else if (c == '-' && i == 0) {
        } else if (c < '0' || c > '9') {
            return false;
        }
    }
    return true;
}

static void appendConfigValue(CommandBinaryFrame& outFrame,
                              const char* name,
                              const char* value,
                              bool numeric)
{
    outFrame.AppendString(name);
    outFrame.AppendU8(numeric ? 1 : 0);
    outFrame.AppendString(value == nullptr ? "" : value);
}

static void appendCurrentConfigSnapshot(CommandBinaryFrame& outFrame)
{
    outFrame.Begin(rtls::protocol::FrameType::ConfigSnapshot);
    const size_t groupCountOffset = outFrame.Size();
    outFrame.AppendU16(0);
    uint16_t groupCount = 0;

    etl::vector<IFrontend*, Front::MAX_FRONTENDS>& frontends = Front::Get();
    for (size_t i = 0; i < frontends.size(); i++) {
        IFrontend* frontend = frontends[i];
        if (frontend == nullptr) {
            continue;
        }
        outFrame.AppendString(frontend->GetParamGroup().data());
        const size_t paramCountOffset = outFrame.Size();
        outFrame.AppendU16(0);
        uint16_t paramCount = 0;

        etl::span<const ParamDef> params = frontend->GetParamLayout();
        for (size_t j = 0; j < params.size(); j++) {
            const ParamDef& param = params[j];
            char value[256] = {};
            uint32_t len = sizeof(value) - 1;
            ParamType type = ParamType::UNDEFINED;

            if (frontend->GetParam(param.name, value, len, type) != ErrorParam::OK) {
                continue;
            }
            value[len] = '\0';
            if (frontend->GetParamGroup() == "uwb" && IsUwbShortAddrName(param.name)) {
                const UWBShortAddr* addr = GetUwbShortAddrByName(param.name);
                String shortAddr = addr == nullptr ? String("") : UwbShortAddrToString(*addr);
                appendConfigValue(outFrame, param.name, shortAddr.c_str(), false);
            } else {
                appendConfigValue(outFrame, param.name, value, type != ParamType::STRING && isValueNumeric(value));
            }
            paramCount++;
        }

        outFrame.SetU16(paramCountOffset, paramCount);
        groupCount++;
    }

    outFrame.SetU16(groupCountOffset, groupCount);
    outFrame.Finish();
}

#endif

#ifdef USE_CONSOLE_CONFIG_MGMT
static void appendConfigFileSnapshot(CommandBinaryFrame& outFrame, const char* name)
{
    String path = String(ConfigManager::CONFIG_DIR) + "/" + name + ".txt";
    File file = LittleFS.open(path, "r");
    if (!file) {
        appendAck(outFrame, rtls::protocol::StatusCode::NotFound, "Failed to open config file");
        return;
    }

    outFrame.Begin(rtls::protocol::FrameType::ConfigSnapshot);
    const size_t groupCountOffset = outFrame.Size();
    outFrame.AppendU16(0);
    uint16_t groupCount = 0;
    uint16_t paramCount = 0;
    size_t paramCountOffset = 0;
    String currentGroup = "";

    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.length() == 0 || line.startsWith("#")) {
            continue;
        }

        const int colonIndex = line.indexOf(':');
        if (colonIndex < 0) {
            continue;
        }
        String key = line.substring(0, colonIndex);
        String value = line.substring(colonIndex + 1);
        key.trim();
        value.trim();

        const int dotIndex = key.indexOf('.');
        if (dotIndex < 0) {
            continue;
        }
        String group = key.substring(0, dotIndex);
        String param = key.substring(dotIndex + 1);

        if (group != currentGroup) {
            if (paramCountOffset != 0) {
                outFrame.SetU16(paramCountOffset, paramCount);
            }
            currentGroup = group;
            outFrame.AppendString(group.c_str());
            paramCountOffset = outFrame.Size();
            outFrame.AppendU16(0);
            paramCount = 0;
            groupCount++;
        }

        appendConfigValue(outFrame, param.c_str(), value.c_str(), isValueNumeric(value.c_str()));
        paramCount++;
    }

    if (paramCountOffset != 0) {
        outFrame.SetU16(paramCountOffset, paramCount);
    }
    outFrame.SetU16(groupCountOffset, groupCount);
    outFrame.Finish();
    file.close();
}
#endif

static void errorCallback(cmd_error* c);

#ifdef USE_CONSOLE_PARAM_RW
static void readCallback(cmd* c);
static void readAllCallback(cmd* c);
static void writeCallback(cmd* c);
#endif

#ifdef USE_UWB_MODE_TDOA_ANCHOR
static void tdoaDistancesCallback(cmd* c);
#endif

#ifdef USE_UWB_MODE_TDOA_TAG
static void tdoaAnchorModelResetCallback(cmd* c);
static void tdoaAnchorModelCollectStartCallback(cmd* c);
static void tdoaAnchorModelCollectStatusCallback(cmd* c);
static void tdoaAnchorModelLockCallback(cmd* c);
static void tdoaAnchorModelStatusCallback(cmd* c);
static void tdoaAnchorModelExportCallback(cmd* c);
static void tdoaEstimatorStatsResetCallback(cmd* c);
#endif

#ifdef USE_CONSOLE_CONFIG_MGMT
static void loadConfigCallback(cmd* c);
static void saveConfigCallback(cmd* c);
static void backupConfigCallback(cmd* c);

// Multi-config management callbacks
static void listConfigsCallback(cmd* c);
static void saveConfigAsCallback(cmd* c);
static void loadConfigNamedCallback(cmd* c);
static void readConfigNamedCallback(cmd* c);
static void deleteConfigCallback(cmd* c);
#endif

#ifdef USE_CONSOLE_LED_CONTROL
// LED 2 control callbacks
static void toggleLed2Callback(cmd* c);
static void getLed2StateCallback(cmd* c);
#endif

// Helper functions for parameter read/write (used by PARAM_RW and CONFIG_MGMT)
#if defined(USE_CONSOLE_PARAM_RW) || defined(USE_CONSOLE_CONFIG_MGMT)
static bool IsUwbShortAddrName(const char* name) {
    if (strcmp(name, "devShortAddr") == 0) {
        return true;
    }
    if (strncmp(name, "devId", 5) == 0) {
        char idx = name[5];
        return idx >= '1' && idx <= '6' && name[6] == '\0';
    }
    return false;
}

static const UWBShortAddr* GetUwbShortAddrByName(const char* name) {
    const UWBParams& params = Front::uwbLittleFSFront.GetParams();
    if (strcmp(name, "devShortAddr") == 0) return &params.devShortAddr;
    if (strcmp(name, "devId1") == 0) return &params.devId1;
    if (strcmp(name, "devId2") == 0) return &params.devId2;
    if (strcmp(name, "devId3") == 0) return &params.devId3;
    if (strcmp(name, "devId4") == 0) return &params.devId4;
    if (strcmp(name, "devId5") == 0) return &params.devId5;
    if (strcmp(name, "devId6") == 0) return &params.devId6;
    return nullptr;
}

static String UwbShortAddrToString(const UWBShortAddr& addr) {
    char buf[3] = {};
    size_t len = 0;
    if (addr[0] != '\0') {
        buf[len++] = addr[0];
    }
    if (addr[1] != '\0') {
        buf[len++] = addr[1];
    }
    if (len == 0) {
        return String("0");
    }
    buf[len] = '\0';
    return String(buf);
}
#endif // USE_CONSOLE_PARAM_RW || USE_CONSOLE_CONFIG_MGMT

#ifdef USE_CONSOLE_PARAM_RW
static bool IsDigitChar(char c) {
    return c >= '0' && c <= '9';
}

static bool ParseShortAddrDigits(String input, char out[2], uint32_t& outLen) {
    input.trim();
    if (input.length() != 1) {
        return false;
    }
    if (!IsDigitChar(input[0])) {
        return false;
    }
    uint8_t numeric = static_cast<uint8_t>(input[0] - '0');
    if (numeric > 7) {
        return false;
    }

    out[0] = input[0];
    out[1] = '\0';
    outLen = 1;
    return true;
}

static void TrimQuotedString(String& value) {
    value.trim();
    if (value.length() >= 2 && value[0] == '"' && value[value.length() - 1] == '"') {
        value = value.substring(1, value.length() - 1);
    }
}
#endif // USE_CONSOLE_PARAM_RW

#ifdef USE_CONSOLE_CONFIG_MGMT
static String escapeJsonString(const String& str) {
    String escaped;
    for (int i = 0; i < str.length(); i++) {
        char c = str[i];
        switch (c) {
            case '\"': escaped += "\\\""; break;
            case '\\': escaped += "\\\\"; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped += c; break;
        }
    }
    return escaped;
}
#endif // USE_CONSOLE_CONFIG_MGMT

void CommandHandler::Init()
{
    commandQueueMutex = xSemaphoreCreateMutex();

    if (commandQueueMutex == NULL)
    {
        LOG_ERROR("Failed to create command queue mutex");
    }

#ifdef USE_CONSOLE_PARAM_RW
    Command readAll = simpleCLI.addCommand("readall", readAllCallback);
    readAll.addPositionalArgument("group", "all");

    // Read parameter: read --group <group parameter name> --name <parameter name>
    Command readCmd = simpleCLI.addCommand("read", readCallback);
    readCmd.addArgument("group");
    readCmd.addArgument("name");

    // Write command: write --group <group parameter name> --name <parameter name> --data <string to write>
    Command writeCmd = simpleCLI.addCommand("write", writeCallback);
    writeCmd.addArgument("group");
    writeCmd.addArgument("name");
    writeCmd.addArgument("data");
#endif // USE_CONSOLE_PARAM_RW

    // Reboot command: reboot (always available)
    Command rebootCmd = simpleCLI.addCommand("reboot", [](cmd* c) {
        ESP.restart();
    });

    // Firmware info command: firmware-info (always available)
    // Returns JSON with device info, version, board type, and build date/time
    Command firmwareInfoCmd = simpleCLI.addCommand("firmware-info", [](cmd* c) {
        commandResult = "{\"device\":\"";
        commandResult += DEVICE_TYPE;
        commandResult += "\",\"version\":\"";
        commandResult += FIRMWARE_VERSION;
        commandResult += "\",\"board\":\"";
        commandResult += BOARD_TYPE;
        commandResult += "\",\"buildDate\":\"";
        commandResult += BUILD_DATE;
        commandResult += "\",\"buildTime\":\"";
        commandResult += BUILD_TIME;
        commandResult += "\"}";
    });

#ifdef USE_UWB_MODE_TDOA_ANCHOR
    // Diagnostic/calibration helper: get latest inter-anchor ToF distances (raw ticks)
    // Returns JSON: {"anchorId":0,"antennaDelay":16580,"activeSlots":4,"distances":[...]}
    simpleCLI.addCommand("tdoa-distances", tdoaDistancesCallback);
#endif

#ifdef USE_UWB_MODE_TDOA_TAG
    simpleCLI.addCommand("tdoa-anchor-model-reset", tdoaAnchorModelResetCallback);
    simpleCLI.addCommand("tdoa-anchor-model-collect-start", tdoaAnchorModelCollectStartCallback);
    simpleCLI.addCommand("tdoa-anchor-model-collect-status", tdoaAnchorModelCollectStatusCallback);
    simpleCLI.addCommand("tdoa-anchor-model-lock", tdoaAnchorModelLockCallback);
    simpleCLI.addCommand("tdoa-anchor-model-status", tdoaAnchorModelStatusCallback);
    simpleCLI.addCommand("tdoa-anchor-model-export", tdoaAnchorModelExportCallback);
    simpleCLI.addCommand("tdoa-estimator-stats-reset", tdoaEstimatorStatsResetCallback);
#endif

#ifdef USE_CONSOLE_CONFIG_MGMT
    // LittleFS parameter management commands
    Command loadConfigCmd = simpleCLI.addCommand("load-config", loadConfigCallback);
    Command saveConfigCmd = simpleCLI.addCommand("save-config", saveConfigCallback);
    Command backupConfigCmd = simpleCLI.addCommand("backup-config", backupConfigCallback);

    // Multi-config management commands
    Command listConfigsCmd = simpleCLI.addCommand("list-configs", listConfigsCallback);

    Command saveConfigAsCmd = simpleCLI.addCommand("save-config-as", saveConfigAsCallback);
    saveConfigAsCmd.addArgument("name");

    Command loadConfigNamedCmd = simpleCLI.addCommand("load-config-named", loadConfigNamedCallback);
    loadConfigNamedCmd.addArgument("name");

    Command readConfigNamedCmd = simpleCLI.addCommand("read-config-named", readConfigNamedCallback);
    readConfigNamedCmd.addArgument("name");

    Command deleteConfigCmd = simpleCLI.addCommand("delete-config", deleteConfigCallback);
    deleteConfigCmd.addArgument("name");
#endif // USE_CONSOLE_CONFIG_MGMT

#ifdef USE_CONSOLE_LED_CONTROL
    // LED 2 control commands
    Command toggleLed2Cmd = simpleCLI.addCommand("toggle-led2", toggleLed2Callback);
    Command getLed2StateCmd = simpleCLI.addCommand("get-led2-state", getLed2StateCallback);
#endif // USE_CONSOLE_LED_CONTROL

    simpleCLI.setOnError(errorCallback);
}

String CommandHandler::ExecuteCommand(const char* command)
{
    if (xSemaphoreTake(commandQueueMutex, portMAX_DELAY) == pdTRUE) {
        // Parse command and execute
        simpleCLI.parse(command);
        String tmp = std::move(commandResult);
        xSemaphoreGive(commandQueueMutex);
        return tmp;
    }
    return "Failed to accuire command mutex";
}

bool CommandHandler::TryExecuteBinaryCommand(const char* command, CommandBinaryFrame& outFrame)
{
    if (command == nullptr) {
        return false;
    }

    const bool knownBinaryCommand =
        commandStartsWith(command, "firmware-info")
        || commandStartsWith(command, "tdoa-distances")
        || commandStartsWith(command, "backup-config")
        || commandStartsWith(command, "list-configs")
        || commandStartsWith(command, "read-config-named")
        || commandStartsWith(command, "save-config-as")
        || commandStartsWith(command, "load-config-named")
        || commandStartsWith(command, "delete-config")
        || commandStartsWith(command, "toggle-led2")
        || commandStartsWith(command, "get-led2-state")
        || commandStartsWith(command, "tdoa-anchor-model-reset")
        || commandStartsWith(command, "tdoa-anchor-model-collect-start")
        || commandStartsWith(command, "tdoa-anchor-model-collect-status")
        || commandStartsWith(command, "tdoa-anchor-model-lock")
        || commandStartsWith(command, "tdoa-anchor-model-status")
        || commandStartsWith(command, "tdoa-anchor-model-export")
        || commandStartsWith(command, "tdoa-estimator-stats-reset");

    if (!knownBinaryCommand) {
        return false;
    }

    if (xSemaphoreTake(commandQueueMutex, portMAX_DELAY) != pdTRUE) {
        appendAck(outFrame, rtls::protocol::StatusCode::Error, "Failed to acquire command mutex");
        return true;
    }

    if (commandStartsWith(command, "firmware-info")) {
        outFrame.Begin(rtls::protocol::FrameType::FirmwareInfo);
        outFrame.AppendString(DEVICE_TYPE);
        outFrame.AppendString(FIRMWARE_VERSION);
        outFrame.AppendString(BOARD_TYPE);
        outFrame.AppendString(BUILD_DATE);
        outFrame.AppendString(BUILD_TIME);
        outFrame.Finish();
        xSemaphoreGive(commandQueueMutex);
        return true;
    }

#ifdef USE_UWB_MODE_TDOA_ANCHOR
    if (commandStartsWith(command, "tdoa-distances")) {
        const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
        if (uwbParams.mode != UWBMode::ANCHOR_TDOA) {
            appendAck(outFrame, rtls::protocol::StatusCode::InvalidMode, "Not in ANCHOR_TDOA mode");
            xSemaphoreGive(commandQueueMutex);
            return true;
        }

        uint16_t distances[8] = {};
        if (!uwbTdoa2AnchorGetDistances(distances, 8)) {
            appendAck(outFrame, rtls::protocol::StatusCode::Error, "TDoA anchor algorithm not initialized");
            xSemaphoreGive(commandQueueMutex);
            return true;
        }

        outFrame.Begin(rtls::protocol::FrameType::TdoaDistances);
        outFrame.AppendU8(uwbTdoa2AnchorGetAnchorId());
        outFrame.AppendU8((uwbParams.tdoaSlotCount == 0) ? 8 : uwbParams.tdoaSlotCount);
        outFrame.AppendU16(uwbTdoa2AnchorGetAntennaDelay());
        for (uint8_t i = 0; i < 8; i++) {
            outFrame.AppendU16(distances[i]);
        }
        outFrame.Finish();
        xSemaphoreGive(commandQueueMutex);
        return true;
    }
#endif

#ifdef USE_CONSOLE_CONFIG_MGMT
    if (commandStartsWith(command, "backup-config")) {
        appendCurrentConfigSnapshot(outFrame);
        xSemaphoreGive(commandQueueMutex);
        return true;
    }

    if (commandStartsWith(command, "list-configs")) {
        outFrame.Begin(rtls::protocol::FrameType::ConfigList);
        String active = ConfigManager::GetActiveConfig();
        outFrame.AppendString(active.c_str());
        const size_t count = ConfigManager::GetConfigCount();
        const uint8_t encodedCount = count > UINT8_MAX ? UINT8_MAX : static_cast<uint8_t>(count);
        outFrame.AppendU8(encodedCount);
        for (size_t i = 0; i < encodedCount; i++) {
            outFrame.AppendString(ConfigManager::GetConfigName(i));
        }
        outFrame.Finish();
        xSemaphoreGive(commandQueueMutex);
        return true;
    }

    if (commandStartsWith(command, "read-config-named")) {
        String name = commandNameArg(command);
        appendConfigFileSnapshot(outFrame, name.c_str());
        xSemaphoreGive(commandQueueMutex);
        return true;
    }

    if (commandStartsWith(command, "save-config-as")
        || commandStartsWith(command, "load-config-named")
        || commandStartsWith(command, "delete-config")) {
        String name = commandNameArg(command);
        ConfigError result = ConfigError::OK;
        if (commandStartsWith(command, "save-config-as")) {
            result = ConfigManager::SaveConfigAs(name.c_str());
        } else if (commandStartsWith(command, "load-config-named")) {
            result = ConfigManager::LoadConfigNamed(name.c_str());
        } else {
            result = ConfigManager::DeleteConfig(name.c_str());
        }

        rtls::protocol::StatusCode status = rtls::protocol::StatusCode::Ok;
        const char* message = "OK";
        switch (result) {
            case ConfigError::OK:
                break;
            case ConfigError::CONFIG_NOT_FOUND:
                status = rtls::protocol::StatusCode::NotFound;
                message = "Configuration not found";
                break;
            case ConfigError::INVALID_NAME:
                status = rtls::protocol::StatusCode::InvalidName;
                message = "Invalid config name";
                break;
            case ConfigError::FILE_SYSTEM_ERROR:
                status = rtls::protocol::StatusCode::FileSystemError;
                message = "File system error";
                break;
            case ConfigError::INVALID_CONFIG:
                status = rtls::protocol::StatusCode::Error;
                message = "Invalid configuration";
                break;
            default:
                status = rtls::protocol::StatusCode::Error;
                message = "Config operation failed";
                break;
        }
        appendAck(outFrame, status, message);
        xSemaphoreGive(commandQueueMutex);
        return true;
    }
#endif

#ifdef USE_CONSOLE_LED_CONTROL
    if (commandStartsWith(command, "toggle-led2") || commandStartsWith(command, "get-led2-state")) {
        if (!Front::appLittleFSFront.IsLed2Configured()) {
            outFrame.Begin(rtls::protocol::FrameType::LedState, rtls::protocol::StatusCode::Error);
            outFrame.AppendBool(false);
            outFrame.AppendBool(false);
            outFrame.Finish();
            xSemaphoreGive(commandQueueMutex);
            return true;
        }
        if (commandStartsWith(command, "toggle-led2")) {
            Front::appLittleFSFront.ToggleLed2();
        }
        outFrame.Begin(rtls::protocol::FrameType::LedState);
        outFrame.AppendBool(true);
        outFrame.AppendBool(Front::appLittleFSFront.GetLed2State());
        outFrame.Finish();
        xSemaphoreGive(commandQueueMutex);
        return true;
    }
#endif

#ifdef USE_UWB_MODE_TDOA_TAG
    if (commandStartsWith(command, "tdoa-anchor-model-reset")) {
        if (!IsTagTdoaMode()) {
            appendAck(outFrame, rtls::protocol::StatusCode::InvalidMode, "Not in TAG_TDOA mode");
        } else {
            TDoAAnchorModelCommands::Reset();
            appendAck(outFrame, rtls::protocol::StatusCode::Ok, "OK");
        }
        xSemaphoreGive(commandQueueMutex);
        return true;
    }

    if (commandStartsWith(command, "tdoa-anchor-model-collect-start")) {
        if (!IsTagTdoaMode()) {
            appendAck(outFrame, rtls::protocol::StatusCode::InvalidMode, "Not in TAG_TDOA mode");
        } else {
            const bool ok = TDoAAnchorModelCommands::StartCollection();
            appendAck(outFrame, ok ? rtls::protocol::StatusCode::Ok : rtls::protocol::StatusCode::Error, ok ? "OK" : "Failed");
        }
        xSemaphoreGive(commandQueueMutex);
        return true;
    }

    if (commandStartsWith(command, "tdoa-anchor-model-lock")) {
        if (!IsTagTdoaMode()) {
            appendAck(outFrame, rtls::protocol::StatusCode::InvalidMode, "Not in TAG_TDOA mode");
        } else {
            const bool ok = TDoAAnchorModelCommands::Lock();
            appendAck(outFrame, ok ? rtls::protocol::StatusCode::Ok : rtls::protocol::StatusCode::Error, ok ? "OK" : "Failed");
        }
        xSemaphoreGive(commandQueueMutex);
        return true;
    }

    if (commandStartsWith(command, "tdoa-anchor-model-collect-status")
        || commandStartsWith(command, "tdoa-anchor-model-status")
        || commandStartsWith(command, "tdoa-anchor-model-export")) {
        if (!IsTagTdoaMode()) {
            appendAck(outFrame, rtls::protocol::StatusCode::InvalidMode, "Not in TAG_TDOA mode");
        } else {
            const uint8_t view = commandStartsWith(command, "tdoa-anchor-model-collect-status") ? 1
                : commandStartsWith(command, "tdoa-anchor-model-export") ? 2
                : 0;
            TDoAAnchorModelCommands::AppendBinaryStatus(outFrame, view);
        }
        xSemaphoreGive(commandQueueMutex);
        return true;
    }

    if (commandStartsWith(command, "tdoa-estimator-stats-reset")) {
        if (!IsTagTdoaMode()) {
            appendAck(outFrame, rtls::protocol::StatusCode::InvalidMode, "Not in TAG_TDOA mode");
        } else {
            TDoAAnchorModelCommands::ResetEstimatorStats();
            appendAck(outFrame, rtls::protocol::StatusCode::Ok, "OK");
        }
        xSemaphoreGive(commandQueueMutex);
        return true;
    }
#endif

    appendAck(outFrame, rtls::protocol::StatusCode::NotSupported, "Binary response not supported by this firmware build");
    xSemaphoreGive(commandQueueMutex);
    return true;
}

// ********** Command Callbacks **********

#ifdef USE_CONSOLE_PARAM_RW
static void readCallback(cmd* c)
{
    Command cmd(c);

    Argument groupArg = cmd.getArgument("group");
    Argument nameArg = cmd.getArgument("name");

    String paramGroup = groupArg.getValue();
    String valueGroup = nameArg.getValue();

    if (paramGroup == "uwb" && IsUwbShortAddrName(valueGroup.c_str())) {
        const UWBShortAddr* addr = GetUwbShortAddrByName(valueGroup.c_str());
        if (!addr) {
            commandResult = "Name not found";
            return;
        }
        commandResult = "Param: " + UwbShortAddrToString(*addr);
        return;
    }

    // Read parameter
    char inData[128];
    uint32_t inDataLen = 0;
    ParamType inDataType = ParamType::UNDEFINED;
    ErrorParam ret = Front::ReadGlobalParam(paramGroup.c_str(), valueGroup.c_str(), inData, inDataLen, inDataType);
    inData[inDataLen] = '\0';

    switch (ret)
    {
    case ErrorParam::OK:
        // Respond with string
        // For now with the arguments that we received
        commandResult = "Param: " + String(inData, inDataLen);
        return;
        break;
    case ErrorParam::GROUP_NOT_FOUND:
        commandResult = "Group not found";
        break;
    case ErrorParam::NAME_NOT_FOUND:
        commandResult = "Name not found";
        break;
    case ErrorParam::FAILED_TO_READ:
        commandResult = "Failed to read";
        break;
    case ErrorParam::PARAM_TOO_LONG:
        commandResult = "Invalid param";
        break;
    }
}

static void writeCallback(cmd* c)
{
    Command cmd(c);

    Argument groupArg = cmd.getArgument("group");
    Argument nameArg = cmd.getArgument("name");
    Argument dataArg = cmd.getArgument("data");

    String paramGroup = groupArg.getValue();
    String valueGroup = nameArg.getValue();
    String data = dataArg.getValue();
    TrimQuotedString(data);

    if (paramGroup == "uwb" && IsUwbShortAddrName(valueGroup.c_str())) {
        char addrBytes[2] = {};
        uint32_t addrLen = 0;
        if (!ParseShortAddrDigits(data, addrBytes, addrLen)) {
            commandResult = "Invalid short address (expected single digit 0-7)";
            return;
        }
        ErrorParam ret = Front::WriteGlobalParam(paramGroup.c_str(), valueGroup.c_str(), addrBytes, addrLen);
        switch (ret)
        {
        case ErrorParam::OK:
            commandResult = "Param written";
            return;
        case ErrorParam::GROUP_NOT_FOUND:
            commandResult = "Group not found";
            break;
        case ErrorParam::NAME_NOT_FOUND:
            commandResult = "Name not found";
            break;
        case ErrorParam::FAILED_TO_WRITE:
            commandResult = "Failed to write";
            break;
        case ErrorParam::PARAM_TOO_LONG:
            commandResult = "Param too long";
            break;
        case ErrorParam::INVALID_DATA:
            commandResult = "Invalid data";
            break;
        default:
            commandResult = "Failed to write";
            break;
        }
        return;
    }

    // Write parameter
    ErrorParam ret = Front::WriteGlobalParam(paramGroup.c_str(), valueGroup.c_str(), data.c_str(), data.length());

    switch (ret)
    {
    case ErrorParam::OK:
        // Respond with string
        commandResult = "Param written";
        return;
        break;
    case ErrorParam::GROUP_NOT_FOUND:
        commandResult = "Group not found";
        break;
    case ErrorParam::NAME_NOT_FOUND:
        commandResult = "Name not found";
        break;
    case ErrorParam::FAILED_TO_WRITE:
        commandResult = "Failed to write";
        break;
    case ErrorParam::PARAM_TOO_LONG:
        commandResult = "Param too long";
        break;
    case ErrorParam::INVALID_DATA:
        commandResult = "Invalid data";
        break;
    }
}


static void readAllCallback(cmd* c)
{
    commandResult.clear();
    commandResult.reserve(512);
    
    Command cmd(c);
    Argument groupArg = cmd.getArgument("group");
    String group = groupArg.getValue();

    // Read layout of all frontends and read corresponding parameters

    // For now this is extreamly inefficient since the way we have to retreve those parameters, should use flatmap or something similar
    etl::vector<IFrontend*, Front::MAX_FRONTENDS>& frontends = Front::Get();

    // Iterate over frontends, retrieve layout and read all parameters
    for (size_t i = 0; i < frontends.size(); i++)
    {
        IFrontend* frontend = frontends[i];
        const etl::span<const ParamDef>& layout = frontend->GetParamLayout();
        const etl::string_view frt_group = frontend->GetParamGroup();
        if (frt_group == group.c_str() || group == "all")
        {
            for (const ParamDef& param : layout)
            {
                char inData[128];
                uint32_t inDataLen = 0;
                ParamType inDataType = ParamType::UNDEFINED;
                ErrorParam ret = Front::ReadGlobalParam(frontend->GetParamGroup().data(), param.name, inData, inDataLen, inDataType);

                switch (ret)
                {
                case ErrorParam::OK:{
                    // Respond with string
                    String allparameters = String(frontend->GetParamGroup().data());
                    allparameters += ".";
                    allparameters += param.name;
                    allparameters += ": ";
                    allparameters += String(inData, inDataLen);
                    allparameters += "\n";
                    commandResult += allparameters;
                    break;
                }
                case ErrorParam::GROUP_NOT_FOUND:
                    commandResult += "Group not found\n";
                    break;
                case ErrorParam::NAME_NOT_FOUND:
                    commandResult += "Name not found\n";
                    break;
                case ErrorParam::FAILED_TO_READ:
                    commandResult += "Failed to read\n";
                    break;
                case ErrorParam::PARAM_TOO_LONG:
                    commandResult += "Invalid param\n";
                    break;
                }
            }
        }
    }
}
#endif // USE_CONSOLE_PARAM_RW

#ifdef USE_UWB_MODE_TDOA_ANCHOR
static void tdoaDistancesCallback(cmd* c)
{
    (void)c;

    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
    if (uwbParams.mode != UWBMode::ANCHOR_TDOA) {
        commandResult = "{\"error\":\"Not in ANCHOR_TDOA mode\"}";
        return;
    }

    uint16_t distances[8] = {};
    bool ok = uwbTdoa2AnchorGetDistances(distances, 8);
    if (!ok) {
        commandResult = "{\"error\":\"TDoA anchor algorithm not initialized\"}";
        return;
    }

    const uint8_t activeSlots = (uwbParams.tdoaSlotCount == 0) ? 8 : uwbParams.tdoaSlotCount;
    const uint8_t anchorId = uwbTdoa2AnchorGetAnchorId();
    const uint16_t antennaDelay = uwbTdoa2AnchorGetAntennaDelay();

    commandResult.reserve(256);
    commandResult = "{\"anchorId\":";
    commandResult += String(anchorId);
    commandResult += ",\"antennaDelay\":";
    commandResult += String(static_cast<unsigned int>(antennaDelay));
    commandResult += ",\"activeSlots\":";
    commandResult += String(static_cast<unsigned int>(activeSlots));
    commandResult += ",\"distances\":[";
    for (size_t i = 0; i < 8; i++) {
        if (i > 0) commandResult += ",";
        commandResult += String(static_cast<unsigned int>(distances[i]));
    }
    commandResult += "]}";
}
#endif // USE_UWB_MODE_TDOA_ANCHOR

#ifdef USE_UWB_MODE_TDOA_TAG
static bool IsTagTdoaMode()
{
    return Front::uwbLittleFSFront.GetParams().mode == UWBMode::TAG_TDOA;
}

static String WrapJson(const String& payload, const char* fieldName, bool success)
{
    String result;
    result.reserve(payload.length() + 32);
    result = "{\"success\":";
    result += success ? "true" : "false";
    result += ",\"";
    result += fieldName;
    result += "\":";
    result += payload;
    result += "}";
    return result;
}

static void tdoaAnchorModelResetCallback(cmd* c)
{
    (void)c;
    if (!IsTagTdoaMode()) {
        commandResult = "{\"success\":false,\"error\":\"Not in TAG_TDOA mode\"}";
        return;
    }
    TDoAAnchorModelCommands::Reset();
    commandResult = "{\"success\":true}";
}

static void tdoaAnchorModelCollectStartCallback(cmd* c)
{
    (void)c;
    if (!IsTagTdoaMode()) {
        commandResult = "{\"success\":false,\"error\":\"Not in TAG_TDOA mode\"}";
        return;
    }
    const bool ok = TDoAAnchorModelCommands::StartCollection();
    commandResult = ok ? "{\"success\":true}" : "{\"success\":false}";
}

static void tdoaAnchorModelCollectStatusCallback(cmd* c)
{
    (void)c;
    if (!IsTagTdoaMode()) {
        commandResult = "{\"success\":false,\"error\":\"Not in TAG_TDOA mode\"}";
        return;
    }
    commandResult = WrapJson(TDoAAnchorModelCommands::CollectStatusJson(), "collect", true);
}

static void tdoaAnchorModelLockCallback(cmd* c)
{
    (void)c;
    if (!IsTagTdoaMode()) {
        commandResult = "{\"success\":false,\"error\":\"Not in TAG_TDOA mode\"}";
        return;
    }
    const bool ok = TDoAAnchorModelCommands::Lock();
    commandResult = ok ? "{\"success\":true}" : WrapJson(TDoAAnchorModelCommands::CollectStatusJson(), "collect", false);
}

static void tdoaAnchorModelStatusCallback(cmd* c)
{
    (void)c;
    if (!IsTagTdoaMode()) {
        commandResult = "{\"success\":false,\"error\":\"Not in TAG_TDOA mode\"}";
        return;
    }
    commandResult = WrapJson(TDoAAnchorModelCommands::StatusJson(), "status", true);
}

static void tdoaAnchorModelExportCallback(cmd* c)
{
    (void)c;
    if (!IsTagTdoaMode()) {
        commandResult = "{\"success\":false,\"error\":\"Not in TAG_TDOA mode\"}";
        return;
    }
    commandResult = WrapJson(TDoAAnchorModelCommands::ExportJson(), "model", true);
}

static void tdoaEstimatorStatsResetCallback(cmd* c)
{
    (void)c;
    if (!IsTagTdoaMode()) {
        commandResult = "{\"success\":false,\"error\":\"Not in TAG_TDOA mode\"}";
        return;
    }
    TDoAAnchorModelCommands::ResetEstimatorStats();
    commandResult = "{\"success\":true}";
}
#endif // USE_UWB_MODE_TDOA_TAG

static void errorCallback(cmd_error* c)
{
    CommandError cmdError(c);

    commandResult = "Error: " + cmdError.toString() + "\n";

    // Respond with error message
    if (cmdError.hasCommand())
    {
        commandResult += "Did you mean: " + cmdError.getCommand().toString() + "?";
    }
}

#ifdef USE_CONSOLE_CONFIG_MGMT
static void loadConfigCallback(cmd* c)
{
    ErrorParam result = Front::LoadAllParams();
    
    switch (result) {
        case ErrorParam::OK:
            commandResult = "Configuration loaded successfully from /params.txt";
            break;
        case ErrorParam::FILE_NOT_FOUND:
            commandResult = "Warning: /params.txt not found, using default parameters";
            break;
        case ErrorParam::FILE_SYSTEM_ERROR:
            commandResult = "Error: Failed to initialize LittleFS";
            break;
        case ErrorParam::INVALID_DATA:
            commandResult = "Error: Invalid parameter format in /params.txt";
            break;
        default:
            commandResult = "Error: Failed to load configuration";
            break;
    }
}

static void saveConfigCallback(cmd* c)
{
    ErrorParam result = Front::SaveAllParams();
    
    switch (result) {
        case ErrorParam::OK:
            commandResult = "Configuration saved successfully to /params.txt";
            break;
        case ErrorParam::FILE_SYSTEM_ERROR:
            commandResult = "Error: Failed to write to LittleFS";
            break;
        default:
            commandResult = "Error: Failed to save configuration";
            break;
    }
}

static void backupConfigCallback(cmd* c)
{
    commandResult = "Current configuration:\n";
    commandResult += "{\n";
    
    etl::vector<IFrontend*, Front::MAX_FRONTENDS>& frontends = Front::Get();
    bool first_group = true;
    
    for (size_t i = 0; i < frontends.size(); i++) {
        IFrontend* frontend = frontends[i];
        
        if (!first_group) {
            commandResult += ",\n";
        }
        first_group = false;
        
        commandResult += "  \"" + String(frontend->GetParamGroup().data()) + "\": {\n";
        
        etl::span<const ParamDef> params = frontend->GetParamLayout();
        bool first_param = true;
        
        for (size_t j = 0; j < params.size(); j++) {
            const ParamDef& param = params[j];
            
            if (!first_param) {
                commandResult += ",\n";
            }
            first_param = false;
            
            char value[256];
            uint32_t len = sizeof(value);
            ParamType type;
            
            if (frontend->GetParam(param.name, value, len, type) == ErrorParam::OK) {
                commandResult += "    \"" + String(param.name) + "\": ";

                if (frontend->GetParamGroup() == "uwb" && IsUwbShortAddrName(param.name)) {
                    const UWBShortAddr* addr = GetUwbShortAddrByName(param.name);
                    if (addr) {
                        commandResult += "\"" + UwbShortAddrToString(*addr) + "\"";
                    } else {
                        commandResult += "\"\"";
                    }
                } else if (type == ParamType::STRING) {
                    commandResult += "\"" + escapeJsonString(String(value)) + "\"";
                } else {
                    commandResult += String(value);
                }
            }
        }
        
        commandResult += "\n  }";
    }
    
    commandResult += "\n}";
}

// ********** Multi-Config Management Callbacks **********

static void listConfigsCallback(cmd* c)
{
    commandResult = ConfigManager::ListConfigsJson();
}

static void saveConfigAsCallback(cmd* c)
{
    Command cmd(c);
    Argument nameArg = cmd.getArgument("name");
    String name = nameArg.getValue();

    ConfigError result = ConfigManager::SaveConfigAs(name.c_str());

    switch (result) {
        case ConfigError::OK:
            commandResult = "{\"success\":true,\"message\":\"Configuration saved as '" + name + "'\"}";
            break;
        case ConfigError::INVALID_NAME:
            commandResult = "{\"success\":false,\"error\":\"Invalid config name. Use only letters, numbers, underscores, and hyphens.\"}";
            break;
        case ConfigError::NAME_TOO_LONG:
            commandResult = "{\"success\":false,\"error\":\"Config name too long (max 32 characters)\"}";
            break;
        case ConfigError::MAX_CONFIGS_REACHED:
            commandResult = "{\"success\":false,\"error\":\"Maximum number of configurations reached (10)\"}";
            break;
        case ConfigError::FILE_SYSTEM_ERROR:
            commandResult = "{\"success\":false,\"error\":\"File system error\"}";
            break;
        case ConfigError::INVALID_CONFIG:
            commandResult = "{\"success\":false,\"error\":\"Invalid configuration\"}";
            break;
        default:
            commandResult = "{\"success\":false,\"error\":\"Unknown error\"}";
            break;
    }
}

static void loadConfigNamedCallback(cmd* c)
{
    Command cmd(c);
    Argument nameArg = cmd.getArgument("name");
    String name = nameArg.getValue();

    ConfigError result = ConfigManager::LoadConfigNamed(name.c_str());

    switch (result) {
        case ConfigError::OK:
            commandResult = "{\"success\":true,\"message\":\"Configuration '" + name + "' loaded\"}";
            break;
        case ConfigError::CONFIG_NOT_FOUND:
            commandResult = "{\"success\":false,\"error\":\"Configuration not found\"}";
            break;
        case ConfigError::INVALID_NAME:
            commandResult = "{\"success\":false,\"error\":\"Invalid config name\"}";
            break;
        case ConfigError::FILE_SYSTEM_ERROR:
            commandResult = "{\"success\":false,\"error\":\"File system error\"}";
            break;
        default:
            commandResult = "{\"success\":false,\"error\":\"Unknown error\"}";
            break;
    }
}

static void readConfigNamedCallback(cmd* c)
{
    Command cmd(c);
    Argument nameArg = cmd.getArgument("name");
    String name = nameArg.getValue();

    commandResult = ConfigManager::ReadConfigNamedJson(name.c_str());
}

static void deleteConfigCallback(cmd* c)
{
    Command cmd(c);
    Argument nameArg = cmd.getArgument("name");
    String name = nameArg.getValue();

    ConfigError result = ConfigManager::DeleteConfig(name.c_str());

    switch (result) {
        case ConfigError::OK:
            commandResult = "{\"success\":true,\"message\":\"Configuration '" + name + "' deleted\"}";
            break;
        case ConfigError::CONFIG_NOT_FOUND:
            commandResult = "{\"success\":false,\"error\":\"Configuration not found\"}";
            break;
        case ConfigError::INVALID_NAME:
            commandResult = "{\"success\":false,\"error\":\"Invalid config name\"}";
            break;
        default:
            commandResult = "{\"success\":false,\"error\":\"Unknown error\"}";
            break;
    }
}
#endif // USE_CONSOLE_CONFIG_MGMT

// ********** LED 2 Control Callbacks **********

#ifdef USE_CONSOLE_LED_CONTROL
static void toggleLed2Callback(cmd* c)
{
    if (!Front::appLittleFSFront.IsLed2Configured()) {
        commandResult = "{\"success\":false,\"error\":\"LED 2 pin not configured\"}";
        return;
    }

    Front::appLittleFSFront.ToggleLed2();
    bool newState = Front::appLittleFSFront.GetLed2State();
    commandResult = "{\"success\":true,\"led2State\":" + String(newState ? "true" : "false") + "}";
}

static void getLed2StateCallback(cmd* c)
{
    if (!Front::appLittleFSFront.IsLed2Configured()) {
        commandResult = "{\"configured\":false}";
        return;
    }

    bool state = Front::appLittleFSFront.GetLed2State();
    commandResult = "{\"configured\":true,\"state\":" + String(state ? "true" : "false") + "}";
}
#endif // USE_CONSOLE_LED_CONTROL
