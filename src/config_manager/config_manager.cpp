#include "config_manager.hpp"
#include "front.hpp"
#include "logging/logging.hpp"
#include "uwb/uwb_params.hpp"

#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>

namespace {

constexpr uint8_t kMaxConfigAnchors = 8;

struct StoredAnchorConfig {
    bool devIdPresent = false;
    bool xPresent = false;
    bool yPresent = false;
    bool zPresent = false;
    uint8_t devId = 0;
};

bool parseU8Strict(String value, uint8_t& out)
{
    value.trim();
    if (value.length() == 0) {
        return false;
    }

    uint16_t parsed = 0;
    for (uint16_t i = 0; i < value.length(); i++) {
        const char c = value.charAt(i);
        if (!isdigit(static_cast<unsigned char>(c))) {
            return false;
        }
        parsed = static_cast<uint16_t>(parsed * 10 + (c - '0'));
        if (parsed > UINT8_MAX) {
            return false;
        }
    }

    out = static_cast<uint8_t>(parsed);
    return true;
}

bool parseFiniteFloatStrict(String value)
{
    value.trim();
    if (value.length() == 0) {
        return false;
    }

    char* end = nullptr;
    errno = 0;
    const float parsed = strtof(value.c_str(), &end);
    if (end == value.c_str() || errno == ERANGE || !std::isfinite(parsed)) {
        return false;
    }
    while (end != nullptr && *end != '\0') {
        if (!isspace(static_cast<unsigned char>(*end))) {
            return false;
        }
        end++;
    }
    return true;
}

bool parseAnchorSlotParam(const String& paramName, char prefix, uint8_t& outSlot)
{
    if (paramName.length() != 2 || paramName.charAt(0) != prefix) {
        return false;
    }
    const char slotChar = paramName.charAt(1);
    if (slotChar < '1' || slotChar > '8') {
        return false;
    }
    outSlot = static_cast<uint8_t>(slotChar - '1');
    return true;
}

bool parseAnchorDevIdParam(const String& paramName, uint8_t& outSlot)
{
    if (!paramName.startsWith("devId") || paramName.length() != 6) {
        return false;
    }
    const char slotChar = paramName.charAt(5);
    if (slotChar < '1' || slotChar > '8') {
        return false;
    }
    outSlot = static_cast<uint8_t>(slotChar - '1');
    return true;
}

ConfigError validateStoredUwbAnchorConfig(const char* path)
{
    File file = LittleFS.open(path, "r");
    if (!file) {
        LOG_ERROR("ConfigManager: Failed to open config file for validation: %s", path);
        return ConfigError::FILE_SYSTEM_ERROR;
    }

    StoredAnchorConfig anchors[kMaxConfigAnchors] = {};
    bool tagTdoaMode = false;
    bool anchorCountPresent = false;
    uint8_t anchorCount = 0;
    bool valid = true;

    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.length() == 0 || line.startsWith("#")) {
            continue;
        }

        const int colonIndex = line.indexOf(':');
        if (colonIndex == -1) {
            continue;
        }

        String key = line.substring(0, colonIndex);
        String value = line.substring(colonIndex + 1);
        key.trim();
        value.trim();

        const int dotIndex = key.indexOf('.');
        if (dotIndex == -1) {
            continue;
        }

        const String group = key.substring(0, dotIndex);
        if (group != "uwb") {
            continue;
        }

        const String paramName = key.substring(dotIndex + 1);
        if (paramName == "mode") {
            uint8_t parsed = 0;
            tagTdoaMode = parseU8Strict(value, parsed)
                && parsed == static_cast<uint8_t>(UWBMode::TAG_TDOA);
            continue;
        }

        if (paramName == "anchorCount") {
            uint8_t parsed = 0;
            if (!parseU8Strict(value, parsed) || parsed > kMaxConfigAnchors) {
                valid = false;
                continue;
            }
            anchorCountPresent = true;
            anchorCount = parsed;
            continue;
        }

        uint8_t slot = 0;
        if (parseAnchorDevIdParam(paramName, slot)) {
            uint8_t devId = 0;
            if (!parseU8Strict(value, devId) || devId >= kMaxConfigAnchors) {
                valid = false;
                continue;
            }
            anchors[slot].devIdPresent = true;
            anchors[slot].devId = devId;
            continue;
        }

        if (parseAnchorSlotParam(paramName, 'x', slot)) {
            anchors[slot].xPresent = parseFiniteFloatStrict(value);
            valid = anchors[slot].xPresent;
            continue;
        }
        if (parseAnchorSlotParam(paramName, 'y', slot)) {
            anchors[slot].yPresent = parseFiniteFloatStrict(value);
            valid = anchors[slot].yPresent;
            continue;
        }
        if (parseAnchorSlotParam(paramName, 'z', slot)) {
            anchors[slot].zPresent = parseFiniteFloatStrict(value);
            valid = anchors[slot].zPresent;
            continue;
        }
    }

    file.close();

    if (!tagTdoaMode) {
        return ConfigError::OK;
    }

    if (!valid || !anchorCountPresent || anchorCount == 0) {
        if (valid) {
            LOG_ERROR("ConfigManager: Config %s missing positive UWB anchorCount", path);
        }
        return ConfigError::INVALID_CONFIG;
    }

    bool seen[kMaxConfigAnchors] = {};
    for (uint8_t slot = 0; slot < anchorCount; slot++) {
        const StoredAnchorConfig& anchor = anchors[slot];
        if (!anchor.devIdPresent || !anchor.xPresent || !anchor.yPresent || !anchor.zPresent) {
            LOG_ERROR("ConfigManager: Config %s missing geometry for UWB anchor slot %u",
                      path, static_cast<unsigned int>(slot + 1));
            return ConfigError::INVALID_CONFIG;
        }
        if (anchor.devId >= anchorCount || seen[anchor.devId]) {
            LOG_ERROR("ConfigManager: Config %s has non-contiguous or duplicate UWB anchor id %u",
                      path, static_cast<unsigned int>(anchor.devId));
            return ConfigError::INVALID_CONFIG;
        }
        seen[anchor.devId] = true;
    }

    for (uint8_t id = 0; id < anchorCount; id++) {
        if (!seen[id]) {
            LOG_ERROR("ConfigManager: Config %s missing UWB anchor id %u",
                      path, static_cast<unsigned int>(id));
            return ConfigError::INVALID_CONFIG;
        }
    }

    return ConfigError::OK;
}

} // namespace

// Static member definitions
etl::string<ConfigManager::MAX_NAME_LENGTH> ConfigManager::s_ActiveConfig;
etl::vector<ConfigInfo, ConfigManager::MAX_CONFIGS> ConfigManager::s_Configs;
bool ConfigManager::s_Initialized = false;

bool ConfigManager::Init() {
    if (s_Initialized) {
        return true;
    }

    if (!LittleFS.begin(true)) {
        LOG_ERROR("ConfigManager: Failed to initialize LittleFS");
        return false;
    }

    if (!EnsureConfigDir()) {
        LOG_ERROR("ConfigManager: Failed to create config directory");
        return false;
    }

    LoadMetadata();
    s_Initialized = true;
    LOG_INFO("ConfigManager: Initialized with %d configs, active: %s",
             s_Configs.size(), s_ActiveConfig.empty() ? "none" : s_ActiveConfig.c_str());
    return true;
}

bool ConfigManager::EnsureConfigDir() {
    if (!LittleFS.exists(CONFIG_DIR)) {
        if (!LittleFS.mkdir(CONFIG_DIR)) {
            LOG_ERROR("ConfigManager: Failed to create directory %s", CONFIG_DIR);
            return false;
        }
    }
    return true;
}

bool ConfigManager::LoadMetadata() {
    s_Configs.clear();
    s_ActiveConfig.clear();

    File file = LittleFS.open(METADATA_FILE, "r");
    if (!file) {
        LOG_INFO("ConfigManager: No metadata file found, starting fresh");
        return true;
    }

    // Simple JSON parsing (format: {"activeConfig":"name","configs":["name1","name2"]})
    String content = file.readString();
    file.close();

    // Parse activeConfig
    int activeIdx = content.indexOf("\"activeConfig\":\"");
    if (activeIdx >= 0) {
        activeIdx += 16; // length of "activeConfig":"
        int endIdx = content.indexOf("\"", activeIdx);
        if (endIdx > activeIdx) {
            String active = content.substring(activeIdx, endIdx);
            s_ActiveConfig.assign(active.c_str());
        }
    }

    // Parse configs array
    int configsIdx = content.indexOf("\"configs\":[");
    if (configsIdx >= 0) {
        configsIdx += 11; // length of "configs":[
        int endArrayIdx = content.indexOf("]", configsIdx);
        if (endArrayIdx > configsIdx) {
            String configsStr = content.substring(configsIdx, endArrayIdx);

            // Parse each config name in the array
            int pos = 0;
            while (pos < configsStr.length() && s_Configs.size() < MAX_CONFIGS) {
                int startQuote = configsStr.indexOf("\"", pos);
                if (startQuote < 0) break;
                int endQuote = configsStr.indexOf("\"", startQuote + 1);
                if (endQuote < 0) break;

                String name = configsStr.substring(startQuote + 1, endQuote);
                if (name.length() > 0 && name.length() <= MAX_NAME_LENGTH) {
                    ConfigInfo info;
                    info.name.assign(name.c_str());
                    s_Configs.push_back(info);
                }
                pos = endQuote + 1;
            }
        }
    }

    return true;
}

bool ConfigManager::SaveMetadata() {
    File file = LittleFS.open(METADATA_FILE, "w");
    if (!file) {
        LOG_ERROR("ConfigManager: Failed to open metadata file for writing");
        return false;
    }

    // Build JSON manually
    String json = "{\"activeConfig\":\"";
    json += s_ActiveConfig.c_str();
    json += "\",\"configs\":[";

    for (size_t i = 0; i < s_Configs.size(); i++) {
        if (i > 0) json += ",";
        json += "\"";
        json += s_Configs[i].name.c_str();
        json += "\"";
    }
    json += "]}";

    file.print(json);
    file.close();
    return true;
}

bool ConfigManager::IsValidConfigName(const char* name) {
    if (!name || strlen(name) == 0 || strlen(name) > MAX_NAME_LENGTH) {
        return false;
    }

    // Only allow alphanumeric, underscore, and hyphen
    for (size_t i = 0; i < strlen(name); i++) {
        char c = name[i];
        if (!isalnum(c) && c != '_' && c != '-') {
            return false;
        }
    }
    return true;
}

String ConfigManager::GetConfigPath(const char* name) {
    String path = CONFIG_DIR;
    path += "/";
    path += name;
    path += ".txt";
    return path;
}

bool ConfigManager::CopyFile(const char* src, const char* dst) {
    File srcFile = LittleFS.open(src, "r");
    if (!srcFile) {
        LOG_ERROR("ConfigManager: Failed to open source file %s", src);
        return false;
    }

    File dstFile = LittleFS.open(dst, "w");
    if (!dstFile) {
        LOG_ERROR("ConfigManager: Failed to open destination file %s", dst);
        srcFile.close();
        return false;
    }

    // Copy in chunks, checking for write errors
    bool success = true;
    uint8_t buffer[256];
    while (srcFile.available()) {
        size_t bytesRead = srcFile.read(buffer, sizeof(buffer));
        size_t bytesWritten = dstFile.write(buffer, bytesRead);
        if (bytesWritten != bytesRead) {
            LOG_ERROR("ConfigManager: Write error copying file (wrote %d of %d bytes)",
                      bytesWritten, bytesRead);
            success = false;
            break;
        }
    }

    srcFile.close();
    dstFile.close();

    // If copy failed, remove the incomplete destination file
    if (!success) {
        LittleFS.remove(dst);
    }

    return success;
}

String ConfigManager::ListConfigsJson() {
    if (!s_Initialized) {
        Init();
    }

    String json = "{\"activeConfig\":\"";
    json += s_ActiveConfig.c_str();
    json += "\",\"configs\":[";

    for (size_t i = 0; i < s_Configs.size(); i++) {
        if (i > 0) json += ",";
        json += "{\"name\":\"";
        json += s_Configs[i].name.c_str();
        json += "\"}";
    }
    json += "]}";

    return json;
}

ConfigError ConfigManager::SaveConfigAs(const char* name) {
    if (!s_Initialized) {
        Init();
    }

    if (!IsValidConfigName(name)) {
        return ConfigError::INVALID_NAME;
    }

    if (strlen(name) > MAX_NAME_LENGTH) {
        return ConfigError::NAME_TOO_LONG;
    }

    // First, save current parameters to /params.txt
    ErrorParam saveResult = Front::SaveAllParams();
    if (saveResult != ErrorParam::OK) {
        LOG_ERROR("ConfigManager: Failed to save current params before creating config");
        return ConfigError::FILE_SYSTEM_ERROR;
    }

    // Check if config with this name already exists
    bool exists = false;
    for (const auto& config : s_Configs) {
        if (config.name == name) {
            exists = true;
            break;
        }
    }

    // If doesn't exist and we're at max, return error
    if (!exists && s_Configs.size() >= MAX_CONFIGS) {
        return ConfigError::MAX_CONFIGS_REACHED;
    }

    // Copy current params.txt to the config file
    String configPath = GetConfigPath(name);
    if (!CopyFile("/params.txt", configPath.c_str())) {
        return ConfigError::FILE_SYSTEM_ERROR;
    }

    // Add to configs list if new
    if (!exists) {
        ConfigInfo info;
        info.name.assign(name);
        s_Configs.push_back(info);
    }

    // Set as active
    s_ActiveConfig.assign(name);

    // Save metadata
    if (!SaveMetadata()) {
        return ConfigError::FILE_SYSTEM_ERROR;
    }

    LOG_INFO("ConfigManager: Saved config '%s'", name);
    return ConfigError::OK;
}

ConfigError ConfigManager::LoadConfigNamed(const char* name) {
    if (!s_Initialized) {
        Init();
    }

    if (!IsValidConfigName(name)) {
        return ConfigError::INVALID_NAME;
    }

    // Check if config exists
    bool found = false;
    for (const auto& config : s_Configs) {
        if (config.name == name) {
            found = true;
            break;
        }
    }

    if (!found) {
        return ConfigError::CONFIG_NOT_FOUND;
    }

    String configPath = GetConfigPath(name);
    if (!LittleFS.exists(configPath)) {
        return ConfigError::CONFIG_NOT_FOUND;
    }

    const ConfigError validationResult = validateStoredUwbAnchorConfig(configPath.c_str());
    if (validationResult != ConfigError::OK) {
        return validationResult;
    }

    // Copy config file to params.txt
    if (!CopyFile(configPath.c_str(), "/params.txt")) {
        return ConfigError::FILE_SYSTEM_ERROR;
    }

    // Update active config
    s_ActiveConfig.assign(name);
    SaveMetadata();

    // Reload all parameters and check for errors
    ErrorParam loadResult = Front::LoadAllParams();
    if (loadResult != ErrorParam::OK && loadResult != ErrorParam::FILE_NOT_FOUND) {
        LOG_WARN("ConfigManager: Failed to reload parameters after loading config");
        return ConfigError::FILE_SYSTEM_ERROR;
    }

    LOG_INFO("ConfigManager: Loaded config '%s'", name);
    return ConfigError::OK;
}

ConfigError ConfigManager::DeleteConfig(const char* name) {
    if (!s_Initialized) {
        Init();
    }

    if (!IsValidConfigName(name)) {
        return ConfigError::INVALID_NAME;
    }

    // Find and remove from list
    bool found = false;
    for (auto it = s_Configs.begin(); it != s_Configs.end(); ++it) {
        if (it->name == name) {
            s_Configs.erase(it);
            found = true;
            break;
        }
    }

    if (!found) {
        return ConfigError::CONFIG_NOT_FOUND;
    }

    // Delete the config file
    String configPath = GetConfigPath(name);
    if (LittleFS.exists(configPath)) {
        LittleFS.remove(configPath);
    }

    // Clear active config if it was the deleted one
    if (s_ActiveConfig == name) {
        s_ActiveConfig.clear();
    }

    // Save metadata
    SaveMetadata();

    LOG_INFO("ConfigManager: Deleted config '%s'", name);
    return ConfigError::OK;
}

String ConfigManager::GetActiveConfig() {
    if (!s_Initialized) {
        Init();
    }
    return String(s_ActiveConfig.c_str());
}

size_t ConfigManager::GetConfigCount() {
    if (!s_Initialized) {
        Init();
    }
    return s_Configs.size();
}

const char* ConfigManager::GetConfigName(size_t index) {
    if (!s_Initialized) {
        Init();
    }
    if (index >= s_Configs.size()) {
        return "";
    }
    return s_Configs[index].name.c_str();
}

ConfigError ConfigManager::SetActiveConfig(const char* name) {
    if (!s_Initialized) {
        Init();
    }

    if (!IsValidConfigName(name)) {
        return ConfigError::INVALID_NAME;
    }

    // Check if config exists
    bool found = false;
    for (const auto& config : s_Configs) {
        if (config.name == name) {
            found = true;
            break;
        }
    }

    if (!found) {
        return ConfigError::CONFIG_NOT_FOUND;
    }

    s_ActiveConfig.assign(name);
    SaveMetadata();

    return ConfigError::OK;
}

String ConfigManager::ReadConfigNamedJson(const char* name) {
    if (!s_Initialized) {
        Init();
    }

    if (!IsValidConfigName(name)) {
        return "{\"error\":\"Invalid config name\"}";
    }

    // Check if config exists
    bool found = false;
    for (const auto& config : s_Configs) {
        if (config.name == name) {
            found = true;
            break;
        }
    }

    if (!found) {
        return "{\"error\":\"Config not found\"}";
    }

    String configPath = GetConfigPath(name);
    File file = LittleFS.open(configPath, "r");
    if (!file) {
        return "{\"error\":\"Failed to open config file\"}";
    }

    // Parse the config file and build JSON
    // File format: group.paramName: value
    String json = "{\n";
    String currentGroup = "";
    bool firstGroup = true;
    bool firstParam = true;

    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();

        // Skip empty lines and comments
        if (line.length() == 0 || line.startsWith("#")) {
            continue;
        }

        // Find the colon separator
        int colonIndex = line.indexOf(':');
        if (colonIndex == -1) {
            continue;
        }

        String key = line.substring(0, colonIndex);
        String value = line.substring(colonIndex + 1);
        key.trim();
        value.trim();

        // Find the dot separator for group.param
        int dotIndex = key.indexOf('.');
        if (dotIndex == -1) {
            continue;
        }

        String group = key.substring(0, dotIndex);
        String paramName = key.substring(dotIndex + 1);

        // Check if we're starting a new group
        if (group != currentGroup) {
            // Close previous group if exists
            if (!firstGroup) {
                json += "\n  },\n";
            }
            firstGroup = false;
            firstParam = true;
            currentGroup = group;
            json += "  \"" + group + "\": {\n";
        }

        // Add parameter
        if (!firstParam) {
            json += ",\n";
        }
        firstParam = false;

        json += "    \"" + paramName + "\": ";

        // Determine if value is numeric or string
        bool isNumeric = true;
        bool hasDot = false;
        for (size_t i = 0; i < value.length(); i++) {
            char c = value.charAt(i);
            if (c == '.') {
                if (hasDot) {
                    isNumeric = false;
                    break;
                }
                hasDot = true;
            } else if (c == '-' && i == 0) {
                // Allow leading minus
            } else if (!isdigit(c)) {
                isNumeric = false;
                break;
            }
        }

        // Handle empty values as strings
        if (value.length() == 0) {
            isNumeric = false;
        }

        if (isNumeric) {
            json += value;
        } else {
            // Escape special characters in string values
            String escaped = "";
            for (size_t i = 0; i < value.length(); i++) {
                char c = value.charAt(i);
                if (c == '"') {
                    escaped += "\\\"";
                } else if (c == '\\') {
                    escaped += "\\\\";
                } else if (c == '\n') {
                    escaped += "\\n";
                } else if (c == '\r') {
                    escaped += "\\r";
                } else if (c == '\t') {
                    escaped += "\\t";
                } else {
                    escaped += c;
                }
            }
            json += "\"" + escaped + "\"";
        }
    }

    file.close();

    // Close last group
    if (!firstGroup) {
        json += "\n  }";
    }

    json += "\n}";

    return json;
}
