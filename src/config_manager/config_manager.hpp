#pragma once

#include <Arduino.h>
#include <LittleFS.h>

#include <etl/vector.h>
#include <etl/string.h>

struct ConfigInfo {
    etl::string<32> name;
};

enum class ConfigError {
    OK = 0,
    FILE_SYSTEM_ERROR,
    CONFIG_NOT_FOUND,
    CONFIG_EXISTS,
    NAME_TOO_LONG,
    MAX_CONFIGS_REACHED,
    INVALID_NAME,
    INVALID_CONFIG,
    CANNOT_DELETE_ACTIVE
};

class ConfigManager {
public:
    static constexpr size_t MAX_CONFIGS = 10;
    static constexpr size_t MAX_NAME_LENGTH = 32;
    static constexpr const char* CONFIG_DIR = "/configs";
    static constexpr const char* METADATA_FILE = "/configs/metadata.json";

    // Initialize the config manager (ensure directory exists)
    static bool Init();

    // List all configurations - returns JSON string
    static String ListConfigsJson();

    // Save current parameters as named config
    static ConfigError SaveConfigAs(const char* name);

    // Load named configuration
    static ConfigError LoadConfigNamed(const char* name);

    // Delete a configuration
    static ConfigError DeleteConfig(const char* name);

    // Get active configuration name
    static String GetActiveConfig();

    static size_t GetConfigCount();
    static const char* GetConfigName(size_t index);

    // Set active configuration (updates metadata only)
    static ConfigError SetActiveConfig(const char* name);

    // Read a named configuration and return as JSON (without loading it)
    static String ReadConfigNamedJson(const char* name);

private:
    static bool EnsureConfigDir();
    static bool LoadMetadata();
    static bool SaveMetadata();
    static bool IsValidConfigName(const char* name);
    static String GetConfigPath(const char* name);
    static bool CopyFile(const char* src, const char* dst);

    static etl::string<MAX_NAME_LENGTH> s_ActiveConfig;
    static etl::vector<ConfigInfo, MAX_CONFIGS> s_Configs;
    static bool s_Initialized;
};
