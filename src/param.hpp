#pragma once

#include <Arduino.h>

#include <etl/array.h>
#include <etl/type_lookup.h>

enum class ParamType {
    UINT8 = 0,
    UINT16,
    UINT32,
    INT8,
    INT16,
    INT32,
    FLOAT,
    DOUBLE,
    STRING,
    BOOL,
    ENUM,
    UNDEFINED
};

enum class ErrorParam {
    OK = 0,
    NAME_NOT_FOUND,
    GROUP_NOT_FOUND,
    FAILED_TO_WRITE,
    FAILED_TO_READ,
    PARAM_TOO_LONG,
    INVALID_DATA,
    FILE_SYSTEM_ERROR,
    FILE_NOT_FOUND
};

struct ParamDef {
    uint16_t address;
    uint16_t len;
    ParamType type;
    const char* name;
};

/**
 * @brief There must be a better way...
 * For now we support the following types:
 * - uint32_t
 * - uint16_t
 * - uint8_t
 * - int32_t
 * - int16_t
 * - int8_t
 * - float
 * - double
 * - bool
 * - string : etl::array<char, size>, any size will default to string
 * 
 * - UWBMode
 * - WiFiMode
 * @todo: Do this properly in the future, for now we had some fun with template metaprogramming.
 */

// All enums will be treated as uint8_t
enum class UWBMode : uint8_t;
enum class WifiMode : uint8_t;
enum class ZCalcMode : uint8_t;
enum class OutputBackend : uint8_t;

using uint32_t_type = etl::type_id_pair<uint32_t, static_cast<size_t>(ParamType::UINT32)>;
using uint16_t_type = etl::type_id_pair<uint16_t, static_cast<size_t>(ParamType::UINT16)>;
using uint8_t_type = etl::type_id_pair<uint8_t, static_cast<size_t>(ParamType::UINT8)>;
using int32_t_type = etl::type_id_pair<int32_t, static_cast<size_t>(ParamType::INT32)>;
using int16_t_type = etl::type_id_pair<int16_t, static_cast<size_t>(ParamType::INT16)>;
using int8_t_type = etl::type_id_pair<int8_t, static_cast<size_t>(ParamType::INT8)>;
using float_type = etl::type_id_pair<float, static_cast<size_t>(ParamType::FLOAT)>;
using double_type = etl::type_id_pair<double, static_cast<size_t>(ParamType::DOUBLE)>;
using bool_type = etl::type_id_pair<bool, static_cast<size_t>(ParamType::BOOL)>;

using wifimode_type = etl::type_id_pair<WifiMode, static_cast<size_t>(ParamType::ENUM)>;
using uwbmode_type = etl::type_id_pair<UWBMode, static_cast<size_t>(ParamType::ENUM)>;
using zcalcmode_type = etl::type_id_pair<ZCalcMode, static_cast<size_t>(ParamType::ENUM)>;
using outputbackend_type = etl::type_id_pair<OutputBackend, static_cast<size_t>(ParamType::ENUM)>;

template <size_t size>
using string_type = etl::type_id_pair<etl::array<char, size>, static_cast<size_t>(ParamType::STRING)>;

template <size_t size>
using TypeIdLookup = etl::type_id_lookup<
                        uint8_t_type, uint16_t_type, uint32_t_type,
                        int8_t_type, int16_t_type, int32_t_type,
                        float_type, double_type, string_type<size>,
                        bool_type, wifimode_type, uwbmode_type, zcalcmode_type,
                        outputbackend_type>;

/**
 * @brief Parameter construction at compile-time.
 * 
 */
#define PARAM_DEF(structure, attrib) ParamDef{offsetof(structure, attrib), sizeof(structure::attrib), static_cast<ParamType>(TypeIdLookup<sizeof(structure::attrib)>::id_from_type_v<decltype(structure::attrib)>), #attrib}

