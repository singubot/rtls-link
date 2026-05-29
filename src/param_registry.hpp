#pragma once

#include <stddef.h>
#include <stdint.h>

#include "param.hpp"

namespace rtls::params {

struct RegistryEntry {
    const char* id;
    const char* group;
    const char* name;
};

size_t Count();
const RegistryEntry* Get(size_t index);
const RegistryEntry* FindById(const char* id, size_t len);

ErrorParam Read(const RegistryEntry& entry, char* value, uint32_t& len, ParamType& type);
ErrorParam Write(const RegistryEntry& entry, const char* value, uint32_t len);

} // namespace rtls::params
