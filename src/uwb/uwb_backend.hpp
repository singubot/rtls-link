#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "bsp/board.hpp"

#include "uwb_params.hpp"
#include "uwb_config.hpp"

class UWBBackend {
public:
    explicit UWBBackend(const bsp::UWBConfig& spi_config);
    virtual ~UWBBackend() = default;

    virtual void Update() = 0;
    virtual void SetEnabled(bool enabled) { (void)enabled; }

    virtual uint32_t GetNumberOfConnectedDevices() { return 0; }
};
