#pragma once

#include "config/features.hpp"

#ifdef USE_UWB_MODE_TDOA_ANCHOR

#include <Arduino.h>
#include <SPI.h>

#include "uwb_ops.hpp"

extern "C" {
    #include "libdw1000.h"
}

#include "freertos/FreeRTOS.h"

#include "uwb_params.hpp"
#include "uwb_backend.hpp"

#include "anchor/tdoa_anchor.hpp"
#include "anchor/tdoa_anchor_api.h"

#include "utils/dispatcher.hpp"

class UWBAnchorTDoA : public UWBBackend {
public:
    UWBAnchorTDoA(const bsp::UWBConfig& uwb_config, UWBShortAddr shortAddr, uint16_t antennaDelay);

    template <libDw1000::IsrFlags TFlags>
    void OnEvent();             // Called outside ISR context

    void Update() override;
    void SetEnabled(bool enabled) override;
    void RadioTask();

    static constexpr uint32_t STALL_TIMEOUT_MS = 150;

private:
    static constexpr BaseType_t UWB_TASK_CORE_ID = 1;
    static constexpr uint32_t UWB_TASK_WAIT_MS = 10;

    void ServicePendingInterrupts(uint32_t notifyCount);
    void DispatchPendingEvents();
    void CheckStallWatchdog();
    void ApplyRuntimeParams();
    void KickStartRadio();
    void SuspendRadio();
    void ResumeRadio();
    void ClearPendingInterruptAccounting();
    void AttachInterruptHandler();
    void DetachInterruptHandler();
    uint32_t PackCurrentSysStatusLow32() const;

    uint32_t m_lastEventTimeMs = 0;
    uint16_t m_broadcastAntennaDelay = 0;
    volatile bool m_startRequested = false;
    volatile bool m_radioEnabled = true;
    bool m_radioSuspended = false;
    bool m_interruptAttached = false;
    int m_interruptPin = -1;
    // Libdw1000 device
    dwDevice_t m_Device;
    dwOps_t m_Ops = {
        .spiRead = libDw1000::SpiRead,
        .spiWrite = libDw1000::SpiWrite,
        .spiSetSpeed = libDw1000::SpiSetSpeed,
        .delayms = libDw1000::DelayMs,
        .reset = libDw1000::Reset,
    };

    // User data for libdw1000
    libDw1000::DwData m_DwData = {
        .rst_pin = bsp::kBoardConfig.uwb.pins.reset_pin,
        .cs_pin = bsp::kBoardConfig.uwb.pins.spi_cs_pin,
        .interrupt_flags = 0,
    };

    // Uwb algorithm config
    uwbConfig_t m_UwbConfig = {
        .mode = 0,  // Legacy used for chosing tdoa2 or 3.
        .address = {0,0,0,0,0,0,0xcf,0xbc}, // Index 0 will be anchor id
        .anchorListSize = UWBParams::maxAnchorCount,
        .anchors = {},
        .position = {},
        .positionEnabled = true,

        .smartPower = false,    // Unused for now
        .forceTxPower = true,   // Unused for now
        .txPower = 0x1F1F1F1Ful, // Using defaults, unused for now

        .lowBitrate = true,     // Unused for now
        .longPreamble = false,  // Unused for now

        // TDMA schedule (TDoA anchors)
        .tdoaSlotCount = 0,
        .tdoaSlotDurationUs = 0,

        .antennaDelay = 0
    };
};


using AnchorTDoADispatcher = Dispatcher<libDw1000::IsrFlags, UWBAnchorTDoA,
        DispatchEntry<libDw1000::IsrFlags, libDw1000::IsrFlags::RX_DONE, UWBAnchorTDoA, &UWBAnchorTDoA::OnEvent<libDw1000::IsrFlags::RX_DONE>>,
        DispatchEntry<libDw1000::IsrFlags, libDw1000::IsrFlags::TX_DONE, UWBAnchorTDoA, &UWBAnchorTDoA::OnEvent<libDw1000::IsrFlags::TX_DONE>>,
        DispatchEntry<libDw1000::IsrFlags, libDw1000::IsrFlags::RX_TIMEOUT, UWBAnchorTDoA, &UWBAnchorTDoA::OnEvent<libDw1000::IsrFlags::RX_TIMEOUT>>,
        DispatchEntry<libDw1000::IsrFlags, libDw1000::IsrFlags::RX_FAILED, UWBAnchorTDoA, &UWBAnchorTDoA::OnEvent<libDw1000::IsrFlags::RX_FAILED>>>;

#endif // USE_UWB_MODE_TDOA_ANCHOR
