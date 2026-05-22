#include "config/features.hpp"

#ifdef USE_UWB_MODE_TDOA_ANCHOR

#include <Arduino.h>
#include "logging/logging.hpp"

#include "uwb_tdoa_anchor.hpp"
#include "uwb_frontend_littlefs.hpp"
#include "dw1000_radio_config.hpp"
#include "tdoa_common.hpp"

#include "SPI.h"

#include <freertos/task.h>

#define DEFAULT_RX_TIMEOUT 10000

static FAST_CODE void anchorInterruptISR();
static FAST_CODE void txCallback(dwDevice_t *dev);
static FAST_CODE void rxCallback(dwDevice_t *dev);
static FAST_CODE void rxTimeoutCallback(dwDevice_t *dev);
static FAST_CODE void rxFailedCallback(dwDevice_t *dev);

static volatile bool isr_flag = false;

UWBAnchorTDoA::UWBAnchorTDoA(const bsp::UWBConfig& uwb_config, UWBShortAddr shortAddr, uint16_t antennaDelay)
    : UWBBackend(uwb_config)
{
    // NOTE: Look into short data fast accuracy...
    // Using a lambda to attach the class method as an interrupt handler

    LOG_INFO("--- UWB Anchor TDOA Mode ---");

    uint8_t parsedAnchorId = 0;
    if (!tdoa::ParseAnchorId(shortAddr, parsedAnchorId)) {
        LOG_WARN("Invalid TDoA anchor short address '%c%c', forcing anchor ID 0",
                 shortAddr[0], shortAddr[1]);
    }
    m_UwbConfig.address[0] = parsedAnchorId;
    m_UwbConfig.address[1] = 0;

    // Spi pins already setup on uwb_backend
    dwInit(&m_Device, &m_Ops);          // Initialize the driver. Init resets user data!
    m_Device.userdata = &m_DwData;

    int result = dwConfigure(&m_Device);      // Configure the DW1000
    if (result != 0) {
        LOG_WARN("DW1000 configuration failed, devid: %u", static_cast<uint32_t>(result));
    }
    LOG_INFO("DW1000 Configured (Anchor TDoA)");

    dwEnableAllLeds(&m_Device);

    // vTaskDelay(pdMS_TO_TICKS(500));

    dwTime_t delay = {.full = 0};
    dwSetAntenaDelay(&m_Device, delay);
    dwAttachSentHandler(&m_Device, txCallback);
    dwAttachReceivedHandler(&m_Device, rxCallback);
    dwAttachReceiveTimeoutHandler(&m_Device, rxTimeoutCallback);
    dwAttachReceiveFailedHandler(&m_Device, rxFailedCallback);
    dwNewConfiguration(&m_Device);
    dwSetDefaults(&m_Device);

    // Get UWB radio settings from parameters
    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
    // TDMA schedule (handled by TDoA anchor algorithm)
    m_UwbConfig.tdoaSlotCount = uwbParams.tdoaSlotCount;
    m_UwbConfig.tdoaSlotDurationUs = uwbParams.tdoaSlotDurationUs;

    dw1000_radio::ApplyTdoaRadioParams(&m_Device, uwbParams);

    dwSetReceiveWaitTimeout(&m_Device, DEFAULT_RX_TIMEOUT);

    dwCommitConfiguration(&m_Device);

    uint32_t dev_id = dwGetDeviceId(&m_Device);
    LOG_INFO("Initialized TDoA Anchor - DevID: 0x%08X, AnchorID: %u",
             dev_id, static_cast<unsigned int>(parsedAnchorId));
    LOG_INFO("  Radio: mode=%u, ch=%u, txPower=%u, smartPwr=%s",
             uwbParams.dwMode, uwbParams.channel, uwbParams.txPowerLevel,
             uwbParams.smartPowerEnable ? "on" : "off");
    LOG_INFO("  TDMA: slots=%u, slotUs=%u%s",
             (uwbParams.tdoaSlotCount == 0) ? 8 : uwbParams.tdoaSlotCount,
             uwbParams.tdoaSlotDurationUs,
             (uwbParams.tdoaSlotDurationUs == 0) ? " (legacy)" : "");
    LOG_INFO("  Antenna delay: %u", antennaDelay);

    m_broadcastAntennaDelay = antennaDelay;

    // Pass antenna delay to algorithm so it's broadcast in TX packets
    m_UwbConfig.antennaDelay = antennaDelay;

    // Init the tdoa anchor algorithm
    uwbTdoa2Algorithm.init(&m_UwbConfig, &m_Device);

    attachInterrupt(digitalPinToInterrupt(uwb_config.pins.int_pin),
        anchorInterruptISR, RISING);

    vTaskDelay(pdMS_TO_TICKS(300));

    uwbTdoa2Algorithm.onEvent(&m_Device, uwbEvent_t::eventTimeout);
}


void UWBAnchorTDoA::Update()
{
    while (isr_flag) {
        // Clear the flag before handling the interrupt to avoid losing a new
        // interrupt that arrives during dwHandleInterrupt().
        isr_flag = false;
        dwHandleInterrupt(&m_Device);
        m_lastEventTimeMs = millis();
    }

    // Stall watchdog: if no DW1000 interrupt for longer than several TDMA
    // frame periods, the radio is stuck (e.g. failed delayed TX/RX).
    // Force idle and trigger resync.
    uint32_t now = millis();
    if (m_lastEventTimeMs != 0 && (now - m_lastEventTimeMs) > STALL_TIMEOUT_MS) {
        LOG_WARN("UWB stall detected (%lu ms without interrupt), reinitializing",
                 (unsigned long)(now - m_lastEventTimeMs));
        dwIdle(&m_Device);
        uwbTdoa2AnchorRecordStallReset();
        // Full reinit: resets FSM to syncTdmaState and clears stale
        // timestamps so the resync path computes timing from the current
        // DW1000 system clock (not the stale TDMA frame start).
        uwbTdoa2Algorithm.init(&m_UwbConfig, &m_Device);
        uwbTdoa2Algorithm.onEvent(&m_Device, uwbEvent_t::eventTimeout);
        m_lastEventTimeMs = now;
    }

    // If the antenna delay parameter is updated at runtime (via the websocket
    // param interface), propagate it to the TDoA anchor algorithm immediately
    // so it is reflected in outgoing packets without requiring a reboot.
    const uint16_t desiredDelay = Front::uwbLittleFSFront.GetParams().ADelay;
    if (desiredDelay != m_broadcastAntennaDelay) {
        m_broadcastAntennaDelay = desiredDelay;
        m_UwbConfig.antennaDelay = desiredDelay;
        uwbTdoa2AnchorSetAntennaDelay(desiredDelay);
        LOG_INFO("Updated broadcast antenna delay: %u", static_cast<unsigned int>(desiredDelay));
    }

    if (m_DwData.interrupt_flags != 0) {
        AnchorTDoADispatcher dispatcher(this);
        dispatcher.Dispatch(static_cast<libDw1000::IsrFlags>(m_DwData.interrupt_flags));
    }
}

template<libDw1000::IsrFlags TFlags>
void UWBAnchorTDoA::OnEvent()
{
    if constexpr (TFlags == libDw1000::RX_DONE) {
        uwbTdoa2Algorithm.onEvent(&m_Device, uwbEvent_t::eventPacketReceived);
    } else if constexpr (TFlags == libDw1000::TX_DONE) {
        uwbTdoa2Algorithm.onEvent(&m_Device, uwbEvent_t::eventPacketSent);
    } else if constexpr (TFlags == libDw1000::RX_TIMEOUT) {
        uwbTdoa2Algorithm.onEvent(&m_Device, uwbEvent_t::eventReceiveTimeout);
    } else if constexpr (TFlags == libDw1000::RX_FAILED) {
        uwbTdoa2Algorithm.onEvent(&m_Device, uwbEvent_t::eventReceiveFailed);
    }
    m_DwData.interrupt_flags &= ~TFlags;  // Clear the specific flag at the end
}

static FAST_CODE void anchorInterruptISR() {
    isr_flag = true;
}

/* TODO: Move to FreeRTOS notifications */
static FAST_CODE void txCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::TX_DONE;
}

static FAST_CODE void rxCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::RX_DONE;
}

static FAST_CODE void rxTimeoutCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::RX_TIMEOUT;
}

static FAST_CODE void rxFailedCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::RX_FAILED;
}

#endif // USE_UWB_MODE_TDOA_ANCHOR
