#include "config/features.hpp"

#ifdef USE_UWB_MODE_TDOA_ANCHOR

#include <Arduino.h>
#include "logging/logging.hpp"

#include "uwb_tdoa_anchor.hpp"
#include "uwb_frontend_littlefs.hpp"
#include "dw1000_radio_config.hpp"
#include "tdoa_common.hpp"
#include "scheduler.hpp"

#include "SPI.h"

#include <freertos/task.h>

#define DEFAULT_RX_TIMEOUT 10000

static FAST_CODE void anchorInterruptISR();
static FAST_CODE void txCallback(dwDevice_t *dev);
static FAST_CODE void rxCallback(dwDevice_t *dev);
static FAST_CODE void rxTimeoutCallback(dwDevice_t *dev);
static FAST_CODE void rxFailedCallback(dwDevice_t *dev);
static void anchorRadioTaskEntry();

static UWBAnchorTDoA* s_anchorInstance = nullptr;
static TaskHandle_t s_anchorRadioTaskHandle = nullptr;
static portMUX_TYPE s_irqStatsMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_pendingIrqCount = 0;
static volatile uint32_t s_firstPendingIrqUs = 0;

// AsyncTCP runs at priority 3 by default. Keep the radio task below it so
// OTA/WebSocket traffic can complete even when UWB IRQs are dense.
static constexpr uint32_t kAnchorRadioTaskPriority = 2;

static StaticTaskHolder<etl::delegate<void()>, 4096, TaskType::CONTINUOUS> anchor_radio_task = {
    "UwbAnchorTask",
    0,
    kAnchorRadioTaskPriority,
    etl::delegate<void()>(),
    {},
    {}
};

UWBAnchorTDoA::UWBAnchorTDoA(const bsp::UWBConfig& uwb_config, UWBShortAddr shortAddr, uint16_t antennaDelay)
    : UWBBackend(uwb_config)
{
    s_anchorInstance = this;
    m_interruptPin = uwb_config.pins.int_pin;

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

    anchor_radio_task.taskFunction = etl::delegate<void()>::create<&anchorRadioTaskEntry>();
    Scheduler::scheduler.CreateStaticPinnedTask(anchor_radio_task, UWB_TASK_CORE_ID);
    s_anchorRadioTaskHandle = anchor_radio_task.handle;

    AttachInterruptHandler();

    vTaskDelay(pdMS_TO_TICKS(300));

    KickStartRadio();
}

static void anchorRadioTaskEntry()
{
    if (s_anchorInstance != nullptr) {
        s_anchorInstance->RadioTask();
    } else {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void UWBAnchorTDoA::Update()
{
    // DW1000 event servicing is owned by RadioTask(). Keeping Update() empty
    // avoids a second loop-polled path touching radio state.
}

void UWBAnchorTDoA::SetEnabled(bool enabled)
{
    if (enabled == m_radioEnabled) {
        return;
    }

    m_radioEnabled = enabled;

    if (!enabled) {
        DetachInterruptHandler();
        ClearPendingInterruptAccounting();
        LOG_INFO("UWB anchor radio suspend requested");
    } else {
        if (!m_radioSuspended) {
            AttachInterruptHandler();
            m_startRequested = true;
        }
        LOG_INFO("UWB anchor radio resume requested");
    }

    if (s_anchorRadioTaskHandle != nullptr) {
        xTaskNotifyGive(s_anchorRadioTaskHandle);
    }
}

void UWBAnchorTDoA::RadioTask()
{
    const uint32_t notifyCount = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(UWB_TASK_WAIT_MS));
    uint32_t irqNotifyCount = notifyCount;

    if (!m_radioEnabled) {
        SuspendRadio();
        return;
    }

    if (m_radioSuspended) {
        ResumeRadio();
    }

    if (m_startRequested) {
        m_startRequested = false;
        uwbTdoa2Algorithm.onEvent(&m_Device, uwbEvent_t::eventTimeout);
        m_lastEventTimeMs = millis();
        if (irqNotifyCount > 0) {
            irqNotifyCount--;
        }
    }

    if (irqNotifyCount > 0) {
        ServicePendingInterrupts(irqNotifyCount);

        uint32_t extraNotifyCount = 0;
        while ((extraNotifyCount = ulTaskNotifyTake(pdTRUE, 0)) > 0) {
            ServicePendingInterrupts(extraNotifyCount);
        }
    }

    CheckStallWatchdog();
    ApplyRuntimeParams();
}

void UWBAnchorTDoA::ServicePendingInterrupts(uint32_t notifyCount)
{
    uint32_t pendingIrqCount = 0;
    uint32_t firstPendingIrqUs = 0;

    portENTER_CRITICAL(&s_irqStatsMux);
    pendingIrqCount = s_pendingIrqCount;
    firstPendingIrqUs = s_firstPendingIrqUs;
    s_pendingIrqCount = 0;
    s_firstPendingIrqUs = 0;
    portEXIT_CRITICAL(&s_irqStatsMux);

    if (pendingIrqCount == 0) {
        pendingIrqCount = notifyCount;
    }

    if (firstPendingIrqUs != 0) {
        uwbTdoa2AnchorRecordIrqService(pendingIrqCount, micros() - firstPendingIrqUs);
    } else {
        uwbTdoa2AnchorRecordIrqService(pendingIrqCount, 0);
    }

    for (uint32_t i = 0; i < notifyCount; i++) {
        const uint32_t handleStartUs = micros();
        dwHandleInterrupt(&m_Device);
        uwbTdoa2AnchorRecordDwHandleInterrupt(micros() - handleStartUs);
        m_lastEventTimeMs = millis();
        DispatchPendingEvents();
    }
}

void UWBAnchorTDoA::DispatchPendingEvents()
{
    if (m_DwData.interrupt_flags != 0) {
        const uint32_t hardStartUs = micros();
        AnchorTDoADispatcher dispatcher(this);
        dispatcher.Dispatch(static_cast<libDw1000::IsrFlags>(m_DwData.interrupt_flags));
        uwbTdoa2AnchorRecordHardPath(micros() - hardStartUs);
    }
}

void UWBAnchorTDoA::CheckStallWatchdog()
{
    const uint32_t now = millis();
    if (m_lastEventTimeMs == 0 || (now - m_lastEventTimeMs) <= STALL_TIMEOUT_MS) {
        return;
    }

    LOG_WARN("UWB stall detected (%lu ms without interrupt), reinitializing",
             static_cast<unsigned long>(now - m_lastEventTimeMs));
    uwbTdoa2AnchorRecordLastDwStatusBeforeStall(PackCurrentSysStatusLow32());
    dwIdle(&m_Device);
    uwbTdoa2AnchorRecordStallReset();
    uwbTdoa2Algorithm.init(&m_UwbConfig, &m_Device);
    uwbTdoa2Algorithm.onEvent(&m_Device, uwbEvent_t::eventTimeout);
    m_lastEventTimeMs = now;
}

void UWBAnchorTDoA::ApplyRuntimeParams()
{
    const uint16_t desiredDelay = Front::uwbLittleFSFront.GetParams().ADelay;
    if (desiredDelay == m_broadcastAntennaDelay) {
        return;
    }

    m_broadcastAntennaDelay = desiredDelay;
    m_UwbConfig.antennaDelay = desiredDelay;
    uwbTdoa2AnchorSetAntennaDelay(desiredDelay);
    LOG_INFO("Updated broadcast antenna delay: %u", static_cast<unsigned int>(desiredDelay));
}

void UWBAnchorTDoA::KickStartRadio()
{
    m_startRequested = true;
    if (s_anchorRadioTaskHandle != nullptr) {
        xTaskNotifyGive(s_anchorRadioTaskHandle);
    }
}

void UWBAnchorTDoA::SuspendRadio()
{
    if (m_radioSuspended) {
        return;
    }

    dwIdle(&m_Device);
    m_DwData.interrupt_flags = 0;
    m_lastEventTimeMs = 0;
    m_startRequested = false;
    m_radioSuspended = true;
    LOG_INFO("UWB anchor radio suspended");
}

void UWBAnchorTDoA::ResumeRadio()
{
    ClearPendingInterruptAccounting();
    m_DwData.interrupt_flags = 0;
    uwbTdoa2Algorithm.init(&m_UwbConfig, &m_Device);
    m_radioSuspended = false;

    AttachInterruptHandler();

    m_startRequested = true;
    LOG_INFO("UWB anchor radio resumed");
}

void UWBAnchorTDoA::ClearPendingInterruptAccounting()
{
    portENTER_CRITICAL(&s_irqStatsMux);
    s_pendingIrqCount = 0;
    s_firstPendingIrqUs = 0;
    portEXIT_CRITICAL(&s_irqStatsMux);
}

void UWBAnchorTDoA::AttachInterruptHandler()
{
    if (m_interruptPin < 0 || m_interruptAttached) {
        return;
    }

    attachInterrupt(digitalPinToInterrupt(m_interruptPin),
        anchorInterruptISR, RISING);
    m_interruptAttached = true;
}

void UWBAnchorTDoA::DetachInterruptHandler()
{
    if (m_interruptPin < 0 || !m_interruptAttached) {
        return;
    }

    detachInterrupt(digitalPinToInterrupt(m_interruptPin));
    m_interruptAttached = false;
}

uint32_t UWBAnchorTDoA::PackCurrentSysStatusLow32() const
{
    return static_cast<uint32_t>(m_Device.sysstatus[0])
        | (static_cast<uint32_t>(m_Device.sysstatus[1]) << 8)
        | (static_cast<uint32_t>(m_Device.sysstatus[2]) << 16)
        | (static_cast<uint32_t>(m_Device.sysstatus[3]) << 24);
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
    const uint32_t nowUs = micros();
    portENTER_CRITICAL_ISR(&s_irqStatsMux);
    if (s_pendingIrqCount == 0) {
        s_firstPendingIrqUs = nowUs;
    }
    if (s_pendingIrqCount < 0xffffffffu) {
        s_pendingIrqCount++;
    }
    portEXIT_CRITICAL_ISR(&s_irqStatsMux);

    if (s_anchorRadioTaskHandle != nullptr) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(s_anchorRadioTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
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
