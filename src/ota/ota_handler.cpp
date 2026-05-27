/**
 * @file ota_handler.cpp
 * @brief OTA update handler implementation
 */

#include "ota_handler.hpp"

#ifdef USE_OTA_WEB

#include <Update.h>
#include <Arduino.h>
#include "version.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "logging/logging.hpp"
#include "uwb/uwb_frontend_littlefs.hpp"

namespace ota {

// Delay reboot long enough for the HTTP response to leave the TCP stack.
static constexpr uint32_t REBOOT_DELAY_MS = 1500;
static TimerHandle_t rebootTimer = nullptr;

#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
static bool uwbSuspendedForOta = false;

static void suspendUwbForOtaIfSupported() {
    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
    if (uwbParams.uwbEnable == 0 || uwbParams.mode != UWBMode::ANCHOR_TDOA) {
        return;
    }

    LOG_INFO("[OTA] Suspending anchor UWB radio for update");
    Front::uwbLittleFSFront.SetRuntimeEnabled(false);
    uwbSuspendedForOta = true;
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void resumeUwbAfterFailedOta() {
    if (!uwbSuspendedForOta) {
        return;
    }

    LOG_WARN("[OTA] Update failed, resuming suspended anchor UWB radio");
    Front::uwbLittleFSFront.SetRuntimeEnabled(true);
    uwbSuspendedForOta = false;
}
#endif

static void rebootTimerCallback(TimerHandle_t timer) {
    LOG_INFO("[OTA] Reboot timer elapsed, rebooting...");
    if (rebootTimer != nullptr) {
        xTimerDelete(rebootTimer, 0);
        rebootTimer = nullptr;
    }
    ESP.restart();
}

static void scheduleReboot() {
    if (rebootTimer != nullptr) {
        xTimerStop(rebootTimer, 0);
        xTimerDelete(rebootTimer, 0);
        rebootTimer = nullptr;
    }

    rebootTimer = xTimerCreate(
        "rebootTimer",
        pdMS_TO_TICKS(REBOOT_DELAY_MS),
        pdFALSE,
        nullptr,
        rebootTimerCallback
    );

    if (rebootTimer != nullptr) {
        xTimerStart(rebootTimer, 0);
        LOG_INFO("[OTA] Update complete, reboot scheduled in %lums", REBOOT_DELAY_MS);
    } else {
        LOG_ERROR("[OTA] Failed to create reboot timer");
    }
}

void initOtaRoutes(AsyncWebServer& server) {
    // Handle CORS preflight for /update
    server.on("/update", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
        AsyncWebServerResponse* response = request->beginResponse(200);
        response->addHeader("Access-Control-Allow-Origin", "*");
        response->addHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
        response->addHeader("Access-Control-Allow-Headers", "Content-Type");
        request->send(response);
    });

    // POST /update - Handle firmware upload
    server.on("/update", HTTP_POST,
        // Response handler (called after upload completes)
        [](AsyncWebServerRequest* request) {
            bool success = !Update.hasError();

            String responseBody;
            if (success) {
                responseBody = "OK ";
                responseBody += FIRMWARE_VERSION;
            } else {
                responseBody = "ERROR ";
                responseBody += Update.errorString();
            }

            AsyncWebServerResponse* response = request->beginResponse(
                success ? 200 : 500,
                "text/plain",
                responseBody
            );
            response->addHeader("Access-Control-Allow-Origin", "*");
            response->addHeader("Connection", "close");
            request->send(response);

            if (success) {
                scheduleReboot();
#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
            } else {
                resumeUwbAfterFailedOta();
#endif
            }
        },
        // File upload handler (called for each chunk)
        [](AsyncWebServerRequest* request, const String& filename, size_t index,
           uint8_t* data, size_t len, bool final) {

            if (index == 0) {
                LOG_INFO("[OTA] Starting update: %s", filename.c_str());
#ifdef USE_RUNTIME_SUBSYSTEM_TOGGLES
                suspendUwbForOtaIfSupported();
#endif

                // Calculate available space
                size_t maxSize = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
                LOG_INFO("[OTA] Max firmware size: %u bytes", maxSize);

                if (!Update.begin(maxSize, U_FLASH)) {
                    LOG_ERROR("[OTA] Update.begin failed: %s", Update.errorString());
                    return;
                }
            }

            // Write chunk
            if (Update.write(data, len) != len) {
                LOG_ERROR("[OTA] Update.write failed: %s", Update.errorString());
                return;
            }

            if (final) {
                if (Update.end(true)) {
                    LOG_INFO("[OTA] Update complete. Size: %u bytes", index + len);
                } else {
                    LOG_ERROR("[OTA] Update.end failed: %s", Update.errorString());
                }
            }
        }
    );

    LOG_INFO("[OTA] HTTP OTA upload route registered at /update");
}

} // namespace ota

#endif // USE_OTA_WEB
