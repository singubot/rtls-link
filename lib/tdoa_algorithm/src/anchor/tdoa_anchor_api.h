/**
 * @file tdoa_anchor_api.h
 * @brief Public API helpers for the TDoA anchor algorithm.
 *
 * The upstream Bitcraze TDoA anchor implementation keeps all algorithm state in a
 * single internal static context. For calibration and diagnostics we expose a
 * small, C-linkage API to:
 *  - read the latest inter-anchor raw ToF measurements (DW1000 timestamp units)
 *  - get/set the antenna delay value broadcast in anchor packets
 *
 * NOTE: The inter-anchor distances are raw (uncorrected) and include antenna
 * delays from both endpoints. A consumer must subtract both anchors' antenna
 * delays to obtain a corrected distance.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define UWB_TDOA2_ANCHOR_STATS_VERSION 1
#define UWB_TDOA2_ANCHOR_STATS_SLOT_COUNT 8

typedef struct uwbTdoa2AnchorSlotStats_s {
  uint32_t goodRx;
  uint32_t rxTimeout;
  uint32_t rxFailed;
  uint32_t unexpectedPacket;
  uint32_t validDistance;
  uint32_t invalidDistance;
  uint32_t packetIdMismatch;
} uwbTdoa2AnchorSlotStats_t;

typedef struct uwbTdoa2AnchorStats_s {
  uint8_t version;
  uint8_t anchorId;
  uint8_t activeSlots;
  uint8_t state;
  uint8_t slotState;
  uint8_t slot;
  uint8_t nextSlot;
  uint8_t txEnabled;
  uint16_t antennaDelay;
  uint32_t slotDurationUs;
  uint32_t frameDurationUs;
  uint8_t slot0MissStreak;
  uint32_t slot0Misses;
  uint32_t syncAcquisitions;
  uint32_t syncLosses;
  uint32_t resyncs;
  uint32_t stallResets;
  uint32_t txScheduled;
  uint32_t txDone;
  uint8_t packetIds[UWB_TDOA2_ANCHOR_STATS_SLOT_COUNT];
  uint16_t distances[UWB_TDOA2_ANCHOR_STATS_SLOT_COUNT];
  uwbTdoa2AnchorSlotStats_t slots[UWB_TDOA2_ANCHOR_STATS_SLOT_COUNT];
} uwbTdoa2AnchorStats_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Copy the latest inter-anchor distances for this anchor.
 *
 * @param[out] out_distances Pointer to a buffer to receive distances.
 * @param[in]  max_len Maximum number of entries to copy.
 * @return true if copied, false if invalid args or not initialized.
 */
bool uwbTdoa2AnchorGetDistances(uint16_t* out_distances, uint8_t max_len);

/**
 * @brief Copy the current TDoA anchor diagnostics snapshot.
 *
 * The snapshot is intentionally counter-oriented. Expensive DW1000 RX quality
 * reads should be added as sampled/optional fields in a future schema version.
 */
bool uwbTdoa2AnchorGetStats(uwbTdoa2AnchorStats_t* out_stats);

/**
 * @brief Record a wrapper-level stall watchdog reset in anchor diagnostics.
 */
void uwbTdoa2AnchorRecordStallReset(void);

/**
 * @brief Get the current anchor ID (0..7) used by the TDMA schedule.
 */
uint8_t uwbTdoa2AnchorGetAnchorId(void);

/**
 * @brief Get the antenna delay (DW1000 ticks) currently broadcast by this anchor.
 */
uint16_t uwbTdoa2AnchorGetAntennaDelay(void);

/**
 * @brief Set the antenna delay (DW1000 ticks) to be broadcast by this anchor.
 *
 * This does not affect the raw inter-anchor ToF computation (which is kept
 * uncorrected in the anchors), but it enables immediate correction on the tag
 * side without requiring an anchor reboot.
 */
void uwbTdoa2AnchorSetAntennaDelay(uint16_t delay);

#ifdef __cplusplus
} // extern "C"
#endif
