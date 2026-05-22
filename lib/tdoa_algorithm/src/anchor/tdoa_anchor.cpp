#include "tdoa_anchor.hpp"
#include "tdoa_anchor_api.h"

/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* uwb_tdoa2.c: Uwb TDOA anchor, version with anchor-computed distances */

/*
 * This anchor algorithm uses TDMA to divide frames into timeslots. Each
 * anchor sends a packet in one timeslot, anchor n sends its packet in
 * timeslot n.
 *
 * The number of active slots per frame and the slot duration are configurable
 * (legacy behavior: 8 slots of ~2ms).
 *
 * Each packet contains (assuming the packet is sent by anchor n):
 *   - A list of 8 IDs that contains the sequence number of the packets
 *     - At index n: The sequence number of this packet
 *     - At index != n: The sequence number of the last packet received by
 *       anchor 'index'
 *   - A list of 8 timestamps that contains
 *     - At index n: The TX timestamp of the current packet in anchor n time
 *     - At index != n: The RX timestamp of all other packets from previous
 *                      frame in anchor n clock. If the previous packet was
 *                      invalid the timestamp is 0
 *   - A list of 7 distances, the distance from this anchor to the other
 *     anchors in the system expressed in this anchor clock. The distance to
 *     the current anchor is reserved.
 *
 * This is enough info for an observer to calculate the time of departure
 * of any packets in this anchor clock, and so to calculate the difference time
 * of arrivale of the packets at the tag.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "mac.h"

#include "tdoa_anchor.hpp"

#define debug(...) printf(__VA_ARGS__)

// Still using modulo 2 calculation for slots
// TODO: If A0 is the TDMA master it could transmit slots parameters and frame
//       start so that we would not be limited to modulo 2 anymore
#define NSLOTS 8

static constexpr uint8_t kLegacySlotCount = NSLOTS;
static constexpr uint64_t kLegacySlotLenTicks = (1ull << (26 + 1)); // Legacy ~2ms timeslot

// DW1000 delayed TX/RX requires time aligned to 512 ticks (9 LSB = 0).
static constexpr uint64_t kDelayedTxRxAlignMask = (1ull << 9) - 1;

// DW1000 timestamp ticks per microsecond: 499.2 MHz * 128 / 1e6 = 63897.6
// Represented as a rational to avoid floating point: 63897.6 = 638976 / 10
static constexpr uint64_t kDwTicksPerUsNum = 638976;
static constexpr uint64_t kDwTicksPerUsDen = 10;

static uint64_t usToDwTicks(uint32_t us)
{
  return (static_cast<uint64_t>(us) * kDwTicksPerUsNum + (kDwTicksPerUsDen / 2)) / kDwTicksPerUsDen;
}

static uint64_t alignDwTicks512(uint64_t ticks)
{
  return (ticks + kDelayedTxRxAlignMask) & ~kDelayedTxRxAlignMask;
}

// Time length of the preamble
#define PREAMBLE_LENGTH_S ( 128 * 1017.63e-9 )
#define PREAMBLE_LENGTH (uint64_t)( PREAMBLE_LENGTH_S * 499.2e6 * 128 )

// Guard length to account for clock drift and time of flight
#define TDMA_GUARD_LENGTH_S ( 1e-6 )
#define TDMA_GUARD_LENGTH (uint64_t)( TDMA_GUARD_LENGTH_S * 499.2e6 * 128 )

// Timeout for receiving a packet in a timeslot
#define RECEIVE_TIMEOUT 300

// Timeout while searching for the master anchor (slot 0) during sync.
// Use a longer window to avoid duty-cycled listen gaps during acquisition.
#define RECEIVE_SYNC_TIMEOUT 20000

#define TS_TX_SIZE 4

// Useful constants
static const uint8_t base_address[] = {0,0,0,0,0,0,0xcf,0xbc};

static constexpr uint8_t kSlot0MissThreshold = 3;

// FSM states
enum state_e {
  syncTdmaState = 0, // Anchors 1 to 5 starts here and rise up to synchronizedState
  syncTimeState,
  synchronizedState, // Anchor 0 is always here!
};

enum slotState_e {
  slotRxDone,
  slotTxDone,
};

static uint32_t dwTicksToUs(uint64_t ticks)
{
  return static_cast<uint32_t>((ticks * kDwTicksPerUsDen + (kDwTicksPerUsNum / 2)) / kDwTicksPerUsNum);
}

static void inc32(uint32_t& value)
{
  if (value < 0xffffffffu) {
    value++;
  }
}

// This context struct contains all the requied global values of the algorithm
static struct ctx_s {
  int anchorId;
  enum state_e state;
  enum slotState_e slotState;
  uint8_t slot0MissCount;

  // Current and next TDMA slot
  int slot;
  int nextSlot;

  // Current packet id and tx timestamps
  uint8_t pid;

  // TDMA start of frame in local clock
  dwTime_t tdmaFrameStart;

  // TDMA schedule parameters
  uint8_t activeSlots; // 2..NSLOTS
  uint64_t slotLen;    // DW1000 ticks
  uint64_t frameLen;   // DW1000 ticks
  bool txEnabled;      // false if anchorId >= activeSlots

  // list of timestamps and ids for last frame.
  uint8_t packetIds[NSLOTS];
  uint32_t rxTimestamps[NSLOTS];
  uint32_t txTimestamps[NSLOTS];

  uint16_t distances[NSLOTS];
  uint16_t antennaDelay;
  uwbTdoa2AnchorStats_t stats;
} ctx;

static bool s_initialized = false;

static uint64_t tdmaLastFrame(uint64_t now)
{
  if (ctx.frameLen == 0) {
    return 0;
  }
  return now - (now % ctx.frameLen);
}

static uint8_t statsSlotIndex(int slot)
{
  if (slot < 0 || slot >= NSLOTS) {
    return 0;
  }
  return static_cast<uint8_t>(slot);
}

static void resetVolatileStatsFields()
{
  ctx.stats.version = UWB_TDOA2_ANCHOR_STATS_VERSION;
  ctx.stats.anchorId = static_cast<uint8_t>(ctx.anchorId);
  ctx.stats.activeSlots = ctx.activeSlots;
  ctx.stats.state = static_cast<uint8_t>(ctx.state);
  ctx.stats.slotState = static_cast<uint8_t>(ctx.slotState);
  ctx.stats.slot = static_cast<uint8_t>(ctx.slot);
  ctx.stats.nextSlot = static_cast<uint8_t>(ctx.nextSlot);
  ctx.stats.txEnabled = ctx.txEnabled ? 1 : 0;
  ctx.stats.antennaDelay = ctx.antennaDelay;
  ctx.stats.slotDurationUs = dwTicksToUs(ctx.slotLen);
  ctx.stats.frameDurationUs = dwTicksToUs(ctx.frameLen);
  ctx.stats.slot0MissStreak = ctx.slot0MissCount;
}

static void refreshStatsSnapshot()
{
  resetVolatileStatsFields();
  memcpy(ctx.stats.packetIds, ctx.packetIds, sizeof(ctx.stats.packetIds));
  memcpy(ctx.stats.distances, ctx.distances, sizeof(ctx.stats.distances));
}

static void recordSyncAcquisition()
{
  inc32(ctx.stats.syncAcquisitions);
}

static void recordSyncLoss()
{
  inc32(ctx.stats.syncLosses);
  inc32(ctx.stats.resyncs);
}

// Packet formats
#define PACKET_TYPE_TDOA2 0x22

typedef struct rangePacket_s {
  uint8_t type;
  uint8_t pid[NSLOTS];  // Packet id of the timestamps
  uint8_t timestamps[NSLOTS][TS_TX_SIZE];  // Relevant time for anchors
  uint16_t distances[NSLOTS];
  uint16_t antennaDelay;  // This anchor's configured antenna delay (DW1000 ticks)
} __attribute__((packed)) rangePacket_t;

/* Adjust time for schedule transfer by DW1000 radio. Set 9 LSB to 0 */
static uint32_t adjustTxRxTime(dwTime_t *time)
{
  uint32_t added = (1<<9) - (time->low32 & ((1<<9)-1));

  time->low32 = (time->low32 & ~((1<<9)-1)) + (1<<9);

  return added;
}

/* Calculate the transmit time for a given timeslot in the current frame */
static dwTime_t transmitTimeForSlot(int slot)
{
  dwTime_t transmitTime = { .full = 0 };

  // Calculate start of the slot
  transmitTime.full = ctx.tdmaFrameStart.full + static_cast<uint64_t>(slot) * ctx.slotLen;
  // Add guard and preamble time
  transmitTime.full += TDMA_GUARD_LENGTH;
  transmitTime.full += PREAMBLE_LENGTH;

  // DW1000 can only schedule time with 9 LSB at 0, adjust for it
  adjustTxRxTime(&transmitTime);

  return transmitTime;
}

static void handleFailedRx(dwDevice_t *dev)
{
  (void)dev;

  ctx.rxTimestamps[ctx.slot] = 0;
  ctx.distances[ctx.slot] = 0;

  // Missing slot 0 (master anchor) occasionally is expected in real RF
  // environments. Only drop sync after a few consecutive misses to reduce
  // sync oscillation.
  if (ctx.slot == 0) {
    inc32(ctx.stats.slot0Misses);
    if (ctx.slot0MissCount < 0xff) {
      ctx.slot0MissCount++;
    }
    if (ctx.slot0MissCount >= kSlot0MissThreshold) {
      ctx.state = syncTdmaState;
      ctx.slot0MissCount = 0;
      recordSyncLoss();
    }
  }
}

static bool calculateDistance(int slot, int newId, int remotePid, uint32_t remoteTx, uint32_t remoteRx, uint32_t ts)
{
  // Check that the 2 last packets are consecutive packets and that our last packet is in beteen
  if ((ctx.packetIds[slot] == ((newId-1) & 0x0ff)) && remotePid == ctx.packetIds[ctx.anchorId]) {
    double tround1 = remoteRx - ctx.txTimestamps[ctx.slot];
    double treply1 = ctx.txTimestamps[ctx.anchorId] - ctx.rxTimestamps[ctx.slot];
    double tround2 = ts - ctx.txTimestamps[ctx.anchorId];
    double treply2 = remoteTx - remoteRx;

    uint32_t distance = ((tround2 * tround1)-(treply1 * treply2)) / (2*(treply1 + tround2));
    ctx.distances[slot] = distance & 0xfffful;
    return true;
  } else {
    ctx.distances[slot] = 0;
    return false;
  }
}

static void handleRxPacket(dwDevice_t *dev)
{
  static packet_t rxPacket;
  dwTime_t rxTime = { .full = 0 };

  dwGetRawReceiveTimestamp(dev, &rxTime);
  dwCorrectTimestamp(dev, &rxTime);

  int dataLength = dwGetDataLength(dev);
  rxPacket.payload[0] = 0;
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  if (dataLength == 0 || rxPacket.payload[0] != PACKET_TYPE_TDOA2 || rxPacket.sourceAddress[0] != ctx.slot) {
    inc32(ctx.stats.slots[statsSlotIndex(ctx.slot)].unexpectedPacket);
    handleFailedRx(dev);
    return;
  }
  rangePacket_t * rangePacket = (rangePacket_t *)rxPacket.payload;
  uwbTdoa2AnchorSlotStats_t& slotStats = ctx.stats.slots[statsSlotIndex(ctx.slot)];
  inc32(slotStats.goodRx);

  uint32_t remoteTx;
  memcpy(&remoteTx, rangePacket->timestamps[ctx.slot], 4);
  uint32_t remoteRx;
  memcpy(&remoteRx, rangePacket->timestamps[ctx.anchorId], 4);

  const bool distanceValid = calculateDistance(ctx.slot, rangePacket->pid[ctx.slot], rangePacket->pid[ctx.anchorId],
                                               remoteTx, remoteRx, rxTime.low32);
  if (distanceValid) {
    inc32(slotStats.validDistance);
  } else {
    inc32(slotStats.invalidDistance);
    inc32(slotStats.packetIdMismatch);
  }

  ctx.packetIds[ctx.slot] = rangePacket->pid[ctx.slot];
  ctx.rxTimestamps[ctx.slot] = rxTime.low32;
  memcpy(&ctx.txTimestamps[ctx.slot], &rangePacket->timestamps[ctx.slot], 4);

  // Resync TDMA and save useful anchor 0 information
  if (ctx.slot == 0) {
    ctx.slot0MissCount = 0;
    // Resync local frame start to packet from anchor 0
    dwTime_t pkTxTime = { .full = 0 };
    memcpy(&pkTxTime, rangePacket->timestamps[ctx.slot], TS_TX_SIZE);
    ctx.tdmaFrameStart.full = rxTime.full - (pkTxTime.full - tdmaLastFrame(pkTxTime.full));

    //TODO: Save relevant data to calculate masterTime
  }
}

// Setup the radio to receive a packet in the next timeslot
static void setupRx(dwDevice_t *dev)
{
  dwTime_t receiveTime = { .full = 0 };

  // Calculate start of the slot
  receiveTime.full = ctx.tdmaFrameStart.full + static_cast<uint64_t>(ctx.nextSlot) * ctx.slotLen;
  adjustTxRxTime(&receiveTime);

  dwSetReceiveWaitTimeout(dev, RECEIVE_TIMEOUT);
  dwWriteSystemConfigurationRegister(dev);

  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwSetTxRxTime(dev, receiveTime);
  dwStartReceive(dev);
}

// Set TX data in the radio TX buffer
static void setTxData(dwDevice_t *dev)
{
  static packet_t txPacket;
  static bool firstEntry = true;

  if (firstEntry) {
    MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);

    memcpy(txPacket.sourceAddress, base_address, 8);
    txPacket.sourceAddress[0] = ctx.anchorId;
    memcpy(txPacket.destAddress, base_address, 8);
    txPacket.destAddress[0] = 0xff;

    txPacket.payload[0] = PACKET_TYPE_TDOA2;

    firstEntry = false;
  }

  rangePacket_t *rangePacket = (rangePacket_t *)txPacket.payload;

  for (int i=0; i<NSLOTS; i++) {
    rangePacket->pid[i] = ctx.packetIds[i];
    memcpy(rangePacket->timestamps[i], &ctx.rxTimestamps[i], TS_TX_SIZE);
  }
  memcpy(rangePacket->timestamps[ctx.anchorId], &ctx.txTimestamps[ctx.anchorId], TS_TX_SIZE);
  memcpy(rangePacket->distances, ctx.distances, sizeof(ctx.distances));
  rangePacket->antennaDelay = ctx.antennaDelay;

  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + sizeof(rangePacket_t));
}

// Setup the radio to send a packet in the next timeslot
static void setupTx(dwDevice_t *dev)
{
  ctx.packetIds[ctx.anchorId] = ctx.pid++;
  dwTime_t txTime = transmitTimeForSlot(ctx.nextSlot);
  ctx.txTimestamps[ctx.anchorId] = txTime.low32;
  inc32(ctx.stats.txScheduled);

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  setTxData(dev);
  dwSetTxRxTime(dev, txTime);

  dwWaitForResponse(dev, false);
  dwStartTransmit(dev);
}

// Increment the slot variables and, if required, switch tdmaStartFrame to next
// frame state time
static void updateSlot()
{
  ctx.slot = ctx.nextSlot;
  ctx.nextSlot = ctx.nextSlot + 1;
  if (ctx.nextSlot >= ctx.activeSlots) {
    ctx.nextSlot = 0;
  }

  // If the next slot is 0, the next schedule has to be in the next frame!
  if (ctx.nextSlot == 0) {
    ctx.tdmaFrameStart.full += ctx.frameLen;
  }
}

// slotStep is called once per timeslot as long as TDMA is synched and setup
// the next timeslot action
static uint32_t slotStep(dwDevice_t *dev, uwbEvent_t event)
{
  switch (ctx.slotState) {
    case slotRxDone:
      if (event == eventPacketReceived) {
        handleRxPacket(dev);
      } else {
        uwbTdoa2AnchorSlotStats_t& slotStats = ctx.stats.slots[statsSlotIndex(ctx.slot)];
        if (event == eventReceiveFailed) {
          inc32(slotStats.rxFailed);
        } else {
          inc32(slotStats.rxTimeout);
        }
        handleFailedRx(dev);
      }

      // Quickly setup transfer to next slot
      if (ctx.txEnabled && ctx.nextSlot == ctx.anchorId) {
        setupTx(dev);
        ctx.slotState = slotTxDone;
        updateSlot();
      } else {
        setupRx(dev);
        ctx.slotState = slotRxDone;
        updateSlot();
      }

      break;
    case slotTxDone:
      if (event == eventPacketSent) {
        inc32(ctx.stats.txDone);
      }
      setupRx(dev);
      ctx.slotState = slotRxDone;
      updateSlot();
      break;
  }

  return MAX_TIMEOUT;
}

// Initialize/reset the agorithm
static void tdoa2Init(uwbConfig_t * config, dwDevice_t *dev)
{
  ctx.anchorId = config->address[0];

  // TDMA schedule configuration
  uint8_t slotCount = config->tdoaSlotCount;
  if (slotCount == 0) {
    slotCount = kLegacySlotCount;
  }
  if (slotCount < 2) {
    slotCount = 2;
  }
  if (slotCount > NSLOTS) {
    slotCount = NSLOTS;
  }
  ctx.activeSlots = slotCount;
  ctx.txEnabled = (ctx.anchorId < ctx.activeSlots);

  uint64_t slotLen = 0;
  if (config->tdoaSlotDurationUs == 0) {
    slotLen = kLegacySlotLenTicks;
  } else {
    slotLen = alignDwTicks512(usToDwTicks(config->tdoaSlotDurationUs));
    if (slotLen < (1ull << 9)) {
      slotLen = (1ull << 9);
    }
  }

  uint64_t frameLen = slotLen * ctx.activeSlots;
  if (frameLen >= (1ull << 32)) {
    // The protocol only carries 32-bit timestamps; keep the frame length below
    // the wrap-around period to avoid sync issues.
    uint64_t maxSlotLen = ((1ull << 32) - 1) / ctx.activeSlots;
    maxSlotLen &= ~kDelayedTxRxAlignMask;
    if (maxSlotLen < (1ull << 9)) {
      maxSlotLen = (1ull << 9);
    }
    if (slotLen > maxSlotLen) {
      slotLen = maxSlotLen;
      frameLen = slotLen * ctx.activeSlots;
      debug("TDMA slot duration too long, clamped (slots=%u)\r\n", ctx.activeSlots);
    }
  }

  ctx.slotLen = slotLen;
  ctx.frameLen = frameLen;

  ctx.state = syncTdmaState;
  ctx.slot0MissCount = 0;
  ctx.slot = ctx.activeSlots - 1;
  ctx.nextSlot = 0;
  ctx.pid = 0;
  ctx.antennaDelay = config->antennaDelay;
  memset(ctx.packetIds, 0, sizeof(ctx.packetIds));
  memset(ctx.txTimestamps, 0, sizeof(ctx.txTimestamps));
  memset(ctx.rxTimestamps, 0, sizeof(ctx.rxTimestamps));
  memset(ctx.distances, 0, sizeof(ctx.distances));

  if (!s_initialized) {
    memset(&ctx.stats, 0, sizeof(ctx.stats));
  }
  refreshStatsSnapshot();

  s_initialized = true;
}

// Called for each DW radio event
static uint32_t tdoa2UwbEvent(dwDevice_t *dev, uwbEvent_t event)
{
  if (ctx.state == synchronizedState) {
    return slotStep(dev, event);
  } else {
    if (ctx.anchorId == 0) {
      // Compute next frame start from the CURRENT DW1000 system time so that
      // delayed TX is always scheduled in the future.  Using the stale
      // ctx.tdmaFrameStart would produce a past time after a prolonged stall
      // (e.g. watchdog recovery), causing the DW1000 to silently reject the
      // delayed TX and stalling the master anchor permanently.
      dwTime_t sysTime = { .full = 0 };
      dwGetSystemTimestamp(dev, &sysTime);
      ctx.tdmaFrameStart.full = tdmaLastFrame(sysTime.full) + 2 * ctx.frameLen;
      ctx.state = synchronizedState;
      recordSyncAcquisition();
      setupTx(dev);

      ctx.slotState = slotTxDone;
      updateSlot();
    } else {
      switch (event) {
        case eventPacketReceived: {
            static packet_t rxPacket;
            int dataLength = dwGetDataLength(dev);
            dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

            if (rxPacket.sourceAddress[0] == 0 && rxPacket.payload[0] == PACKET_TYPE_TDOA2) {
              // Treat this as slot 0 in a synchronized frame and let the normal
              // state machine schedule the next slot.
              ctx.slot = 0;
              ctx.nextSlot = 1;
              ctx.slotState = slotRxDone;
              ctx.state = synchronizedState;
              recordSyncAcquisition();
              return slotStep(dev, eventPacketReceived);
            } else {
              // Start the receiver waiting for a packet from anchor 0
              dwIdle(dev);
              dwSetReceiveWaitTimeout(dev, RECEIVE_SYNC_TIMEOUT);
              dwWriteSystemConfigurationRegister(dev);

              dwNewReceive(dev);
              dwSetDefaults(dev);
              dwStartReceive(dev);
            }
          }
          break;
        default:
          // Start the receiver waiting for a packet from anchor 0
          dwIdle(dev);
          dwSetReceiveWaitTimeout(dev, RECEIVE_SYNC_TIMEOUT);
          dwWriteSystemConfigurationRegister(dev);

          dwNewReceive(dev);
          dwSetDefaults(dev);
          dwStartReceive(dev);
          break;
      }
    }
  }

  return MAX_TIMEOUT;
}

uwbAlgorithm_t uwbTdoa2Algorithm = {
  .init = tdoa2Init,
  .onEvent = tdoa2UwbEvent,
};

// -----------------------------------------------------------------------------
// Public API (C-linkage) for diagnostics / calibration tooling
// -----------------------------------------------------------------------------

extern "C" bool uwbTdoa2AnchorGetDistances(uint16_t* out_distances, uint8_t max_len)
{
  if (!s_initialized || out_distances == nullptr || max_len == 0) {
    return false;
  }

  uint8_t count = max_len;
  if (count > NSLOTS) {
    count = NSLOTS;
  }

  for (uint8_t i = 0; i < count; i++) {
    out_distances[i] = ctx.distances[i];
  }
  return true;
}

extern "C" bool uwbTdoa2AnchorGetStats(uwbTdoa2AnchorStats_t* out_stats)
{
  if (!s_initialized || out_stats == nullptr) {
    return false;
  }

  refreshStatsSnapshot();
  memcpy(out_stats, &ctx.stats, sizeof(ctx.stats));
  return true;
}

extern "C" void uwbTdoa2AnchorRecordStallReset(void)
{
  if (!s_initialized) {
    return;
  }

  inc32(ctx.stats.stallResets);
  inc32(ctx.stats.resyncs);
}

extern "C" uint8_t uwbTdoa2AnchorGetAnchorId(void)
{
  if (!s_initialized) {
    return 0;
  }
  return static_cast<uint8_t>(ctx.anchorId);
}

extern "C" uint16_t uwbTdoa2AnchorGetAntennaDelay(void)
{
  if (!s_initialized) {
    return 0;
  }
  return ctx.antennaDelay;
}

extern "C" void uwbTdoa2AnchorSetAntennaDelay(uint16_t delay)
{
  if (!s_initialized) {
    return;
  }
  ctx.antennaDelay = delay;
}
