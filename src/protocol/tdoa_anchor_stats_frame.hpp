#pragma once

#include "config/features.hpp"

#ifdef USE_UWB_MODE_TDOA_ANCHOR

#include "protocol/rtls_binary_protocol.hpp"
#include "anchor/tdoa_anchor_api.h"

namespace rtls::protocol {

template <size_t Capacity>
inline void AppendTdoaAnchorStatsFrame(BinaryFrameBuilder<Capacity>& frame,
                                       const uwbTdoa2AnchorStats_t& stats)
{
    frame.Begin(FrameType::TdoaAnchorStats);
    frame.AppendU8(stats.version);
    frame.AppendU8(stats.anchorId);
    frame.AppendU8(stats.activeSlots);
    frame.AppendU8(stats.state);
    frame.AppendU8(stats.slotState);
    frame.AppendU8(stats.slot);
    frame.AppendU8(stats.nextSlot);
    frame.AppendBool(stats.txEnabled != 0);
    frame.AppendU16(stats.antennaDelay);
    frame.AppendU32(stats.slotDurationUs);
    frame.AppendU32(stats.frameDurationUs);
    frame.AppendU8(stats.slot0MissStreak);
    frame.AppendU32(stats.slot0Misses);
    frame.AppendU32(stats.syncAcquisitions);
    frame.AppendU32(stats.syncLosses);
    frame.AppendU32(stats.resyncs);
    frame.AppendU32(stats.stallResets);
    frame.AppendU32(stats.txScheduled);
    frame.AppendU32(stats.txDone);
    frame.AppendU32(stats.irqCount);
    frame.AppendU32(stats.irqToServiceLastUs);
    frame.AppendU32(stats.irqToServiceMaxUs);
    frame.AppendU32(stats.dwHandleInterruptLastUs);
    frame.AppendU32(stats.dwHandleInterruptMaxUs);
    frame.AppendU32(stats.uwbHardPathLastUs);
    frame.AppendU32(stats.uwbHardPathMaxUs);
    frame.AppendU32(stats.slotSlackMinUs);
    frame.AppendU32(stats.rxArmLateCount);
    frame.AppendU32(stats.txArmLateCount);
    frame.AppendU32(stats.missedDeadlineCount);
    frame.AppendU32(stats.guardedTxCount);
    frame.AppendU32(stats.lastDwStatusBeforeStall);

    uint8_t slotCount = stats.activeSlots;
    if (slotCount == 0 || slotCount > UWB_TDOA2_ANCHOR_STATS_SLOT_COUNT) {
        slotCount = UWB_TDOA2_ANCHOR_STATS_SLOT_COUNT;
    }
    frame.AppendU8(slotCount);

    for (uint8_t i = 0; i < slotCount; i++) {
        frame.AppendU8(stats.packetIds[i]);
    }
    for (uint8_t i = 0; i < slotCount; i++) {
        frame.AppendU16(stats.distances[i]);
    }
    for (uint8_t i = 0; i < slotCount; i++) {
        const uwbTdoa2AnchorSlotStats_t& slot = stats.slots[i];
        frame.AppendU32(slot.goodRx);
        frame.AppendU32(slot.rxTimeout);
        frame.AppendU32(slot.rxFailed);
        frame.AppendU32(slot.unexpectedPacket);
        frame.AppendU32(slot.validDistance);
        frame.AppendU32(slot.invalidDistance);
        frame.AppendU32(slot.packetIdMismatch);
    }

    frame.Finish();
}

} // namespace rtls::protocol

#endif // USE_UWB_MODE_TDOA_ANCHOR
