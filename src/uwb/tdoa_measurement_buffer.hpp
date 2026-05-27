#pragma once

#include <stddef.h>
#include <stdint.h>

#include <etl/array.h>

namespace tdoa {

struct MeasurementSlot {
    float tdoa = 0.0f;          // canonical: distance(a) - distance(b) with a < b
    uint64_t timestamp_us = 0;  // esp_timer_get_time() when last updated
    uint8_t anchor_a = 0;       // canonical (smaller) anchor id
    uint8_t anchor_b = 0;       // canonical (larger) anchor id
    bool fresh = false;         // true if updated since last consume
};

struct MeasurementSnapshotResult {
    bool haveEnough = false;
    size_t copied = 0;
    size_t measurementCountForStats = 0;
    uint32_t expired = 0;
    uint32_t consumed = 0;
};

template <size_t PairCount, size_t AnchorCount>
MeasurementSnapshotResult SnapshotFreshMeasurements(
    etl::array<MeasurementSlot, PairCount>& slots,
    const etl::array<bool, AnchorCount>& configuredAnchors,
    uint64_t nowUs,
    uint64_t staleThresholdUs,
    size_t minMeasurements,
    MeasurementSlot* out,
    size_t outCapacity,
    uint8_t minUniqueAnchors = 0)
{
    MeasurementSnapshotResult result;
    size_t usableFresh = 0;
    etl::array<bool, AnchorCount> usableAnchorSeen = {};
    uint8_t usableUniqueAnchors = 0;

    for (auto& slot : slots) {
        if (!slot.fresh) {
            continue;
        }
        if ((nowUs - slot.timestamp_us) > staleThresholdUs
            || slot.anchor_a >= AnchorCount
            || slot.anchor_b >= AnchorCount
            || !configuredAnchors[slot.anchor_a]
            || !configuredAnchors[slot.anchor_b]) {
            slot.fresh = false;
            ++result.expired;
            continue;
        }
        ++usableFresh;
        if (!usableAnchorSeen[slot.anchor_a]) {
            usableAnchorSeen[slot.anchor_a] = true;
            ++usableUniqueAnchors;
        }
        if (!usableAnchorSeen[slot.anchor_b]) {
            usableAnchorSeen[slot.anchor_b] = true;
            ++usableUniqueAnchors;
        }
    }

    result.haveEnough = usableFresh >= minMeasurements
                     && usableUniqueAnchors >= minUniqueAnchors;
    result.measurementCountForStats = usableFresh;
    if (!result.haveEnough) {
        return result;
    }

    for (auto& slot : slots) {
        if (!slot.fresh) {
            continue;
        }
        if (result.copied >= outCapacity) {
            break;
        }
        out[result.copied++] = slot;
        slot.fresh = false;
        ++result.consumed;
    }
    result.measurementCountForStats = result.copied;
    return result;
}

} // namespace tdoa
