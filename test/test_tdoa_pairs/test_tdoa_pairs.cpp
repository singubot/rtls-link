#include <gtest/gtest.h>

#include "uwb/tdoa_common.hpp"
#include "uwb/tdoa_measurement_buffer.hpp"
#include "uwb/tdoa_pairs.hpp"
#include "utils/fixed_json_builder.hpp"
#include "utils/running_stats.hpp"

TEST(TDoAPairs, PairIndexMatchesExpectedFourAnchorOrder)
{
    EXPECT_EQ(tdoa::PairCount(4), 6);
    EXPECT_EQ(tdoa::PairIndex(0, 1, 4), 0);
    EXPECT_EQ(tdoa::PairIndex(0, 2, 4), 1);
    EXPECT_EQ(tdoa::PairIndex(0, 3, 4), 2);
    EXPECT_EQ(tdoa::PairIndex(1, 2, 4), 3);
    EXPECT_EQ(tdoa::PairIndex(1, 3, 4), 4);
    EXPECT_EQ(tdoa::PairIndex(2, 3, 4), 5);

    const tdoa::AnchorPair pair{1, 3};
    EXPECT_EQ(tdoa::PairIndexCanonical(pair, 4), 4);
}

TEST(TDoAPairs, PairIndexMatchesExpectedEightAnchorBoundary)
{
    EXPECT_EQ(tdoa::PairCount(8), 28);
    EXPECT_EQ(tdoa::PairIndex(0, 1, 8), 0);
    EXPECT_EQ(tdoa::PairIndex(0, 7, 8), 6);
    EXPECT_EQ(tdoa::PairIndex(1, 2, 8), 7);
    EXPECT_EQ(tdoa::PairIndex(1, 7, 8), 12);
    EXPECT_EQ(tdoa::PairIndex(6, 7, 8), 27);

    constexpr tdoa::AnchorPair last = tdoa::PairByIndex<8>(27);
    EXPECT_EQ(last.a, 6);
    EXPECT_EQ(last.b, 7);
}

TEST(TDoAPairs, PairIndexIsOrderIndependent)
{
    EXPECT_EQ(tdoa::PairIndex(1, 0, 4), 0);
    EXPECT_EQ(tdoa::PairIndex(3, 0, 4), 2);
    EXPECT_EQ(tdoa::PairIndex(3, 2, 4), 5);
}

TEST(TDoAPairs, CanonicalizeReportsReverseOrder)
{
    tdoa::AnchorPair pair;
    bool reversed = false;

    ASSERT_TRUE(tdoa::CanonicalizePair(3, 1, 4, pair, reversed));
    EXPECT_EQ(pair.a, 1);
    EXPECT_EQ(pair.b, 3);
    EXPECT_TRUE(reversed);

    ASSERT_TRUE(tdoa::CanonicalizePair(1, 3, 4, pair, reversed));
    EXPECT_EQ(pair.a, 1);
    EXPECT_EQ(pair.b, 3);
    EXPECT_FALSE(reversed);
}

TEST(TDoAPairs, RejectsInvalidPairs)
{
    tdoa::AnchorPair pair;
    bool reversed = false;

    EXPECT_FALSE(tdoa::CanonicalizePair(1, 1, 4, pair, reversed));
    EXPECT_EQ(tdoa::PairIndex(4, 1, 4), -1);
    EXPECT_EQ(tdoa::PairIndex(1, 4, 4), -1);
}

TEST(TDoAPairs, PairByIndexMatchesFourAnchorOrder)
{
    constexpr tdoa::AnchorPair p0 = tdoa::PairByIndex<4>(0);
    constexpr tdoa::AnchorPair p5 = tdoa::PairByIndex<4>(5);

    EXPECT_EQ(p0.a, 0);
    EXPECT_EQ(p0.b, 1);
    EXPECT_EQ(p5.a, 2);
    EXPECT_EQ(p5.b, 3);
}

TEST(TDoACommon, ParseAnchorIdAcceptsOneAndTwoDigitIds)
{
    uint8_t anchorId = 0;

    EXPECT_TRUE(tdoa::ParseAnchorId(etl::array<char, 2>{'3', '\0'}, anchorId));
    EXPECT_EQ(anchorId, 3);

    EXPECT_TRUE(tdoa::ParseAnchorId(etl::array<char, 2>{'1', '2'}, anchorId, 13));
    EXPECT_EQ(anchorId, 12);

    EXPECT_TRUE(tdoa::ParseAnchorId(etl::array<char, 2>{'7', '\0'}, anchorId));
    EXPECT_EQ(anchorId, 7);
}

TEST(TDoACommon, ParseAnchorIdRejectsInvalidIds)
{
    uint8_t anchorId = 0;

    EXPECT_FALSE(tdoa::ParseAnchorId(etl::array<char, 2>{'a', '\0'}, anchorId));
    EXPECT_FALSE(tdoa::ParseAnchorId(etl::array<char, 2>{'1', 'x'}, anchorId));
    EXPECT_FALSE(tdoa::ParseAnchorId(etl::array<char, 2>{'8', '\0'}, anchorId));
    EXPECT_FALSE(tdoa::ParseAnchorId(etl::array<char, 2>{'4', '\0'}, anchorId, 4));
}

TEST(TDoAMeasurementBuffer, SnapshotExpiresStaleAndConsumesOnlyWhenEnough)
{
    etl::array<tdoa::MeasurementSlot, 3> slots = {};
    etl::array<bool, 4> configured = {true, true, true, true};
    tdoa::MeasurementSlot snapshot[3] = {};

    slots[0] = tdoa::MeasurementSlot{1.0f, 90, 0, 1, true};
    slots[1] = tdoa::MeasurementSlot{2.0f, 20, 1, 2, true};
    slots[2] = tdoa::MeasurementSlot{3.0f, 80, 2, 3, true};

    const tdoa::MeasurementSnapshotResult result =
        tdoa::SnapshotFreshMeasurements(slots, configured, 100, 50, 2, snapshot, 3);

    EXPECT_TRUE(result.haveEnough);
    EXPECT_EQ(result.copied, 2);
    EXPECT_EQ(result.measurementCountForStats, 2);
    EXPECT_EQ(result.expired, 1);
    EXPECT_EQ(result.consumed, 2);
    EXPECT_EQ(snapshot[0].tdoa, 1.0f);
    EXPECT_EQ(snapshot[1].tdoa, 3.0f);
    EXPECT_FALSE(slots[0].fresh);
    EXPECT_FALSE(slots[1].fresh);
    EXPECT_FALSE(slots[2].fresh);
}

TEST(TDoAMeasurementBuffer, SnapshotLeavesPartialFreshBatchUnconsumed)
{
    etl::array<tdoa::MeasurementSlot, 2> slots = {};
    etl::array<bool, 3> configured = {true, true, true};
    tdoa::MeasurementSlot snapshot[2] = {};

    slots[0] = tdoa::MeasurementSlot{1.0f, 90, 0, 1, true};

    const tdoa::MeasurementSnapshotResult result =
        tdoa::SnapshotFreshMeasurements(slots, configured, 100, 50, 2, snapshot, 2);

    EXPECT_FALSE(result.haveEnough);
    EXPECT_EQ(result.copied, 0);
    EXPECT_EQ(result.measurementCountForStats, 1);
    EXPECT_EQ(result.expired, 0);
    EXPECT_EQ(result.consumed, 0);
    EXPECT_TRUE(slots[0].fresh);
}

TEST(TDoAMeasurementBuffer, SnapshotLeavesBatchWithoutEnoughUniqueAnchorsUnconsumed)
{
    etl::array<tdoa::MeasurementSlot, 5> slots = {};
    etl::array<bool, 5> configured = {true, true, true, true, true};
    tdoa::MeasurementSlot snapshot[5] = {};

    slots[0] = tdoa::MeasurementSlot{1.0f, 90, 0, 1, true};
    slots[1] = tdoa::MeasurementSlot{2.0f, 90, 0, 2, true};
    slots[2] = tdoa::MeasurementSlot{3.0f, 90, 0, 3, true};
    slots[3] = tdoa::MeasurementSlot{4.0f, 90, 1, 2, true};
    slots[4] = tdoa::MeasurementSlot{5.0f, 90, 2, 3, true};

    const tdoa::MeasurementSnapshotResult result =
        tdoa::SnapshotFreshMeasurements(slots, configured, 100, 50, 5, snapshot, 5, 5);

    EXPECT_FALSE(result.haveEnough);
    EXPECT_EQ(result.copied, 0);
    EXPECT_EQ(result.measurementCountForStats, 5);
    EXPECT_EQ(result.expired, 0);
    EXPECT_EQ(result.consumed, 0);
    for (const auto& slot : slots) {
        EXPECT_TRUE(slot.fresh);
    }
}

TEST(TDoAMeasurementBuffer, SnapshotLeavesWideSpanBatchUnconsumed)
{
    etl::array<tdoa::MeasurementSlot, 5> slots = {};
    etl::array<bool, 5> configured = {true, true, true, true, true};
    tdoa::MeasurementSlot snapshot[5] = {};

    slots[0] = tdoa::MeasurementSlot{1.0f, 100, 0, 1, true};
    slots[1] = tdoa::MeasurementSlot{2.0f, 120, 1, 2, true};
    slots[2] = tdoa::MeasurementSlot{3.0f, 140, 2, 3, true};
    slots[3] = tdoa::MeasurementSlot{4.0f, 160, 3, 4, true};
    slots[4] = tdoa::MeasurementSlot{5.0f, 250, 0, 4, true};

    const tdoa::MeasurementSnapshotResult result =
        tdoa::SnapshotFreshMeasurements(slots, configured, 260, 300, 5, snapshot, 5, 5, 120);

    EXPECT_FALSE(result.haveEnough);
    EXPECT_EQ(result.copied, 0);
    EXPECT_EQ(result.measurementCountForStats, 5);
    EXPECT_EQ(result.expired, 0);
    EXPECT_EQ(result.consumed, 0);
    for (const auto& slot : slots) {
        EXPECT_TRUE(slot.fresh);
    }
}

TEST(TDoAMeasurementBuffer, SnapshotExpiresInvalidAnchorSlotsBeforeIndexingConfiguredAnchors)
{
    etl::array<tdoa::MeasurementSlot, 1> slots = {};
    etl::array<bool, 4> configured = {true, true, true, true};
    tdoa::MeasurementSlot snapshot[1] = {};

    slots[0] = tdoa::MeasurementSlot{1.0f, 90, 0, 9, true};

    const tdoa::MeasurementSnapshotResult result =
        tdoa::SnapshotFreshMeasurements(slots, configured, 100, 50, 1, snapshot, 1);

    EXPECT_FALSE(result.haveEnough);
    EXPECT_EQ(result.copied, 0);
    EXPECT_EQ(result.expired, 1);
    EXPECT_EQ(result.consumed, 0);
    EXPECT_FALSE(slots[0].fresh);
}

TEST(TDoAMeasurementBuffer, SnapshotSupportsEightAnchorBoundaryPair)
{
    etl::array<tdoa::MeasurementSlot, tdoa::PairCount(8)> slots = {};
    etl::array<bool, 8> configured = {true, true, true, true, true, true, true, true};
    tdoa::MeasurementSlot snapshot[tdoa::PairCount(8)] = {};

    slots[tdoa::PairIndex(0, 1, 8)] = tdoa::MeasurementSlot{1.0f, 90, 0, 1, true};
    slots[tdoa::PairIndex(6, 7, 8)] = tdoa::MeasurementSlot{2.0f, 95, 6, 7, true};

    const tdoa::MeasurementSnapshotResult result =
        tdoa::SnapshotFreshMeasurements(slots, configured, 100, 50, 2, snapshot, tdoa::PairCount(8));

    EXPECT_TRUE(result.haveEnough);
    EXPECT_EQ(result.copied, 2);
    EXPECT_EQ(snapshot[0].anchor_a, 0);
    EXPECT_EQ(snapshot[0].anchor_b, 1);
    EXPECT_EQ(snapshot[1].anchor_a, 6);
    EXPECT_EQ(snapshot[1].anchor_b, 7);
}

TEST(RunningStats, AppendsJsonToFixedBuilder)
{
    Utils::RunningStats stats;
    Utils::UpdateRunningStats(stats, 1.0);
    Utils::UpdateRunningStats(stats, 2.0);
    Utils::UpdateRunningStats(stats, 3.0);

    Utils::FixedJsonBuilder<128> json;
    Utils::AppendRunningStatsJson(json, stats, 2);

    EXPECT_FALSE(json.Truncated());
    EXPECT_STREQ(json.CStr(), "{\"count\":3,\"mean\":2.00,\"std\":0.82,\"min\":1.00,\"max\":3.00}");
}

TEST(FixedJsonBuilder, ReportsTruncationWithoutOverflow)
{
    Utils::FixedJsonBuilder<8> json;

    EXPECT_FALSE(json.Append("0123456789"));
    EXPECT_TRUE(json.Truncated());
    EXPECT_STREQ(json.CStr(), "01234567");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
