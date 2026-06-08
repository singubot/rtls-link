#include <gtest/gtest.h>

#include <cmath>

#include "../../lib/tdoa_algorithm/src/tag/dynamicAnchorPositions.cpp"

namespace {

DynamicAnchorConfig baseConfig(uint8_t anchorCount)
{
    DynamicAnchorConfig config{};
    config.layout = 0;
    config.anchorCount = anchorCount;
    config.anchorHeight = 1.5f;
    config.anchorPlaneSeparation = 2.0f;
    config.avgSampleCount = 1;
    config.lockedMask = 0;
    return config;
}

void feedRectangle(DynamicAnchorPositionCalculator& calc, float x, float y)
{
    calc.updateDistance(0, 1, x);
    calc.updateDistance(0, 3, y);
    calc.updateDistance(0, 2, std::sqrt(x * x + y * y));
}

void feedBaseRectangle(DynamicAnchorPositionCalculator& calc)
{
    feedRectangle(calc, 5.0f, 3.0f);
}

void feedBaseRectangleAt(DynamicAnchorPositionCalculator& calc, uint32_t timestamp)
{
    calc.updateDistanceAt(0, 1, 5.0f, timestamp);
    calc.updateDistanceAt(0, 3, 3.0f, timestamp);
    calc.updateDistanceAt(0, 2, std::sqrt(34.0f), timestamp);
}

void feedVerticalPairs(DynamicAnchorPositionCalculator& calc, float separation)
{
    calc.updateDistance(0, 4, separation);
    calc.updateDistance(1, 5, separation);
    calc.updateDistance(2, 6, separation);
    calc.updateDistance(3, 7, separation);
}

void feedUpperRectangle(DynamicAnchorPositionCalculator& calc, float x = 5.0f, float y = 3.0f)
{
    calc.updateDistance(4, 5, x);
    calc.updateDistance(4, 7, y);
    calc.updateDistance(4, 6, std::sqrt(x * x + y * y));
}

void feedCrossPlaneChecks(DynamicAnchorPositionCalculator& calc, float separation, float x = 5.0f, float y = 3.0f)
{
    calc.updateDistance(0, 5, std::sqrt(x * x + separation * separation));
    calc.updateDistance(0, 7, std::sqrt(y * y + separation * separation));
    calc.updateDistance(0, 6, std::sqrt(x * x + y * y + separation * separation));
}

void feedEightAnchorLayout(DynamicAnchorPositionCalculator& calc, float separation, float x = 5.0f, float y = 3.0f)
{
    feedRectangle(calc, x, y);
    feedVerticalPairs(calc, separation);
    feedUpperRectangle(calc, x, y);
    feedCrossPlaneChecks(calc, separation, x, y);
}

void expectPoint(const point_t& p, float x, float y, float z)
{
    EXPECT_NEAR(p.x, x, 1e-5f);
    EXPECT_NEAR(p.y, y, 1e-5f);
    EXPECT_NEAR(p.z, z, 1e-5f);
}

} // namespace

TEST(DynamicAnchorPositions, FourAnchorLayoutKeepsSinglePlaneSemantics)
{
    DynamicAnchorPositionCalculator calc;
    calc.init(baseConfig(4));
    feedBaseRectangle(calc);

    ASSERT_TRUE(calc.canCalculate());

    point_t positions[4] = {};
    ASSERT_TRUE(calc.calculatePositions(positions, 4));

    expectPoint(positions[0], 0.0f, 0.0f, -1.5f);
    expectPoint(positions[1], 5.0f, 0.0f, -1.5f);
    expectPoint(positions[2], 5.0f, 3.0f, -1.5f);
    expectPoint(positions[3], 0.0f, 3.0f, -1.5f);
}

TEST(DynamicAnchorPositions, EightAnchorLayoutPlacesUpperPlaneAboveLowerPlane)
{
    DynamicAnchorPositionCalculator calc;
    calc.init(baseConfig(8));
    feedEightAnchorLayout(calc, 2.0f);

    ASSERT_TRUE(calc.canCalculate());

    point_t positions[8] = {};
    ASSERT_TRUE(calc.calculatePositions(positions, 8));

    expectPoint(positions[0], 0.0f, 0.0f, -1.5f);
    expectPoint(positions[1], 5.0f, 0.0f, -1.5f);
    expectPoint(positions[2], 5.0f, 3.0f, -1.5f);
    expectPoint(positions[3], 0.0f, 3.0f, -1.5f);
    expectPoint(positions[4], 0.0f, 0.0f, -3.5f);
    expectPoint(positions[5], 5.0f, 0.0f, -3.5f);
    expectPoint(positions[6], 5.0f, 3.0f, -3.5f);
    expectPoint(positions[7], 0.0f, 3.0f, -3.5f);
}

TEST(DynamicAnchorPositions, EightAnchorLayoutRequiresVerticalPairs)
{
    DynamicAnchorPositionCalculator calc;
    calc.init(baseConfig(8));
    feedBaseRectangle(calc);

    EXPECT_FALSE(calc.canCalculate());
}

TEST(DynamicAnchorPositions, EightAnchorLayoutRejectsMismatchedPlaneSeparation)
{
    DynamicAnchorPositionCalculator calc;
    calc.init(baseConfig(8));
    feedEightAnchorLayout(calc, 2.0f);
    feedVerticalPairs(calc, 4.0f);

    ASSERT_TRUE(calc.canCalculate());

    point_t positions[8] = {};
    EXPECT_FALSE(calc.calculatePositions(positions, 8));
}

TEST(DynamicAnchorPositions, FourAnchorLayoutRejectsBadDiagonal)
{
    DynamicAnchorPositionCalculator calc;
    calc.init(baseConfig(4));
    calc.updateDistance(0, 1, 5.0f);
    calc.updateDistance(0, 3, 3.0f);
    calc.updateDistance(0, 2, 12.0f);

    ASSERT_TRUE(calc.canCalculate());

    point_t positions[4] = {};
    EXPECT_FALSE(calc.calculatePositions(positions, 4));
}

TEST(DynamicAnchorPositions, EightAnchorLayoutRequiresUpperPlaneEvidence)
{
    DynamicAnchorPositionCalculator calc;
    calc.init(baseConfig(8));
    feedBaseRectangle(calc);
    feedVerticalPairs(calc, 2.0f);

    EXPECT_FALSE(calc.canCalculate());
}

TEST(DynamicAnchorPositions, EightAnchorLayoutRejectsBadUpperPlaneGeometry)
{
    DynamicAnchorPositionCalculator calc;
    calc.init(baseConfig(8));
    feedBaseRectangle(calc);
    feedVerticalPairs(calc, 2.0f);
    calc.updateDistance(4, 5, 8.0f);
    calc.updateDistance(4, 7, 3.0f);
    calc.updateDistance(4, 6, std::sqrt(34.0f));
    feedCrossPlaneChecks(calc, 2.0f);

    ASSERT_TRUE(calc.canCalculate());

    point_t positions[8] = {};
    EXPECT_FALSE(calc.calculatePositions(positions, 8));
}

TEST(DynamicAnchorPositions, FinalizedDistancesExpire)
{
    DynamicAnchorPositionCalculator calc;
    calc.init(baseConfig(4));
    feedBaseRectangleAt(calc, 100);

    EXPECT_TRUE(calc.canCalculateAt(100 + STALENESS_TIMEOUT_TICKS));
    EXPECT_FALSE(calc.canCalculateAt(101 + STALENESS_TIMEOUT_TICKS));
}

TEST(DynamicAnchorPositions, LockMaskPreservesFourAnchorPositionUntilUnlock)
{
    DynamicAnchorPositionCalculator calc;
    calc.init(baseConfig(4));
    feedBaseRectangle(calc);

    point_t positions[4] = {};
    ASSERT_TRUE(calc.calculatePositions(positions, 4));
    expectPoint(positions[1], 5.0f, 0.0f, -1.5f);

    calc.setLockedMask(1 << 1);
    feedRectangle(calc, 7.0f, 4.0f);
    ASSERT_TRUE(calc.calculatePositions(positions, 4));
    expectPoint(positions[1], 5.0f, 0.0f, -1.5f);
    expectPoint(positions[2], 7.0f, 4.0f, -1.5f);

    calc.setLockedMask(0);
    ASSERT_TRUE(calc.calculatePositions(positions, 4));
    expectPoint(positions[1], 7.0f, 0.0f, -1.5f);
}

TEST(DynamicAnchorPositions, LockMaskPreservesEightAnchorPositionUntilUnlock)
{
    DynamicAnchorPositionCalculator calc;
    calc.init(baseConfig(8));
    feedEightAnchorLayout(calc, 2.0f);

    point_t positions[8] = {};
    ASSERT_TRUE(calc.calculatePositions(positions, 8));
    expectPoint(positions[5], 5.0f, 0.0f, -3.5f);

    calc.setLockedMask(1 << 5);
    feedEightAnchorLayout(calc, 2.0f, 7.0f, 4.0f);
    ASSERT_TRUE(calc.calculatePositions(positions, 8));
    expectPoint(positions[5], 5.0f, 0.0f, -3.5f);
    expectPoint(positions[6], 7.0f, 4.0f, -3.5f);

    calc.setLockedMask(0);
    ASSERT_TRUE(calc.calculatePositions(positions, 8));
    expectPoint(positions[5], 7.0f, 0.0f, -3.5f);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
