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

void feedBaseRectangle(DynamicAnchorPositionCalculator& calc)
{
    calc.updateDistance(0, 1, 5.0f);
    calc.updateDistance(0, 3, 3.0f);
    calc.updateDistance(0, 2, std::sqrt(34.0f));
}

void feedVerticalPairs(DynamicAnchorPositionCalculator& calc, float separation)
{
    calc.updateDistance(0, 4, separation);
    calc.updateDistance(1, 5, separation);
    calc.updateDistance(2, 6, separation);
    calc.updateDistance(3, 7, separation);
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
    feedBaseRectangle(calc);
    feedVerticalPairs(calc, 2.0f);

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
    feedBaseRectangle(calc);
    feedVerticalPairs(calc, 4.0f);

    ASSERT_TRUE(calc.canCalculate());

    point_t positions[8] = {};
    EXPECT_FALSE(calc.calculatePositions(positions, 8));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
