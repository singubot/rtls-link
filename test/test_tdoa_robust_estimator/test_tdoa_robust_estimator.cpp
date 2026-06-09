#include <gtest/gtest.h>

#include <vector>

#include "tdoa_newton_raphson.hpp"
#include "tdoa_robust_estimator.hpp"

namespace {

using Scalar = tdoa_estimator::Scalar;

tdoa_estimator::PosMatrix makeEightAnchorRoom()
{
    tdoa_estimator::PosMatrix anchors(8, 3);
    anchors << -4.0f, -3.0f, 0.0f,
                4.0f, -3.0f, 0.0f,
               -4.0f,  3.0f, 0.0f,
                4.0f,  3.0f, 0.0f,
               -4.0f, -3.0f, 3.0f,
                4.0f, -3.0f, 3.0f,
               -4.0f,  3.0f, 3.0f,
                4.0f,  3.0f, 3.0f;
    return anchors;
}

tdoa_estimator::PosVector3D centroid(const tdoa_estimator::PosMatrix& anchors)
{
    tdoa_estimator::PosVector3D out = tdoa_estimator::PosVector3D::Zero();
    for (int i = 0; i < anchors.rows(); i++) {
        out += anchors.row(i).transpose();
    }
    return out / static_cast<Scalar>(anchors.rows());
}

void buildAllPairMatrices(const tdoa_estimator::PosMatrix& anchors,
                          const tdoa_estimator::PosVector3D& tag,
                          tdoa_estimator::PosMatrix& L,
                          tdoa_estimator::PosMatrix& R,
                          tdoa_estimator::DynVector& tdoas,
                          std::vector<tdoa_estimator::RobustTdoaRow>& rows)
{
    rows.clear();
    rows.reserve(28);
    L.resize(28, 3);
    R.resize(28, 3);
    tdoas.resize(28);

    int idx = 0;
    for (uint8_t a = 0; a < 8; a++) {
        for (uint8_t b = a + 1; b < 8; b++) {
            const Scalar da = (anchors.row(a).transpose() - tag).norm();
            const Scalar db = (anchors.row(b).transpose() - tag).norm();
            const Scalar tdoa = da - db;

            L.row(idx) = anchors.row(a);
            R.row(idx) = anchors.row(b);
            tdoas(idx) = tdoa;

            tdoa_estimator::RobustTdoaRow row;
            row.anchor_a = a;
            row.anchor_b = b;
            row.anchor_a_pos = anchors.row(a).transpose();
            row.anchor_b_pos = anchors.row(b).transpose();
            row.tdoa = tdoa;
            row.age_us = static_cast<uint32_t>(idx * 2000);
            row.nominal_sigma_m = 0.15f;
            row.health = 1.0f;
            rows.push_back(row);
            idx++;
        }
    }
}

} // namespace

TEST(TDoARobustEstimator, WeightedSolveMatchesUnweightedWhenWeightsAreEqual)
{
    const tdoa_estimator::PosMatrix anchors = makeEightAnchorRoom();
    tdoa_estimator::PosVector3D tag;
    tag << 0.8f, -0.4f, 1.2f;

    tdoa_estimator::PosMatrix L;
    tdoa_estimator::PosMatrix R;
    tdoa_estimator::DynVector tdoas;
    std::vector<tdoa_estimator::RobustTdoaRow> rows;
    buildAllPairMatrices(anchors, tag, L, R, tdoas, rows);

    tdoa_estimator::DynVector weights(tdoas.size());
    weights.setOnes();

    const tdoa_estimator::PosVector3D initial = centroid(anchors);
    const auto unweighted = tdoa_estimator::newtonRaphson(L, R, tdoas, initial, 10);
    const auto weighted = tdoa_estimator::newtonRaphsonWeighted(L, R, tdoas, weights, initial, 10);

    ASSERT_TRUE(unweighted.valid);
    ASSERT_TRUE(weighted.valid);
    EXPECT_LT((unweighted.position - weighted.position).norm(), 1e-4f);
    EXPECT_NEAR(unweighted.rmse, weighted.rmse, 1e-5f);
}

TEST(TDoARobustEstimator, HuberPassDownweightsLargePairOutlier)
{
    const tdoa_estimator::PosMatrix anchors = makeEightAnchorRoom();
    tdoa_estimator::PosVector3D tag;
    tag << 0.7f, 0.6f, 1.4f;

    tdoa_estimator::PosMatrix L;
    tdoa_estimator::PosMatrix R;
    tdoa_estimator::DynVector tdoas;
    std::vector<tdoa_estimator::RobustTdoaRow> rows;
    buildAllPairMatrices(anchors, tag, L, R, tdoas, rows);

    constexpr size_t kOutlier = 3;
    tdoas(kOutlier) += 3.0f;
    rows[kOutlier].tdoa += 3.0f;

    const tdoa_estimator::PosVector3D initial = centroid(anchors);
    const auto legacy = tdoa_estimator::newtonRaphson(L, R, tdoas, initial, 10);

    tdoa_estimator::RobustEstimatorOptions options;
    options.enable_pair_selection = false;
    options.enable_robust_pass = true;
    options.max_selected_rows = 28;
    options.min_residual_scale_m = 0.03f;

    const auto robust = tdoa_estimator::estimateRobust3D(rows.data(), rows.size(), initial, options);

    ASSERT_TRUE(legacy.valid);
    ASSERT_TRUE(robust.solve.valid);
    EXPECT_TRUE(robust.robust_pass_used);
    EXPECT_LT((robust.solve.position - tag).norm(), (legacy.position - tag).norm());
    EXPECT_LT(robust.final_weights[kOutlier], 0.5f);
}

TEST(TDoARobustEstimator, PairSelectionBoundsRowsAndKeepsAnchorCoverage)
{
    const tdoa_estimator::PosMatrix anchors = makeEightAnchorRoom();
    tdoa_estimator::PosVector3D tag;
    tag << 1.0f, -0.5f, 1.1f;

    tdoa_estimator::PosMatrix L;
    tdoa_estimator::PosMatrix R;
    tdoa_estimator::DynVector tdoas;
    std::vector<tdoa_estimator::RobustTdoaRow> rows;
    buildAllPairMatrices(anchors, tag, L, R, tdoas, rows);

    tdoa_estimator::RobustEstimatorOptions options;
    options.enable_pair_selection = true;
    options.enable_robust_pass = false;
    options.max_selected_rows = 10;
    options.min_rows = 5;
    options.min_unique_anchors = 4;

    const auto result = tdoa_estimator::estimateRobust3D(rows.data(), rows.size(), centroid(anchors), options);

    ASSERT_TRUE(result.solve.valid);
    EXPECT_TRUE(result.pair_selection_used);
    EXPECT_LE(result.selected_rows, 10);
    EXPECT_GE(result.selected_rows, 5);
    EXPECT_GE(result.unique_anchors, 4);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
