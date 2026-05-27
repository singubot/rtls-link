#include <Eigen.h>
#include <Eigen/Eigenvalues>  // For SelfAdjointEigenSolver
#include <gtest/gtest.h>

#include "tdoa_newton_raphson.hpp"

#include <vector>
#include <random>
#include <iostream>

class TDOANewtonRaphsonTest : public ::testing::Test {
protected:
    std::default_random_engine generator;

    void SetUp() override {
        std::cout << "\n=== TDOA Newton-Raphson Algorithm Test Suite ===\n" << std::endl;
    }

    using Scalar = tdoa_estimator::Scalar;

    // Helper function to generate random anchor positions
    tdoa_estimator::PosMatrix generateAnchorPositions(int numAnchors, double spaceSize = 10.0) {
        std::uniform_real_distribution<double> distribution(-spaceSize, spaceSize);
        tdoa_estimator::PosMatrix anchors(numAnchors, 3);

        for(int i = 0; i < numAnchors; ++i) {
            for(int j = 0; j < 3; ++j) {
                anchors(i, j) = static_cast<Scalar>(distribution(generator));
            }
        }
        return anchors;
    }

    // Calculate true TDOAs
    std::tuple<tdoa_estimator::PosMatrix, tdoa_estimator::PosMatrix, tdoa_estimator::DynVector> calculateTrueTDOAs(
        const tdoa_estimator::PosMatrix& anchors,
        const std::vector<std::pair<int, int>>& measurementAnchorIds,
        const tdoa_estimator::PosVector3D& truePosition) {

        const int numMeasurements = measurementAnchorIds.size();
        tdoa_estimator::PosMatrix anchorPositionsLeft(numMeasurements, 3);
        tdoa_estimator::PosMatrix anchorPositionsRight(numMeasurements, 3);
        tdoa_estimator::DynVector tdoas(numMeasurements);

        for(int i = 0; i < numMeasurements; ++i) {
            anchorPositionsLeft.row(i) = anchors.row(measurementAnchorIds[i].first);
            anchorPositionsRight.row(i) = anchors.row(measurementAnchorIds[i].second);

            Scalar distanceLeft = (anchorPositionsLeft.row(i).transpose() - truePosition).norm();
            Scalar distanceRight = (anchorPositionsRight.row(i).transpose() - truePosition).norm();
            tdoas(i) = distanceLeft - distanceRight;
        }

        return {anchorPositionsLeft, anchorPositionsRight, tdoas};
    }

    // Calculate initial guess (centroid of anchor bbox)
    tdoa_estimator::PosVector3D calculateInitialGuess(const tdoa_estimator::PosMatrix& anchors) {
        if (anchors.rows() == 0) {
            return tdoa_estimator::PosVector3D::Zero();
        }

        tdoa_estimator::PosVector3D minCoords = anchors.row(0).transpose();
        tdoa_estimator::PosVector3D maxCoords = anchors.row(0).transpose();

        for (int i = 1; i < anchors.rows(); ++i) {
            for (int j = 0; j < 3; ++j) {
                if (anchors(i, j) < minCoords(j)) minCoords(j) = anchors(i, j);
                if (anchors(i, j) > maxCoords(j)) maxCoords(j) = anchors(i, j);
            }
        }
        return (minCoords + maxCoords) * Scalar(0.5);
    }

    // Add noise to TDOAs
    tdoa_estimator::DynVector addNoiseToTDOAs(const tdoa_estimator::DynVector& tdoas, double noiseStd) {
        std::normal_distribution<double> distribution(0.0, noiseStd);
        tdoa_estimator::DynVector noisyTdoas = tdoas;

        for(int i = 0; i < tdoas.size(); ++i) {
            noisyTdoas(i) += static_cast<Scalar>(distribution(generator));
        }
        return noisyTdoas;
    }
};

// --- 3D Tests ---

TEST_F(TDOANewtonRaphsonTest, PerfectConditions3D) {
    std::cout << "\nTesting 3D estimation under perfect conditions..." << std::endl;

    tdoa_estimator::PosVector3D truePosition;
    truePosition << 2.0f, 1.0f, 2.5f;

    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << -5.0f, -5.0f, 0.0f,
               5.0f, -5.0f, 0.0f,
               -5.0f, 5.0f, 0.0f,
               5.0f, 5.0f, 1.5f,
               2.5f, 2.5f, 0.0f;

    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}
    };

    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] =
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);

    tdoa_estimator::PosVector3D initialGuess = calculateInitialGuess(anchors);

    auto result = tdoa_estimator::newtonRaphson(
        anchorPositionsLeft, anchorPositionsRight, tdoas, initialGuess, 10);

    Scalar error = (result.position - truePosition).norm();
    std::cout << "True: " << truePosition.transpose()
              << " Est: " << result.position.transpose()
              << " err=" << error << " iters=" << result.iterations << std::endl;

    EXPECT_LT(error, 0.01f);
    EXPECT_TRUE(result.converged);
}

TEST_F(TDOANewtonRaphsonTest, WithNoise3D) {
    std::cout << "\nTesting 3D estimation with noisy measurements..." << std::endl;

    tdoa_estimator::PosVector3D truePosition;
    truePosition << 2.0f, -3.0f, 4.2f;

    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << -5.0f, -5.0f, 2.0f,
               5.0f, -5.0f, 0.0f,
               -5.0f, 5.0f, 0.0f,
               5.0f, 5.0f, 2.0f,
               2.5f, 2.5f, 0.0f;

    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}
    };

    std::vector<double> noiseLevels = {0.01, 0.05, 0.1, 0.5};

    for(double noiseStd : noiseLevels) {
        auto [anchorPositionsLeft, anchorPositionsRight, tdoas] =
            calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);

        tdoa_estimator::DynVector noisyTdoas = addNoiseToTDOAs(tdoas, noiseStd);
        tdoa_estimator::PosVector3D initialGuess = tdoa_estimator::PosVector3D::Zero();

        auto result = tdoa_estimator::newtonRaphson(
            anchorPositionsLeft, anchorPositionsRight, noisyTdoas, initialGuess, 10);

        Scalar error = (result.position - truePosition).norm();
        std::cout << "noise=" << noiseStd << " err=" << error
                  << " iters=" << result.iterations << std::endl;

        EXPECT_LT(error, static_cast<Scalar>(noiseStd * 3.0));
    }
}

TEST_F(TDOANewtonRaphsonTest, DifferentAnchorConfigurations3D) {
    std::cout << "\nTesting 3D estimation different anchor configurations..." << std::endl;

    tdoa_estimator::PosVector3D truePosition;
    truePosition << 2.0f, 3.0f, 1.0f;

    std::vector<int> numAnchorsToTest = {5, 8, 10};

    for(int numAnchors : numAnchorsToTest) {
        tdoa_estimator::PosMatrix anchors = generateAnchorPositions(numAnchors);

        std::vector<std::pair<int, int>> measurementAnchorIds;
        for(int i = 0; i < numAnchors; ++i) {
            measurementAnchorIds.push_back({i, (i + 1) % numAnchors});
        }

        auto [anchorPositionsLeft, anchorPositionsRight, tdoas] =
            calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);

        tdoa_estimator::PosVector3D initialGuess = calculateInitialGuess(anchors);

        auto result = tdoa_estimator::newtonRaphson(
            anchorPositionsLeft, anchorPositionsRight, tdoas, initialGuess, 10);

        Scalar error = (result.position - truePosition).norm();
        std::cout << "n=" << numAnchors << " err=" << error
                  << " iters=" << result.iterations << std::endl;

        EXPECT_LT(error, 0.1f);
    }
}

TEST_F(TDOANewtonRaphsonTest, EightAnchorAllPairs3D) {
    tdoa_estimator::PosMatrix anchors(8, 3);
    anchors << -4.0f, -3.0f, 0.0f,
                4.0f, -3.0f, 0.0f,
               -4.0f,  3.0f, 0.0f,
                4.0f,  3.0f, 0.0f,
               -4.0f, -3.0f, 3.0f,
                4.0f, -3.0f, 3.0f,
               -4.0f,  3.0f, 3.0f,
                4.0f,  3.0f, 3.0f;

    std::vector<std::pair<int, int>> measurementAnchorIds;
    for (int a = 0; a < 8; ++a) {
        for (int b = a + 1; b < 8; ++b) {
            measurementAnchorIds.push_back({a, b});
        }
    }
    ASSERT_EQ(measurementAnchorIds.size(), 28u);
    ASSERT_LE(measurementAnchorIds.size(), tdoa_estimator::kMaxCapacity);

    tdoa_estimator::PosVector3D truePosition;
    truePosition << 1.2f, -0.7f, 1.1f;

    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] =
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);

    tdoa_estimator::PosVector3D initialGuess = calculateInitialGuess(anchors);

    auto result = tdoa_estimator::newtonRaphson(
        anchorPositionsLeft, anchorPositionsRight, tdoas, initialGuess, 15);

    Scalar error = (result.position - truePosition).norm();
    std::cout << "8-anchor all-pairs err=" << error
              << " iters=" << result.iterations << std::endl;

    EXPECT_TRUE(result.valid);
    EXPECT_LT(error, 0.05f);
}

TEST_F(TDOANewtonRaphsonTest, RealAnchorLayout3D) {
    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << 0.0f, 0.0f, 1.8f,
               5.5f, 0.0f, 1.8f,
               0.0f, 2.5f, 1.8f,
               5.5f, 2.5f, 1.8f,
               2.7f, 0.0f, 1.1f;

    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {1, 2}, {2, 3}, {0, 1}, {4, 0}, {3, 4}, {1, 4}
    };

    tdoa_estimator::PosVector3D truePosition;
    truePosition << 3.0f, 1.0f, 2.0f;

    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] =
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);

    tdoa_estimator::PosVector3D initialGuess = calculateInitialGuess(anchors);

    auto result = tdoa_estimator::newtonRaphson(
        anchorPositionsLeft, anchorPositionsRight, tdoas, initialGuess, 10);

    Scalar error = (result.position - truePosition).norm();
    std::cout << "RealLayout err=" << error << " iters=" << result.iterations << std::endl;
    EXPECT_LT(error, 0.01f);
}

// --- 2D Tests ---

TEST_F(TDOANewtonRaphsonTest, PerfectConditions2D) {
    tdoa_estimator::PosVector3D truePosition3D;
    truePosition3D << 2.0f, 1.0f, 2.5f;
    tdoa_estimator::PosVector2D truePosition2D = truePosition3D.head<2>();

    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << -5.0f, -5.0f, 0.0f,
               5.0f, -5.0f, 0.0f,
               -5.0f, 5.0f, 0.0f,
               5.0f, 5.0f, 1.5f,
               2.5f, 2.5f, 0.0f;

    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}
    };

    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] =
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition3D);

    tdoa_estimator::PosVector3D centroid3D = calculateInitialGuess(anchors);
    tdoa_estimator::PosVector2D initialGuessXY = centroid3D.head<2>();

    Scalar fixedZ = truePosition3D(2);

    auto result = tdoa_estimator::newtonRaphson2D(
        anchorPositionsLeft, anchorPositionsRight, tdoas, initialGuessXY, fixedZ, 10);

    Scalar errorXY = (result.position - truePosition2D).norm();
    std::cout << "2D err=" << errorXY << " iters=" << result.iterations << std::endl;
    EXPECT_LT(errorXY, 0.01f);
    EXPECT_TRUE(result.converged);
}

TEST_F(TDOANewtonRaphsonTest, WithNoise2D) {
    tdoa_estimator::PosVector3D truePosition3D;
    truePosition3D << 2.0f, -3.0f, 4.2f;
    tdoa_estimator::PosVector2D truePosition2D = truePosition3D.head<2>();

    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << -5.0f, -5.0f, 2.0f,
               5.0f, -5.0f, 0.0f,
               -5.0f, 5.0f, 0.0f,
               5.0f, 5.0f, 2.0f,
               2.5f, 2.5f, 0.0f;

    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}
    };

    double noiseStd = 0.1;
    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] =
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition3D);
    tdoa_estimator::DynVector noisyTdoas = addNoiseToTDOAs(tdoas, noiseStd);

    tdoa_estimator::PosVector3D centroid3D = calculateInitialGuess(anchors);
    tdoa_estimator::PosVector2D initialGuessXY = centroid3D.head<2>();
    Scalar fixedZ = truePosition3D(2);

    auto result = tdoa_estimator::newtonRaphson2D(
        anchorPositionsLeft, anchorPositionsRight, noisyTdoas, initialGuessXY, fixedZ, 10);

    Scalar errorXY = (result.position - truePosition2D).norm();
    std::cout << "2D noise=" << noiseStd << " err=" << errorXY << std::endl;
    EXPECT_LT(errorXY, static_cast<Scalar>(noiseStd * 5.0));
}

// --- Covariance Tests ---

TEST_F(TDOANewtonRaphsonTest, CovarianceComputation3D) {
    tdoa_estimator::PosVector3D truePosition;
    truePosition << 2.0f, 1.0f, 2.5f;

    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << -5.0f, -5.0f, 0.0f,
               5.0f, -5.0f, 0.0f,
               -5.0f, 5.0f, 0.0f,
               5.0f, 5.0f, 1.5f,
               2.5f, 2.5f, 0.0f;

    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}
    };

    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] =
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);

    tdoa_estimator::PosVector3D initialGuess = calculateInitialGuess(anchors);

    auto result = tdoa_estimator::newtonRaphson(
        anchorPositionsLeft, anchorPositionsRight, tdoas, initialGuess, 10);

    EXPECT_TRUE(result.covarianceValid);
    EXPECT_TRUE(result.converged);
    EXPECT_TRUE(result.valid);

    EXPECT_NEAR(result.positionCovariance(0, 1), result.positionCovariance(1, 0), 1e-10);
    EXPECT_NEAR(result.positionCovariance(0, 2), result.positionCovariance(2, 0), 1e-10);
    EXPECT_NEAR(result.positionCovariance(1, 2), result.positionCovariance(2, 1), 1e-10);

    Eigen::SelfAdjointEigenSolver<tdoa_estimator::CovMatrix3D> eigensolver(result.positionCovariance);
    auto eigenvalues = eigensolver.eigenvalues();
    for (int i = 0; i < 3; ++i) {
        EXPECT_GE(eigenvalues(i), -1e-10) << "Covariance should be PSD";
    }

    EXPECT_GT(result.positionCovariance(0, 0), 0.0);
    EXPECT_GT(result.positionCovariance(1, 1), 0.0);
    EXPECT_GT(result.positionCovariance(2, 2), 0.0);
}

TEST_F(TDOANewtonRaphsonTest, CovarianceScalesWithNoise) {
    tdoa_estimator::PosVector3D truePosition;
    truePosition << 2.0f, -3.0f, 4.2f;

    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << -5.0f, -5.0f, 2.0f,
               5.0f, -5.0f, 0.0f,
               -5.0f, 5.0f, 0.0f,
               5.0f, 5.0f, 2.0f,
               2.5f, 2.5f, 0.0f;

    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}
    };

    std::vector<double> noiseLevels = {0.01, 0.1, 0.5};
    std::vector<double> avgVariances;

    for (double noiseStd : noiseLevels) {
        auto [anchorPositionsLeft, anchorPositionsRight, tdoas] =
            calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);
        tdoa_estimator::DynVector noisyTdoas = addNoiseToTDOAs(tdoas, noiseStd);
        tdoa_estimator::PosVector3D initialGuess = tdoa_estimator::PosVector3D::Zero();

        auto result = tdoa_estimator::newtonRaphson(
            anchorPositionsLeft, anchorPositionsRight, noisyTdoas, initialGuess, 10);

        double avgVar = (result.positionCovariance(0,0)
                       + result.positionCovariance(1,1)
                       + result.positionCovariance(2,2)) / 3.0;
        avgVariances.push_back(avgVar);
    }

    EXPECT_LT(avgVariances[0], avgVariances[1]);
    EXPECT_LT(avgVariances[1], avgVariances[2]);
}

TEST_F(TDOANewtonRaphsonTest, Covariance2DMode) {
    tdoa_estimator::PosVector3D truePosition3D;
    truePosition3D << 2.0f, 1.0f, 2.5f;

    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << -5.0f, -5.0f, 0.0f,
               5.0f, -5.0f, 0.0f,
               -5.0f, 5.0f, 0.0f,
               5.0f, 5.0f, 1.5f,
               2.5f, 2.5f, 0.0f;

    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}
    };

    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] =
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition3D);

    tdoa_estimator::PosVector3D centroid3D = calculateInitialGuess(anchors);
    tdoa_estimator::PosVector2D initialGuessXY = centroid3D.head<2>();
    Scalar fixedZ = truePosition3D(2);

    auto result = tdoa_estimator::newtonRaphson2D(
        anchorPositionsLeft, anchorPositionsRight, tdoas, initialGuessXY, fixedZ, 10);

    EXPECT_TRUE(result.covarianceValid);
    EXPECT_TRUE(result.converged);
    EXPECT_TRUE(result.valid);

    EXPECT_NEAR(result.positionCovariance(0, 1), result.positionCovariance(1, 0), 1e-10);
    EXPECT_GT(result.positionCovariance(0, 0), 0.0);
    EXPECT_GT(result.positionCovariance(1, 1), 0.0);

    Eigen::SelfAdjointEigenSolver<tdoa_estimator::CovMatrix2D> eigensolver(result.positionCovariance);
    auto eigenvalues = eigensolver.eigenvalues();
    for (int i = 0; i < 2; ++i) {
        EXPECT_GE(eigenvalues(i), -1e-10) << "2D Covariance should be PSD";
    }
}

// --- Numerical stability tests (the cases that broke prior float attempts) ---

// Tag at the centroid of the anchor set: J rows become near-linearly dependent.
// In normal-equations form the squared condition would blow float; QR-on-J + LM
// damping must keep it tractable. One anchor lifted out of plane so Z is
// observable (otherwise the system has a Z-mirror ambiguity around the plane).
TEST_F(TDOANewtonRaphsonTest, CentroidStability3D) {
    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << -5.0f, -5.0f, 0.0f,
               5.0f, -5.0f, 0.0f,
               -5.0f, 5.0f, 0.0f,
               5.0f, 5.0f, 0.0f,
               0.0f, 0.0f, 3.5f;  // lifted anchor breaks Z symmetry

    tdoa_estimator::PosVector3D truePosition;
    truePosition << 0.05f, 0.05f, 1.5f;  // very near centroid in XY

    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 3}, {3, 2}, {2, 0}, {0, 4}, {1, 4}, {2, 4}, {3, 4}
    };

    auto [L, R, tdoas] = calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);
    tdoa_estimator::PosVector3D initialGuess = calculateInitialGuess(anchors);

    auto result = tdoa_estimator::newtonRaphson(L, R, tdoas, initialGuess, 10);
    Scalar err = (result.position - truePosition).norm();
    std::cout << "Centroid err=" << err << " iters=" << result.iterations
              << " converged=" << result.converged << std::endl;

    EXPECT_FALSE(result.position.hasNaN());
    EXPECT_LT(err, 0.1f);
}

// Hyperboloid asymptote: tag chosen so dL ≈ dR for several pairs. That row
// of J is near-zero — any unguarded division produces noise. Distance floor
// + LM should keep the solver pointed in a sensible direction.
TEST_F(TDOANewtonRaphsonTest, HyperboloidAsymptote2D) {
    tdoa_estimator::PosMatrix anchors(4, 3);
    anchors << -3.0f, 0.0f, 1.8f,
               3.0f, 0.0f, 1.8f,
               0.0f, -3.0f, 1.8f,
               0.0f, 3.0f, 1.8f;

    // Tag on the perpendicular bisector of the (0,1) pair: dL == dR for that pair.
    tdoa_estimator::PosVector3D truePosition;
    truePosition << 0.0f, 1.0f, 0.0f;

    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {2, 3}, {0, 2}, {1, 3}, {0, 3}
    };

    auto [L, R, tdoas] = calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);

    tdoa_estimator::PosVector2D initialGuess;
    initialGuess << 0.5f, 0.5f;
    Scalar fixedZ = 0.0f;

    auto result = tdoa_estimator::newtonRaphson2D(L, R, tdoas, initialGuess, fixedZ, 10);

    Scalar errXY = (result.position - truePosition.head<2>()).norm();
    std::cout << "Asymptote err=" << errXY << " iters=" << result.iterations
              << " converged=" << result.converged << std::endl;
    EXPECT_FALSE(result.position.hasNaN());
    EXPECT_LT(errXY, 0.1f);
}

// Initial guess outside the anchor envelope: validates LM keeps the iteration
// descending instead of oscillating or producing NaN. Tag-side warm start in
// production stays within or near the anchor envelope; we test ~10m offset
// (a tag walking just outside the anchor area, e.g. between flights).
TEST_F(TDOANewtonRaphsonTest, OffsetInitialGuess3D) {
    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << 0.0f, 0.0f, 1.8f,
               5.5f, 0.0f, 1.8f,
               0.0f, 2.5f, 1.8f,
               5.5f, 2.5f, 1.8f,
               2.7f, 0.0f, 1.1f;

    tdoa_estimator::PosVector3D truePosition;
    truePosition << 3.0f, 1.0f, 1.5f;

    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}, {0, 2}
    };

    auto [L, R, tdoas] = calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);

    tdoa_estimator::PosVector3D initialGuess;
    initialGuess << 13.0f, 12.0f, 5.0f;

    auto result = tdoa_estimator::newtonRaphson(L, R, tdoas, initialGuess, 15);

    Scalar err = (result.position - truePosition).norm();
    std::cout << "Offset init err=" << err << " iters=" << result.iterations
              << " converged=" << result.converged << std::endl;
    EXPECT_FALSE(result.position.hasNaN());
    EXPECT_LT(err, 0.5f);
}

// Iteration count regression: warm-started, well-conditioned solve should
// converge in ≤ 3 iterations.
TEST_F(TDOANewtonRaphsonTest, WarmStartConvergesQuickly) {
    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << 0.0f, 0.0f, 1.8f,
               5.5f, 0.0f, 1.8f,
               0.0f, 2.5f, 1.8f,
               5.5f, 2.5f, 1.8f,
               2.7f, 0.0f, 1.1f;

    tdoa_estimator::PosVector3D truePosition;
    truePosition << 2.5f, 1.2f, 1.0f;

    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}, {0, 2}
    };

    auto [L, R, tdoas] = calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);

    // Warm start within 10 cm of solution.
    tdoa_estimator::PosVector3D initialGuess = truePosition;
    initialGuess(0) += 0.05f;
    initialGuess(1) -= 0.07f;
    initialGuess(2) += 0.03f;

    auto result = tdoa_estimator::newtonRaphson(L, R, tdoas, initialGuess, 5);
    std::cout << "Warm start iters=" << result.iterations
              << " err=" << (result.position - truePosition).norm() << std::endl;
    EXPECT_LE(result.iterations, 3);
    EXPECT_TRUE(result.converged);
}

#ifndef ARDUINO

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    if (RUN_ALL_TESTS())
    ;
    return 0;
}

#else // ARDUINO

#include <Arduino.h>

void setup() {
  int argc = 1;
  char *argv[] = { (char *)"dummy" };
  ::testing::InitGoogleTest(&argc, argv);
  RUN_ALL_TESTS();
}

void loop() {
  delay(100);
}

#endif // ARDUINO
