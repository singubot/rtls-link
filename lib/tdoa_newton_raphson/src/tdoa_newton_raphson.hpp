#pragma once

#include <Eigen.h>
#include <Eigen/QR>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace tdoa_estimator {

    static constexpr size_t kMaxCapacity = 32; // Max anchor pairs per solve

    // Solver scalar type: float for ESP32 hardware FPU.
    // Stability is preserved by solving J*Δ = r via QR (not normal equations)
    // and Levenberg-Marquardt damping inside the iteration.
    using Scalar = float;

    // Eigen types (iteration in float):
    using PosMatrix = Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, kMaxCapacity, 3>;
    using DynVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, kMaxCapacity, 1>;
    using PosVector2D = Eigen::Matrix<Scalar, 2, 1>;
    using PosVector3D = Eigen::Matrix<Scalar, 3, 1>;

    // 2D-Jacobian shape (Nx2) used by the 2D solver.
    using PosMatrix2 = Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, kMaxCapacity, 2>;

    // Covariance kept in double - matrix inverse is precision-sensitive.
    using CovMatrix3D = Eigen::Matrix<double, 3, 3>;
    using CovMatrix2D = Eigen::Matrix<double, 2, 2>;

    struct SolverResult {
        PosVector3D position;            // 3D position
        Scalar rmse;                     // Root Mean Square Error of residuals (m)
        int iterations;                  // Number of iterations performed
        bool converged;                  // True if step/residual delta met threshold
        bool valid;                      // True if converged, within RMSE threshold, and observable
        CovMatrix3D positionCovariance;  // 3x3 position covariance (double)
        bool covarianceValid;            // True if covariance computation succeeded
    };

    struct SolverResult2D {
        PosVector2D position;
        Scalar rmse;
        int iterations;
        bool converged;
        bool valid;                      // True if converged and within RMSE threshold
        CovMatrix2D positionCovariance;
        bool covarianceValid;
    };

    struct TDoAMeasurement {
        int anchor_a;
        int anchor_b;
        Scalar tdoa;
        uint64_t timestamp;
    };

    // Main Newton-Raphson function (3D). Levenberg-Marquardt damped Gauss-Newton
    // with QR-based step solve and warm-start. Defaults tuned for UWB noise (~5-10cm).
    SolverResult newtonRaphson(const PosMatrix& anchorPositionsLeft,
                               const PosMatrix& anchorPositionsRight,
                               const DynVector& doas,
                               PosVector3D initialPos,
                               int maxIterations = 5,
                               Scalar convergenceThreshold = 1e-3f,
                               Scalar rmseThreshold = 0.8f);

    SolverResult newtonRaphsonWeighted(const PosMatrix& anchorPositionsLeft,
                                       const PosMatrix& anchorPositionsRight,
                                       const DynVector& doas,
                                       const DynVector& weights,
                                       PosVector3D initialPos,
                                       int maxIterations = 5,
                                       Scalar convergenceThreshold = 1e-3f,
                                       Scalar rmseThreshold = 0.8f);

    void computeResiduals3D(const PosMatrix& anchorPositionsLeft,
                            const PosMatrix& anchorPositionsRight,
                            const DynVector& doas,
                            const PosVector3D& position,
                            DynVector& residuals);

    // Main Newton-Raphson function (2D) - solves XY with Z fixed.
    SolverResult2D newtonRaphson2D(const PosMatrix& anchorPositionsLeft,
                                   const PosMatrix& anchorPositionsRight,
                                   const DynVector& doas,
                                   PosVector2D initialPos,
                                   Scalar fixedZ,
                                   int maxIterations = 5,
                                   Scalar convergenceThreshold = 1e-3f,
                                   Scalar rmseThreshold = 0.8f);

}
