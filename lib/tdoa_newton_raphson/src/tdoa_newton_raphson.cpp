#include "tdoa_newton_raphson.hpp"

#include <Eigen/QR>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <limits>

namespace tdoa_estimator {

    // Minimum measurement variance floor (1cm std dev squared)
    static constexpr double kMinMeasurementVariance = 0.0001;

    // Tikhonov regularization threshold for covariance computation.
    static constexpr double kRegularizationThreshold = 1e-8;
    static constexpr double kRegularizationFactor = 1e-6;
    static constexpr double kMinGeometryEigenRatio3D = 1e-5;
    static constexpr Scalar kResidualConvergenceThreshold = static_cast<Scalar>(1e-6);

    // Distance floor: prevents division-by-near-zero in Jacobian rows when the
    // tag sits effectively on top of an anchor. 1e-4 m = 0.1 mm — well below
    // sensor noise but large enough to keep float arithmetic stable.
    static constexpr Scalar kDistanceFloor = static_cast<Scalar>(1e-4);

    // Levenberg-Marquardt parameters.
    static constexpr Scalar kLMInitialFactor = static_cast<Scalar>(1e-3);
    static constexpr Scalar kLMShrinkFactor = static_cast<Scalar>(0.5);
    static constexpr Scalar kLMGrowFactor = static_cast<Scalar>(4.0);
    static constexpr Scalar kLMMaxLambda = static_cast<Scalar>(1e6);
    static constexpr Scalar kLMMinLambda = static_cast<Scalar>(1e-12);

    namespace {

        // Per-call scratch / cached state. All buffers respect kMaxCapacity so
        // they live on the stack for the duration of the solve.
        struct SolverContext3D {
            DynVector residuals;       // (numAnchors)
            DynVector distancesL;      // (numAnchors) - cached norms
            DynVector distancesR;      // (numAnchors) - cached norms
            PosMatrix jacobian;        // (numAnchors, 3) - cached for covariance
            // Augmented system buffers for LM: [J; sqrt(λ)·I] · Δ = [r; 0]
            PosMatrix jaug;            // (numAnchors+3, 3)
            DynVector raug;            // (numAnchors+3)
        };

        struct SolverContext2D {
            DynVector residuals;
            DynVector distancesL;
            DynVector distancesR;
            PosMatrix2 jacobian;       // (numAnchors, 2)
            PosMatrix2 jaug;           // (numAnchors+2, 2)
            DynVector raug;            // (numAnchors+2)
        };

        // Compute residuals and cache per-row distances. The Jacobian builders
        // reuse these distances, so the expensive `.norm()` calls only happen
        // here — once per iteration instead of three times.
        inline void computeResidualsAndDistances3D(const PosMatrix& L,
                                                    const PosMatrix& R,
                                                    const DynVector& doas,
                                                    const PosVector3D& pos,
                                                    DynVector& residuals,
                                                    DynVector& distancesL,
                                                    DynVector& distancesR)
        {
            const int n = static_cast<int>(L.rows());
            residuals.resize(n);
            distancesL.resize(n);
            distancesR.resize(n);

            for (int i = 0; i < n; ++i) {
                Scalar dL = (L.row(i).transpose() - pos).norm();
                Scalar dR = (R.row(i).transpose() - pos).norm();
                if (dL < kDistanceFloor) dL = kDistanceFloor;
                if (dR < kDistanceFloor) dR = kDistanceFloor;
                distancesL(i) = dL;
                distancesR(i) = dR;
                residuals(i) = (dL - dR) - doas(i);
            }
        }

        // Build the 3D Jacobian using already-cached distances.
        inline void buildJacobian3D(const PosMatrix& L,
                                    const PosMatrix& R,
                                    const PosVector3D& pos,
                                    const DynVector& distancesL,
                                    const DynVector& distancesR,
                                    PosMatrix& J)
        {
            const int n = static_cast<int>(L.rows());
            J.resize(n, 3);
            for (int i = 0; i < n; ++i) {
                J.row(i) = (pos - L.row(i).transpose()).transpose() / distancesL(i)
                         - (pos - R.row(i).transpose()).transpose() / distancesR(i);
            }
        }

        // Build the 2D Jacobian (Nx2) using cached distances.
        inline void buildJacobian2D(const PosMatrix& L,
                                    const PosMatrix& R,
                                    const PosVector3D& pos3D,
                                    const DynVector& distancesL,
                                    const DynVector& distancesR,
                                    PosMatrix2& J)
        {
            const int n = static_cast<int>(L.rows());
            J.resize(n, 2);
            for (int i = 0; i < n; ++i) {
                Eigen::Matrix<Scalar, 1, 3> diffL = pos3D.transpose() - L.row(i);
                Eigen::Matrix<Scalar, 1, 3> diffR = pos3D.transpose() - R.row(i);
                Eigen::Matrix<Scalar, 1, 3> gradL = diffL / distancesL(i);
                Eigen::Matrix<Scalar, 1, 3> gradR = diffR / distancesR(i);
                J.row(i) = (gradL - gradR).template head<2>();
            }
        }

        // Solve [J; sqrt(λ)·I] · Δ = [r; 0] via HouseholderQR. Operating on the
        // augmented system in float keeps the condition number bounded and
        // avoids the squared-condition penalty of normal equations — this is
        // the path that lets us use float without losing centroid stability.
        template <int Cols, typename JType, typename JaugType, typename RaugType, typename DeltaType>
        inline void solveLMStep(const JType& J,
                                const DynVector& residuals,
                                Scalar lambda,
                                JaugType& jaug,
                                RaugType& raug,
                                DeltaType& delta)
        {
            const int n = static_cast<int>(J.rows());
            jaug.resize(n + Cols, Cols);
            raug.resize(n + Cols);

            jaug.template topRows(n) = J;
            jaug.template bottomRows(Cols) = Eigen::Matrix<Scalar, Cols, Cols>::Identity()
                                             * std::sqrt(lambda);
            raug.head(n) = residuals;
            raug.tail(Cols).setZero();

            delta = jaug.householderQr().solve(raug);
        }

        inline Scalar trace3(const PosMatrix& J) {
            // trace(JᵀJ) = sum of squared column norms
            return J.colwise().squaredNorm().sum();
        }

        inline Scalar trace2(const PosMatrix2& J) {
            return J.colwise().squaredNorm().sum();
        }

        bool matrixObservable3D(const CovMatrix3D& matrix, double trace)
        {
            if (trace < 1e-10) {
                return false;
            }

            Eigen::SelfAdjointEigenSolver<CovMatrix3D> eigensolver(matrix, Eigen::EigenvaluesOnly);
            if (eigensolver.info() != Eigen::Success) {
                return false;
            }

            const double minEigen = eigensolver.eigenvalues().minCoeff();
            return minEigen > kMinGeometryEigenRatio3D * trace;
        }

        bool anchorLayoutObservable3D(const PosMatrix& L, const PosMatrix& R)
        {
            const int n = static_cast<int>(L.rows());
            if (n < 4 || R.rows() != L.rows()) {
                return false;
            }

            Eigen::Matrix<double, 3, 1> centroid = Eigen::Matrix<double, 3, 1>::Zero();
            for (int i = 0; i < n; ++i) {
                centroid += L.row(i).transpose().template cast<double>();
                centroid += R.row(i).transpose().template cast<double>();
            }
            centroid /= static_cast<double>(2 * n);

            CovMatrix3D scatter = CovMatrix3D::Zero();
            for (int i = 0; i < n; ++i) {
                Eigen::Matrix<double, 3, 1> dl =
                    L.row(i).transpose().template cast<double>() - centroid;
                Eigen::Matrix<double, 3, 1> dr =
                    R.row(i).transpose().template cast<double>() - centroid;
                scatter += dl * dl.transpose();
                scatter += dr * dr.transpose();
            }

            return matrixObservable3D(scatter, scatter.trace());
        }

        // Compute 3D covariance from the cached float Jacobian. JᵀJ is built in
        // double — small (3x3), runs once at convergence, worth the precision.
        bool computePositionCovariance3D(const PosMatrix& J,
                                          double measurementVariance,
                                          CovMatrix3D& outCovariance)
        {
            const int n = static_cast<int>(J.rows());
            CovMatrix3D JtJ = CovMatrix3D::Zero();
            for (int i = 0; i < n; ++i) {
                double j0 = static_cast<double>(J(i, 0));
                double j1 = static_cast<double>(J(i, 1));
                double j2 = static_cast<double>(J(i, 2));
                JtJ(0, 0) += j0 * j0; JtJ(0, 1) += j0 * j1; JtJ(0, 2) += j0 * j2;
                JtJ(1, 1) += j1 * j1; JtJ(1, 2) += j1 * j2;
                JtJ(2, 2) += j2 * j2;
            }
            JtJ(1, 0) = JtJ(0, 1);
            JtJ(2, 0) = JtJ(0, 2);
            JtJ(2, 1) = JtJ(1, 2);

            double trace = JtJ.trace();
            double minDiag = JtJ.diagonal().minCoeff();

            if (trace < 1e-10) {
                outCovariance = CovMatrix3D::Identity() * 100.0;
                return false;
            }

            if (!matrixObservable3D(JtJ, trace)) {
                outCovariance = CovMatrix3D::Identity() * 100.0;
                return false;
            }

            if (minDiag < kRegularizationThreshold * trace) {
                JtJ += (kRegularizationFactor * trace) * CovMatrix3D::Identity();
            }

            Eigen::LDLT<CovMatrix3D> ldlt(JtJ);
            if (ldlt.info() != Eigen::Success) {
                outCovariance = CovMatrix3D::Identity() * 100.0;
                return false;
            }

            outCovariance = ldlt.solve(CovMatrix3D::Identity()) * measurementVariance;
            outCovariance = (outCovariance + outCovariance.transpose()) / 2.0;
            return true;
        }

        bool computePositionCovariance2D(const PosMatrix2& J,
                                          double measurementVariance,
                                          CovMatrix2D& outCovariance)
        {
            const int n = static_cast<int>(J.rows());
            CovMatrix2D JtJ = CovMatrix2D::Zero();
            for (int i = 0; i < n; ++i) {
                double j0 = static_cast<double>(J(i, 0));
                double j1 = static_cast<double>(J(i, 1));
                JtJ(0, 0) += j0 * j0;
                JtJ(0, 1) += j0 * j1;
                JtJ(1, 1) += j1 * j1;
            }
            JtJ(1, 0) = JtJ(0, 1);

            double trace = JtJ.trace();
            double minDiag = JtJ.diagonal().minCoeff();

            if (trace < 1e-10) {
                outCovariance = CovMatrix2D::Identity() * 100.0;
                return false;
            }

            if (minDiag < kRegularizationThreshold * trace) {
                JtJ += (kRegularizationFactor * trace) * CovMatrix2D::Identity();
            }

            Eigen::LDLT<CovMatrix2D> ldlt(JtJ);
            if (ldlt.info() != Eigen::Success) {
                outCovariance = CovMatrix2D::Identity() * 100.0;
                return false;
            }

            outCovariance = ldlt.solve(CovMatrix2D::Identity()) * measurementVariance;
            outCovariance = (outCovariance + outCovariance.transpose()) / 2.0;
            return true;
        }

    } // namespace

    // ----- 3D solver -----

    SolverResult newtonRaphson(const PosMatrix& L,
                               const PosMatrix& R,
                               const DynVector& doas,
                               PosVector3D initialPos,
                               int maxIterations,
                               Scalar convergenceThreshold,
                               Scalar rmseThreshold)
    {
        SolverResult result;
        result.position = initialPos;
        result.converged = false;
        result.valid = true;
        result.iterations = 0;
        result.rmse = std::numeric_limits<Scalar>::infinity();
        result.covarianceValid = false;
        result.positionCovariance = CovMatrix3D::Identity();

        if (!anchorLayoutObservable3D(L, R)) {
            result.valid = false;
            return result;
        }

        SolverContext3D ctx;

        PosVector3D pos = initialPos;
        computeResidualsAndDistances3D(L, R, doas, pos, ctx.residuals, ctx.distancesL, ctx.distancesR);
        Scalar prevSse = ctx.residuals.squaredNorm();
        const Scalar rmse_convergence_sse =
            kResidualConvergenceThreshold * kResidualConvergenceThreshold
            * static_cast<Scalar>(doas.size());
        if (prevSse <= rmse_convergence_sse) {
            result.converged = true;
        }

        // LM lambda initialized from the scale of the problem.
        buildJacobian3D(L, R, pos, ctx.distancesL, ctx.distancesR, ctx.jacobian);
        Scalar lambda = kLMInitialFactor * trace3(ctx.jacobian) / Scalar(3);
        if (lambda < kLMMinLambda) lambda = kLMMinLambda;

        for (int iter = 0; !result.converged && iter < maxIterations; ++iter) {
            result.iterations++;

            buildJacobian3D(L, R, pos, ctx.distancesL, ctx.distancesR, ctx.jacobian);

            PosVector3D delta;
            solveLMStep<3>(ctx.jacobian, ctx.residuals, lambda, ctx.jaug, ctx.raug, delta);

            // Trial: pos_new = pos - delta (J*Δ = r ⇒ x_{k+1} = x_k - Δ).
            PosVector3D posTrial = pos - delta;

            DynVector trialResiduals;
            DynVector trialDL, trialDR;
            computeResidualsAndDistances3D(L, R, doas, posTrial, trialResiduals, trialDL, trialDR);
            Scalar trialSse = trialResiduals.squaredNorm();

            if (trialSse < prevSse) {
                // Accept step.
                pos = posTrial;
                ctx.residuals = trialResiduals;
                ctx.distancesL = trialDL;
                ctx.distancesR = trialDR;

                Scalar stepNorm = delta.norm();
                Scalar relImprove = (prevSse > Scalar(0))
                    ? (prevSse - trialSse) / prevSse : Scalar(0);
                prevSse = trialSse;

                lambda *= kLMShrinkFactor;
                if (lambda < kLMMinLambda) lambda = kLMMinLambda;

                if (stepNorm < convergenceThreshold || relImprove < convergenceThreshold) {
                    result.converged = true;
                    break;
                }
            } else {
                // Reject step, increase damping, retry from same pos.
                lambda *= kLMGrowFactor;
                if (lambda > kLMMaxLambda) {
                    // Damping saturated — geometry is bad, give up.
                    break;
                }
            }
        }

        result.position = pos;

        // Final residuals (prevSse already reflects last accepted pos).
        result.rmse = std::sqrt(prevSse / static_cast<Scalar>(doas.size()));
        if (result.rmse > rmseThreshold) {
            result.valid = false;
        }
        if (!result.converged) {
            result.valid = false;
        }

        if (result.valid && result.converged) {
            // Rebuild Jacobian one last time at the converged pos. Cheap (one
            // pass over residuals already-cached distances), and keeps the
            // covariance computation independent of the LM trial state.
            computeResidualsAndDistances3D(L, R, doas, pos, ctx.residuals, ctx.distancesL, ctx.distancesR);
            buildJacobian3D(L, R, pos, ctx.distancesL, ctx.distancesR, ctx.jacobian);

            const int n = static_cast<int>(doas.size());
            const int stateDim = 3;
            double sse = static_cast<double>(ctx.residuals.squaredNorm());
            double measurementVariance = (n > stateDim)
                ? sse / static_cast<double>(n - stateDim)
                : static_cast<double>(result.rmse) * static_cast<double>(result.rmse);
            measurementVariance = std::max(measurementVariance, kMinMeasurementVariance);

            result.covarianceValid = computePositionCovariance3D(
                ctx.jacobian, measurementVariance, result.positionCovariance);
            if (!result.covarianceValid) {
                result.valid = false;
            }
        }

        return result;
    }

    // ----- 2D solver -----

    SolverResult2D newtonRaphson2D(const PosMatrix& L,
                                   const PosMatrix& R,
                                   const DynVector& doas,
                                   PosVector2D initialPos,
                                   Scalar fixedZ,
                                   int maxIterations,
                                   Scalar convergenceThreshold,
                                   Scalar rmseThreshold)
    {
        SolverResult2D result;
        result.position = initialPos;
        result.converged = false;
        result.valid = true;
        result.iterations = 0;
        result.rmse = std::numeric_limits<Scalar>::infinity();
        result.covarianceValid = false;
        result.positionCovariance = CovMatrix2D::Identity();

        SolverContext2D ctx;

        PosVector2D posXY = initialPos;
        PosVector3D pos3D;
        pos3D << posXY(0), posXY(1), fixedZ;

        computeResidualsAndDistances3D(L, R, doas, pos3D, ctx.residuals, ctx.distancesL, ctx.distancesR);
        Scalar prevSse = ctx.residuals.squaredNorm();
        const Scalar rmse_convergence_sse =
            kResidualConvergenceThreshold * kResidualConvergenceThreshold
            * static_cast<Scalar>(doas.size());
        if (prevSse <= rmse_convergence_sse) {
            result.converged = true;
        }

        buildJacobian2D(L, R, pos3D, ctx.distancesL, ctx.distancesR, ctx.jacobian);
        Scalar lambda = kLMInitialFactor * trace2(ctx.jacobian) / Scalar(2);
        if (lambda < kLMMinLambda) lambda = kLMMinLambda;

        for (int iter = 0; !result.converged && iter < maxIterations; ++iter) {
            result.iterations++;

            pos3D << posXY(0), posXY(1), fixedZ;
            buildJacobian2D(L, R, pos3D, ctx.distancesL, ctx.distancesR, ctx.jacobian);

            PosVector2D delta;
            solveLMStep<2>(ctx.jacobian, ctx.residuals, lambda, ctx.jaug, ctx.raug, delta);

            PosVector2D posTrialXY = posXY - delta;
            PosVector3D trialPos3D;
            trialPos3D << posTrialXY(0), posTrialXY(1), fixedZ;

            DynVector trialResiduals, trialDL, trialDR;
            computeResidualsAndDistances3D(L, R, doas, trialPos3D, trialResiduals, trialDL, trialDR);
            Scalar trialSse = trialResiduals.squaredNorm();

            if (trialSse < prevSse) {
                posXY = posTrialXY;
                ctx.residuals = trialResiduals;
                ctx.distancesL = trialDL;
                ctx.distancesR = trialDR;

                Scalar stepNorm = delta.norm();
                Scalar relImprove = (prevSse > Scalar(0))
                    ? (prevSse - trialSse) / prevSse : Scalar(0);
                prevSse = trialSse;

                lambda *= kLMShrinkFactor;
                if (lambda < kLMMinLambda) lambda = kLMMinLambda;

                if (stepNorm < convergenceThreshold || relImprove < convergenceThreshold) {
                    result.converged = true;
                    break;
                }
            } else {
                lambda *= kLMGrowFactor;
                if (lambda > kLMMaxLambda) {
                    break;
                }
            }
        }

        result.position = posXY;
        result.rmse = std::sqrt(prevSse / static_cast<Scalar>(doas.size()));
        if (result.rmse > rmseThreshold) {
            result.valid = false;
        }
        if (!result.converged) {
            result.valid = false;
        }

        if (result.valid && result.converged) {
            pos3D << posXY(0), posXY(1), fixedZ;
            computeResidualsAndDistances3D(L, R, doas, pos3D, ctx.residuals, ctx.distancesL, ctx.distancesR);
            buildJacobian2D(L, R, pos3D, ctx.distancesL, ctx.distancesR, ctx.jacobian);

            const int n = static_cast<int>(doas.size());
            const int stateDim = 2;
            double sse = static_cast<double>(ctx.residuals.squaredNorm());
            double measurementVariance = (n > stateDim)
                ? sse / static_cast<double>(n - stateDim)
                : static_cast<double>(result.rmse) * static_cast<double>(result.rmse);
            measurementVariance = std::max(measurementVariance, kMinMeasurementVariance);

            result.covarianceValid = computePositionCovariance2D(
                ctx.jacobian, measurementVariance, result.positionCovariance);
        }

        return result;
    }

}
