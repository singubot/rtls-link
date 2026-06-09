#pragma once

#include "tdoa_newton_raphson.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace tdoa_estimator {

enum class EstimatorStrategy : uint8_t {
    Legacy = 0,
    Robust = 1,
    Compare = 2,
};

struct RobustTdoaRow {
    uint8_t anchor_a = 0;
    uint8_t anchor_b = 0;
    PosVector3D anchor_a_pos = PosVector3D::Zero();
    PosVector3D anchor_b_pos = PosVector3D::Zero();
    Scalar tdoa = 0.0f;
    uint32_t age_us = 0;
    Scalar nominal_sigma_m = 0.15f;
    Scalar health = 1.0f;
};

struct RobustEstimatorOptions {
    uint8_t min_rows = 5;
    uint8_t min_unique_anchors = 4;
    uint8_t max_selected_rows = 20;
    uint8_t max_iterations = 10;
    Scalar convergence_threshold = 1e-3f;
    Scalar rmse_threshold = 0.8f;
    bool enable_pair_selection = true;
    bool enable_robust_pass = true;
    uint32_t age_half_life_us = 60000;
    Scalar reference_sigma_m = 0.15f;
    Scalar min_weight = 0.05f;
    Scalar huber_k = 1.5f;
    Scalar min_residual_scale_m = 0.05f;
};

struct RobustEstimatorResult {
    SolverResult solve;
    uint8_t input_rows = 0;
    uint8_t selected_rows = 0;
    uint8_t unique_anchors = 0;
    uint8_t selected_indices[kMaxCapacity] = {};
    Scalar base_weights[kMaxCapacity] = {};
    Scalar final_weights[kMaxCapacity] = {};
    Scalar residuals[kMaxCapacity] = {};
    Scalar residual_scale_m = 0.0f;
    bool robust_pass_used = false;
    bool pair_selection_used = false;
};

RobustEstimatorResult estimateRobust3D(const RobustTdoaRow* rows,
                                       size_t row_count,
                                       const PosVector3D& initial_position,
                                       const RobustEstimatorOptions& options = {});

} // namespace tdoa_estimator
