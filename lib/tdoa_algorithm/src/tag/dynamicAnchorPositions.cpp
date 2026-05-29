/**
 * @file dynamicAnchorPositions.cpp
 * @brief Implementation of dynamic anchor position calculation
 */

#include "dynamicAnchorPositions.hpp"
#include <algorithm>
#include <cstring>

namespace {

struct LayoutRoles {
    uint8_t xAnchor;
    uint8_t yAnchor;
    uint8_t cornerAnchor;
};

bool getLayoutRoles(uint8_t layout, LayoutRoles& roles)
{
    switch (layout) {
        case 0: roles = {1, 3, 2}; return true;
        case 1: roles = {1, 2, 3}; return true;
        case 2: roles = {3, 1, 2}; return true;
        case 3: roles = {2, 3, 1}; return true;
        default: return false;
    }
}

bool distanceMatches(float measured, float expected, float relativeTolerance, float absoluteTolerance)
{
    if (measured <= 0.0f || expected <= 0.0f || !std::isfinite(measured) || !std::isfinite(expected)) {
        return false;
    }
    const float tolerance = std::max(absoluteTolerance, expected * relativeTolerance);
    return std::abs(measured - expected) <= tolerance;
}

} // namespace

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
static const char* TAG = "DynAnchorPos";
#endif

void DynamicAnchorPositionCalculator::init(const DynamicAnchorConfig& config) {
    m_config = config;

    // Validate and clamp configuration parameters
    if (m_config.avgSampleCount < 1) {
        m_config.avgSampleCount = 1;
    }
    if (m_config.avgSampleCount > 1000) {
        m_config.avgSampleCount = 1000;
    }
    if (m_config.anchorHeight < 0) {
        m_config.anchorHeight = 0;
    }
    if (m_config.anchorPlaneSeparation < 0) {
        m_config.anchorPlaneSeparation = 0;
    }
    if (m_config.anchorCount > MAX_DYNAMIC_ANCHORS) {
        m_config.anchorCount = MAX_DYNAMIC_ANCHORS;
    }

    reset();
}

void DynamicAnchorPositionCalculator::reset() {
    // Reset all accumulators
    for (uint8_t i = 0; i < MAX_DYNAMIC_ANCHORS; i++) {
        for (uint8_t j = 0; j < MAX_DYNAMIC_ANCHORS; j++) {
            m_accumulators[i][j].reset();
            m_averagedDistances[i][j] = 0.0f;
            m_averagedDistanceLastUpdate[i][j] = 0;
        }
        m_validDistanceMask[i] = 0;
        m_lockedPositions[i] = {0, 0, 0, 0};
        m_lastCalculatedPositions[i] = {0, 0, 0, 0};
    }
}

void DynamicAnchorPositionCalculator::updateDistance(uint8_t fromAnchor, uint8_t toAnchor, float distanceMeters) {
#ifdef ESP_PLATFORM
    uint32_t now = xTaskGetTickCount();
#else
    uint32_t now = 0;  // Fallback for native testing
#endif
    updateDistanceAt(fromAnchor, toAnchor, distanceMeters, now);
}

void DynamicAnchorPositionCalculator::updateDistanceAt(uint8_t fromAnchor,
                                                       uint8_t toAnchor,
                                                       float distanceMeters,
                                                       uint32_t timestamp) {
    // Validate inputs
    if (fromAnchor >= MAX_DYNAMIC_ANCHORS || toAnchor >= MAX_DYNAMIC_ANCHORS || fromAnchor == toAnchor) {
        return;
    }

    // Filter out invalid distances
    if (distanceMeters <= 0.0f || !std::isfinite(distanceMeters)) {
        return;
    }

    // Add to accumulators (symmetric - distance A->B == B->A)
    m_accumulators[fromAnchor][toAnchor].add(distanceMeters, timestamp);
    m_accumulators[toAnchor][fromAnchor].add(distanceMeters, timestamp);

#ifdef ESP_PLATFORM
    // Log progress every 10 samples to help diagnose accumulation issues
    uint16_t count = m_accumulators[fromAnchor][toAnchor].count;
    if (count % 10 == 0 && count > 0) {
        ESP_LOGD(TAG, "Distance %d-%d: %d/%d samples (avg: %.3fm)",
                 fromAnchor, toAnchor, count, m_config.avgSampleCount,
                 m_accumulators[fromAnchor][toAnchor].average());
    }
#endif

    // Check if we have enough samples to finalize this pair
    if (m_accumulators[fromAnchor][toAnchor].isReady(m_config.avgSampleCount)) {
        // Transfer accumulated average to final value
        float avgDistance = m_accumulators[fromAnchor][toAnchor].average();
        m_averagedDistances[fromAnchor][toAnchor] = avgDistance;
        m_averagedDistances[toAnchor][fromAnchor] = avgDistance;
        m_averagedDistanceLastUpdate[fromAnchor][toAnchor] = timestamp;
        m_averagedDistanceLastUpdate[toAnchor][fromAnchor] = timestamp;

        // Mark as valid
        m_validDistanceMask[fromAnchor] |= (1 << toAnchor);
        m_validDistanceMask[toAnchor] |= (1 << fromAnchor);

        // Reset accumulators for next averaging cycle
        m_accumulators[fromAnchor][toAnchor].reset();
        m_accumulators[toAnchor][fromAnchor].reset();
    }
}

bool DynamicAnchorPositionCalculator::canCalculate() const {
#ifdef ESP_PLATFORM
    return canCalculateAt(xTaskGetTickCount());
#else
    return canCalculateAt(0);
#endif
}

bool DynamicAnchorPositionCalculator::canCalculateAt(uint32_t currentTime) const {
    if (m_config.anchorCount != 4 && m_config.anchorCount != 8) {
        return false;
    }

    // Determine required distance pairs based on layout.
    // A0 is always at origin. Layout determines which anchors are on +X and +Y axes.
    // Layout 0: +X=A1, +Y=A3 -> need d01, d03
    // Layout 1: +X=A1, +Y=A2 -> need d01, d02
    // Layout 2: +X=A3, +Y=A1 -> need d03, d01
    // Layout 3: +X=A2, +Y=A3 -> need d02, d03
    LayoutRoles roles{};
    if (!getLayoutRoles(m_config.layout, roles)) {
        return false;
    }

    if (!hasFreshDistance(0, roles.xAnchor, currentTime)
        || !hasFreshDistance(0, roles.yAnchor, currentTime)
        || !hasFreshDistance(0, roles.cornerAnchor, currentTime)) {
        return false;
    }

    if (m_config.anchorCount == 8) {
        if (m_config.anchorPlaneSeparation <= 0.0f) {
            return false;
        }
        for (uint8_t lower = 0; lower < 4; lower++) {
            const uint8_t upper = lower + 4;
            if (!hasFreshDistance(lower, upper, currentTime)) {
                return false;
            }
        }

        const uint8_t upperBase = 4;
        const uint8_t upperX = roles.xAnchor + 4;
        const uint8_t upperY = roles.yAnchor + 4;
        const uint8_t upperCorner = roles.cornerAnchor + 4;
        if (!hasFreshDistance(upperBase, upperX, currentTime)
            || !hasFreshDistance(upperBase, upperY, currentTime)
            || !hasFreshDistance(upperBase, upperCorner, currentTime)
            || !hasFreshDistance(0, upperX, currentTime)
            || !hasFreshDistance(0, upperY, currentTime)
            || !hasFreshDistance(0, upperCorner, currentTime)) {
            return false;
        }
    }

    return true;
}

bool DynamicAnchorPositionCalculator::calculatePositions(point_t* positions, uint8_t maxCount) {
    if (positions == nullptr || maxCount == 0) {
        return false;
    }

    uint8_t count = (maxCount < m_config.anchorCount) ? maxCount : m_config.anchorCount;

    // Calculate new positions based on layout
    point_t newPositions[MAX_DYNAMIC_ANCHORS];
    bool success = false;

    switch (m_config.layout) {
        case 0:  // RECTANGULAR_A1X_A3Y
        case 1:  // RECTANGULAR_A1X_A2Y
        case 2:  // RECTANGULAR_A3X_A1Y
        case 3:  // RECTANGULAR_A2X_A3Y
            success = calculateRectangular(newPositions, count);
            break;
        default:
            // CUSTOM or unknown - not supported yet
            return false;
    }

    if (!success) {
        return false;
    }

    // Apply locking: use locked positions for locked anchors, calculated for others
    for (uint8_t i = 0; i < count; i++) {
        if (isAnchorLocked(i)) {
            positions[i] = m_lockedPositions[i];
        } else {
            positions[i] = newPositions[i];
            m_lastCalculatedPositions[i] = newPositions[i];
        }
    }

    return true;
}

bool DynamicAnchorPositionCalculator::calculateRectangular(point_t* positions, uint8_t count) {
    if (count < 4) {
        return false;
    }

    // A0 is always at origin. Layout determines which anchors are on +X and +Y axes.
    // Layout 0: +X=A1, +Y=A3 -> dX=d01, dY=d03, corner(dX,dY)=A2
    // Layout 1: +X=A1, +Y=A2 -> dX=d01, dY=d02, corner(dX,dY)=A3
    // Layout 2: +X=A3, +Y=A1 -> dX=d03, dY=d01, corner(dX,dY)=A2
    // Layout 3: +X=A2, +Y=A3 -> dX=d02, dY=d03, corner(dX,dY)=A1
    LayoutRoles roles{};
    if (!getLayoutRoles(m_config.layout, roles)) {
        return false;
    }

    float dX = m_averagedDistances[0][roles.xAnchor];
    float dY = m_averagedDistances[0][roles.yAnchor];
    float dDiag = m_averagedDistances[0][roles.cornerAnchor];

    if (dX <= 0.0f || dY <= 0.0f || dDiag <= 0.0f) {
        return false;
    }
    if (!validateRectangular(dX, dY, dDiag)) {
        return false;
    }

    if (count >= 8) {
        const float expectedSeparation = m_config.anchorPlaneSeparation;
        if (expectedSeparation <= 0.0f) {
            return false;
        }

        const float tolerance = std::max(0.5f, expectedSeparation * 0.35f);
        for (uint8_t lower = 0; lower < 4; lower++) {
            const uint8_t upper = lower + 4;
            const float measuredSeparation = m_averagedDistances[lower][upper];
            if (measuredSeparation <= 0.0f
                || std::abs(measuredSeparation - expectedSeparation) > tolerance) {
                return false;
            }
        }

        const uint8_t upperBase = 4;
        const uint8_t upperX = roles.xAnchor + 4;
        const uint8_t upperY = roles.yAnchor + 4;
        const uint8_t upperCorner = roles.cornerAnchor + 4;
        const float upperDX = m_averagedDistances[upperBase][upperX];
        const float upperDY = m_averagedDistances[upperBase][upperY];
        const float upperDiag = m_averagedDistances[upperBase][upperCorner];
        if (!distanceMatches(upperDX, dX, 0.1f, 0.25f)
            || !distanceMatches(upperDY, dY, 0.1f, 0.25f)
            || !validateRectangular(upperDX, upperDY, upperDiag)
            || !distanceMatches(upperDiag, dDiag, 0.1f, 0.25f)) {
            return false;
        }

        const float expectedCrossX = std::sqrt(dX * dX + expectedSeparation * expectedSeparation);
        const float expectedCrossY = std::sqrt(dY * dY + expectedSeparation * expectedSeparation);
        const float expectedCrossDiag = std::sqrt(dDiag * dDiag + expectedSeparation * expectedSeparation);
        if (!distanceMatches(m_averagedDistances[0][upperX], expectedCrossX, 0.15f, 0.35f)
            || !distanceMatches(m_averagedDistances[0][upperY], expectedCrossY, 0.15f, 0.35f)
            || !distanceMatches(m_averagedDistances[0][upperCorner], expectedCrossDiag, 0.15f, 0.35f)) {
            return false;
        }
    }

    // Calculate Z coordinate (NED convention: Z = -height)
    float z = -m_config.anchorHeight;

    // A0 always at origin
    positions[0] = {0, 0.0f, 0.0f, z};

    // Place each anchor based on its role
    // +X anchor at (dX, 0, Z)
    // +Y anchor at (0, dY, Z)
    // Corner anchor at (dX, dY, Z)
    for (uint8_t i = 1; i < 4; i++) {
        if (i == roles.xAnchor) {
            positions[i] = {0, dX, 0.0f, z};
        } else if (i == roles.yAnchor) {
            positions[i] = {0, 0.0f, dY, z};
        } else if (i == roles.cornerAnchor) {
            positions[i] = {0, dX, dY, z};
        }
    }

    if (count >= 8) {
        const float upperZ = -(m_config.anchorHeight + m_config.anchorPlaneSeparation);
        for (uint8_t lower = 0; lower < 4; lower++) {
            const uint8_t upper = lower + 4;
            positions[upper] = positions[lower];
            positions[upper].z = upperZ;
        }
    }

    return true;
}

bool DynamicAnchorPositionCalculator::validateRectangular(float dX, float dY, float dDiag) {
    // For a rectangle, the diagonal from A0 to the corner at (dX, dY) should
    // equal sqrt(dX^2 + dY^2) by the Pythagorean theorem.
    float expectedDiagonal = std::sqrt(dX * dX + dY * dY);

    // Allow 10% tolerance for measurement noise
    float tolerance = 0.1f;

    float diagError = std::abs(dDiag - expectedDiagonal);
    if (expectedDiagonal > 0.0f && (diagError / expectedDiagonal) > tolerance) {
        return false;
    }

    return true;
}

void DynamicAnchorPositionCalculator::lockAnchor(uint8_t anchorId) {
    if (anchorId >= MAX_DYNAMIC_ANCHORS) {
        return;
    }

    // Store current calculated position before locking
    m_lockedPositions[anchorId] = m_lastCalculatedPositions[anchorId];
    m_config.lockedMask |= (1 << anchorId);
}

void DynamicAnchorPositionCalculator::unlockAnchor(uint8_t anchorId) {
    if (anchorId >= MAX_DYNAMIC_ANCHORS) {
        return;
    }

    m_config.lockedMask &= ~(1 << anchorId);
}

bool DynamicAnchorPositionCalculator::isAnchorLocked(uint8_t anchorId) const {
    if (anchorId >= MAX_DYNAMIC_ANCHORS) {
        return false;
    }

    return (m_config.lockedMask & (1 << anchorId)) != 0;
}

float DynamicAnchorPositionCalculator::getAveragedDistance(uint8_t from, uint8_t to) const {
    if (from >= MAX_DYNAMIC_ANCHORS || to >= MAX_DYNAMIC_ANCHORS) {
        return 0.0f;
    }

    return m_averagedDistances[from][to];
}

uint16_t DynamicAnchorPositionCalculator::getSampleCount(uint8_t from, uint8_t to) const {
    if (from >= MAX_DYNAMIC_ANCHORS || to >= MAX_DYNAMIC_ANCHORS) {
        return 0;
    }

    return m_accumulators[from][to].count;
}

bool DynamicAnchorPositionCalculator::isDistanceReady(uint8_t from, uint8_t to) const {
    if (from >= MAX_DYNAMIC_ANCHORS || to >= MAX_DYNAMIC_ANCHORS) {
        return false;
    }

    return (m_validDistanceMask[from] & (1 << to)) != 0;
}

bool DynamicAnchorPositionCalculator::hasFreshDistance(uint8_t from, uint8_t to, uint32_t currentTime) const {
    if (!isDistanceReady(from, to)) {
        return false;
    }
    if (currentTime == 0) {
        return true;
    }
    return (currentTime - m_averagedDistanceLastUpdate[from][to]) <= STALENESS_TIMEOUT_TICKS;
}

void DynamicAnchorPositionCalculator::finalizeAverages() {
    // This is called internally when accumulators are ready
    // Already handled in updateDistance() for immediate finalization
}
