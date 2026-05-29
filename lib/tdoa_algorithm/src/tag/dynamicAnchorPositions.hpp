/**
 * @file dynamicAnchorPositions.hpp
 * @brief Dynamic anchor position calculation from inter-anchor ToF measurements
 *
 * This module calculates anchor positions dynamically from the inter-anchor
 * distance measurements broadcast in TDoA packets. It supports rectangular
 * 4-anchor single-plane layouts and 8-anchor two-plane layouts, and provides
 * averaging/filtering of distance measurements.
 *
 * The calculator uses the distances array from TDoA anchor packets, which
 * contains the measured time-of-flight distances between anchors.
 */

#pragma once

#include "stabilizer_types.h"
#include <cstdint>
#include <cmath>

// DW1000 timestamp unit to meters conversion factor
// 1 DW1000 timestamp unit = ~15.65 picoseconds
// Speed of light = 299,792,458 m/s
// 1 DW1000 unit * c / 2 (round-trip) = distance in meters
constexpr float DW1000_TIME_TO_METERS = 0.004691763978616f;

// Maximum number of anchors supported
constexpr uint8_t MAX_DYNAMIC_ANCHORS = 8;

// Staleness timeout for accumulated distances (in FreeRTOS ticks, configTICK_RATE_HZ = 1000)
// User preference: 5 seconds timeout, warn only (don't reset data)
constexpr uint32_t STALENESS_TIMEOUT_TICKS = 5000;

/**
 * @brief Configuration for dynamic anchor position calculation
 */
struct DynamicAnchorConfig {
    uint8_t layout;              // AnchorLayout enum value
    uint8_t anchorCount;         // Number of anchors in the layout (4 for 2D, 8 for 3D)
    float anchorHeight;          // Lower-plane height (NED: Z = -anchorHeight)
    float anchorPlaneSeparation; // Vertical distance from lower plane to upper plane
    uint16_t avgSampleCount;     // Samples to average before calculating (default: 50)
    uint8_t lockedMask;          // Bitmask of locked anchor positions
};

/**
 * @brief Accumulator for averaging distance measurements
 *
 * Collects multiple distance samples and computes their average
 * for more stable position calculations.
 */
struct DistanceAccumulator {
    float sum;
    uint16_t count;
    uint32_t lastUpdate;

    void reset() {
        sum = 0.0f;
        count = 0;
        lastUpdate = 0;
    }

    void add(float distance, uint32_t timestamp) {
        sum += distance;
        count++;
        lastUpdate = timestamp;
    }

    float average() const {
        return count > 0 ? sum / static_cast<float>(count) : 0.0f;
    }

    bool isReady(uint16_t targetCount) const {
        return count >= targetCount;
    }

    /**
     * @brief Check if accumulated data is stale (no updates for timeout period)
     * @param currentTime Current timestamp (FreeRTOS ticks)
     * @param timeoutTicks Staleness threshold in ticks
     * @return true if data hasn't been updated within timeout period
     */
    bool isStale(uint32_t currentTime, uint32_t timeoutTicks) const {
        if (count == 0) return false;  // No data = not stale (nothing to be stale)
        return (currentTime - lastUpdate) > timeoutTicks;
    }
};

/**
 * @brief Calculates anchor positions dynamically from inter-anchor distances
 *
 * This class receives inter-anchor distance measurements extracted from TDoA
 * packets and calculates the 2D/3D positions of anchors based on the configured
 * layout. The 8-anchor layout places A4 above A0, A5 above A1, A6 above A2,
 * and A7 above A3. It supports position locking for individual anchors and
 * provides averaging to smooth out measurement noise.
 *
 * Example usage:
 * @code
 * DynamicAnchorPositionCalculator calc;
 * DynamicAnchorConfig config = {
 *     .layout = 0,  // RECTANGULAR_A1X_A3Y (A0 at origin, +X=A1, +Y=A3)
 *     .anchorCount = 4,
 *     .anchorHeight = 2.0f,  // lower plane 2 meters high
 *     .anchorPlaneSeparation = 0.0f,
 *     .avgSampleCount = 50,
 *     .lockedMask = 0
 * };
 * calc.init(config);
 *
 * // Feed distance measurements from TDoA packets
 * calc.updateDistance(0, 1, 5.0f);  // 5 meters between A0 and A1
 * calc.updateDistance(0, 3, 3.0f);  // 3 meters between A0 and A3
 * // ... etc
 *
 * // When enough samples collected, calculate positions
 * if (calc.canCalculate()) {
 *     point_t positions[4];
 *     calc.calculatePositions(positions, 4);
 * }
 * @endcode
 */
class DynamicAnchorPositionCalculator {
public:
    /**
     * @brief Initialize the calculator with configuration
     * @param config Configuration parameters
     */
    void init(const DynamicAnchorConfig& config);

    /**
     * @brief Update with a new inter-anchor distance measurement
     * @param fromAnchor Source anchor ID (0-7)
     * @param toAnchor Destination anchor ID (0-7)
     * @param distanceMeters Distance in meters
     */
    void updateDistance(uint8_t fromAnchor, uint8_t toAnchor, float distanceMeters);

    /**
     * @brief Check if enough data is available to calculate positions
     * @return true if calculation can proceed
     */
    bool canCalculate() const;

    /**
     * @brief Calculate anchor positions from averaged distances
     * @param positions Output array for calculated positions (NED coordinates)
     * @param maxCount Maximum number of positions to calculate
     * @return true if calculation succeeded
     */
    bool calculatePositions(point_t* positions, uint8_t maxCount);

    /**
     * @brief Reset all accumulated data and start fresh
     */
    void reset();

    // ---- Locking control ----

    /**
     * @brief Set the locked anchor bitmask
     * @param mask Bitmask where bit N indicates anchor N is locked
     */
    void setLockedMask(uint8_t mask) { m_config.lockedMask = mask; }

    /**
     * @brief Get the current locked anchor bitmask
     */
    uint8_t getLockedMask() const { return m_config.lockedMask; }

    /**
     * @brief Lock an anchor's position at its current calculated value
     * @param anchorId Anchor ID to lock (0-7)
     */
    void lockAnchor(uint8_t anchorId);

    /**
     * @brief Unlock an anchor to allow position updates
     * @param anchorId Anchor ID to unlock (0-7)
     */
    void unlockAnchor(uint8_t anchorId);

    /**
     * @brief Check if an anchor is locked
     * @param anchorId Anchor ID to check (0-7)
     * @return true if anchor position is locked
     */
    bool isAnchorLocked(uint8_t anchorId) const;

    // ---- Status and diagnostics ----

    /**
     * @brief Get the averaged distance between two anchors
     * @param from Source anchor ID
     * @param to Destination anchor ID
     * @return Averaged distance in meters, or 0 if not available
     */
    float getAveragedDistance(uint8_t from, uint8_t to) const;

    /**
     * @brief Get the number of samples collected for a distance pair
     * @param from Source anchor ID
     * @param to Destination anchor ID
     * @return Number of samples collected
     */
    uint16_t getSampleCount(uint8_t from, uint8_t to) const;

    /**
     * @brief Check if a specific distance pair has enough samples
     * @param from Source anchor ID
     * @param to Destination anchor ID
     * @return true if ready for averaging
     */
    bool isDistanceReady(uint8_t from, uint8_t to) const;

private:
    DynamicAnchorConfig m_config;

    // Distance accumulation for averaging
    DistanceAccumulator m_accumulators[MAX_DYNAMIC_ANCHORS][MAX_DYNAMIC_ANCHORS];

    // Final averaged distance values (used for position calculation)
    float m_averagedDistances[MAX_DYNAMIC_ANCHORS][MAX_DYNAMIC_ANCHORS];

    // Locked positions (preserved when anchor is locked)
    point_t m_lockedPositions[MAX_DYNAMIC_ANCHORS];

    // Bitmask indicating which averaged distances are valid
    uint8_t m_validDistanceMask[MAX_DYNAMIC_ANCHORS];

    // Last calculated positions (for locking)
    point_t m_lastCalculatedPositions[MAX_DYNAMIC_ANCHORS];

    /**
     * @brief Calculate positions for rectangular layout
     * @param positions Output array
     * @param count Number of anchors
     * @return true if calculation succeeded
     */
    bool calculateRectangular(point_t* positions, uint8_t count);

    /**
     * @brief Validate that distances form a valid rectangular layout
     * @param dX Distance from A0 to the +X axis anchor
     * @param dY Distance from A0 to the +Y axis anchor
     * @param dDiag Distance from A0 to the diagonal corner anchor
     * @return true if distances are geometrically valid
     */
    bool validateRectangular(float dX, float dY, float dDiag);

    /**
     * @brief Finalize averages when accumulator is ready
     *
     * Transfers accumulated values to the averaged distances array
     * when enough samples have been collected.
     */
    void finalizeAverages();
};
