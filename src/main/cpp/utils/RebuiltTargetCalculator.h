//====================================================================================================================================================
// Copyright 2026 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================
#pragma once

#include "auton/AllianceZoneManager.h"
#include "fielddata/FieldConstants.h"
#include "utils/DragonField.h"
#include "utils/TargetCalculator.h"

#include <array>
#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <string>
#include <units/length.h>
#include <units/time.h>

/**
 * \class RebuiltTargetCalculator
 * \brief Season-specific target calculator for 2026 Rebuilt
 *
 * Provides real-time target acquisition and launcher angle calculations for the 2026 FRC season.
 * This singleton class extends TargetCalculator with 2026-specific configuration and performance
 * optimizations through intelligent caching and field element lookup.
 *
 * \section Features
 * - **Alliance-aware target selection**: Automatically selects appropriate hub or passing targets based on alliance color
 * - **Smart caching**: Caches alliance-specific field elements and positions to avoid repeated lookups
 * - **Launcher angle calculation**: Computes optimal launcher angles within mechanical constraints (91-267 degrees)
 * - **Driver offset adjustment**: Supports real-time target offset tuning via controller input for precision alignment
 * - **Field visualization**: Updates field visualization objects for debugging and monitoring
 *
 * \section Target Selection Strategy
 * - In alliance zone: Targets the hub center
 * - Outside alliance zone: Targets the closest passing target (either outpost or depot)
 * - Both targets support adjustable X/Y offsets for fine-tuning
 *
 * \section Caching Strategy for Performance
 * - Field element enums cached on alliance change (RED_HUB_CENTER, BLUE_HUB_CENTER, etc.)
 * - Field positions (Pose2d) cached on alliance change to avoid repeated FieldConstants lookups
 * - Static string constants avoid repeated std::string construction for field object names
 *
 * \section Mechanism Offset
 * The launcher mechanism is offset from the robot center in the robot frame:
 * - Default X offset: -3.333 inches (behind center)
 * - Default Y offset: 4.604 inches (starboard/right of center)
 *
 * \TODO Integrate with FieldElementCalculator and ZoneCalculator when available for dynamic field element updates
 */
class RebuiltTargetCalculator : public TargetCalculator
{
public:
    /**
     * \brief Get singleton instance
     * \return Pointer to the RebuiltTargetCalculator singleton
     */
    static RebuiltTargetCalculator *GetInstance();

    /**
     * \brief Get the current target position based on robot location and alliance
     *
     * Determines target position using the following logic:
     * - If in alliance zone: returns hub center position + applied offsets
     * - If outside alliance zone: returns closest passing target (outpost or depot) + applied offsets
     *
     * \return Translation2d with target position in meters (world frame)
     *
     * \note This method calls ValidateAlliance() to ensure cached data reflects current alliance
     * \note Applies manual target offset adjustments set via UpdateTargetOffset()
     */
    frc::Translation2d GetTargetPosition() override;

    /**
     * \brief Calculate optimal launcher angle to hit the current target
     *
     * Computes the launcher angle required to acquire the current target, accounting for:
     * - Current robot pose and orientation
     * - Lookahead time for target prediction
     * - Mechanism offset from robot center
     * - Mechanical constraints (min: 91°, max: 267°)
     * - Current launcher angle to minimize movement
     *
     * The algorithm searches for the closest valid angle to the ideal angle within mechanical limits,
     * considering 360-degree rotations to find the minimum travel path.
     *
     * \param lookAheadTime Time in seconds to predict target position movement
     * \param currentLauncherAngle Current launcher angle in degrees (used for minimum error optimization)
     * \return Angle in rotations (0-1.0 scale, where 1.0 = 360°)
     *
     * \note Updates the field visualization with calculated launcher position
     * \see GetMechanismWorldPosition(), CalculateMechanismAngleToTarget()
     */
    units::angle::degree_t GetLauncherTarget(units::angle::degree_t currentLauncherAngle);

    units::angle::degree_t GetChassisTargetForLaunching();

    /**
     * \brief Process controller input to adjust target positions
     *
     * Handles button and axis input to fine-tune target positions:
     * - **Discrete buttons** (5-inch steps):
     *   - Up/Down: Adjust main target X offset
     *   - Left/Right: Adjust main target Y offset
     * - **Analog axes**: Adjust passing target offsets
     *   - Depot X/Y offsets
     *   - Outpost X/Y offsets
     *
     * Adjustments are direction-adjusted based on alliance (red/blue) for intuitive control.
     * Uses button state tracking to ensure single adjustments per button press.
     *
     * \note Alliance sign is cached for consistency with RefreshAllianceCache()
     * \note Updates field visualization via UpdatePassingTargetsOnField()
     */
    void UpdateTargetOffset();
    bool IsValidTurretAngle() { return m_hasFoundValidAngle; }

private:
    /**
     * \brief Check if alliance has changed and refresh cache if necessary
     *
     * This method implements lazy validation of the alliance state. It only performs
     * a full cache refresh when:
     * - Alliance color has changed (detected via FMSData)
     * - This is the first validation while enabled
     *
     * \return true if cache was refreshed (alliance changed), false if cache still valid
     * \note Maintains m_validatedWhileEnabled to avoid redundant validation when disabled
     * \see RefreshAllianceCache()
     */
    bool ValidateAlliance();

    /**
     * \brief Constructor - private for singleton pattern, initializes with default mechanism offset
     *
     * Initialization steps:
     * 1. Sets mechanism offset to (-3.333 inches, 4.604 inches) in robot frame
     * 2. Obtains singleton references to DragonField, FieldConstants, and AllianceZoneManager
     * 3. Caches current alliance color and calls RefreshAllianceCache()
     * 4. Registers field visualization objects for debugging and telemetry
     */
    RebuiltTargetCalculator();

    /**
     * \brief Get the X-axis passing target offset for a given field element
     *
     * \param fieldElement The field element enum (RED_OUTPOST, BLUE_DEPOT, etc.)
     * \return X offset in inches; positive values move target away from friendly alliance
     *
     * \note Returns either outpost or depot X offset based on field element type
     */
    units::length::inch_t GetPassingTargetXOffset(FieldConstants::FIELD_ELEMENT fieldElement);

    /**
     * \brief Get the Y-axis passing target offset for a given field element
     *
     * \param fieldElement The field element enum (RED_OUTPOST, BLUE_DEPOT, etc.)
     * \return Y offset in inches; sign depends on alliance and element type
     *
     * \note Returns either outpost or depot Y offset based on field element type
     */
    units::length::inch_t GetPassingTargetYOffset(FieldConstants::FIELD_ELEMENT fieldElement);

    /**
     * \brief Update field visualization positions for passing targets
     *
     * Recalculates and updates field object positions based on current offset values.
     * Called after any offset modification to keep visualizations in sync.
     *
     * Updated field objects:
     * - "Depot Passing Target Position": m_depotPassingPosition + m_passingDepotTargetOffset
     * - "Outpost Passing Target Position": m_outpostPassingPosition + m_passingOutpostTargetOffset
     */
    void UpdatePassingTargetsOnField();

    /**
     * \brief Refresh all cached alliance-specific field elements and positions
     *
     * Called when alliance changes or on first initialization. Updates:
     * - m_hubCenter: Alliance-specific hub center field element
     * - m_outpostPassingTarget: Alliance-specific outpost passing target
     * - m_depotPassingTarget: Alliance-specific depot passing target
     * - m_hubCenterPosition: Cached world position of hub
     * - m_outpostPassingPosition: Cached world position of outpost target
     * - m_depotPassingPosition: Cached world position of depot target
     *
     * \note Looks up cached positions from FieldConstants only once per alliance change
     * \see ValidateAlliance()
     */
    void RefreshAllianceCache();

    /**
     * \brief Compute the lookahead time for virtual-target compensation based on distance to target.
     *
     * Returns 0 s when the robot is stationary (no virtual-target offset needed).
     * When moving, interpolates flight time from a distance → time lookup table so that
     * faster projectiles at longer ranges are correctly modeled without the caller
     * needing to know or pass the value.
     *
     * \return Lookahead time in seconds (0 s when stationary)
     */
    units::time::second_t GetLookAheadTime();

    // Distance (inches) → lookahead time (seconds) lookup table.
    static constexpr std::array<units::length::inch_t, 13> m_LookAheadDistances{
        46.1_in, 68.5_in, 87.6_in, 97.8_in, 110.3_in, 127.4_in, 149.3_in,
        167.8_in, 183.3_in, 197.0_in, 238.7_in, 277.5_in, 314.9_in};
    static constexpr std::array<units::time::second_t, 13> m_LookAheadTimes{
        1.07_s, 1.01_s, 0.88_s, 0.92_s, 0.966_s, 0.90_s, 0.96_s,
        0.97_s, 1.10_s, 1.20_s, 1.30_s, 1.40_s, 1.60_s};

    static RebuiltTargetCalculator *m_instance;

    /// \name String Constants
    /// Static string constants to avoid repeated std::string construction
    /// These are used as keys for field visualization objects
    /// @{

    /// Field object name for outpost passing target visualization
    static const std::string kOutpostPassingTargetName;

    /// Field object name for depot passing target visualization
    static const std::string kDepotPassingTargetName;

    /// Field object name for current target visualization
    static const std::string kCurrentTargetName;

    /// Field object name for launcher position/angle visualization
    static const std::string kLauncherPositionName;

    /// @}

    /// \name Mechanism Configuration
    /// @{

    /// Mechanism position offset from robot center in robot frame (meters)
    /// Default: -3.333 inches (behind center), 4.604 inches (starboard/right of center)
    static constexpr frc::Translation2d m_mechanismOffset{-4.6_in, 4.3_in};

    /// @}

    /// \name Singleton References
    /// @{

    /// Reference to field visualization object for debugging and telemetry
    DragonField *m_field;

    /// Reference to field constants for field element position lookups
    FieldConstants *m_fieldConstants;

    /// Reference to zone manager for alliance zone detection
    AllianceZoneManager *m_zoneManager;

    /// @}

    /// \name Launcher Mechanical Constraints
    /// @{

    /// Minimum launcher angle in degrees (soft limit, angles below this are clamped)
    static constexpr units::degree_t m_minLauncherAngle = 90_deg;

    /// Maximum launcher angle in degrees (soft limit, angles above this are clamped)
    static constexpr units::degree_t m_maxLauncherAngle = 265_deg;

    /// @}

    /// \name Cached Alliance-Specific Field Elements
    /// These enums are cached to avoid repeated alliance checks during each cycle
    /// They are refreshed in RefreshAllianceCache() when alliance changes
    /// @{

    /// Cached hub center field element (RED_HUB_CENTER or BLUE_HUB_CENTER)
    FieldConstants::FIELD_ELEMENT m_hubCenter;

    /// Cached outpost passing target (RED_OUTPOST_PASSING_TARGET or BLUE_OUTPOST_PASSING_TARGET)
    FieldConstants::FIELD_ELEMENT m_outpostPassingTarget;

    /// Cached depot passing target (RED_DEPOT_PASSING_TARGET or BLUE_DEPOT_PASSING_TARGET)
    FieldConstants::FIELD_ELEMENT m_depotPassingTarget;

    /// @}

    /// \name Cached Field Positions
    /// These world-frame positions are cached to avoid repeated FieldConstants lookups
    /// They are updated in RefreshAllianceCache() when alliance changes
    /// @{

    /// Cached world position of hub center target (meters)
    frc::Translation2d m_hubCenterPosition;

    /// Cached world position of outpost passing target (meters)
    frc::Translation2d m_outpostPassingPosition;

    /// Cached world position of depot passing target (meters)
    frc::Translation2d m_depotPassingPosition;

    /// @}

    /// \name Alliance Tracking
    /// @{

    /// Cached alliance color for detecting changes; updated in RefreshAllianceCache()
    frc::DriverStation::Alliance m_cachedAlliance;

    /// Tracks whether alliance validation has been performed while robot is enabled
    /// Used to optimize ValidateAlliance() to skip unnecessary validation
    bool m_validatedWhileEnabled = false;

    /// @}

    /// \name Main Target Offset
    /// Manual adjustments to the primary target position (hub or passing target)
    /// @{

    /// X-axis offset for main target in inches
    units::length::inch_t m_xTargetOffset = 0_in;

    /// Y-axis offset for main target in inches
    units::length::inch_t m_yTargetOffset = 0_in;

    /// @}

    /// \name Passing Target Offsets
    /// Separate offset tuning for depot and outpost passing targets
    /// @{

    /// X-axis offset for depot passing target in inches
    units::length::inch_t m_passingDepotTargetXOffset = 0_in;

    /// Y-axis offset for depot passing target in inches
    units::length::inch_t m_passingDepotTargetYOffset = 0_in;

    /// X-axis offset for outpost passing target in inches
    units::length::inch_t m_passingOutpostTargetXOffset = 0_in;

    /// Y-axis offset for outpost passing target in inches
    units::length::inch_t m_passingOutpostTargetYOffset = 0_in;

    /// @}

    /// \name Button State Tracking
    /// Tracks previous button states to implement single-press behavior for discrete adjustments
    /// @{

    /// Previous state of up button (for detecting rising edge)
    bool m_prevUpPressed = false;

    /// Previous state of down button (for detecting rising edge)
    bool m_prevDownPressed = false;

    /// Previous state of left button (for detecting rising edge)
    bool m_prevLeftPressed = false;

    /// Previous state of right button (for detecting rising edge)
    bool m_prevRightPressed = false;

    bool m_hasFoundValidAngle = false;

    /// @}

    /// \name Cached Calculation Results
    /// Cache for expensive calculations to avoid recalculation when pose hasn't changed
    /// @{

    /// Cached launcher target angle from last calculation (in turns)
    units::angle::turn_t m_cachedLauncherTarget = 0_tr;

    /// @}
};
