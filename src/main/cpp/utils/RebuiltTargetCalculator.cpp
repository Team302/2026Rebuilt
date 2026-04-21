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

#include "utils/RebuiltTargetCalculator.h"
#include "teleopcontrol/TeleopControl.h"
#include "utils/AngleUtils.h"
#include "utils/FMSData.h"
#include "utils/InterpolateUtils.h"
#include "utils/PoseUtils.h"
#include "utils/logging/debug/Logger.h"
// Static string constants — constructed once, never copied on each call
/// Identifies the outpost passing target visualization object on the field
const std::string RebuiltTargetCalculator::kOutpostPassingTargetName = "Outpost Passing Target Position";

/// Identifies the depot passing target visualization object on the field
const std::string RebuiltTargetCalculator::kDepotPassingTargetName = "Depot Passing Target Position";

/// Identifies the current target visualization object on the field
const std::string RebuiltTargetCalculator::kCurrentTargetName = "Current Target Position";

/// Identifies the launcher position/angle visualization object on the field
const std::string RebuiltTargetCalculator::kLauncherPositionName = "Launcher Position";

RebuiltTargetCalculator::RebuiltTargetCalculator() : TargetCalculator()
{
    SetMechanismOffset(m_mechanismOffset);

    m_field = DragonField::GetInstance();
    m_fieldConstants = FieldConstants::GetInstance();
    m_zoneManager = AllianceZoneManager::GetInstance();

    // Cache alliance-specific field elements and positions once
    m_cachedAlliance = FMSData::GetAllianceColor();
    RefreshAllianceCache();

    m_field->AddObject(kOutpostPassingTargetName, frc::Pose2d(m_outpostPassingPosition, frc::Rotation2d()), true);
    m_field->AddObject(kDepotPassingTargetName, frc::Pose2d(m_depotPassingPosition, frc::Rotation2d()), true);
    m_field->AddObject(kCurrentTargetName, frc::Pose2d());
    m_field->AddObject(kLauncherPositionName, frc::Pose2d());
}

RebuiltTargetCalculator *RebuiltTargetCalculator::m_instance = nullptr;

/**
 * Returns the singleton instance of RebuiltTargetCalculator, creating it if necessary.
 * Thread-safe singleton pattern implementation.
 *
 * \return Pointer to the single RebuiltTargetCalculator instance
 */
RebuiltTargetCalculator *RebuiltTargetCalculator::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new RebuiltTargetCalculator();
    }
    return m_instance;
}

/**
 * Validates whether the alliance has changed since the last check and refreshes cached
 * alliance-specific data if needed.
 *
 * Performance optimization using lazy validation:
 * - Only refreshes cache when alliance changes
 * - Tracks enabled state to avoid unnecessary validation when robot is disabled
 *
 * \return true if alliance changed and cache was refreshed, false if no change
 */
bool RebuiltTargetCalculator::ValidateAlliance()
{
    if (m_validatedWhileEnabled)
    {
        return false; // no change since last validation, skip
    }

    m_validatedWhileEnabled = frc::DriverStation::IsEnabled();

    auto currentAlliance = FMSData::GetAllianceColor();

    if (currentAlliance != m_cachedAlliance)
    {
        m_cachedAlliance = currentAlliance;
        RefreshAllianceCache();
        return true; // alliance changed, cache refreshed
    }
    return false; // no change, cache still valid
}

/**
 * Determines the current target position based on robot location and alliance.
 *
 * Target selection logic:
 * - If in alliance zone: targets the hub center
 * - If outside alliance zone: targets the closest passing target (outpost or depot)
 *
 * Applies manual offset adjustments (m_xTargetOffset, m_yTargetOffset) for driver tuning.
 * Automatically validates alliance and refreshes cache if needed.
 *
 * \return Target position in world frame (meters) as Translation2d
 */
frc::Translation2d RebuiltTargetCalculator::GetTargetPosition()
{
    ValidateAlliance();

    bool isInAllianceZone = m_zoneManager->IsInAllianceZone();
    frc::Translation2d targetPosition{};

    if (m_fieldConstants != nullptr)
    {
        if (isInAllianceZone)
        {
            // Use pre-cached hub center position
            targetPosition = m_hubCenterPosition;
        }
        else
        {
            // Determine closest passing target using cached enum values
            auto fieldElement = PoseUtils::GetClosestFieldElement(GetChassisPose(), m_outpostPassingTarget, m_depotPassingTarget);

            auto xPassingOffset = GetPassingTargetXOffset(fieldElement);
            auto yPassingOffset = GetPassingTargetYOffset(fieldElement);

            // Use pre-cached base positions instead of looking them up each cycle
            auto &basePosition = (fieldElement == m_outpostPassingTarget) ? m_outpostPassingPosition : m_depotPassingPosition;
            targetPosition = basePosition + frc::Translation2d(xPassingOffset, yPassingOffset);
        }
    }

    targetPosition = targetPosition + frc::Translation2d(m_xTargetOffset, m_yTargetOffset);

    return targetPosition;
}

/**
 * Calculates the optimal launcher angle to acquire and hit the current target.
 *
 * Algorithm:
 * 1. Validates alliance and refreshes cache if needed
 * 2. Predicts target position using lookahead time
 * 3. Calculates field angle from mechanism position to target
 * 4. Converts to robot-relative angle accounting for current robot orientation
 * 5. Searches for closest valid angle within mechanical constraints (91-267 degrees)
 * 6. Considers 360-degree rotations to minimize launcher movement
 * 7. If no valid angle found in range, clamps to nearest limit
 *
 * \param lookAheadTime Time in seconds to predict target position movement
 * \param currentLauncherAngle Current launcher angle in degrees (for minimum-error optimization)
 * \return Optimal launcher angle in rotations (0-1.0 scale, where 1.0 = 360°)
 */
units::angle::degree_t RebuiltTargetCalculator::GetLauncherTarget(units::angle::degree_t currentLauncherAngle)
{
    ValidateAlliance();
    UpdateChassisSpeeds();
    UpdateChassisPose();

    if (!m_isMoving)
    {
        return m_cachedLauncherTarget;
    }

    auto lookAheadTime = GetLookAheadTime();

    m_field->UpdateObject(kCurrentTargetName, GetVirtualTargetPose(lookAheadTime));

    units::degree_t fieldAngleToTarget = CalculateMechanismAngleToTarget(lookAheadTime);
    auto robotPose = GetChassisPose();

    frc::Rotation2d relativeRot = frc::Rotation2d(fieldAngleToTarget) - robotPose.Rotation();
    units::degree_t robotRelativeGoal = relativeRot.Degrees();

    units::degree_t bestAngle = 0_deg;
    m_hasFoundValidAngle = false;
    units::degree_t minError = 360_deg;

    for (int i = -1; i <= 1; i++)
    {
        units::degree_t potentialSetpoint = robotRelativeGoal + (360_deg * i);

        if (potentialSetpoint >= m_minLauncherAngle && potentialSetpoint <= m_maxLauncherAngle)
        {
            auto error = units::math::abs(potentialSetpoint - currentLauncherAngle);
            if (error < minError)
            {
                bestAngle = potentialSetpoint;
                minError = error;
                m_hasFoundValidAngle = true;
            }
        }
    }

    if (!m_hasFoundValidAngle)
    {
        units::degree_t normalizedGoal = relativeRot.Degrees();
        if (normalizedGoal < 0_deg)
            normalizedGoal += 360_deg;
        bestAngle = std::clamp(normalizedGoal, m_minLauncherAngle, m_maxLauncherAngle);
    }

    m_field->UpdateObject(kLauncherPositionName, frc::Pose2d(GetMechanismWorldPosition(), robotPose.Rotation() + frc::Rotation2d(bestAngle)));
    m_cachedLauncherTarget = bestAngle;

    return m_cachedLauncherTarget;
}
units::angle::degree_t RebuiltTargetCalculator::GetChassisTargetForLaunching()
{
    ValidateAlliance();
    UpdateChassisSpeeds();
    UpdateChassisPose();

    auto lookAheadTime = GetLookAheadTime();

    m_field->UpdateObject(kCurrentTargetName, GetVirtualTargetPose(lookAheadTime));

    units::degree_t fieldAngleToTarget = CalculateMechanismAngleToTarget(lookAheadTime);

    return AngleUtils::GetEquivAngle(fieldAngleToTarget);
}

/**
 * Processes controller input to adjust target positions in real-time for driver tuning.
 *
 * Discrete button adjustments (single press = 5-inch step):
 * - Update up button: increase main target X offset
 * - Update down button: decrease main target X offset
 * - Update left button: increase main target Y offset
 * - Update right button: decrease main target Y offset
 *
 * Analog axis adjustments (continuous):
 * - Depot passing target X/Y offsets
 * - Outpost passing target X/Y offsets
 *
 * Direction adjustments:
 * - Blue alliance: natural forward/starboard adjustments
 * - Red alliance: reversed adjustments for intuitive driver control
 *
 * Uses button state tracking to detect rising edges and prevent drift.
 * Updates field visualization after offset changes.
 */
void RebuiltTargetCalculator::UpdateTargetOffset()
{
    ValidateAlliance();

    auto teleopControl = TeleopControl::GetInstance();
    auto isBlue = (m_cachedAlliance == frc::DriverStation::Alliance::kBlue);

    if (teleopControl != nullptr)
    {
        bool isUpPressed = teleopControl->IsButtonPressed(TeleopControlFunctions::UPDATE_TARGET_OFFSET_UP);
        bool isDownPressed = teleopControl->IsButtonPressed(TeleopControlFunctions::UPDATE_TARGET_OFFSET_DOWN);
        bool isLeftPressed = teleopControl->IsButtonPressed(TeleopControlFunctions::UPDATE_TARGET_OFFSET_LEFT);
        bool isRightPressed = teleopControl->IsButtonPressed(TeleopControlFunctions::UPDATE_TARGET_OFFSET_RIGHT);

        auto sign5in = isBlue ? 5_in : -5_in;

        if (isUpPressed && !m_prevUpPressed)
        {
            m_xTargetOffset += sign5in;
        }
        if (isDownPressed && !m_prevDownPressed)
        {
            m_xTargetOffset -= sign5in;
        }
        if (isLeftPressed && !m_prevLeftPressed)
        {
            m_yTargetOffset += sign5in;
        }
        if (isRightPressed && !m_prevRightPressed)
        {
            m_yTargetOffset -= sign5in;
        }

        m_prevUpPressed = isUpPressed;
        m_prevDownPressed = isDownPressed;
        m_prevLeftPressed = isLeftPressed;
        m_prevRightPressed = isRightPressed;

        // Passing target offsets — use cached alliance sign
        auto xSign = isBlue ? 1_in : -1_in;
        auto ySign = isBlue ? -1_in : 1_in;
        m_passingDepotTargetXOffset += teleopControl->GetAxisValue(TeleopControlFunctions::UPDATE_DEPOT_PASSING_TARGET_X) * xSign;
        m_passingDepotTargetYOffset += teleopControl->GetAxisValue(TeleopControlFunctions::UPDATE_DEPOT_PASSING_TARGET_Y) * ySign;
        m_passingOutpostTargetXOffset += teleopControl->GetAxisValue(TeleopControlFunctions::UPDATE_OUTPOST_PASSING_TARGET_X) * xSign;
        m_passingOutpostTargetYOffset += teleopControl->GetAxisValue(TeleopControlFunctions::UPDATE_OUTPOST_PASSING_TARGET_Y) * ySign;
    }

    UpdatePassingTargetsOnField();
}

/**
 * Returns the current X-axis offset for a passing target element.
 *
 * \param fieldElement The field element to query (RED_OUTPOST_PASSING_TARGET, BLUE_DEPOT_PASSING_TARGET, etc.)
 * \return X offset in inches; positive values move target away from friendly alliance zone
 */
units::length::inch_t RebuiltTargetCalculator::GetPassingTargetXOffset(FieldConstants::FIELD_ELEMENT fieldElement)
{
    return (fieldElement == m_outpostPassingTarget) ? m_passingOutpostTargetXOffset : m_passingDepotTargetXOffset;
}

/**
 * Returns the current Y-axis offset for a passing target element.
 *
 * \param fieldElement The field element to query (RED_OUTPOST_PASSING_TARGET, BLUE_DEPOT_PASSING_TARGET, etc.)
 * \return Y offset in inches; sign is alliance-dependent for intuitive control
 */
units::length::inch_t RebuiltTargetCalculator::GetPassingTargetYOffset(FieldConstants::FIELD_ELEMENT fieldElement)
{
    return (fieldElement == m_outpostPassingTarget) ? m_passingOutpostTargetYOffset : m_passingDepotTargetYOffset;
}

/**
 * Updates field visualization objects with current passing target positions.
 *
 * Recalculates positions based on:
 * - Cached base positions (m_depotPassingPosition, m_outpostPassingPosition)
 * - Current offset values (m_passingDepotTargetXOffset, m_passingOutpostTargetXOffset, etc.)
 *
 * Called after any offset modification to keep visualizations synchronized with calculated positions.
 * Useful for driver tuning and debugging target acquisition.
 */
void RebuiltTargetCalculator::UpdatePassingTargetsOnField()
{
    frc::Translation2d passingDepotOffset = frc::Translation2d(m_passingDepotTargetXOffset, m_passingDepotTargetYOffset);
    frc::Translation2d passingOutpostOffset = frc::Translation2d(m_passingOutpostTargetXOffset, m_passingOutpostTargetYOffset);

    // Use pre-cached base positions instead of looking them up each cycle
    frc::Pose2d depotPose = frc::Pose2d(m_depotPassingPosition + passingDepotOffset, frc::Rotation2d());
    frc::Pose2d outpostPose = frc::Pose2d(m_outpostPassingPosition + passingOutpostOffset, frc::Rotation2d());

    m_field->UpdateObject(kDepotPassingTargetName, depotPose);
    m_field->UpdateObject(kOutpostPassingTargetName, outpostPose);
}

/**
 * Computes the lookahead time for virtual-target compensation based on distance to target.
 *
 * When the robot is stationary, returns 0 s so no virtual offset is applied.
 * When moving, uses CalculateMechanismDistanceToTarget (with 0 s lookahead for the
 * current real distance) and interpolates the flight time from the lookup table.
 *
 * \return Lookahead time in seconds
 */
units::time::second_t RebuiltTargetCalculator::GetLookAheadTime()
{
    if (!m_isMoving)
    {
        return 0_s;
    }

    units::length::inch_t distance = CalculateMechanismDistanceToTarget(0_s);
    auto lookAheadtime = InterpolateUtils::linearInterpolate(m_LookAheadDistances, m_LookAheadTimes, distance);

    return (lookAheadtime + m_lookAheadTimeOffset);
}

/**
 * Refreshes all cached alliance-specific field elements and their world positions.
 *
 * Called on:
 * - Initialization in constructor
 * - Alliance change (via ValidateAlliance())
 *
 * Updates these cache members:
 * - m_hubCenter: RED_HUB_CENTER or BLUE_HUB_CENTER
 * - m_outpostPassingTarget: RED_OUTPOST_PASSING_TARGET or BLUE_OUTPOST_PASSING_TARGET
 * - m_depotPassingTarget: RED_DEPOT_PASSING_TARGET or BLUE_DEPOT_PASSING_TARGET
 * - m_hubCenterPosition: Cached position of hub center
 * - m_outpostPassingPosition: Cached position of outpost target
 * - m_depotPassingPosition: Cached position of depot target
 *
 * Performance optimization: Performs FieldConstants lookups only once per alliance change
 * instead of on every GetTargetPosition() call.
 */
void RebuiltTargetCalculator::RefreshAllianceCache()
{
    bool isRed = (m_cachedAlliance == frc::DriverStation::Alliance::kRed);

    m_hubCenter = isRed ? FieldConstants::FIELD_ELEMENT::RED_HUB_CENTER
                        : FieldConstants::FIELD_ELEMENT::BLUE_HUB_CENTER;
    m_outpostPassingTarget = isRed ? FieldConstants::FIELD_ELEMENT::RED_OUTPOST_PASSING_TARGET
                                   : FieldConstants::FIELD_ELEMENT::BLUE_OUTPOST_PASSING_TARGET;
    m_depotPassingTarget = isRed ? FieldConstants::FIELD_ELEMENT::RED_DEPOT_PASSING_TARGET
                                 : FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_PASSING_TARGET;

    if (m_fieldConstants != nullptr)
    {
        m_hubCenterPosition = m_fieldConstants->GetFieldElementPose2d(m_hubCenter).Translation();
        m_outpostPassingPosition = m_fieldConstants->GetFieldElementPose2d(m_outpostPassingTarget).Translation();
        m_depotPassingPosition = m_fieldConstants->GetFieldElementPose2d(m_depotPassingTarget).Translation();
    }
}
