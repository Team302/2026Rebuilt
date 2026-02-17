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
#include "chassis/commands/season_specific_commands/DriveOverBump.h"
#include "fielddata/BumpHelper.h"

#include "auton/AllianceZoneManager.h"
#include "auton/NeutralZoneManager.h"
#include "fielddata/FieldOffsetValues.h"
#include "utils/PoseUtils.h"

//------------------------------------------------------------------
/// @brief      Constructor for DriveOverBump command
/// @param[in]  chassis - Pointer to the swerve drive subsystem
/// @details    Initializes the base DriveToPose command with the chassis.
///             This command autonomously drives the robot over a bump,
///             using a two-stage approach (midpoint and endpoint) to ensure
///             proper trajectory over the field obstacles. The command
///             determines the nearest bump and calculates appropriate poses
///             based on whether the robot is in the alliance or neutral zone.
//------------------------------------------------------------------
DriveOverBump::DriveOverBump(subsystems::CommandSwerveDrivetrain *chassis) : DriveToPose(chassis), m_midPose(), m_endPose()
{
}

//------------------------------------------------------------------
/// @brief      Calculates the target poses for driving over the bump
/// @return     frc::Pose2d - The initial midpoint pose to drive to first
/// @details    This method determines the robot's path over the nearest bump
///             by calculating two key poses:
///             1. m_midPose: The position at the top of the bump (returned)
///             2. m_endPose: The final destination on the other side
///
///             The calculation process:
///             - Uses BumpHelper to identify the nearest bump (Red/Blue, Depot/Outpost)
///             - Checks if robot is in neutral zone or alliance zone
///             - Retrieves field-relative X/Y coordinates from FieldOffsetValues
///             - Determines appropriate rotation angle based on bump location and direction
///             - Sets distance threshold to 1 foot for completion detection
///
///             Direction logic:
///             - If in neutral zone: Drive toward alliance zone over bump
///             - If in alliance zone: Drive toward neutral zone over bump
///
/// @note       Sets m_beforeMidPose flag to true to enable two-stage navigation
/// @note       Returns default origin pose if BumpHelper is unavailable
//------------------------------------------------------------------
frc::Pose2d DriveOverBump::GetEndPose()
{
    m_beforeMidPose = true;
    frc::Pose2d endPose{};
    frc::Pose2d midPose{};
    units::angle::degree_t rotation{45_deg};

    // Get the BumpHelper singleton to determine which bump to drive over
    auto bumpHelper = BumpHelper::GetInstance();
    if (bumpHelper != nullptr)
    {
        // Identify the nearest bump based on robot's current position
        auto bump = bumpHelper->CalcNearestBump();

        // Determine if robot is currently in the neutral zone
        auto isInNeutralZone = NeutralZoneManager::GetInstance()->IsInNeutralZone();

        // Calculate the appropriate rotation angle for the bump and direction
        auto rotation = GetRotation(bump, isInNeutralZone);

        // Determine if this is a red alliance bump (depot or outpost)
        auto isRed = (bump == BUMP_ID::RED_DEPOT_BUMP || bump == BUMP_ID::RED_OUTPOST_BUMP);

        // Retrieve field coordinates for both sides of the bump
        auto offsetVals = FieldOffsetValues::GetInstance();
        auto neutralX = offsetVals->GetValue(isRed, FIELD_OFFSET_ITEMS::NEUTRAL_BUMP_X);
        auto neutralY = offsetVals->GetValue(isRed, FIELD_OFFSET_ITEMS::NEUTRAL_BUMP_Y);

        auto allianceX = offsetVals->GetValue(isRed, FIELD_OFFSET_ITEMS::ALLIANCE_BUMP_X);
        auto allianceY = offsetVals->GetValue(isRed, FIELD_OFFSET_ITEMS::ALLIANCE_BUMP_Y);

        if (isInNeutralZone) // Drive from neutral zone over bump to alliance zone
        {
            // First go to neutral side of bump, then to alliance side
            m_midPose = frc::Pose2d(neutralX, neutralY, frc::Rotation2d(rotation));
            m_endPose = frc::Pose2d(allianceX, allianceY, frc::Rotation2d(rotation));
        }
        else // Drive from alliance zone over bump to neutral zone
        {
            // First go to alliance side of bump, then to neutral side
            m_midPose = frc::Pose2d(allianceX, allianceY, frc::Rotation2d(rotation));
            m_endPose = frc::Pose2d(neutralX, neutralY, frc::Rotation2d(rotation));
        }

        // Set distance threshold for pose completion detection (1 foot tolerance)
        SetDistanceThreshold(1_ft);
    }

    return m_midPose;
}

//------------------------------------------------------------------
/// @brief      Determines the appropriate rotation angle for driving over a bump
/// @param[in]  bump - The identifier for which bump (Red/Blue, Depot/Outpost)
/// @param[in]  isInNeutralZone - True if robot is in neutral zone, false if in alliance zone
/// @return     units::angle::degree_t - The rotation angle in degrees for the robot heading
/// @details    This method returns the correct robot heading based on:
///             - Which bump is being crossed (4 possibilities)
///             - Direction of travel (neutral->alliance or alliance->neutral)
///
///             Rotation angles are defined as static constexpr members:
///             - Red Depot: 315° from alliance, 315° from neutral
///             - Red Outpost: 45° from alliance, 45° from neutral
///             - Blue Depot: 45° from alliance, 45° from neutral
///             - Blue Outpost: 315° from alliance, 315° from neutral
///
///             These angles ensure the robot approaches and crosses the bump
///             at the optimal heading toward the hub center.
//------------------------------------------------------------------
units::angle::degree_t DriveOverBump::GetRotation(BUMP_ID bump, bool isInNeutralZone) const
{
    switch (bump)
    {
    case BUMP_ID::RED_OUTPOST_BUMP:
        // Red outpost side: 45° heading regardless of direction
        return isInNeutralZone ? NeutralZoneTowardHubRedOutpost : RedAllianceOutpostWallTowardHub;
    case BUMP_ID::BLUE_OUTPOST_BUMP:
        // Blue outpost side: 315° heading regardless of direction
        return isInNeutralZone ? NeutralZoneTowardHubBlueOutpost : BlueAllianceOutpostWallTowardHub;
    case BUMP_ID::RED_DEPOT_BUMP:
        // Red depot side: 315° heading regardless of direction
        return isInNeutralZone ? NeutralZoneTowardHubRedDepot : RedAllianceDepotWallTowardHub;
    case BUMP_ID::BLUE_DEPOT_BUMP:
    default:
        // Blue depot side (default): 45° heading regardless of direction
        return isInNeutralZone ? NeutralZoneTowardHubBlueDepot : BlueAllianceDepotWallTowardHub;
    }
}

//------------------------------------------------------------------
/// @brief      Checks if the DriveOverBump command has finished execution
/// @return     true if the command should terminate, false if it should continue
/// @details    This method implements a two-stage completion check:
///
///             Stage 1: Error checking
///             - Returns true immediately if m_endPose is at origin (calculation error)
///
///             Stage 2: Two-phase navigation
///             - First phase: Drive to m_midPose (top of bump)
///               * When reached, updates target to m_endPose and continues
///             - Second phase: Drive to m_endPose (other side of bump)
///               * When reached, command completes
///
///             The m_beforeMidPose flag tracks which phase we're in:
///             - true: Still navigating to midpoint
///             - false: Navigating to final endpoint
///
///             This two-stage approach ensures the robot successfully crosses
///             the bump rather than trying to drive directly through it.
///
/// @note       Uses PoseUtils::IsPoseAtOrigin with 1cm tolerance for error detection
/// @note       Delegates to base class DriveToPose::IsFinished() for actual completion check
//------------------------------------------------------------------
bool DriveOverBump::IsFinished()
{
    // Safety check: If end pose wasn't calculated properly, stop immediately
    if (PoseUtils::IsPoseAtOrigin(m_endPose, units::length::centimeter_t{1.0}))
    {
        return true;
    }

    // Check if we've reached the current target pose (either mid or end)
    auto finished = DriveToPose::IsFinished();

    // Two-stage navigation logic
    if (m_beforeMidPose && finished)
    {
        // Just finished reaching the midpoint (top of bump)
        // Now update the target to the final endpoint
        SetEndPose(m_endPose);
        m_beforeMidPose = false;
        return false; // Continue to second stage
    }

    // Either still driving to first pose, or finished with second pose
    return finished;
}
