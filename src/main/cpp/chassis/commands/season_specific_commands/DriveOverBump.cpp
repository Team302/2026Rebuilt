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
DriveOverBump::DriveOverBump(subsystems::CommandSwerveDrivetrain *chassis) : DriveToPose(chassis)
{
    // Set distance threshold for pose completion detection (1 foot tolerance)
    SetDistanceThreshold(kDistanceThreshold);
    SetAngleTolerance(kAngleTolerance);
    SetYTransitionToEndPointTolerance(kYTransitionToEndPointTolerance); // Allow extra tolerance for Y due to bump crossing dynamics
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
        return isInNeutralZone ? kNeutralZoneTowardHubRedOutpost : kRedAllianceOutpostWallTowardHub;
    case BUMP_ID::BLUE_OUTPOST_BUMP:
        // Blue outpost side: 315° heading regardless of direction
        return isInNeutralZone ? kNeutralZoneTowardHubBlueOutpost : kBlueAllianceOutpostWallTowardHub;
    case BUMP_ID::RED_DEPOT_BUMP:
        // Red depot side: 315° heading regardless of direction
        return isInNeutralZone ? kNeutralZoneTowardHubRedDepot : kRedAllianceDepotWallTowardHub;
    case BUMP_ID::BLUE_DEPOT_BUMP:
    default:
        // Blue depot side (default): 45° heading regardless of direction
        return isInNeutralZone ? kNeutralZoneTowardHubBlueDepot : kBlueAllianceDepotWallTowardHub;
    }
}

//------------------------------------------------------------------
/// @brief      Calculates the target poses for driving over a bump
/// @return     DriveToPoses struct containing midpoint and endpoint poses
/// @details    Determines which bump is nearest and calculates a two-stage path:
///
///             **Stage 1 (Mid Pose):**
///             - If in neutral zone: Target is neutral side of bump
///             - If in alliance zone: Target is alliance side of bump
///
///             **Stage 2 (End Pose):**
///             - If in neutral zone: Target is alliance side of bump
///             - If in alliance zone: Target is neutral side of bump
///
///             The method uses BumpHelper to identify the nearest bump,
///             NeutralZoneManager to determine current zone, and
///             FieldOffsetValues to retrieve the exact field coordinates.
///
///             Rotation angles are set to point toward the hub center
///             (45° or 315° depending on which bump is being crossed).
///
/// @note       This override enables the two-stage navigation required
///             to safely cross over field bumps
/// @see        GetRotation() for rotation angle calculation
//------------------------------------------------------------------
struct DriveToPoses DriveOverBump::GetDriveToPoses()
{
    struct DriveToPoses poses;
    poses.hasMidPose = true;

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
        auto allianceY = neutralY; // Y coordinate is the same for both sides of the same bump

        if (isInNeutralZone) // Drive from neutral zone over bump to alliance zone
        {
            // First go to neutral side of bump, then to alliance side
            poses.midPose = frc::Pose2d(neutralX, neutralY, frc::Rotation2d(rotation));
            poses.endPose = frc::Pose2d(allianceX, allianceY, frc::Rotation2d(rotation));
        }
        else // Drive from alliance zone over bump to neutral zone
        {
            // First go to alliance side of bump, then to neutral side
            poses.midPose = frc::Pose2d(allianceX, allianceY, frc::Rotation2d(rotation));
            poses.endPose = frc::Pose2d(neutralX, neutralY, frc::Rotation2d(rotation));
        }
    }
    return poses;
}