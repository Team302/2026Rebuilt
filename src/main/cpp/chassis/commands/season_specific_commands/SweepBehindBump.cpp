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
#include "chassis/commands/season_specific_commands/SweepBehindBump.h"
#include "fielddata/FieldOffsetValues.h"
#include "utils/PoseUtils.h"

//------------------------------------------------------------------
/// @brief      Constructor for SweepBehindBump command
/// @param[in]  chassis - Pointer to the swerve drive subsystem
/// @details    Initializes the base DriveToPose command with the chassis.
///             This command autonomously drives the robot over a bump,
///             using a two-stage approach (midpoint and endpoint) to ensure
///             proper trajectory over the field obstacles. The command
///             determines the nearest bump and calculates appropriate poses
///             based on whether the robot is in the alliance or neutral zone.
//------------------------------------------------------------------
// TODO: not sure where to put this but were doing eighht of these so one on each side of each bump, using location to decide which one to use instead of what allience were on--i think drive to depot/outpost should have smth to ref
SweepBehindBump::SweepBehindBump(subsystems::CommandSwerveDrivetrain *chassis) : DriveToPose(chassis)
{
    // Set distance threshold for pose completion detection (1 foot tolerance)
    SetDistanceThreshold(kDistanceThreshold);
    SetAngleTolerance(kAngleTolerance);
    SetYTransitionToEndPointTolerance(kYTransitionToEndPointTolerance); // Allow extra tolerance for Y due to bump crossing dynamics
}
// might want to change threshold making it smaller bc this is a path specificly to get as close to hub as possible--not sure but should ask at very least

//------------------------------------------------------------------
/// @brief      Determines the appropriate rotation angle for driving over a bump
/// @param[in]  bump - The identifier for which bump (Red/Blue, Depot/Outpost)
/// @return     units::angle::degree_t - The rotation angle in degrees for the robot heading
/// @details    This method returns the correct robot heading based on:
///             - Which bump is being crossed (4 possibilities)
///             - Direction of travel (neutral->alliance or alliance->neutral)
///
///             These angles ensure the robot approaches and crosses the bump
///             at the optimal heading toward the hub center.
//------------------------------------------------------------------
units::angle::degree_t SweepBehindBump::GetRotation(BUMP_ID bump) const
{
    if (bump == BUMP_ID::BLUE_DEPOT_BUMP || bump == BUMP_ID::RED_OUTPOST_BUMP)
    {
        // For Blue Depot and Red Outpost, use 270° heading
        return kBlueDepotRedOutpost;
    }
    else
    {
        // For Blue Outpost and Red Depot, use 90° heading
        return kRedDepotBlueOutpost;
    }
}
// TODO: add if statement saying, if blue depot or red out post set 270  and if else set 90 if

/// @brief      Calculates the target poses for driving over a bump
/// @return     DriveToPoses struct containing midpoint and endpoint poses
/// @details    Determines which bump is nearest and calculates a two-stage path:
///
///             **Mid Pose**
///             - If in neutral zone: Target is neutral side of bump
///             - If in alliance zone: Target is alliance side of bump
///
///             **End Pose**
///             - If in neutral zone: Target is alliance side of bump
///             - If in alliance zone: Target is neutral side of bump
///
///             The method uses BumpHelper to identify the nearest bump,
///             NeutralZoneManager to determine current zone, and
///             FieldOffsetValues to retrieve the exact field coordinates.
///
///             Rotation angles are set to point toward the hub center
///
/// @note       This override enables the two-stage navigation required
///             to safely cross over field bumps
/// @see        GetRotation() for rotation angle calculation
//------------------------------------------------------------------
struct DriveToPoses SweepBehindBump::GetDriveToPoses()
{
    struct DriveToPoses poses;
    poses.hasMidPose = true;

    auto offsetVals = FieldOffsetValues::GetInstance();
    auto nearestBumps = offsetVals->GetNearestAndCrossFieldBumpEdges(NeutralZoneManager::GetInstance()->IsInNeutralZone()); // Get all bump positions for both sides of the field

    if (!nearestBumps.empty())
    {
        auto pose = GetChassis()->GetPose(); // Get current robot pose
        auto bump = nearestBumps.front();    // Get the nearest bump (first in the list)
        auto rotation = GetRotation(bump.bumpId);
        poses.midPose = frc::Pose2d(bump.x, pose.Y(), rotation); // Create a pose for the bump position (rotation will be set later)

        bump = nearestBumps.back(); // Get the cross-field bump (last in the list)
        poses.endPose = frc::Pose2d(bump.x, bump.y, rotation);
    }
    return poses;
}