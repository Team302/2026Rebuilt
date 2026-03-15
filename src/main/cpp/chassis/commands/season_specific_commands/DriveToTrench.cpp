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
#include "chassis/commands/season_specific_commands/DriveToTrench.h"
#include "fielddata/BumpHelper.h"

#include "auton/AllianceZoneManager.h"
#include "auton/NeutralZoneManager.h"
#include "fielddata/FieldOffsetValues.h"
#include "utils/FMSData.h"
#include "utils/PoseUtils.h"

//------------------------------------------------------------------
/// @brief      Constructor for DriveToTrench command
/// @param[in]  chassis - Pointer to the swerve drive subsystem
/// @details    Initializes the base DriveToPose command with the chassis.
///             This command autonomously drives the robot over a bump,
///             using a two-stage approach (midpoint and endpoint) to ensure
///             proper trajectory over the field obstacles. The command
///             determines the nearest bump and calculates appropriate poses
///             based on whether the robot is in the alliance or neutral zone.
//------------------------------------------------------------------
DriveToTrench::DriveToTrench(subsystems::CommandSwerveDrivetrain *chassis) : DriveToPose(chassis)
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
///             These angles ensure the robot approaches and crosses the bump
///             at the optimal heading toward the hub center.
//------------------------------------------------------------------
units::angle::degree_t DriveToTrench::GetRotation() const
{
    return (FMSData::GetAllianceColor() == frc::DriverStation::Alliance::kBlue) ? kTowardBlueAllianceWall : kTowardRedAllianceWall;
}

//------------------------------------------------------------------
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
struct DriveToPoses DriveToTrench::GetDriveToPoses()
{
    struct DriveToPoses poses;
    poses.hasMidPose = true;

    auto targetPoses = TrenchHelper::GetInstance()->GetTrenchDrivePositions(FMSData::GetAllianceColor() == frc::DriverStation::Alliance::kRed); // Get trench drive positions for the current alliance

    auto currentPose = GetChassis()->GetPose();
    auto firstPose = targetPoses.front();
    poses.midPose = frc::Pose2d(currentPose.X(), firstPose.Y(), firstPose.Rotation().Degrees()); // Create a pose for the bump position
    poses.endPose = targetPoses.back();                                                          // End pose is the last position (opposite side)

    return poses;
}
