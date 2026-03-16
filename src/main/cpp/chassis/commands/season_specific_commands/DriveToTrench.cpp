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
#include "utils/FMSData.h"

//------------------------------------------------------------------
/// @brief      Constructor for DriveToTrench command
/// @param[in]  chassis - Pointer to the swerve drive subsystem
/// @details    Initializes the base DriveToPose command with the chassis and
///             configures the distance, angle, and Y-transition tolerances used
///             to detect when each stage of the trench crossing is complete.
//------------------------------------------------------------------
DriveToTrench::DriveToTrench(subsystems::CommandSwerveDrivetrain *chassis) : DriveToPose(chassis)
{
    // Set distance threshold for pose completion detection (1 foot tolerance)
    SetDistanceThreshold(kDistanceThreshold);
    SetAngleTolerance(kAngleTolerance);
    SetYTransitionToEndPointTolerance(kYTransitionToEndPointTolerance);
}

//------------------------------------------------------------------
/// @brief      Calculates the target poses for driving through a trench
/// @return     DriveToPoses struct containing a mid pose and an end pose
/// @details    Delegates to TrenchHelper to obtain the ordered drive positions
///             for the nearest trench on the current alliance side, then builds
///             the two-stage path as follows:
///
///             **Mid Pose**
///             - X: robot's current X-coordinate (no lateral shift yet)
///             - Y: Y-coordinate of the first (near-trench) position returned
///               by TrenchHelper::GetTrenchDrivePositions()
///             - Rotation: as specified by TrenchHelper for the near side
///
///             **End Pose**
///             - The last position in the TrenchHelper vector (far side of trench)
///             - If TrenchHelper returns only one pose, that pose is used for both
///               mid and end
///
///             The mid pose keeps the robot's current X so it first aligns on
///             the Y axis before committing to the full trench crossing.
///
/// @note       This override enables the two-stage navigation required to
///             safely align with and cross through the field trench
//------------------------------------------------------------------
struct DriveToPoses DriveToTrench::GetDriveToPoses()
{
    struct DriveToPoses poses;
    poses.hasMidPose = true;

    auto targetPoses = TrenchHelper::GetInstance()->GetTrenchDrivePositions(FMSData::GetAllianceColor() == frc::DriverStation::Alliance::kRed); // Get trench drive positions for the current alliance

    auto currentPose = GetChassis()->GetPose();

    auto firstPose = targetPoses.front();
    poses.midPose = frc::Pose2d(currentPose.X(), firstPose.Y(), firstPose.Rotation().Degrees());
    poses.endPose = targetPoses.size() > 1 ? targetPoses.back() : firstPose; // If there's only one pose, use it as the end pose as well

    return poses;
}
