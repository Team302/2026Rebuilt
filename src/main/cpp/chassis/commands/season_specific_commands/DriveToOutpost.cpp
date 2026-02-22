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
#include "chassis/commands/season_specific_commands/DriveToOutpost.h"
#include "auton/NeutralZoneManager.h"
#include "fielddata/OutpostHelper.h"
#include "utils/PoseUtils.h"

//------------------------------------------------------------------
/// @brief      Constructor for DriveToOutpost command
/// @param[in]  chassis - Pointer to the swerve drive subsystem
/// @details    Initializes the base DriveToPose command with the chassis.
///             This command autonomously drives the robot to the nearest
///             Outpost on the field.
//------------------------------------------------------------------
DriveToOutpost::DriveToOutpost(subsystems::CommandSwerveDrivetrain *chassis)
    : DriveToPose(chassis)
{
    SetDistanceThreshold(kDistanceThreshold);
    SetAngleTolerance(kAngleTolerance);
    SetYTransitionToEndPointTolerance(kYTransitionToEndPointTolerance);
}

//------------------------------------------------------------------
/// @brief      Calculates the target poses for driving to an outpost
/// @return     DriveToPoses struct containing midpoint and endpoint poses
/// @details    Determines the nearest outpost and returns a two-stage path:
///
///             **Behavior:**
///             - If in neutral zone: Returns current pose (no movement)
///             - If in alliance zone: Calculates two-stage path using OutpostHelper
///
///             **Mid Pose**
///             - Offset position near the outpost for approach alignment
///
///             **End Pose**
///             - Final position at the outpost center
///
///             The command uses OutpostHelper to identify which outpost (red or blue)
///             is closest and calculates both an offset approach pose and the
///             final target pose at the outpost center.
/// @see        OutpostHelper::CalcOutpostPose() for outpost position calculation
/// @see        OutpostHelper::CalcOutpostOffsetPose() for approach position
//------------------------------------------------------------------
struct DriveToPoses DriveToOutpost::GetDriveToPoses()
{
    struct DriveToPoses poses;
    poses.hasMidPose = false;

    if (NeutralZoneManager::GetInstance()->IsInNeutralZone())
    {
        auto chassis = GetChassis();
        if (chassis != nullptr)
        {
            poses.endPose = chassis->GetPose();
        }
        return poses;
    }
    auto outpostHelper = OutpostHelper::GetInstance();
    if (outpostHelper != nullptr)
    {
        poses.endPose = outpostHelper->CalcOutpostPose();
        poses.hasMidPose = true;
        poses.midPose = outpostHelper->CalcOutpostOffsetPose();

        return poses;
    }

    return poses;
}