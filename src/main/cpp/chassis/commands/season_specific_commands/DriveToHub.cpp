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
#include "chassis/commands/season_specific_commands/DriveToHub.h"
#include "auton/NeutralZoneManager.h"
#include "fielddata/HubHelper.h"
#include "utils/PoseUtils.h"

//------------------------------------------------------------------
/// @brief      Constructor for DriveToHub command
/// @param[in]  chassis - Pointer to the swerve drive subsystem
/// @details    Initializes the base DriveToPose command with the chassis.
///             This command autonomously drives the robot to the nearest
///             Hub on the field.
//------------------------------------------------------------------
DriveToHub::DriveToHub(subsystems::CommandSwerveDrivetrain *chassis)
    : DriveToPose(chassis)
{
}

//------------------------------------------------------------------
/// @brief      Calculates the target pose for driving to the hub
/// @return     DriveToPoses struct containing the hub target pose
/// @details    Determines the nearest hub and returns its center position.
///
///             **Behavior:**
///             - If in neutral zone: Returns current pose (no movement)
///             - If in alliance zone: Calculates nearest hub pose using HubHelper
///
///             The command uses HubHelper to identify which hub (red or blue)
///             is closest based on the robot's current position and returns
///             the center coordinates of that hub.
///
/// @note       This is a single-stage navigation (no midpoint)
/// @see        HubHelper::CalcHubPose() for hub position calculation
//------------------------------------------------------------------
struct DriveToPoses DriveToHub::GetDriveToPoses()
{
    struct DriveToPoses poses;
    poses.hasMidPose = false;
    if (NeutralZoneManager::GetInstance()->IsInNeutralZone())
    {
        auto chassis = GetChassis();
        if (chassis != nullptr)
        {
            poses.endPose = chassis->GetPose();
            return poses;
        }
        return poses; // if in the neutral zone but chassis is unavailable, return default poses with hasMidPose=false
    }
    auto hubHelper = HubHelper::GetInstance();
    if (hubHelper != nullptr)
    {
        poses.endPose = hubHelper->CalcHubPose();
        return poses;
    }

    return poses;
}
