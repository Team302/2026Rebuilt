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
/// @brief      Calculates the target end pose for the Hub
/// @return     frc::Pose2d - The target pose at the center of the nearest Hub
/// @details    Uses HubHelper to determine which Hub (red or blue) is
///             closest to the robot and calculates the center pose of that
///             Hub. Returns a default pose if HubHelper is unavailable.
//------------------------------------------------------------------
frc::Pose2d DriveToHub::GetEndPose()
{
    frc::Pose2d endPose;
    if (NeutralZoneManager::GetInstance()->IsInNeutralZone())
    {
        auto chassis = GetChassis();
        if (chassis != nullptr)
        {
            return chassis->GetPose();
        }
        return endPose;
    }
    auto hubHelper = HubHelper::GetInstance();
    if (hubHelper != nullptr)
    {
        return hubHelper->CalcHubPose();
    }
    return endPose;
}
