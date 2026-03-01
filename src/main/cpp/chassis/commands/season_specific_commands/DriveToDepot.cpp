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
#include "chassis/commands/season_specific_commands/DriveToDepot.h"
#include "auton/NeutralZoneManager.h"
#include "fielddata/DepotHelper.h"
#include "utils/PoseUtils.h"

//------------------------------------------------------------------
/// @brief      Constructor for DriveToDepot command
/// @param[in]  chassis - Pointer to the swerve drive subsystem
/// @details    Initializes the base DriveToPose command with the chassis.
///             This command autonomously drives the robot to the nearest
///             depot on the field.
//------------------------------------------------------------------
DriveToDepot::DriveToDepot(subsystems::CommandSwerveDrivetrain *chassis)
    : DriveToPose(chassis)
{
}

//------------------------------------------------------------------
/// @brief      Calculates the target pose for driving to a depot
/// @return     DriveToPoses struct containing the depot target pose
/// @details    Determines the nearest depot and returns its center position.
///
///             **Behavior:**
///             - If in neutral zone: Returns current pose (no movement)
///             - If in alliance zone: Calculates nearest depot pose using DepotHelper
///
///             The command uses DepotHelper to identify which depot (red or blue)
///             is closest based on the robot's current position and returns
///             the center coordinates of that depot.
///
/// @note       This is a single-stage navigation (no midpoint)
/// @see        DepotHelper::CalcDepotPose() for depot position calculation
//------------------------------------------------------------------
struct DriveToPoses DriveToDepot::GetDriveToPoses()
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
    auto depotHelper = DepotHelper::GetInstance();
    if (depotHelper != nullptr)
    {
        poses.endPose = depotHelper->CalcDepotPose();
        return poses;
    }

    return poses;
}
