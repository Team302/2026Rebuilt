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
#include "chassis/commands/season_specific_commands/DriveToTower.h"
#include "auton/AllianceZoneManager.h"
#include "utils/PoseUtils.h"
//------------------------------------------------------------------
/// @brief      Constructor for DriveToTower command
/// @param[in]  chassis - Pointer to the swerve drive subsystem
/// @details    Initializes the base DriveToPose command with the chassis.
///             This command autonomously drives the robot to the nearest
///             Tower on the field.
//------------------------------------------------------------------

//------------------------------------------------------------------
/// @brief      Calculates the target end pose for the Tower
/// @return     frc::Pose2d - The target pose at the center of the nearest Tower
/// @details    Uses TowerHelper to determine which Tower (red or blue) is
///             closest to the robot and calculates the center pose of that
///             Tower. Returns a default pose if TowerHelper is unavailable.
//------------------------------------------------------------------
DriveToTower::DriveToTower(subsystems::CommandSwerveDrivetrain *chassis)
    : DriveToPose(chassis)
{
}
//------------------------------------------------------------------
/// @brief      Calculates target poses for tower navigation
/// @return     DriveToPoses struct with poses for tower navigation
/// @details    Overrides base class to provide tower-specific two-stage navigation.
///             Returns current pose if in neutral zone, otherwise calculates nearest tower path.
/// @see        DriveToTower.cpp for full implementation details
//------------------------------------------------------------------
struct DriveToPoses DriveToTower::GetDriveToPoses()
{
    struct DriveToPoses poses;
    poses.hasMidPose = false;

    if (!AllianceZoneManager::GetInstance()->IsInAllianceZone())
    {
        auto chassis = GetChassis();
        if (chassis != nullptr)
        {
            poses.endPose = chassis->GetPose();
        }
        return poses;
    }

    auto towerHelper = TowerHelper::GetInstance();
    if (towerHelper != nullptr)
    {
        poses.endPose = towerHelper->CalcTowerPose();
    }
    return poses;
}
