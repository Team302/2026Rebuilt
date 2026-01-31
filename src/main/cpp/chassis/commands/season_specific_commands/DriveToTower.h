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
#pragma once

#include "chassis/commands/DriveToPose.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"

//====================================================================================================================================================
/// @class DriveToTower
/// @brief Command to autonomously drive the robot to the nearest Tower on the field
///
/// This command extends DriveToPose to provide specific functionality for navigating to Towers.
/// It automatically determines which Tower (red or blue) is closest to the robot's current position
/// and calculates the target pose at the center of that Tower using DriveToTowerHelper.
///
/// The command uses PID control to drive the robot to the calculated Towercenter position,
/// making it useful for autonomous routines or driver assistance features during matches.
//====================================================================================================================================================
class DriveToTower : public DriveToPose
{
public:
    //------------------------------------------------------------------
    /// @brief      Get the singleton instance of TowerHelper
    /// @return     DRiveToTowerHelper* - Pointer to the singleton instance
    //------------------------------------------------------------------
    static DriveToTower *GetInstance();

    //------------------------------------------------------------------
    /// @brief      Constructor for DriveToTower command
    /// @param[in]  chassis - Pointer to the swerve drive subsystem
    /// @details    Initializes the command with the chassis reference for
    ///             autonomous navigation to the nearest Tower
    //------------------------------------------------------------------
    DriveToTower(subsystems::CommandSwerveDrivetrain *chassis);

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    //------------------------------------------------------------------
    ~DriveToTower() = default;

    //------------------------------------------------------------------
    /// @brief      Calculates the target end pose for the Tower
    /// @return     frc::Pose2d - The target pose at the center of the nearest Tower
    /// @details    Overrides the base class method to provide Tower-specific
    ///             target calculation using TowerHelper
    //------------------------------------------------------------------
    frc::Pose2d GetEndPose() override;

    //------------------------------------------------------------------
    /// @brief      Determines if the DriveToTower command has finished execution
    /// @return     true if the command has completed driving to the tower,
    ///             false if the command should continue running
    /// @details    Called repeatedly by the command scheduler to check completion status
    //------------------------------------------------------------------

    bool IsFinished() override;
};