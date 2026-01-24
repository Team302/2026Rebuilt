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
/// @class DriveToDepot
/// @brief Command to autonomously drive the robot to the nearest depot on the field
///
/// This command extends DriveToPose to provide specific functionality for navigating to depots.
/// It automatically determines which depot (red or blue) is closest to the robot's current position
/// and calculates the target pose at the center of that depot using DepotHelper.
///
/// The command uses path following to drive the robot to the calculated depot center position,
/// making it useful for autonomous routines or driver assistance features during matches.
//====================================================================================================================================================
class DriveToDepot : public DriveToPose
{
public:
    //------------------------------------------------------------------
    /// @brief      Constructor for DriveToDepot command
    /// @param[in]  chassis - Pointer to the swerve drive subsystem
    /// @details    Initializes the command with the chassis reference for
    ///             autonomous navigation to the nearest depot
    //------------------------------------------------------------------
    DriveToDepot(subsystems::CommandSwerveDrivetrain *chassis);

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    //------------------------------------------------------------------
    ~DriveToDepot() = default;

    //------------------------------------------------------------------
    /// @brief      Calculates the target end pose for the depot
    /// @return     frc::Pose2d - The target pose at the center of the nearest depot
    /// @details    Overrides the base class method to provide depot-specific
    ///             target calculation using DepotHelper
    //------------------------------------------------------------------
    frc::Pose2d GetEndPose() override;
};