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
#include "utils/DragonField.h"

//====================================================================================================================================================
/// @class DriveToOutpost
/// @brief Command to autonomously drive the robot to the nearest Outpost on the field
///
/// This command extends DriveToPose to provide specific functionality for navigating to Outposts.
/// It automatically determines which Outpost (red or blue) is closest to the robot's current position
/// and calculates the target pose using OutpostHelper. The command uses a two-stage approach with a
/// midpoint pose (offset from outpost) and endpoint pose (at the outpost center).
///
/// The command uses PID control to drive the robot to the calculated Outpost positions,
/// making it useful for autonomous routines or driver assistance features during matches.
//====================================================================================================================================================
class DriveToOutpost : public DriveToPose
{
public:
    //------------------------------------------------------------------
    /// @brief      Constructor for DriveToOutpost command
    /// @param[in]  chassis - Pointer to the swerve drive subsystem
    /// @details    Initializes the command with the chassis reference for
    ///             autonomous navigation to the nearest Outpost
    //------------------------------------------------------------------
    DriveToOutpost(subsystems::CommandSwerveDrivetrain *chassis);

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    //------------------------------------------------------------------
    ~DriveToOutpost() = default;

protected:
    //------------------------------------------------------------------
    /// @brief      Calculates target poses for outpost navigation
    /// @return     DriveToPoses struct with offset approach pose (midpoint) and outpost center (endpoint)
    /// @details    Overrides base class to provide outpost-specific two-stage navigation.
    ///             Returns current pose if in neutral zone, otherwise calculates nearest outpost path.
    /// @see        DriveToOutpost.cpp for full implementation details
    //------------------------------------------------------------------
    struct DriveToPoses GetDriveToPoses() override;

private:
};