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
}

//------------------------------------------------------------------
/// @brief      Calculates the target end pose for the Outpost
/// @return     frc::Pose2d - The target pose at the center of the nearest Outpost
/// @details    Uses OutpostHelper to determine which Outpost (red or blue) is
///             closest to the robot and calculates the center pose of that
///             Outpost. Returns a default pose if OutpostHelper is unavailable.
//------------------------------------------------------------------
frc::Pose2d DriveToOutpost::GetEndPose()
{
    frc::Pose2d endPose;
    auto outpostHelper = OutpostHelper::GetInstance();
    if (outpostHelper != nullptr)
    {
        endPose = outpostHelper->CalcOutpostPose();
    }
    return endPose;
}

//------------------------------------------------------------------
/// @brief Checks if the DriveToOutpost command has finished execution.
///
/// @return true if the end pose is at the origin or if the base class's IsFinished
///         condition is met, false otherwise.
///
/// @details This method determines whether the command should terminate by first checking
///          if the end pose is at the origin (which means we had an error calculating the
///          target position) so we stop immediately.  Otherwise, it delegates to the base class's
///          IsFinished() method to determine completion.
//------------------------------------------------------------------
bool DriveToOutpost::IsFinished()
{
    auto endPose = GetEndPose();
    if (PoseUtils::IsPoseAtOrigin(endPose, units::length::centimeter_t{1.0}))
    {
        return true;
    }

    return DriveToPose::IsFinished(); // call base class's IsFinished method
}
