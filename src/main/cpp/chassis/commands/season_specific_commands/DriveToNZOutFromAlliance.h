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
#include "fielddata/FieldConstants.h"

//====================================================================================================================================================
/// @class DriveToNZOutFromAllience
/// @brief Command to autonomously drive the robot to the nearest bump on the field
///
/// This command extends DriveToPose to provide specific functionality for navigating to NZOutFromAllience.
/// It automatically determines which bump (red or blue) is closest to the robot's current position
/// and calculates the target pose at the center of that bump using DepotHelper.
///
/// The command uses PID control to drive the robot to the calculated bump center position,
/// making it useful for autonomous routines or driver assistance features during matches.
//====================================================================================================================================================
class DriveToNZOutFromAlliance : public DriveToPose
{
public:
    //------------------------------------------------------------------
    /// @brief      Constructor for DriveToNZOutFromAllience command
    /// @param[in]  chassis - Pointer to the swerve drive subsystem
    /// @details    Initializes the command with the chassis reference for
    ///             autonomous navigation to the nearest depot
    //------------------------------------------------------------------
    DriveToNZOutFromAlliance(subsystems::CommandSwerveDrivetrain *chassis);

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    //------------------------------------------------------------------
    ~DriveToNZOutFromAlliance() = default;

    //------------------------------------------------------------------
    /// @brief      Calculates the target end pose for the NZOutFromAllience
    /// @return     frc::Pose2d - The target pose at the center of the nearest depot
    /// @details    Overrides the base class method to provide NZOutFromAllience-specific
    ///             target calculation using DepotHelper
    //------------------------------------------------------------------
    frc::Pose2d GetEndPose() override;

    //------------------------------------------------------------------
    /// @brief      Determines if theDriveToNZOutFromAllience command has finished execution
    /// @return     true if the command has completed driving to the depot,
    ///             false if the command should continue running
    /// @details    Called repeatedly by the command scheduler to check completion status
    //------------------------------------------------------------------

    bool IsFinished() override;
    frc::Pose2d GetNearestBumpPose();
    frc::Pose2d m_robotPose;

private:
    static constexpr units::degree_t RedBumpOutpostToNZ{45.0};
    static constexpr units::degree_t RedBumpOutpostToAlliance{45.0};
    static constexpr units::degree_t RedBumpDepotToNZ{45.0};
    static constexpr units::degree_t RedBumpDepotToAlliance{45.0};

    static constexpr units::degree_t BlueBumpOutpostToNZ{45.0};
    static constexpr units::degree_t BlueBumpOutpostToAlliance{45.0};
    static constexpr units::degree_t BlueBumpDepotToNZ{45.0};
    static constexpr units::degree_t BlueBumpDepotToAlliance{45.0};
};
