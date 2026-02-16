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
#include "fielddata/BumpHelper.h"
#include "fielddata/FieldConstants.h"
#include "units/angle.h"

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
class DriveOverBump : public DriveToPose
{
public:
    //------------------------------------------------------------------
    /// @brief      Constructor for DriveToNZOutFromAllience command
    /// @param[in]  chassis - Pointer to the swerve drive subsystem
    /// @details    Initializes the command with the chassis reference for
    ///             autonomous navigation to the nearest depot
    //------------------------------------------------------------------
    DriveOverBump(subsystems::CommandSwerveDrivetrain *chassis);

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    //------------------------------------------------------------------
    ~DriveOverBump() = default;

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
    frc::Pose2d m_robotPose;
    subsystems::CommandSwerveDrivetrain *m_chassis;

private:
    units::angle::degree_t GetRotation(BUMP_ID bump, bool isInNeutralZone) const;

    frc::Pose2d m_midPose;
    frc::Pose2d m_endPose;
    bool m_beforeMidPose = true;

    static constexpr units::degree_t RedAllianceOutpostWallTowardHub{225.0};
    static constexpr units::degree_t RedAllianceDepotWallTowardHub{135.0};
    static constexpr units::degree_t BlueAllianceOutpostWallTowardHub{315.0};
    static constexpr units::degree_t BlueAllianceDepotWallTowardHub{45.0};
    static constexpr units::degree_t NeutralZoneTowardHubRedDepot{45.0};
    static constexpr units::degree_t NeutralZoneTowardHubBlueDepot{225.0};
    static constexpr units::degree_t NeutralZoneTowardHubRedOutpost{315.0};
    static constexpr units::degree_t NeutralZoneTowardHubBlueOutpost{135.0};
};
