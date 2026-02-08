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

#include <memory>

#include "chassis/commands/TrajectoryDrive.h"
#include "chassis/commands/season_specific_commands/DriveToDepot.h"
#include "chassis/commands/season_specific_commands/DriveToOutpost.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "chassis/generated/Telemetry.h"
#include "frc2/command/CommandPtr.h"
#include "state/IRobotStateChangeSubscriber.h"
#include "teleopcontrol/TeleopControl.h"

//====================================================================================================================================================
/// @class SwerveContainer
/// @brief Container class for managing swerve drivetrain subsystem and associated commands
///
/// This singleton class serves as the primary container for the swerve drive chassis,
/// managing all drive-related commands and their bindings. It implements IRobotStateChangeSubscriber
/// to respond to robot state changes.
///
/// Responsibilities include:
/// - Initializing the swerve chassis subsystem
/// - Creating and managing drive commands (field-oriented, robot-oriented, trajectory, autonomous)
/// - Configuring button bindings for driver control
/// - Setting up telemetry logging
/// - Providing access to trajectory drive commands for autonomous routines
/// - Supporting system identification (SysID) for drivetrain characterization
//====================================================================================================================================================
class SwerveContainer : public IRobotStateChangeSubscriber
{
public:
    //------------------------------------------------------------------
    /// @brief      Get the singleton instance of SwerveContainer
    /// @return     SwerveContainer* - Pointer to the singleton instance
    //------------------------------------------------------------------
    static SwerveContainer *GetInstance();

    //------------------------------------------------------------------
    /// @brief      Get the trajectory drive command for autonomous routines
    /// @return     TrajectoryDrive* - Pointer to the trajectory drive command
    //------------------------------------------------------------------
    TrajectoryDrive *GetTrajectoryDriveCommand() { return m_trajectoryDrive.get(); }
    DriveToDepot *GetDriveToDepotCommand() { return m_driveToDepot.get(); }
    DriveToOutpost *GetDriveToOutpostCommand() { return m_driveToOutpost.get(); }

private:
    //------------------------------------------------------------------
    /// @brief      Private constructor for singleton pattern
    /// @details    Initializes chassis, creates all drive commands, and
    ///             configures bindings
    //------------------------------------------------------------------
    SwerveContainer();

    //------------------------------------------------------------------
    /// @brief      Virtual destructor (default implementation)
    //------------------------------------------------------------------
    virtual ~SwerveContainer() = default;

    /// @brief Singleton instance pointer
    static SwerveContainer *m_instance;

    /// @brief Pointer to the swerve drivetrain subsystem
    subsystems::CommandSwerveDrivetrain *m_chassis;

    /// @brief Maximum linear speed for the drivetrain
    units::meters_per_second_t m_maxSpeed;

    /// @brief Maximum angular rotation rate (1.25 turns per second)
    static constexpr units::radians_per_second_t m_maxAngularRate{1.25_tps};

    /// @brief Telemetry logger for chassis state data
    Telemetry logger;

    /// @brief Field-oriented drive command for teleop control
    frc2::CommandPtr m_fieldDrive;

    /// @brief Robot-oriented drive command for teleop control
    frc2::CommandPtr m_robotDrive;

    /// @brief Trajectory following command for autonomous paths
    std::unique_ptr<TrajectoryDrive> m_trajectoryDrive;

    /// @brief Drive to depot command for season-specific autonomous navigation
    std::unique_ptr<DriveToDepot> m_driveToDepot;

    /// @brief Drive to outpost command for season-specific autonomous navigation
    std::unique_ptr<DriveToOutpost> m_driveToOutpost;

    //------------------------------------------------------------------
    /// @brief      Configures button bindings for chassis control
    /// @details    Sets up controller bindings, telemetry, and SysID options
    //------------------------------------------------------------------
    void ConfigureBindings();

    //------------------------------------------------------------------
    /// @brief      Configures System Identification button bindings
    /// @param[in]  controller - Pointer to the teleop controller
    /// @details    Binds SysID routines for drivetrain characterization
    //------------------------------------------------------------------
    void SetSysIDBinding(TeleopControl *controller);

    //------------------------------------------------------------------
    /// @brief      Creates and binds standard drive commands
    /// @param[in]  controller - Pointer to the teleop controller
    /// @details    Sets up field/robot-oriented drive and reset functions
    //------------------------------------------------------------------
    void CreateStandardDriveCommands(TeleopControl *controller);

    //------------------------------------------------------------------
    /// @brief      Creates and binds season-specific drive commands
    /// @param[in]  controller - Pointer to the teleop controller
    /// @details    Placeholder for binding autonomous drive commands
    //------------------------------------------------------------------
    void CreateRebuiltDriveToCommands(TeleopControl *controller);

    //------------------------------------------------------------------
    /// @brief      Handles robot state change notifications
    /// @param[in]  change - The type of state change that occurred
    /// @param[in]  value - The new value associated with the state change
    /// @details    Overrides IRobotStateChangeSubscriber interface method
    //------------------------------------------------------------------
    void NotifyStateUpdate(RobotStateChanges::StateChange change, int value) override;

    bool m_climbModeStatus = false;
};
