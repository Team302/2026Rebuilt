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
#include "chassis/commands/season_specific_commands/AutoDefend.h"
#include "chassis/commands/season_specific_commands/DriveAlongNearestWall.h"
#include "chassis/commands/season_specific_commands/DriveOverBump.h"
#include "chassis/commands/season_specific_commands/DriveToDepot.h"
#include "chassis/commands/season_specific_commands/DriveToFuel.h"
#include "chassis/commands/season_specific_commands/DriveToHub.h"
#include "chassis/commands/season_specific_commands/DriveToOutpost.h"
#include "chassis/commands/season_specific_commands/DriveToTower.h"
#include "chassis/commands/season_specific_commands/DriveToTrench.h"
#include "chassis/commands/season_specific_commands/SweepBehindHub.h"
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
    // declaring classes
    //------------------------------------------------------------------
    /// @brief      Get the singleton instance of SwerveContainer
    /// @return     SwerveContainer* - Pointer to the singleton instance
    //------------------------------------------------------------------
    static SwerveContainer *GetInstance();

    //------------------------------------------------------------------
    /// @brief      Get the trajectory drive command for autonomous routines
    /// @return     TrajectoryDrive* - Pointer to the trajectory drive command
    /// @details    Provides access to the trajectory following command for
    ///             executing pre-planned autonomous paths
    //------------------------------------------------------------------
    TrajectoryDrive *GetTrajectoryDriveCommand() { return m_trajectoryDrive.get(); }

    //------------------------------------------------------------------
    /// @brief      Get the drive over bump command for autonomous navigation
    /// @return     DriveOverBump* - Pointer to the drive over bump command
    /// @details    Provides access to the command that navigates over bumps
    ///             or obstacles on the field
    //------------------------------------------------------------------
    DriveOverBump *GetDriveOverBumpCommand() { return m_driveOverBump.get(); }

    //------------------------------------------------------------------
    /// @brief      Get the drive to depot command for autonomous navigation
    /// @return     DriveToDepot* - Pointer to the drive to depot command
    /// @details    Provides access to the command that autonomously drives
    ///             the robot to the depot location
    //------------------------------------------------------------------
    DriveToDepot *GetDriveToDepotCommand() { return m_driveToDepot.get(); }

    //------------------------------------------------------------------
    /// @brief      Get the drive to hub command for autonomous navigation
    /// @return     DriveToHub* - Pointer to the drive to hub command
    /// @details    Provides access to the command that autonomously drives
    ///             the robot to the hub location
    //------------------------------------------------------------------
    DriveToHub *GetDriveToHubCommand() { return m_driveToHub.get(); }

    //------------------------------------------------------------------
    /// @brief      Get the drive to outpost command for autonomous navigation
    /// @return     DriveToOutpost* - Pointer to the drive to outpost command
    /// @details    Provides access to the command that autonomously drives
    ///             the robot to the outpost location
    //------------------------------------------------------------------
    DriveToOutpost *GetDriveToOutpostCommand() { return m_driveToOutpost.get(); }

    //------------------------------------------------------------------
    /// @brief      Get the drive to tower command for autonomous navigation (climb mode)
    /// @return     DriveToTower* - Pointer to the drive to tower command
    /// @details    Provides access to the command that autonomously drives
    ///             the robot to the tower location
    //------------------------------------------------------------------
    DriveToTower *GetDriveToTowerCommand() { return m_driveToTower.get(); }

    //------------------------------------------------------------------
    /// @brief      Get the drive along nearest wall command for autonomous navigation
    /// @return     DriveAlongNearestWall* - Pointer to the drive along nearest wall command
    /// @details    Provides access to the command that autonomously drives
    ///             the robot along the nearest wall
    //------------------------------------------------------------------
    DriveAlongNearestWall *GetDriveAlongNearestWallCommand() { return m_driveAlongNearestWall.get(); }

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

    /// @brief Drive over bump command for season-specific autonomous navigation
    std::unique_ptr<DriveOverBump> m_driveOverBump;

    /// @brief Drive to depot command for season-specific autonomous navigation
    std::unique_ptr<DriveToDepot> m_driveToDepot;

    /// @brief Drive to hub command for season-specific autonomous navigation
    std::unique_ptr<DriveToHub> m_driveToHub;

    /// @brief Drive to outpost command for season-specific autonomous navigation
    std::unique_ptr<DriveToOutpost> m_driveToOutpost;

    /// @brief Drive to tower command for season-specific autonomous navigation (climb mode)
    std::unique_ptr<DriveToTower> m_driveToTower;

    /// @brief Sweep behind hub command for season-specific autonomous navigation
    std::unique_ptr<SweepBehindHub> m_sweepBehindHub;

    /// @brief Drive along nearest wall command for season-specific autonomous navigation
    std::unique_ptr<DriveAlongNearestWall> m_driveAlongNearestWall;

    /// @brief Drive to fuel command for season-specific autonomous navigation fueled by vision targeting
    std::unique_ptr<DriveToFuel> m_driveToFuel;

    /// @brief Auto defend command for season-specific autonomous navigation targets bumpers
    std::unique_ptr<AutoDefend> m_autoDefend;
    /// @brief Drive to trench command for season-specific autonomous navigation
    std::unique_ptr<DriveToTrench> m_driveToTrench;

    //------------------------------------------------------------------
    /// @brief      Configures button bindings for chassis control
    /// @details    Sets up controller bindings, telemetry, and SysID options.
    ///             Calls helper methods to configure standard drive commands
    ///             and season-specific drive-to commands.
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
    /// @details    Sets up autonomous navigation commands for drive-to
    ///             locations (depot, hub, outpost) and obstacle navigation
    ///             (drive over bump). Commands are conditionally enabled
    ///             based on climb mode status.
    //------------------------------------------------------------------
    void CreateRebuiltDriveToCommands(TeleopControl *controller);

    //------------------------------------------------------------------
    /// @brief      Handles robot state change notifications
    /// @param[in]  change - The type of state change that occurred
    /// @param[in]  value - The new value associated with the state change
    /// @details    Overrides IRobotStateChangeSubscriber interface method.
    ///             Currently monitors climb mode status changes to adjust
    ///             drive command behavior accordingly.
    //------------------------------------------------------------------
    void NotifyStateUpdate(RobotStateChanges::StateChange change, bool value) override;

    //------------------------------------------------------------------
    /// @brief      Tracks whether the robot is currently in climb mode
    /// @details    When true, certain drive-to commands are disabled and
    ///             alternative climb-specific navigation is enabled
    //------------------------------------------------------------------
    bool m_climbModeStatus = false;
};
