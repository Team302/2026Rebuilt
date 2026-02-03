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

#include "chassis/SwerveContainer.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/commands/TeleopFieldDrive.h"
#include "chassis/commands/TeleopRobotDrive.h"
#include "frc2/command/ProxyCommand.h"
#include "frc2/command/button/RobotModeTriggers.h"
#include "frc2/command/Commands.h"
#include "state/RobotState.h"
#include "utils/logging/debug/Logger.h"

// Season Specific Commands
#include "chassis/commands/season_specific_commands/DriveToDepot.h"
#include "chassis/commands/season_specific_commands/DriveToOutpost.h"

//------------------------------------------------------------------
/// @brief      Static method to create or return the singleton instance
//------------------------------------------------------------------
SwerveContainer *SwerveContainer::m_instance = nullptr;
SwerveContainer *SwerveContainer::GetInstance()
{
    if (SwerveContainer::m_instance == nullptr)
    {
        SwerveContainer::m_instance = new SwerveContainer();
    }
    return SwerveContainer::m_instance;
}

//------------------------------------------------------------------
/// @brief      Constructor for SwerveContainer
/// @details    Initializes the swerve chassis subsystem and creates all
///             drive commands including field-oriented, robot-oriented,
///             trajectory following, and season-specific commands.
///             Also configures button bindings if chassis is available.
//------------------------------------------------------------------
SwerveContainer::SwerveContainer() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                                     m_maxSpeed(ChassisConfigMgr::GetInstance()->GetMaxSpeed()),
                                     m_fieldDrive(std::make_unique<TeleopFieldDrive>(m_chassis, TeleopControl::GetInstance(), m_maxSpeed, m_maxAngularRate)),
                                     m_robotDrive(std::make_unique<TeleopRobotDrive>(m_chassis, TeleopControl::GetInstance(), m_maxSpeed, m_maxAngularRate)),
                                     m_trajectoryDrive(std::make_unique<TrajectoryDrive>(m_chassis)),
                                     m_driveToDepot(std::make_unique<DriveToDepot>(m_chassis)),
                                     m_driveToOutpost(std::make_unique<DriveToOutpost>(m_chassis))
{
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus_Bool);

    if (m_chassis != nullptr)
    {
        ConfigureBindings();
    }
}

//------------------------------------------------------------------
/// @brief      Configures button bindings for chassis control
/// @details    Sets up command bindings for the teleop controller,
///             creates standard drive commands, and registers telemetry.
///             Also provides commented-out SysID binding setup for
///             system identification routines.
//------------------------------------------------------------------
void SwerveContainer::ConfigureBindings()
{
    auto controller = TeleopControl::GetInstance();

    CreateStandardDriveCommands(controller);
    CreateRebuiltDriveToCommands(controller);

    m_chassis->RegisterTelemetry([this](auto const &state)
                                 { logger.Telemeterize(state); });

    // When needed to do this, map the buttons accordingly and uncomment the lines below
    // SetSysIDBinding(controller);
}

//------------------------------------------------------------------
/// @brief      Creates and binds standard drive commands
/// @param[in]  controller - Pointer to the teleop controller
/// @details    Sets up field-oriented drive as default command,
///             configures idle mode during disabled state,
///             and binds reset yaw and robot-oriented drive triggers.
//------------------------------------------------------------------
void SwerveContainer::CreateStandardDriveCommands(TeleopControl *controller)
{
    auto isResetYawSelected = controller->GetCommandTrigger(TeleopControlFunctions::RESET_POSITION);
    auto isRobotOriented = controller->GetCommandTrigger(TeleopControlFunctions::ROBOT_ORIENTED_DRIVE);

    if (m_chassis != nullptr)
    {
        m_chassis->SetDefaultCommand(std::move(m_fieldDrive));

        frc2::RobotModeTriggers::Disabled().WhileTrue(m_chassis->ApplyRequest([]
                                                                              { return swerve::requests::Idle{}; })
                                                          .IgnoringDisable(true));

        isResetYawSelected.OnTrue(m_chassis->RunOnce([this, controller]
                                                     { m_chassis->SeedFieldCentric(); }));
    }

    isRobotOriented.WhileTrue(std::move(m_robotDrive));
}

//------------------------------------------------------------------
/// @brief      Creates and binds rebuilt drive-to commands
/// @param[in]  controller - Pointer to the teleop controller
/// @details    Placeholder method for binding season-specific
///             autonomous drive commands such as DriveToDepot.
///             To be implemented as needed.
//------------------------------------------------------------------
void SwerveContainer::CreateRebuiltDriveToCommands(TeleopControl *controller)
{
    auto driveToDepot = controller->GetCommandTrigger(TeleopControlFunctions::DRIVE_TO_DEPOT);
    auto driveToOutpost = controller->GetCommandTrigger(TeleopControlFunctions::DRIVE_TO_OUTPOST);

    driveToDepot.WhileTrue(frc2::cmd::DeferredProxy([this]() -> frc2::Command *
                                                    {
        if (m_climbModeStatus) {
            // TODO: Replace with actual Drive To Tower Depot Side Command
        }
        else
        {
            return m_driveToDepot.get();
        } }));

    driveToOutpost.WhileTrue(frc2::cmd::DeferredProxy([this]() -> frc2::Command *
                                                      {
        if (m_climbModeStatus) {
            // TODO: Replace with actual Drive To Tower Outpost Side Command
        }
        else
        {
            return m_driveToOutpost.get();
        } }));
}

//------------------------------------------------------------------
/// @brief      Configures button bindings for System Identification (SysID)
/// @param[in]  controller - Pointer to the teleop controller
/// @details    Binds SysID routines to controller buttons for characterization.
///             Includes quasistatic and dynamic tests in both forward and
///             reverse directions. Each routine should be run exactly once
///             in a single log for proper characterization.
//------------------------------------------------------------------
void SwerveContainer::SetSysIDBinding(TeleopControl *controller)
{
    if (controller != nullptr)
    {
        // Run SysId routines when holding Select and A,X,Y,B.
        // Note that each routine should be run exactly once in a single log
        (controller->GetCommandTrigger(TeleopControlFunctions::SYSID_MODIFER) && controller->GetCommandTrigger(TeleopControlFunctions::SYSID_QUASISTATICFORWARD)).WhileTrue(m_chassis->SysIdQuasistatic(frc2::sysid::Direction::kForward)); // A
        (controller->GetCommandTrigger(TeleopControlFunctions::SYSID_MODIFER) && controller->GetCommandTrigger(TeleopControlFunctions::SYSID_QUASISTATICREVERSE)).WhileTrue(m_chassis->SysIdQuasistatic(frc2::sysid::Direction::kReverse)); // B
        (controller->GetCommandTrigger(TeleopControlFunctions::SYSID_MODIFER) && controller->GetCommandTrigger(TeleopControlFunctions::SYSID_DYNAMICFORWARD)).WhileTrue(m_chassis->SysIdDynamic(frc2::sysid::Direction::kForward));         // Y
        (controller->GetCommandTrigger(TeleopControlFunctions::SYSID_MODIFER) && controller->GetCommandTrigger(TeleopControlFunctions::SYSID_DYNAMICREVERSE)).WhileTrue(m_chassis->SysIdDynamic(frc2::sysid::Direction::kReverse));         // X
    }
}

//------------------------------------------------------------------
/// @brief      Handles robot state change notifications
/// @param[in]  change - The type of state change that occurred
/// @param[in]  value - The new value associated with the state change
/// @details    Implements IRobotStateChangeSubscriber interface.
///             Can be used to react to robot state changes such as
///             game mode transitions or mechanism state updates.
///             Currently a placeholder for future implementation.
//------------------------------------------------------------------
void SwerveContainer::NotifyStateUpdate(RobotStateChanges::StateChange change, int value)
{
    if (change == RobotStateChanges::StateChange::ClimbModeStatus_Bool)
    {
        m_climbModeStatus = value;
    }
}