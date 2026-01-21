//====================================================================================================================================================
// Copyright 2025 Lake Orion Robotics FIRST Team 302
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
#include "frc2/command/Commands.h"
#include "frc2/command/button/RobotModeTriggers.h"
#include "chassis/commands/TeleopFieldDrive.h"
#include "chassis/commands/TeleopRobotDrive.h"
#include "chassis/commands/DriveToPose.h"
#include "chassis/commands/VisionDrive.h"
#include "state/RobotState.h"
#include "state/IRobotStateChangeSubscriber.h"
#include "frc2/command/ProxyCommand.h"
#include "utils/logging/debug/Logger.h"
#include "fielddata/FieldConstants.h"

// Season Specific Commands

SwerveContainer *SwerveContainer::m_instance = nullptr;

SwerveContainer *SwerveContainer::GetInstance()
{
    if (SwerveContainer::m_instance == nullptr)
    {
        SwerveContainer::m_instance = new SwerveContainer();
    }
    return SwerveContainer::m_instance;
}

SwerveContainer::SwerveContainer() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                                     m_maxSpeed(ChassisConfigMgr::GetInstance()->GetMaxSpeed()),
                                     m_fieldDrive(std::make_unique<TeleopFieldDrive>(m_chassis, TeleopControl::GetInstance(), m_maxSpeed, m_maxAngularRate)),
                                     m_robotDrive(std::make_unique<TeleopRobotDrive>(m_chassis, TeleopControl::GetInstance(), m_maxSpeed, m_maxAngularRate)),
                                     m_trajectoryDrive(std::make_unique<TrajectoryDrive>(m_chassis))
// TO DO: Decided how to do specified heading for telop drive(Either another child command of DriveToPose or potentially a child of TelopFieldDrive/RobotDrive dpending on what you need)
{

    if (m_chassis != nullptr)
    {
        ConfigureBindings();
    }
}

void SwerveContainer::ConfigureBindings()
{
    auto controller = TeleopControl::GetInstance();

    CreateStandardDriveCommands(controller);

    m_chassis->RegisterTelemetry([this](auto const &state)
                                 { logger.Telemeterize(state); });

    SetSysIDBinding(controller);
}

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

void SwerveContainer::CreateRebuiltDriveToCommands(TeleopControl *controller)
{
}

void SwerveContainer::SetSysIDBinding(TeleopControl *controller)
{
    if (controller != nullptr)
    {
        // Run SysId routines when holding Select and A,X,Y,B.
        // Note that each routine should be run exactly once in a single log.
        (controller->GetCommandTrigger(TeleopControlFunctions::SYSID_MODIFER) && controller->GetCommandTrigger(TeleopControlFunctions::AUTO_ALIGN_HUMAN_PLAYER_STATION)).WhileTrue(m_chassis->SysIdQuasistatic(frc2::sysid::Direction::kForward)); // A
        (controller->GetCommandTrigger(TeleopControlFunctions::SYSID_MODIFER) && controller->GetCommandTrigger(TeleopControlFunctions::AUTO_ALIGN_LEFT)).WhileTrue(m_chassis->SysIdQuasistatic(frc2::sysid::Direction::kReverse));                 // B
        (controller->GetCommandTrigger(TeleopControlFunctions::SYSID_MODIFER) && controller->GetCommandTrigger(TeleopControlFunctions::AUTO_CLIMB)).WhileTrue(m_chassis->SysIdDynamic(frc2::sysid::Direction::kForward));                          // Y
        (controller->GetCommandTrigger(TeleopControlFunctions::SYSID_MODIFER) && controller->GetCommandTrigger(TeleopControlFunctions::AUTO_ALIGN_RIGHT)).WhileTrue(m_chassis->SysIdDynamic(frc2::sysid::Direction::kReverse));                    // X
    }
}

void SwerveContainer::NotifyStateUpdate(RobotStateChanges::StateChange change, int value)
{
    if (change == RobotStateChanges::StateChange::ClimbModeStatus_Int)
    {
        m_climbMode = value;
    }
}