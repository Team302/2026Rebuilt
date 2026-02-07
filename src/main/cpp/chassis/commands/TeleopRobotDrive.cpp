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

#include "chassis/commands/TeleopRobotDrive.h"
#include "state/RobotState.h"
#include "state/RobotStateChanges.h"
#include "utils/logging/debug/Logger.h"

// Note the simplified constructor and AddRequirements call
TeleopRobotDrive::TeleopRobotDrive(subsystems::CommandSwerveDrivetrain *chassis,
                                   TeleopControl *controller,
                                   units::velocity::meters_per_second_t maxSpeed,
                                   units::angular_velocity::degrees_per_second_t maxAngularRate) : m_chassis(chassis),
                                                                                                   m_controller(controller),
                                                                                                   m_maxSpeed(maxSpeed),
                                                                                                   m_currentMaxSpeed(maxSpeed),
                                                                                                   m_maxAngularRate(maxAngularRate),
                                                                                                   m_currentMaxAngularRate(maxAngularRate)
{
    AddRequirements(m_chassis);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::IsLaunching_Bool);
}

/**
 * @brief Initializes the teleop robot drive command.
 *
 * Sets up initial state and registers for robot state change notifications.
 */
void TeleopRobotDrive::Initialize()
{
}

/**
 * @brief Executes the teleop robot drive command.
 *
 * Reads controller inputs for forward, strafe, and rotate axes, scales them by the
 * current maximum speeds, and applies the robot-centric drive request to the chassis.
 */
void TeleopRobotDrive::Execute()
{
    double forward = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD);
    double strafe = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE);
    double rotate = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE);

    m_chassis->SetControl(
        m_RobotDriveRequest.WithVelocityX(forward * m_currentMaxSpeed)
            .WithVelocityY(strafe * m_currentMaxSpeed)
            .WithRotationalRate(rotate * m_currentMaxAngularRate));
}

bool TeleopRobotDrive::IsFinished()
{
    // A default drive command should never finish on its own.
    // It runs until it is interrupted by another command.
    return false;
}

/**
 * @brief Ends the teleop robot drive command.
 *
 * Applies brake control to the chassis when the command ends or is interrupted.
 *
 * @param interrupted Whether the command was interrupted.
 */
void TeleopRobotDrive::End(bool interrupted)
{
    m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
}

/**
 * @brief Handles robot state change notifications.
 *
 * When the robot enters or exits launching state, scales the maximum speeds
 * to reduce chassis movement during launching operations.
 *
 * @param change The type of state change.
 * @param value The new state value (true for launching, false for normal).
 */
void TeleopRobotDrive::NotifyStateUpdate(RobotStateChanges::StateChange change, bool value)
{
    if (change == RobotStateChanges::StateChange::IsLaunching_Bool)
    {
        m_currentMaxSpeed = value ? m_maxSpeed * m_launchingSpeedScale : m_maxSpeed;
        m_currentMaxAngularRate = value ? m_maxAngularRate * m_launchingSpeedScale : m_maxAngularRate;

        Logger::GetLogger()->LogDataDirectlyOverNT(std::string("TeleopRobotDrive"), std::string("Launching "), value);
        Logger::GetLogger()->LogDataDirectlyOverNT(std::string("TeleopRobotDrive"), std::string("m_currentMaxSpeed "), m_currentMaxSpeed.value());
        Logger::GetLogger()->LogDataDirectlyOverNT(std::string("TeleopRobotDrive"), std::string("m_currentMaxAngularRate "), m_currentMaxAngularRate.value());
    }
}