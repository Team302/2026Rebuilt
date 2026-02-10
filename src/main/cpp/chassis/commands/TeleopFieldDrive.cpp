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

#include "chassis/commands/TeleopFieldDrive.h"
#include "state/RobotState.h"
#include "utils/FMSData.h"
#include "vision/DragonVision.h"

// Note the simplified constructor and AddRequirements call
TeleopFieldDrive::TeleopFieldDrive(subsystems::CommandSwerveDrivetrain *chassis,
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
 * @brief Initializes the teleop field drive command.
 *
 * Publishes that drive to field element is not done and sets the vision pipeline
 * to April Tags for field element detection.
 */
void TeleopFieldDrive::Initialize()
{
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFieldElement_Bool, false);

    auto vision = DragonVision::GetDragonVision();
    if (vision != nullptr)
    {
        vision->SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS, DRAGON_LIMELIGHT_PIPELINE::APRIL_TAG);
    }
}

/**
 * @brief Executes the teleop field drive command.
 *
 * Reads controller inputs for forward, strafe, and rotate axes, scales them by the
 * current maximum speeds, and applies the field-centric drive request to the chassis.
 */
void TeleopFieldDrive::Execute()
{
    double forward = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD);
    double strafe = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE);
    double rotate = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE);

    m_chassis->SetControl(
        m_fieldDriveRequest.WithVelocityX(forward * m_currentMaxSpeed)
            .WithVelocityY(strafe * m_currentMaxSpeed)
            .WithRotationalRate(rotate * m_currentMaxAngularRate));
}

/**
 * @brief Checks if the command is finished.
 *
 * This command runs continuously until interrupted by another command.
 *
 * @return Always returns false.
 */
bool TeleopFieldDrive::IsFinished()
{
    // A default drive command should never finish on its own.
    // It runs until it is interrupted by another command.
    return false;
}

/**
 * @brief Ends the teleop field drive command.
 *
 * Applies brake control to the chassis when the command ends or is interrupted.
 *
 * @param interrupted Whether the command was interrupted.
 */
void TeleopFieldDrive::End(bool interrupted)
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
void TeleopFieldDrive::NotifyStateUpdate(RobotStateChanges::StateChange change, bool value)
{
    if (change == RobotStateChanges::StateChange::IsLaunching_Bool)
    {
        m_currentMaxSpeed = value ? m_maxSpeed * m_launchingSpeedScale : m_maxSpeed;
        m_currentMaxAngularRate = value ? m_maxAngularRate * m_launchingSpeedScale : m_maxAngularRate;
    }
}
