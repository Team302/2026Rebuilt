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

#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "state/IRobotStateChangeSubscriber.h"
#include "teleopcontrol/TeleopControl.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"

/**
 * @brief Command for teleop field-centric driving of the swerve drivetrain.
 *
 * This command reads input from the teleop controller and translates it into field-centric
 * drive requests for the swerve drivetrain. It supports dynamic speed scaling based on
 * robot state changes, such as reducing speed during launching operations.
 */
class TeleopFieldDrive : public frc2::CommandHelper<frc2::Command, TeleopFieldDrive>, public IRobotStateChangeSubscriber
{
public:
    /**
     * @brief Constructs a TeleopFieldDrive command.
     *
     * @param chassis Pointer to the CommandSwerveDrivetrain subsystem.
     * @param controller Pointer to the TeleopControl for input.
     * @param maxSpeed Maximum linear velocity for normal operation.
     * @param maxAngularRate Maximum angular velocity for normal operation.
     */
    TeleopFieldDrive(subsystems::CommandSwerveDrivetrain *chassis,
                     TeleopControl *controller,
                     units::velocity::meters_per_second_t maxSpeed,
                     units::angular_velocity::degrees_per_second_t maxAngularRate);

    /**
     * @brief Initializes the command.
     *
     * Sets up initial state before command starts executing.
     */
    void Initialize() override;

    /**
     * @brief Executes the drive command.
     *
     * Reads controller inputs and applies field-centric drive request to the chassis.
     */
    void Execute() override;

    /**
     * @brief Checks if the command is finished.
     *
     * This command runs indefinitely until interrupted.
     *
     * @return Always returns false.
     */
    bool IsFinished() override;

    /**
     * @brief Ends the command.
     *
     * Cleans up any state when the command ends or is interrupted.
     *
     * @param interrupted Whether the command was interrupted.
     */
    void End(bool interrupted) override;

private:
    subsystems::CommandSwerveDrivetrain *m_chassis;                        ///< Pointer to the swerve drivetrain subsystem.
    TeleopControl *m_controller;                                           ///< Pointer to the teleop control input.
    units::velocity::meters_per_second_t m_maxSpeed;                       ///< Maximum linear speed for normal operation.
    units::velocity::meters_per_second_t m_currentMaxSpeed;                ///< Current maximum linear speed (may be scaled).
    units::angular_velocity::degrees_per_second_t m_maxAngularRate;        ///< Maximum angular rate for normal operation.
    units::angular_velocity::degrees_per_second_t m_currentMaxAngularRate; ///< Current maximum angular rate (may be scaled).
    static constexpr double m_launchingSpeedScale = 0.5;                   ///< Scale factor for speed reduction during launching.

    swerve::requests::FieldCentric m_fieldDriveRequest = swerve::requests::FieldCentric{}
                                                             .WithDeadband(m_maxSpeed * 0.1)                                  // TODO: Investigate this deadband vs controller deadband
                                                             .WithRotationalDeadband(m_maxAngularRate * 0.1)                  // TODO: Investigate this deadband vs controller deadband
                                                             .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage) // Use open-loop voltage for drive
                                                             .WithDesaturateWheelSpeeds(true);

    void NotifyStateUpdate(RobotStateChanges::StateChange change, bool value) override;
};
