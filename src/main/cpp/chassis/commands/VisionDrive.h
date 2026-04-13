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

#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "frc/controller/PIDController.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "teleopcontrol/TeleopControl.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"
#include "vision/DragonVision.h"

class VisionDrive : public frc2::Command
{
public:
    VisionDrive(subsystems::CommandSwerveDrivetrain *chassis);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

    virtual units::velocity::meters_per_second_t GetMaxVelocity() const { return kMaxVelocityDefault; }

protected:
    virtual swerve::requests::RobotCentric GetRobotDriveRequest() = 0;

    subsystems::CommandSwerveDrivetrain *m_chassis;
    swerve::requests::RobotCentric m_RobotDriveRequest = swerve::requests::RobotCentric{};

private:
    TeleopControl *m_controller = TeleopControl::GetInstance();
    units::velocity::meters_per_second_t m_maxSpeed = 4_mps;
    units::angular_velocity::degrees_per_second_t m_maxAngularRate = 360_deg_per_s;

    DragonVision *m_vision = DragonVision::GetDragonVision();

    /// @brief Maximum translational velocity for the robot
    units::velocity::meters_per_second_t kMaxVelocity;
    static constexpr units::velocity::meters_per_second_t kMaxVelocityDefault = 4_mps;

    units::acceleration::meters_per_second_squared_t kMaxAcceleration;
    static constexpr units::acceleration::meters_per_second_squared_t kMaxAccelerationDefault = 3_mps_sq;

    swerve::requests::FieldCentric m_fieldDriveRequest = swerve::requests::FieldCentric{}
                                                             .WithDeadband(m_maxSpeed * 0.1)                                  // TODO: Investigate this deadband vs controller deadband
                                                             .WithRotationalDeadband(m_maxAngularRate * 0.1)                  // TODO: Investigate this deadband vs controller deadband
                                                             .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage) // Use open-loop voltage for drive
                                                             .WithDesaturateWheelSpeeds(true);

    int m_visionCacheI = 0;
};