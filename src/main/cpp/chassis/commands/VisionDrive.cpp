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

#include "chassis/commands/VisionDrive.h"
#include "vision/PoseOffsetUtils.h"

// Note the simplified constructor and AddRequirements call
VisionDrive::VisionDrive(subsystems::CommandSwerveDrivetrain *chassis) : m_chassis(chassis)
{
    AddRequirements(m_chassis);
    m_xController.SetIZone(5.0);
    m_yawController.SetIZone(5.0);
    kMaxVelocity = GetMaxVelocity();
    kMaxAcceleration = GetMaxAcceleration();
}

void VisionDrive::Initialize()
{
    m_xController.Reset();
    m_yawController.Reset();
    m_visionCache = GetObjectSelection();
}

void VisionDrive::Execute()
{
    // auto targets = GetObjectSelection();
    // if (!targets.empty() && targets[0] != nullptr)
    // {
    //     // units::angle::degree_t yawError = targets[0]->horizontalOffset;
    //     // units::length::meter_t rangeToTarget = PoseOffsetUtils::CalculateDistanceFromObject(*targets[0], GetObjectHeight());

    //     // auto xVel = units::velocity::meters_per_second_t(m_xController.Calculate(rangeToTarget.value()));
    //     // auto rotVel = units::angular_velocity::degrees_per_second_t(m_yawController.Calculate(yawError.value()));

    //     // m_chassis->SetControl(
    //     //     m_RobotDriveRequest.WithVelocityX(xVel)
    //     //         .WithVelocityY(0.0_mps)
    //     //         .WithRotationalRate(rotVel));
    //     // m_visionCacheI = 0;
    // }
    // else
    // {
    //     double forward = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD);
    //     double strafe = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE);
    //     double rotate = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE);

    //     m_chassis->SetControl(
    //         m_fieldDriveRequest.WithVelocityX(forward * m_maxSpeed)
    //             .WithVelocityY(strafe * m_maxSpeed)
    //             .WithRotationalRate(rotate * m_maxAngularRate));
    // }
    // m_visionCacheI++;
}

bool VisionDrive::IsFinished()
{
    // A default drive command should never finish on its own.
    // It runs until it is interrupted by another command.
    return false;
}

void VisionDrive::End(bool interrupted)
{
    m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
}