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

#include "utils/TargetCalculator.h"

#include <cmath>

TargetCalculator::TargetCalculator()
{
    m_chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();
}

TargetCalculator *TargetCalculator::m_instance = nullptr;

TargetCalculator *TargetCalculator::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new TargetCalculator();
    }
    return m_instance;
}

frc::Translation2d TargetCalculator::GetTargetPosition()
{
    // Default implementation returns origin. Subclasses should override.
    return frc::Translation2d{0_m, 0_m};
}

frc::Translation2d TargetCalculator::CalculateVirtualTarget(
    const frc::Translation2d &realTarget,
    units::second_t lookaheadTime) const
{
    auto velocity = GetChassisVelocity();

    // Calculate the offset based on robot velocity and lookahead time
    // The virtual target is offset backward (opposite of velocity) to compensate
    // for the robot's movement during projectile flight
    units::meter_t offsetX = velocity.vx * lookaheadTime;
    units::meter_t offsetY = velocity.vy * lookaheadTime;

    // Virtual goal = real goal - velocity_offset
    // This compensates for the robot moving toward/away from the goal
    return frc::Translation2d{
        realTarget.X() - offsetX,
        realTarget.Y() - offsetY};
}

frc::Translation2d TargetCalculator::GetLauncherWorldPosition() const
{
    auto pose = GetChassisPose();

    // Get launcher position in robot frame
    auto launcherOffsetRobot = GetLauncherOffset();
    double launcherX_robot = launcherOffsetRobot.X().value();
    double launcherY_robot = launcherOffsetRobot.Y().value();

    // Get robot's rotation
    double robotAngle = pose.Rotation().Radians().value();
    double cosTheta = std::cos(robotAngle);
    double sinTheta = std::sin(robotAngle);

    // Rotate launcher position from robot frame to world frame
    // Rotation matrix: [cos -sin] [x]
    //                 [sin  cos] [y]
    double launcherX_world = launcherX_robot * cosTheta - launcherY_robot * sinTheta;
    double launcherY_world = launcherX_robot * sinTheta + launcherY_robot * cosTheta;

    // Translate to world position (add robot center)
    return frc::Translation2d{
        pose.X() + units::meter_t{launcherX_world},
        pose.Y() + units::meter_t{launcherY_world}};
}

units::meter_t TargetCalculator::CalculateDistanceToTarget(const frc::Translation2d *target)
{
    auto pose = GetChassisPose();
    auto robotPosition = frc::Translation2d{pose.X(), pose.Y()};

    auto targetPos = (target != nullptr) ? *target : GetTargetPosition();

    // Calculate distance from chassis center to target
    return robotPosition.Distance(targetPos);
}

units::meter_t TargetCalculator::CalculateLauncherDistanceToTarget(const frc::Translation2d *target)
{
    frc::Translation2d launcherPos = GetLauncherWorldPosition();
    frc::Translation2d targetPos = (target != nullptr) ? *target : GetTargetPosition();

    // Calculate distance from launcher to target
    return launcherPos.Distance(targetPos);
}

units::degree_t TargetCalculator::CalculateAngleToTarget(const frc::Translation2d *target)
{
    frc::Pose2d pose = GetChassisPose();
    frc::Translation2d robotPosition = frc::Translation2d{pose.X(), pose.Y()};

    frc::Translation2d targetPos = (target != nullptr) ? *target : GetTargetPosition();

    // Calculate vector from robot center to target in world frame
    auto xDistance = (targetPos.X() - robotPosition.X());
    auto yDistance = (targetPos.Y() - robotPosition.Y());

    // Calculate angle in world frame (from +X axis)
    units::angle::degree_t fieldAngleToTarget = units::math::atan2(yDistance, xDistance);
    units::angle::degree_t robotRotation = pose.Rotation().Degrees();

    // Convert to robot frame by subtracting robot's rotation
    units::angle::degree_t angleToTarget = fieldAngleToTarget - robotRotation;

    // Normalize angle to [-pi, pi]
    angleToTarget = units::angle::degree_t{units::math::atan2(units::math::sin(angleToTarget), units::math::cos(angleToTarget))};
    return angleToTarget;
}

units::degree_t TargetCalculator::CalculateLauncherAngleToTarget(const frc::Translation2d *target)
{
    frc::Translation2d launcherPos = GetLauncherWorldPosition();
    frc::Pose2d pose = GetChassisPose();

    frc::Translation2d targetPos = (target != nullptr) ? *target : GetTargetPosition();

    // Calculate vector from launcher to target in world frame
    auto xDistance = (targetPos.X() - launcherPos.X());
    auto yDistance = (targetPos.Y() - launcherPos.Y());

    // Calculate angle in world frame (from +X axis)
    units::angle::degree_t fieldAngleToTarget = units::math::atan2(yDistance, xDistance);
    units::angle::degree_t robotRotation = pose.Rotation().Degrees();

    // Convert to robot frame by subtracting robot's rotation
    units::angle::degree_t angleToTarget = fieldAngleToTarget - robotRotation;

    // Normalize angle to [-pi, pi]
    angleToTarget = units::angle::degree_t{units::math::atan2(units::math::sin(angleToTarget), units::math::cos(angleToTarget))};
    return angleToTarget;
}

void TargetCalculator::SetLauncherOffset(units::meter_t xOffset, units::meter_t yOffset)
{
    // Default implementation does nothing. Subclasses should override if needed. May not need for different mechanisms.
}

frc::Pose2d TargetCalculator::GetChassisPose() const
{
    if (m_chassis != nullptr)
    {
        return m_chassis->GetPose();
    }
    return frc::Pose2d{};
}

frc::ChassisSpeeds TargetCalculator::GetChassisVelocity() const
{
    if (m_chassis != nullptr)
    {
        return m_chassis->GetState().Speeds;
    }
    return frc::ChassisSpeeds{};
}