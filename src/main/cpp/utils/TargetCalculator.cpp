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
    units::time::second_t lookaheadTime) const
{
    auto robotVelocity = GetChassisVelocity();
    auto pose = GetChassisPose();

    // Convert robot-relative speeds to field-relative speeds
    auto fieldVelocity = frc::ChassisSpeeds::FromRobotRelativeSpeeds(
        robotVelocity.vx,
        robotVelocity.vy,
        robotVelocity.omega,
        pose.Rotation());

    units::meter_t offsetX = fieldVelocity.vx * lookaheadTime;
    units::meter_t offsetY = fieldVelocity.vy * lookaheadTime;

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
    auto launcherX_robot = launcherOffsetRobot.X();
    auto launcherY_robot = launcherOffsetRobot.Y();

    // Get robot's rotation
    auto robotAngle = pose.Rotation().Degrees();
    auto cosTheta = units::math::cos(robotAngle);
    auto sinTheta = units::math::sin(robotAngle);

    // Rotate launcher position from robot frame to world frame
    auto launcherX_world = launcherX_robot * cosTheta - launcherY_robot * sinTheta;
    auto launcherY_world = launcherX_robot * sinTheta + launcherY_robot * cosTheta;

    // Translate to world position (add robot center)
    return frc::Translation2d{
        pose.X() + launcherX_world,
        pose.Y() + launcherY_world};
}

units::meter_t TargetCalculator::CalculateDistanceToTarget(units::time::second_t lookaheadTime)
{
    auto pose = GetChassisPose();
    auto robotPosition = frc::Translation2d{pose.X(), pose.Y()};

    auto realTarget = GetTargetPosition();
    auto targetPos = (lookaheadTime > 0_s) ? CalculateVirtualTarget(realTarget, lookaheadTime) : realTarget;

    // Calculate distance from chassis center to target
    return robotPosition.Distance(targetPos);
}

units::meter_t TargetCalculator::CalculateLauncherDistanceToTarget(units::time::second_t lookaheadTime)
{
    frc::Translation2d launcherPos = GetLauncherWorldPosition();

    auto realTarget = GetTargetPosition();
    auto targetPos = (lookaheadTime > 0_s) ? CalculateVirtualTarget(realTarget, lookaheadTime) : realTarget;

    // Calculate distance from launcher to target
    return launcherPos.Distance(targetPos);
}

units::degree_t TargetCalculator::CalculateAngleToTarget(units::time::second_t lookaheadTime)
{
    frc::Pose2d pose = GetChassisPose();
    frc::Translation2d robotPosition = frc::Translation2d{pose.X(), pose.Y()};

    auto realTarget = GetTargetPosition();
    auto targetPos = (lookaheadTime > 0_s) ? CalculateVirtualTarget(realTarget, lookaheadTime) : realTarget;

    // Calculate vector from robot center to target in world frame
    auto xDistance = (targetPos.X() - robotPosition.X());
    auto yDistance = (targetPos.Y() - robotPosition.Y());

    // Calculate angle in world frame (from +X axis / forward direction)
    // This is relative to field forward, not robot heading
    units::angle::degree_t angleToTarget = units::math::atan2(yDistance, xDistance);

    return angleToTarget;
}

units::degree_t TargetCalculator::CalculateLauncherAngleToTarget(units::time::second_t lookaheadTime)
{
    frc::Translation2d launcherPos = GetLauncherWorldPosition();

    auto realTarget = GetTargetPosition();
    auto targetPos = (lookaheadTime > 0_s) ? CalculateVirtualTarget(realTarget, lookaheadTime) : realTarget;

    // Calculate vector from launcher to target in world frame
    auto xDistance = (targetPos.X() - launcherPos.X());
    auto yDistance = (targetPos.Y() - launcherPos.Y());

    // Calculate angle in world frame (from +X axis / forward direction)
    // This is relative to field forward, not robot heading
    units::degree_t angleToTarget = units::math::atan2(yDistance, xDistance);

    return angleToTarget;
}

void TargetCalculator::SetLauncherOffset(units::meter_t xOffset, units::meter_t yOffset)
{
    // Default implementation does nothing. Subclasses should override if needed. May not need for different mechanisms.
}

frc::Pose2d TargetCalculator::GetVirtualTargetPose(
    units::time::second_t lookaheadTime)
{
    auto realTarget = GetTargetPosition();
    return frc::Pose2d{
        CalculateVirtualTarget(realTarget, lookaheadTime),
        frc::Rotation2d{}};
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