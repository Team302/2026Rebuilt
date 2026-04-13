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
#include "utils/logging/debug/Logger.h"

#include <cmath>

TargetCalculator::TargetCalculator()
{
    m_chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();
}

frc::Translation2d TargetCalculator::CalculateVirtualTarget(
    const frc::Translation2d &realTarget,
    units::time::second_t lookaheadTime) const
{
    // Convert robot-relative speeds to field-relative speeds
    auto fieldVelocity = frc::ChassisSpeeds::FromRobotRelativeSpeeds(
        m_currentChassisSpeeds.vx,
        m_currentChassisSpeeds.vy,
        m_currentChassisSpeeds.omega,
        m_chassisPose.Rotation());

    units::meter_t offsetX = fieldVelocity.vx * lookaheadTime;
    units::meter_t offsetY = fieldVelocity.vy * lookaheadTime;

    // Virtual goal = real goal - velocity_offset
    // This compensates for the robot moving toward/away from the goal
    return frc::Translation2d{
        realTarget.X() - offsetX,
        realTarget.Y() - offsetY};
}

frc::Translation2d TargetCalculator::CalculateRotationalMechDelta(units::time::second_t lookaheadTime) const
{
    units::degree_t futureHeading = m_chassisPose.Rotation().Degrees() +
                                    units::degree_t{m_currentChassisSpeeds.omega.value() * lookaheadTime.value()};

    frc::Translation2d currentMechOffset = m_mechanismOffset.RotateBy(m_chassisPose.Rotation());
    frc::Translation2d futureMechOffset = m_mechanismOffset.RotateBy(frc::Rotation2d{futureHeading});
    return futureMechOffset - currentMechOffset;
}

frc::Translation2d TargetCalculator::GetMechanismWorldPosition() const
{
    return m_chassisPose.Translation() + m_mechanismOffset.RotateBy(m_chassisPose.Rotation());
}

units::meter_t TargetCalculator::CalculateDistanceToTarget(units::time::second_t lookaheadTime)
{
    if (m_chassisPose == m_lastChassisPose)
    {
        return m_cachedDistanceToTarget;
    }

    auto robotPosition = frc::Translation2d{m_chassisPose.X(), m_chassisPose.Y()};

    auto realTarget = GetTargetPosition();
    auto targetPos = (lookaheadTime > 0_s) ? CalculateVirtualTarget(realTarget, lookaheadTime) : realTarget;

    m_cachedDistanceToTarget = robotPosition.Distance(targetPos);
    return m_cachedDistanceToTarget;
}

units::meter_t TargetCalculator::CalculateMechanismDistanceToTarget(units::time::second_t lookaheadTime)
{
    if (m_chassisPose == m_lastChassisPose)
    {
        return m_cachedMechanismDistanceToTarget;
    }

    frc::Translation2d mechanismPos = GetMechanismWorldPosition();

    auto realTarget = GetTargetPosition();
    auto targetPos = (lookaheadTime > 0_s) ? CalculateVirtualTarget(realTarget, lookaheadTime) : realTarget;

    m_cachedMechanismDistanceToTarget = mechanismPos.Distance(targetPos);
    return m_cachedMechanismDistanceToTarget;
}

units::degree_t TargetCalculator::CalculateAngleToTarget(units::time::second_t lookaheadTime)
{
    auto realTarget = GetTargetPosition();
    auto targetPos = (lookaheadTime > 0_s) ? CalculateVirtualTarget(realTarget, lookaheadTime) : realTarget;
    frc::Translation2d vectorToTarget = targetPos - m_chassisPose.Translation();

    return vectorToTarget.Angle().Degrees();
}

units::degree_t TargetCalculator::CalculateMechanismAngleToTarget(units::time::second_t lookaheadTime)
{
    if (m_chassisPose == m_lastChassisPose)
    {
        return m_cachedMechanismAngleToTarget;
    }

    frc::Translation2d mechanismPos = GetMechanismWorldPosition();

    auto realTarget = GetTargetPosition();
    auto targetPos = (lookaheadTime > 0_s) ? CalculateVirtualTarget(realTarget, lookaheadTime) : realTarget;

    frc::Translation2d vectorToTarget = targetPos - mechanismPos;

    m_cachedMechanismAngleToTarget = vectorToTarget.Angle().Degrees();
    return m_cachedMechanismAngleToTarget;
}

void TargetCalculator::SetMechanismOffset(frc::Translation2d offset)
{
    m_mechanismOffset = offset;
}

frc::Pose2d TargetCalculator::GetVirtualTargetPose(
    units::time::second_t lookaheadTime)
{
    auto realTarget = GetTargetPosition();
    return frc::Pose2d{
        CalculateVirtualTarget(realTarget, lookaheadTime),
        frc::Rotation2d{}};
}

void TargetCalculator::UpdateChassisPose(bool forceUpdate)
{
    m_lastChassisPose = m_chassisPose;

    m_isMoving = !m_chassis->IsSamePose();

    // Update the pose while moving, when forced, or on the one cycle after we stop.
    // The "just stopped" cycle ensures the cache busts once so all callers recompute
    // with zero speeds and the virtual-target offset unwinds back to the real target.
    if (m_chassis != nullptr && (forceUpdate || m_isMoving || m_wasMoving))
    {
        m_chassisPose = m_chassis->GetPose();
    }

    m_wasMoving = m_isMoving;
}

void TargetCalculator::UpdateChassisSpeeds()
{
    if (m_chassis != nullptr)
    {
        m_currentChassisSpeeds = m_chassis->GetState().Speeds;
    }
}