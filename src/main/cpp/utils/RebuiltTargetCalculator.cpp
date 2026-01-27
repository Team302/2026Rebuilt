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

#include "utils/RebuiltTargetCalculator.h"

RebuiltTargetCalculator::RebuiltTargetCalculator()
{
    // TODO: update launcher offsets and target position(need to use zone logic later and field element calculator to get target position)
    // Mechanism offset initialized in member variable declaration for now

    TargetCalculator();
    SetMechanismOffset(m_mechanismOffset);

    m_field = DragonField::GetInstance();
    m_field->AddPose("TargetPosition", frc::Pose2d(GetTargetPosition(), frc::Rotation2d()));
    m_field->AddPose("LauncherPosition", frc::Pose2d());
}

RebuiltTargetCalculator *RebuiltTargetCalculator::m_instance = nullptr;

RebuiltTargetCalculator *RebuiltTargetCalculator::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new RebuiltTargetCalculator();
    }
    return m_instance;
}

frc::Translation2d RebuiltTargetCalculator::GetTargetPosition()
{
    // TODO: Replace with dynamic target selection based on:
    // - FieldElementCalculator for actual target position
    // - Zone detector for zone-specific targets
    // For now, return hardcoded hub target for testing
    return m_hubTarget;
}

units::angle::degree_t RebuiltTargetCalculator::GetLauncherTarget(units::time::second_t looheadTime, units::angle::degree_t currentLauncherAngle)
{

    m_field->UpdateObject("TargetPosition", GetVirtualTargetPose(looheadTime));

    units::degree_t fieldAngleToTarget = CalculateMechanismAngleToTarget(looheadTime);
    auto robotPose = GetChassisPose();

    frc::Rotation2d relativeRot = frc::Rotation2d(fieldAngleToTarget) - robotPose.Rotation();
    units::degree_t robotRelativeGoal = relativeRot.Degrees();

    units::degree_t bestAngle = 0_deg;
    bool hasFoundValidAngle = false;
    units::degree_t minError = 360_deg; // Extremely large initial value to ensure any valid angle is closer

    for (int i = -1; i <= 1; i++)
    {
        units::degree_t potentialSetpoint = robotRelativeGoal + (360_deg * i);

        if (potentialSetpoint >= m_minLauncherAngle && potentialSetpoint <= m_maxLauncherAngle)
        {
            auto error = units::math::abs(potentialSetpoint - currentLauncherAngle);
            if (error < minError)
            {
                bestAngle = potentialSetpoint;
                minError = error;
                hasFoundValidAngle = true;
            }
        }
    }

    if (!hasFoundValidAngle)
    {
        units::degree_t normalizedGoal = relativeRot.Degrees();
        if (normalizedGoal < 0_deg)
            normalizedGoal += 360_deg;
        bestAngle = std::clamp(normalizedGoal, m_minLauncherAngle, m_maxLauncherAngle);
    }

    m_field->UpdateObject("LauncherPosition", frc::Pose2d(GetMechanismWorldPosition(), robotPose.Rotation() + frc::Rotation2d(bestAngle)));
    return bestAngle;
}