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
#include "teleopcontrol/TeleopControl.h"
#include "utils/FMSData.h"
#include "utils/PoseUtils.h"
#include "utils/logging/debug/Logger.h"

// Static string constants — constructed once, never copied on each call
const std::string RebuiltTargetCalculator::kOutpostPassingTargetName = "Outpost Passing Target Position";
const std::string RebuiltTargetCalculator::kDepotPassingTargetName = "Depot Passing Target Position";
const std::string RebuiltTargetCalculator::kCurrentTargetName = "Current Target Position";
const std::string RebuiltTargetCalculator::kLauncherPositionName = "Launcher Position";

RebuiltTargetCalculator::RebuiltTargetCalculator() : TargetCalculator()
{
    SetMechanismOffset(m_mechanismOffset);

    m_field = DragonField::GetInstance();
    m_fieldConstants = FieldConstants::GetInstance();
    m_zoneManager = AllianceZoneManager::GetInstance();

    // Cache alliance-specific field elements and positions once
    m_cachedAlliance = FMSData::GetAllianceColor();
    RefreshAllianceCache();

    m_field->AddObject(kOutpostPassingTargetName, frc::Pose2d(m_outpostPassingPosition, frc::Rotation2d()), true);
    m_field->AddObject(kDepotPassingTargetName, frc::Pose2d(m_depotPassingPosition, frc::Rotation2d()), true);
    m_field->AddObject(kCurrentTargetName, frc::Pose2d());
    m_field->AddObject(kLauncherPositionName, frc::Pose2d());
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
    bool isInAllianceZone = m_zoneManager->IsInAllianceZone();
    frc::Translation2d targetPosition{};

    if (m_fieldConstants != nullptr)
    {
        if (isInAllianceZone)
        {
            // Use pre-cached hub center position
            targetPosition = m_hubCenterPosition;
        }
        else
        {
            // Determine closest passing target using cached enum values
            auto fieldElement = PoseUtils::GetClosestFieldElement(GetChassisPose(), m_outpostPassingTarget, m_depotPassingTarget);

            auto xPassingOffset = GetPassingTargetXOffset(fieldElement);
            auto yPassingOffset = GetPassingTargetYOffset(fieldElement);

            // Use pre-cached base positions instead of looking them up each cycle
            auto &basePosition = (fieldElement == m_outpostPassingTarget) ? m_outpostPassingPosition : m_depotPassingPosition;
            targetPosition = basePosition + frc::Translation2d(xPassingOffset, yPassingOffset);
        }
    }

    targetPosition = targetPosition + frc::Translation2d(m_xTargetOffset, m_yTargetOffset);

    return targetPosition;
}

units::angle::turn_t RebuiltTargetCalculator::GetLauncherTarget(units::time::second_t looheadTime, units::angle::degree_t currentLauncherAngle)
{
    m_field->UpdateObject(kCurrentTargetName, GetVirtualTargetPose(looheadTime));

    units::degree_t fieldAngleToTarget = CalculateMechanismAngleToTarget(looheadTime);
    auto robotPose = GetChassisPose();

    frc::Rotation2d relativeRot = frc::Rotation2d(fieldAngleToTarget) - robotPose.Rotation();
    units::degree_t robotRelativeGoal = relativeRot.Degrees();

    units::degree_t bestAngle = 0_deg;
    bool hasFoundValidAngle = false;
    units::degree_t minError = 360_deg;

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

    m_field->UpdateObject(kLauncherPositionName, frc::Pose2d(GetMechanismWorldPosition(), robotPose.Rotation() + frc::Rotation2d(bestAngle)));
    return units::angle::turn_t(bestAngle.value());
}

void RebuiltTargetCalculator::UpdateTargetOffset()
{
    auto teleopControl = TeleopControl::GetInstance();

    // Refresh cached alliance data if alliance has changed
    auto currentAlliance = FMSData::GetAllianceColor();
    if (currentAlliance != m_cachedAlliance)
    {
        m_cachedAlliance = currentAlliance;
        RefreshAllianceCache();
    }

    bool isBlue = (m_cachedAlliance == frc::DriverStation::Alliance::kBlue);

    if (teleopControl != nullptr)
    {
        bool isUpPressed = teleopControl->IsButtonPressed(TeleopControlFunctions::UPDATE_TARGET_OFFSET_UP);
        bool isDownPressed = teleopControl->IsButtonPressed(TeleopControlFunctions::UPDATE_TARGET_OFFSET_DOWN);
        bool isLeftPressed = teleopControl->IsButtonPressed(TeleopControlFunctions::UPDATE_TARGET_OFFSET_LEFT);
        bool isRightPressed = teleopControl->IsButtonPressed(TeleopControlFunctions::UPDATE_TARGET_OFFSET_RIGHT);

        auto sign5in = isBlue ? 5_in : -5_in;

        if (isUpPressed && !m_prevUpPressed)
        {
            m_xTargetOffset += sign5in;
        }
        if (isDownPressed && !m_prevDownPressed)
        {
            m_xTargetOffset -= sign5in;
        }
        if (isLeftPressed && !m_prevLeftPressed)
        {
            m_yTargetOffset += sign5in;
        }
        if (isRightPressed && !m_prevRightPressed)
        {
            m_yTargetOffset -= sign5in;
        }

        m_prevUpPressed = isUpPressed;
        m_prevDownPressed = isDownPressed;
        m_prevLeftPressed = isLeftPressed;
        m_prevRightPressed = isRightPressed;

        // Passing target offsets — use cached alliance sign
        auto xSign = isBlue ? 1_in : -1_in;
        auto ySign = isBlue ? -1_in : 1_in;
        m_passingDepotTargetXOffset += teleopControl->GetAxisValue(TeleopControlFunctions::UPDATE_DEPOT_PASSING_TARGET_X) * xSign;
        m_passingDepotTargetYOffset += teleopControl->GetAxisValue(TeleopControlFunctions::UPDATE_DEPOT_PASSING_TARGET_Y) * ySign;
        m_passingOutpostTargetXOffset += teleopControl->GetAxisValue(TeleopControlFunctions::UPDATE_OUTPOST_PASSING_TARGET_X) * xSign;
        m_passingOutpostTargetYOffset += teleopControl->GetAxisValue(TeleopControlFunctions::UPDATE_OUTPOST_PASSING_TARGET_Y) * ySign;
    }

    UpdatePassingTargetsOnField();
}

units::length::inch_t RebuiltTargetCalculator::GetPassingTargetXOffset(FieldConstants::FIELD_ELEMENT fieldElement)
{
    return (fieldElement == m_outpostPassingTarget) ? m_passingOutpostTargetXOffset : m_passingDepotTargetXOffset;
}

units::length::inch_t RebuiltTargetCalculator::GetPassingTargetYOffset(FieldConstants::FIELD_ELEMENT fieldElement)
{
    return (fieldElement == m_outpostPassingTarget) ? m_passingOutpostTargetYOffset : m_passingDepotTargetYOffset;
}

void RebuiltTargetCalculator::UpdatePassingTargetsOnField()
{
    frc::Translation2d passingDepotOffset = frc::Translation2d(m_passingDepotTargetXOffset, m_passingDepotTargetYOffset);
    frc::Translation2d passingOutpostOffset = frc::Translation2d(m_passingOutpostTargetXOffset, m_passingOutpostTargetYOffset);

    // Use pre-cached base positions instead of looking them up each cycle
    frc::Pose2d depotPose = frc::Pose2d(m_depotPassingPosition + passingDepotOffset, frc::Rotation2d());
    frc::Pose2d outpostPose = frc::Pose2d(m_outpostPassingPosition + passingOutpostOffset, frc::Rotation2d());

    m_field->UpdateObject(kDepotPassingTargetName, depotPose);
    m_field->UpdateObject(kOutpostPassingTargetName, outpostPose);
}

void RebuiltTargetCalculator::RefreshAllianceCache()
{
    bool isRed = (m_cachedAlliance == frc::DriverStation::Alliance::kRed);

    m_hubCenter = isRed ? FieldConstants::FIELD_ELEMENT::RED_HUB_CENTER
                        : FieldConstants::FIELD_ELEMENT::BLUE_HUB_CENTER;
    m_outpostPassingTarget = isRed ? FieldConstants::FIELD_ELEMENT::RED_OUTPOST_PASSING_TARGET
                                   : FieldConstants::FIELD_ELEMENT::BLUE_OUTPOST_PASSING_TARGET;
    m_depotPassingTarget = isRed ? FieldConstants::FIELD_ELEMENT::RED_DEPOT_PASSING_TARGET
                                 : FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_PASSING_TARGET;

    if (m_fieldConstants != nullptr)
    {
        m_hubCenterPosition = m_fieldConstants->GetFieldElementPose2d(m_hubCenter).Translation();
        m_outpostPassingPosition = m_fieldConstants->GetFieldElementPose2d(m_outpostPassingTarget).Translation();
        m_depotPassingPosition = m_fieldConstants->GetFieldElementPose2d(m_depotPassingTarget).Translation();
    }
}