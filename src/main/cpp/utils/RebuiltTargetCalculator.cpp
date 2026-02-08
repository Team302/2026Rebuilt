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
#include "utils/FMSData.h"
#include "utils/PoseUtils.h"
#include "teleopcontrol/TeleopControl.h"
#include "utils/logging/debug/Logger.h"

RebuiltTargetCalculator::RebuiltTargetCalculator() : TargetCalculator()
{
    // TODO: update launcher offsets and target position(need to use zone logic later and field element calculator to get target position)
    SetMechanismOffset(m_mechanismOffset);

    m_field = DragonField::GetInstance();
    m_fieldConstants = FieldConstants::GetInstance();
    m_zoneManager = AllianceZoneManager::GetInstance();

    auto outpostPassingTarget = FieldConstants::FIELD_ELEMENT::BLUE_OUTPOST_PASSING_TARGET;
    auto depotPassingTarget = FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_PASSING_TARGET;

    if (FMSData::GetAllianceColor() == frc::DriverStation::Alliance::kRed)
    {
        outpostPassingTarget = FieldConstants::FIELD_ELEMENT::RED_OUTPOST_PASSING_TARGET;
        depotPassingTarget = FieldConstants::FIELD_ELEMENT::RED_DEPOT_PASSING_TARGET;
    }

    m_field->AddObject("Outpost Passing Target Position", frc::Pose2d(m_fieldConstants->GetFieldElementPose2d(outpostPassingTarget).Translation(), frc::Rotation2d()), true);
    m_field->AddObject("Depot Passing Target Position", frc::Pose2d(m_fieldConstants->GetFieldElementPose2d(depotPassingTarget).Translation(), frc::Rotation2d()), true);
    m_field->AddObject("Current Target Position", frc::Pose2d());
    m_field->AddObject("Launcher Position", frc::Pose2d());
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
    auto alliance = FMSData::GetAllianceColor();

    if (m_fieldConstants != nullptr)
    {
        auto fieldElement = FieldConstants::FIELD_ELEMENT::BLUE_HUB_CENTER;
        if (isInAllianceZone)
        {
            fieldElement = alliance == frc::DriverStation::Alliance::kRed
                               ? FieldConstants::FIELD_ELEMENT::RED_HUB_CENTER
                               : fieldElement;
            targetPosition = m_fieldConstants->GetFieldElementPose2d(fieldElement).Translation();
        }
        else
        {
            auto blueOutpost = FieldConstants::FIELD_ELEMENT::BLUE_OUTPOST_PASSING_TARGET;
            auto blueDepot = FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_PASSING_TARGET;
            auto redOutpost = FieldConstants::FIELD_ELEMENT::RED_OUTPOST_PASSING_TARGET;
            auto redDepot = FieldConstants::FIELD_ELEMENT::RED_DEPOT_PASSING_TARGET;

            if (alliance == frc::DriverStation::Alliance::kBlue)
            {
                fieldElement = PoseUtils::GetClosestFieldElement(GetChassisPose(), blueOutpost, blueDepot);
            }
            else
            {
                fieldElement = PoseUtils::GetClosestFieldElement(GetChassisPose(), redOutpost, redDepot);
            }

            auto xPassingOffset = GetPassingTargetXOffset(fieldElement);
            auto yPassingOffset = GetPassingTargetYOffset(fieldElement);
            targetPosition = m_fieldConstants->GetFieldElementPose2d(fieldElement).Translation() + frc::Translation2d(xPassingOffset, yPassingOffset);
        }
    }

    targetPosition = targetPosition + frc::Translation2d(m_xTargetOffset, m_yTargetOffset);

    return targetPosition;
}

units::angle::degree_t RebuiltTargetCalculator::GetLauncherTarget(units::time::second_t looheadTime, units::angle::degree_t currentLauncherAngle)
{

    m_field->UpdateObject("Current Target Position", GetVirtualTargetPose(looheadTime));

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

    m_field->UpdateObject("Launcher Position", frc::Pose2d(GetMechanismWorldPosition(), robotPose.Rotation() + frc::Rotation2d(bestAngle)));
    return bestAngle;
}

void RebuiltTargetCalculator::UpdateTargetOffset()
{
    auto teleopControl = TeleopControl::GetInstance();
    auto alliance = FMSData::GetAllianceColor();

    if (teleopControl != nullptr)
    {
        bool isUpPressed = teleopControl->IsButtonPressed(TeleopControlFunctions::UPDATE_TARGET_OFFSET_UP);
        bool isDownPressed = teleopControl->IsButtonPressed(TeleopControlFunctions::UPDATE_TARGET_OFFSET_DOWN);
        bool isLeftPressed = teleopControl->IsButtonPressed(TeleopControlFunctions::UPDATE_TARGET_OFFSET_LEFT);
        bool isRightPressed = teleopControl->IsButtonPressed(TeleopControlFunctions::UPDATE_TARGET_OFFSET_RIGHT);

        if (isUpPressed && !m_prevUpPressed)
        {
            m_xTargetOffset += alliance == frc::DriverStation::Alliance::kBlue ? 5_in : -5_in;
        }
        if (isDownPressed && !m_prevDownPressed)
        {
            m_xTargetOffset += alliance == frc::DriverStation::Alliance::kBlue ? -5_in : 5_in;
        }
        if (isLeftPressed && !m_prevLeftPressed)
        {
            m_yTargetOffset += alliance == frc::DriverStation::Alliance::kBlue ? 5_in : -5_in;
        }
        if (isRightPressed && !m_prevRightPressed)
        {
            m_yTargetOffset += alliance == frc::DriverStation::Alliance::kBlue ? -5_in : 5_in;
        }

        m_prevUpPressed = isUpPressed;
        m_prevDownPressed = isDownPressed;
        m_prevLeftPressed = isLeftPressed;
        m_prevRightPressed = isRightPressed;

        // Passing target offsets
        m_passingDepotTargetXOffset += teleopControl->GetAxisValue(TeleopControlFunctions::UPDATE_DEPOT_PASSING_TARGET_X) * (alliance == frc::DriverStation::Alliance::kBlue ? 1_in : -1_in);
        m_passingDepotTargetYOffset += teleopControl->GetAxisValue(TeleopControlFunctions::UPDATE_DEPOT_PASSING_TARGET_Y) * (alliance == frc::DriverStation::Alliance::kBlue ? -1_in : 1_in);
        m_passingOutpostTargetXOffset += teleopControl->GetAxisValue(TeleopControlFunctions::UPDATE_OUTPOST_PASSING_TARGET_X) * (alliance == frc::DriverStation::Alliance::kBlue ? 1_in : -1_in);
        m_passingOutpostTargetYOffset += teleopControl->GetAxisValue(TeleopControlFunctions::UPDATE_OUTPOST_PASSING_TARGET_Y) * (alliance == frc::DriverStation::Alliance::kBlue ? -1_in : 1_in);
    }

    UpdatePassingTargetsOnField();
}

units::length::inch_t RebuiltTargetCalculator::GetPassingTargetXOffset(FieldConstants::FIELD_ELEMENT fieldElement)
{
    auto blueOutpost = FieldConstants::FIELD_ELEMENT::BLUE_OUTPOST_PASSING_TARGET;
    auto redOutpost = FieldConstants::FIELD_ELEMENT::RED_OUTPOST_PASSING_TARGET;
    return (fieldElement == blueOutpost || fieldElement == redOutpost) ? m_passingOutpostTargetXOffset : m_passingDepotTargetXOffset;
}

units::length::inch_t RebuiltTargetCalculator::GetPassingTargetYOffset(FieldConstants::FIELD_ELEMENT fieldElement)
{
    auto blueOutpost = FieldConstants::FIELD_ELEMENT::BLUE_OUTPOST_PASSING_TARGET;
    auto redOutpost = FieldConstants::FIELD_ELEMENT::RED_OUTPOST_PASSING_TARGET;
    return (fieldElement == blueOutpost || fieldElement == redOutpost) ? m_passingOutpostTargetYOffset : m_passingDepotTargetYOffset;
}

void RebuiltTargetCalculator::UpdatePassingTargetsOnField()
{
    frc::Translation2d passingDepotOffset = frc::Translation2d(m_passingDepotTargetXOffset, m_passingDepotTargetYOffset);
    frc::Translation2d passingOutpostOffset = frc::Translation2d(m_passingOutpostTargetXOffset, m_passingOutpostTargetYOffset);

    auto depotPassingTarget = FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_PASSING_TARGET;
    auto outpostPassingTarget = FieldConstants::FIELD_ELEMENT::BLUE_OUTPOST_PASSING_TARGET;

    if (FMSData::GetAllianceColor() == frc::DriverStation::Alliance::kRed)
    {
        depotPassingTarget = FieldConstants::FIELD_ELEMENT::RED_DEPOT_PASSING_TARGET;
        outpostPassingTarget = FieldConstants::FIELD_ELEMENT::RED_OUTPOST_PASSING_TARGET;
    }

    frc::Pose2d depotPose = frc::Pose2d(m_fieldConstants->GetFieldElementPose2d(depotPassingTarget).Translation() + passingDepotOffset, frc::Rotation2d());
    frc::Pose2d outpostPose = frc::Pose2d(m_fieldConstants->GetFieldElementPose2d(outpostPassingTarget).Translation() + passingOutpostOffset, frc::Rotation2d());

    m_field->UpdateObject("Depot Passing Target Position", depotPose);
    m_field->UpdateObject("Outpost Passing Target Position", outpostPose);
}