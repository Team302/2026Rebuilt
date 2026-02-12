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

#include "DriveToTowerHelper.h"
#include "chassis/ChassisConfigMgr.h"
#include "frc/geometry/Pose2d.h"
#include "utils/PoseUtils.h"
#include "utils/FMSData.h"
#include "auton/AllianceZoneManager.h"

using frc::DriverStation;
DriveToTowerHelper *DriveToTowerHelper::m_instance = nullptr;

//------------------------------------------------------------------
/// @brief      Get the singleton instance of DriveToTowerHelper
/// @return     DRiveToTowerHelper* - Pointer to the singleton instance
//------------------------------------------------------------------
DriveToTowerHelper *DriveToTowerHelper::GetInstance()
{
    if (DriveToTowerHelper::m_instance == nullptr)
    {
        DriveToTowerHelper::m_instance = new DriveToTowerHelper();
    }
    return DriveToTowerHelper::m_instance;
}

//------------------------------------------------------------------
/// @brief      Constructor for DriveToTowerHelper
/// @details    Initializes the chassis and field constants references
///             Used by GetInstance() to create the singleton
//------------------------------------------------------------------
DriveToTowerHelper::DriveToTowerHelper() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                                           m_fieldConstants(FieldConstants::GetInstance())
{
    m_allianceColor = FMSData::GetAllianceColor();
}

//------------------------------------------------------------------
/// @brief      Calculates the center pose of the nearest Tower
/// @return     frc::Pose2d - The calculated center pose of the Tower
/// @details    Determines which Tower (red or blue) is nearest, then
///             calculates the center point by averaging the X and Y coordinates
///             of the left, right, and neutral side poses. Uses the neutral
///             side's rotation for the resulting pose orientation.
//------------------------------------------------------------------

frc::Pose2d DriveToTowerHelper::CalcTowerPose() const
{
    if (m_chassis == nullptr || m_fieldConstants == nullptr)
    {
        return frc::Pose2d();
    }

    auto isNearestTowerRed = IsNearestTowerRed();
    auto neutralPose = isNearestTowerRed ? m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER)
                                         : m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER);
    if (IsNearestTowerRed())
    {
        auto closestFieldElement = PoseUtils::GetClosestFieldElement(neutralPose, FieldConstants::FIELD_ELEMENT::RED_TOWER_DEPOT_STICK, FieldConstants::FIELD_ELEMENT::RED_TOWER_DEPOT_STICK);
        if (closestFieldElement == FieldConstants::FIELD_ELEMENT::RED_TOWER_DEPOT_STICK)
        {
            return frc::Pose2d(neutralPose.X() - m_towerDepotXOffset, neutralPose.Y() - m_towerDepotYOffset, units::angle::degree_t(0.0));
        }
        else if (closestFieldElement == FieldConstants::FIELD_ELEMENT::RED_TOWER_OUTPOST_STICK)
        {
            return frc::Pose2d(neutralPose.X() + m_towerOutpostXOffset, neutralPose.Y() + m_towerOutpostYOffset, units::angle::degree_t(0.0));
        }

        // equasion to position robot correctly with offset
    }
    else
    {
        auto closestFieldElement = PoseUtils::GetClosestFieldElement(neutralPose, FieldConstants::FIELD_ELEMENT::BLUE_TOWER_DEPOT_STICK, FieldConstants::FIELD_ELEMENT::BLUE_TOWER_DEPOT_STICK);
        if (closestFieldElement == FieldConstants::FIELD_ELEMENT::BLUE_TOWER_DEPOT_STICK)
        {
            return frc::Pose2d(neutralPose.X() - m_towerDepotXOffset, neutralPose.Y() - m_towerDepotYOffset, units::angle::degree_t(0.0));
        }
        else if (closestFieldElement == FieldConstants::FIELD_ELEMENT::BLUE_TOWER_OUTPOST_STICK)
        {
            return frc::Pose2d(neutralPose.X() + m_towerOutpostXOffset, neutralPose.Y() + m_towerOutpostYOffset, units::angle::degree_t(0.0));
        }
    }

    if (AllianceZoneManager::GetInstance()->IsInAllianceZone())
        ;
    {
        if (DriverStation::GetAlliance() == DriverStation::Alliance::kRed)
        {
            auto redDepotPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_DEPOT_STICK);
            auto redOutpostPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_OUTPOST_STICK);
            auto distanceToDepot = CalcDistanceToObject(FieldConstants::FIELD_ELEMENT::RED_TOWER_DEPOT_STICK, neutralPose);
            auto distanceToOutpost = CalcDistanceToObject(FieldConstants::FIELD_ELEMENT::RED_TOWER_OUTPOST_STICK, neutralPose);
            if (distanceToDepot < distanceToOutpost)
            {
                return frc::Pose2d(redDepotPose.X() - m_towerDepotXOffset, redDepotPose.Y() - m_towerDepotYOffset, units::angle::degree_t(0.0));
            }
            else
            {
                return frc::Pose2d(redOutpostPose.X() + m_towerOutpostXOffset, redOutpostPose.Y() + m_towerOutpostYOffset, units::angle::degree_t(0.0));
            }
        }
        else
        {
            auto blueDepotPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_DEPOT_STICK);
            return frc::Pose2d(blueDepotPose.X() - m_towerDepotXOffset, blueDepotPose.Y() - m_towerDepotYOffset, units::angle::degree_t(0.0));
        }
        else
        {
            return frc::Pose2d(blueOutpostPose.X() + m_towerOutpostXOffset, blueOutpostPose.Y() + m_towerOutpostYOffset, units::angle::degree_t(0.0));
        }
    }

    // return frc::Pose2d(neutralPose.X(), neutralPose.Y(), isNearestTowerRed ? 0_deg : 180_deg);

    // neutralPose X accounts for half the robot on the intake side + bumpers + agitator/intake being extended
    // neutralPose Y is center of the depot - no need to average with the side values
    // rotation is based on the color
}

//------------------------------------------------------------------
/// @brief      Calculates the distance from a given pose to a field element
/// @param[in]  element - The field element to measure distance to
/// @param[in]  currentPose - The pose to measure distance from
/// @return     units::length::meter_t - The distance in meters
/// @details    Uses the translation components of both poses to calculate
///             the Euclidean distance between them
//------------------------------------------------------------------
units::length::meter_t DriveToTowerHelper::CalcDistanceToObject(FieldConstants::FIELD_ELEMENT element,
                                                                frc::Pose2d currentPose)
{
    if (m_fieldConstants == nullptr)
    {
        return units::length::meter_t(0.0);
    }
    return PoseUtils::GetDeltaBetweenPoses(currentPose, m_fieldConstants->GetFieldElementPose2d(element));
}
