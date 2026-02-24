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
#include "fieldData/FieldOffsetValues.h"
#include "fielddata/FieldConstants.h"

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
    // refrefnce depothelper
    {
        return frc::Pose2d();
    }
    // red or blue and then if its in alliance zone then do the logic if not then return 00 / origin pose
    auto fieldConstants = FieldConstants::GetInstance();

    if (FMSData::GetAllianceColor() == frc::DriverStation::Alliance::kRed)
    {
        // auto fieldVals = FieldOffsetValues::GetInstance();

        auto closestFieldElement = PoseUtils::GetClosestFieldElement(m_chassis->GetPose(), FieldConstants::FIELD_ELEMENT::RED_TOWER_DEPOT_STICK, FieldConstants::FIELD_ELEMENT::RED_TOWER_OUTPOST_STICK);
        if (closestFieldElement == FieldConstants::FIELD_ELEMENT::RED_TOWER_DEPOT_STICK)
        {
            auto x = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER).X() + m_towerDepotXOffset;
            auto y = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER).Y() + m_towerDepotYOffset;
            auto angle = units::angle::degree_t(0.0);
            return frc::Pose2d(x, y, angle);
        }
        else
        {
            auto x = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER).X() + m_towerOutpostXOffset;
            auto y = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER).Y() - m_towerOutpostYOffset;
            auto angle = units::angle::degree_t(0.0);
            return frc::Pose2d(x, y, angle);
        }

        // equasion to position robot correctly with offset
    }
    else // blue
    {
        auto closestFieldElement = PoseUtils::GetClosestFieldElement(m_chassis->GetPose(), FieldConstants::FIELD_ELEMENT::BLUE_TOWER_DEPOT_STICK, FieldConstants::FIELD_ELEMENT::BLUE_TOWER_OUTPOST_STICK);
        if (closestFieldElement == FieldConstants::FIELD_ELEMENT::BLUE_TOWER_DEPOT_STICK)
        {
            auto x = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER).X() - m_towerDepotXOffset;
            auto y = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER).Y() - m_towerDepotYOffset;
            auto angle = units::angle::degree_t(0.0);
            return frc::Pose2d(x, y, angle);
        }
        else
        {
            auto x = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER).X() - m_towerOutpostXOffset;
            auto y = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER).Y() + m_towerOutpostYOffset;
            auto angle = units::angle::degree_t(0.0);
            return frc::Pose2d(x, y, angle);
        }
    }

    // if (AllianceZoneManager::GetInstance()->IsInAllianceZone())
    // changed fieldConstants to FieldOffsetValues
    // {
    //     if (DriverStation::GetAlliance() == DriverStation::Alliance::kRed)
    //     {
    //         auto [redDepotPose, redOutpostPose] = std::pair{
    //             m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_DEPOT_STICK),
    //             m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_OUTPOST_STICK)};

    //         auto [distanceToDepot] = CalcDistanceToObject(FieldConstants::FIELD_ELEMENT::RED_TOWER_DEPOT_STICK, m_chassis->GetPose());
    //         auto [distanceToOutpost] = CalcDistanceToObject(FieldConstants::FIELD_ELEMENT::RED_TOWER_OUTPOST_STICK, m_chassis->GetPose());

    //         if (distanceToDepot < distanceToOutpost)
    //         {
    //             return frc::Pose2d(redDepotPose.X() - m_towerDepotXOffset, redDepotPose.Y() - m_towerDepotYOffset, units::angle::degree_t(0.0));
    //         }
    //         else
    //         {
    //             return frc::Pose2d(redOutpostPose.X() + m_towerOutpostXOffset, redOutpostPose.Y() + m_towerOutpostYOffset, units::angle::degree_t(0.0));
    //         }
    //     }
    //     else
    //     {
    //         auto blueDepotPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_DEPOT_STICK);
    //         auto blueOutpostPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_OUTPOST_STICK);

    //         // call PoseUtils::GetClosestFieldElement
    //         // call fieldoffsetutils to get pose
    //         PoseUtils::GetClosestFieldElement(m_chassis->GetPose(), FieldConstants::FIELD_ELEMENT::BLUE_TOWER_DEPOT_STICK, FieldConstants::FIELD_ELEMENT::BLUE_TOWER_OUTPOST_STICK);
    //     }
    // }

    // return frc::Pose2d(neutralPose.X(), neutralPose.Y(), isNearestTowerRed ? 0_deg : 180_deg);
    // neutralPose X accounts for half the robot on the intake side + bumpers + agitator/intake being extended
    // neutralPose Y is center of the depot - no need to average with the side values
    // rotation is based on the color
    // if (DriverStation::GetAlliance() == DriverStation::Alliance::kblue)
    //     PoseUtils::GetClosestFieldElement(m_chassis->GetPose(), FieldConstants::FIELD_ELEMENT::BLUE_TOWER_DEPOT_STICK, FieldConstants::FIELD_ELEMENT::BLUE_TOWER_OUTPOST_STICK);
    // auto fieldOffsetValues = FieldOffsetValues::GetInstance();
    // auto x = fieldOffsetValues->GetValue(true, FIELD_OFFSET_ITEMS::TOWER_OUTPOST_X);
    // auto y = fieldOffsetValues->GetValue(true, FIELD_OFFSET_ITEMS::TOWER_OUTPOST_Y);
    // auto x = fieldOffsetValues->GetValue(true, FIELD_OFFSET_ITEMS::TOWER_DEPOT_X);
    // auto y = fieldOffsetValues->GetValue(true, FIELD_OFFSET_ITEMS::TOWER_DEPOT_Y);
    // return frc::Pose2d(x, y, isRed ? 0_deg : 180_deg);

    // NEED TO REF PHOTO TAKEN AND PUT IN VALUES "TOWER_OUTPOST_X,  TOWER_DEPOT_X, TOWER_OUTPOST_Y, TOWER_DEPOT_Y, "i think I did this
}

//------------------------------------------------------------------
/// @brief      Calculates the distance from a given pose to a field element
/// @param[in]  element - The field element to measure distance to
/// @param[in]  currentPose - The pose to measure distance from
/// @return     units::length::meter_t - The distance in meters
/// @details    Uses the translation components of both poses to calculate
///             the Euclidean distance between them
//------------------------------------------------------------------
// units::length::meter_t DriveToTowerHelper::CalcDistanceToObject(FieldConstants::FIELD_ELEMENT element,
//                                                                 frc::Pose2d currentPose) const
// {
//     if (m_fieldConstants == nullptr)
//     {
//         return units::length::meter_t(0.0);
//     }
//     return PoseUtils::GetDeltaBetweenPoses(currentPose, m_fieldConstants->GetFieldElementPose2d(element)); // current pose m_chassis -> getvalue, inline for red or blue tower
// }
