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

#include "fielddata/TowerHelper.h"
#include "chassis/ChassisConfigMgr.h"
#include "fielddata/FieldConstants.h"
#include "fielddata/FieldOffsetValues.h"
#include "frc/geometry/Pose2d.h"
#include "utils/FMSData.h"

using frc::DriverStation;
TowerHelper *TowerHelper::m_instance = nullptr;

//------------------------------------------------------------------
/// @brief      Get the singleton instance of TowerHelper
/// @return     TowerHelper* - Pointer to the singleton instance
//------------------------------------------------------------------
TowerHelper *TowerHelper::GetInstance()
{
    if (TowerHelper::m_instance == nullptr)
    {
        TowerHelper::m_instance = new TowerHelper();
    }
    return TowerHelper::m_instance;
}

//------------------------------------------------------------------
/// @brief      Constructor for TowerHelper
/// @details    Initializes the chassis and field constants references
///             Used by GetInstance() to create the singleton
//------------------------------------------------------------------
TowerHelper::TowerHelper() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
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
frc::Pose2d TowerHelper::CalcTowerPose() const
{
    if (m_chassis == nullptr || m_fieldConstants == nullptr)
    {
        return frc::Pose2d();
    }

    auto fieldVals = FieldOffsetValues::GetInstance();
    if (fieldVals == nullptr)
    {
        return frc::Pose2d();
    }

    // Use cached alliance color instead of re-querying FMSData each call
    auto currentPose = m_chassis->GetPose();
    auto isRed = (m_allianceColor == frc::DriverStation::Alliance::kRed);

    if (isRed)
    {
        // Use cached m_fieldConstants for distance calculations instead of going through PoseUtils
        auto depotPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_DEPOT_STICK);
        auto outpostPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_OUTPOST_STICK);
        auto distToDepot = currentPose.Translation().Distance(depotPose.Translation());
        auto distToOutpost = currentPose.Translation().Distance(outpostPose.Translation());

        if (distToDepot < distToOutpost)
        {
            return frc::Pose2d(fieldVals->GetValue(true, FIELD_OFFSET_ITEMS::TOWER_DEPOT_X),
                               fieldVals->GetValue(true, FIELD_OFFSET_ITEMS::TOWER_DEPOT_Y),
                               units::angle::degree_t(0.0));
        }
        else
        {
            return frc::Pose2d(fieldVals->GetValue(true, FIELD_OFFSET_ITEMS::TOWER_OUTPOST_X),
                               fieldVals->GetValue(true, FIELD_OFFSET_ITEMS::TOWER_OUTPOST_Y),
                               units::angle::degree_t(0.0));
        }
    }
    else // blue
    {
        auto depotPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_DEPOT_STICK);
        auto outpostPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_OUTPOST_STICK);
        auto distToDepot = currentPose.Translation().Distance(depotPose.Translation());
        auto distToOutpost = currentPose.Translation().Distance(outpostPose.Translation());

        if (distToDepot < distToOutpost)
        {
            return frc::Pose2d(fieldVals->GetValue(false, FIELD_OFFSET_ITEMS::TOWER_DEPOT_X),
                               fieldVals->GetValue(false, FIELD_OFFSET_ITEMS::TOWER_DEPOT_Y),
                               units::angle::degree_t(0.0));
        }
        else
        {
            return frc::Pose2d(fieldVals->GetValue(false, FIELD_OFFSET_ITEMS::TOWER_OUTPOST_X),
                               fieldVals->GetValue(false, FIELD_OFFSET_ITEMS::TOWER_OUTPOST_Y),
                               units::angle::degree_t(0.0));
        }
    }
}

//------------------------------------------------------------------
/// @brief      Calculates the distance from a given pose to a field element
/// @param[in]  element - The field element to measure distance to
/// @param[in]  currentPose - The pose to measure distance from
/// @return     units::length::meter_t - The distance in meters
/// @details    Uses the translation components of both poses to calculate
///             the Euclidean distance between them
//------------------------------------------------------------------
// units::length::meter_t TowerHelper::CalcDistanceToObject(FieldConstants::FIELD_ELEMENT element,
//                                                                 frc::Pose2d currentPose) const
// {
//     if (m_fieldConstants == nullptr)
//     {
//         return units::length::meter_t(0.0);
//     }
//     return PoseUtils::GetDeltaBetweenPoses(currentPose, m_fieldConstants->GetFieldElementPose2d(element)); // current pose m_chassis -> getvalue, inline for red or blue tower
// }
