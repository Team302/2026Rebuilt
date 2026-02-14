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

#include "fielddata/BumpHelper.h"
#include "auton/NeutralZoneManager.h"
#include "chassis/ChassisConfigMgr.h"
#include "frc/geometry/Pose2d.h"
#include "utils/PoseUtils.h"

BumpHelper *BumpHelper::m_instance = nullptr;

//------------------------------------------------------------------
/// @brief      Get the singleton instance of DepotHelper
/// @return     DepotHelper* - Pointer to the singleton instance
//------------------------------------------------------------------
BumpHelper *BumpHelper::GetInstance()
{
    if (BumpHelper::m_instance == nullptr)
    {
        BumpHelper::m_instance = new BumpHelper();
    }
    return BumpHelper::m_instance;
}

//------------------------------------------------------------------
/// @brief      Constructor for DepotHelper
/// @details    Initializes the chassis and field constants references
///             Used by GetInstance() to create the singleton
//------------------------------------------------------------------
BumpHelper::BumpHelper() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                           m_fieldConstants(FieldConstants::GetInstance())
{
}

BUMP_ID BumpHelper::CalcNearestBump() const
{
    auto chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();
    auto currentPose = (chassis != nullptr) ? chassis->GetPose() : frc::Pose2d();

    // Check whether robot is closer to red or blue side by comparing distance to the depots.
    auto closestDepot = PoseUtils::GetClosestFieldElement(currentPose, FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE, FieldConstants::FIELD_ELEMENT::RED_DEPOT_NEUTRAL_SIDE);
    if (closestDepot == FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE) // closer to blue side
    {
        // Check whether robot is closer to outpost or depot to set the trench pose to the correct one
        auto blueElement = PoseUtils::GetClosestFieldElement(currentPose, FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE, FieldConstants::FIELD_ELEMENT::BLUE_OUTPOST_CENTER);
        if (blueElement == FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE) // blue depot bump
        {
            return BUMP_ID::BLUE_DEPOT_BUMP;
        }
        return BUMP_ID::BLUE_OUTPOST_BUMP;
    }

    // Check whether robot is closer to outpost or depot to set the trench pose to the correct one
    auto redElement = PoseUtils::GetClosestFieldElement(currentPose, FieldConstants::FIELD_ELEMENT::RED_DEPOT_NEUTRAL_SIDE, FieldConstants::FIELD_ELEMENT::RED_OUTPOST_CENTER);
    if (redElement == FieldConstants::FIELD_ELEMENT::RED_DEPOT_NEUTRAL_SIDE) // red depot bump
    {
        return BUMP_ID::RED_DEPOT_BUMP;
    }
    return BUMP_ID::RED_OUTPOST_BUMP;
}

frc::Pose2d BumpHelper::CalcNearestBumpCenter() const
{
    units::length::meter_t centerX = 0_m;
    units::length::meter_t centerY = 0_m;
    units::angle::degree_t rotation = 0_deg;
    frc::Pose2d hubCenterPose{};
    frc::Pose2d trenchCenterPose{};

    auto isInNeutralZone = NeutralZoneManager::GetInstance()->IsInNeutralZone();
    auto chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();
    auto currentPose = (chassis != nullptr) ? chassis->GetPose() : frc::Pose2d();

    // Check whether robot is closer to red or blue side by comparing distance to the depots.
    auto closestDepot = PoseUtils::GetClosestFieldElement(currentPose, FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE, FieldConstants::FIELD_ELEMENT::RED_DEPOT_NEUTRAL_SIDE);
    if (closestDepot == FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE) // closer to blue side
    {
        // set the hub center pose to the blue hub
        hubCenterPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_HUB_CENTER);

        // Check whether robot is closer to outpost or depot to set the trench pose to the correct one
        auto blueElement = PoseUtils::GetClosestFieldElement(currentPose, FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE, FieldConstants::FIELD_ELEMENT::BLUE_OUTPOST_CENTER);
        if (blueElement == FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE) // blue depot bump
        {
            // if in neutral zone and closer to depot, set trench pose to the neutral trench pose
            trenchCenterPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TRENCH_NEUTRAL_DEPOT);
            rotation = (isInNeutralZone) ? 135_deg : 45_deg;
        }
        else // blue outpost bump
        {
            trenchCenterPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TRENCH_ALLIANCE_OUTPOST);
            rotation = (isInNeutralZone) ? 45_deg : 135_deg;
        }
    }
    else
    {
        // set the hub center pose to the red hub
        hubCenterPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_HUB_CENTER);

        // Check whether robot is closer to outpost or depot to set the trench pose to the correct one
        auto redElement = PoseUtils::GetClosestFieldElement(currentPose, FieldConstants::FIELD_ELEMENT::RED_DEPOT_NEUTRAL_SIDE, FieldConstants::FIELD_ELEMENT::RED_OUTPOST_CENTER);
        if (redElement == FieldConstants::FIELD_ELEMENT::RED_DEPOT_NEUTRAL_SIDE) // red depot bump
        {
            // red 0, blue 180
            // if in neutral zone and closer to depot, set trench pose to the neutral trench pose
            trenchCenterPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TRENCH_NEUTRAL_DEPOT);
            rotation = (isInNeutralZone) ? 45_deg : 135_deg;
        }
        else // red outpost bump
        {
            trenchCenterPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TRENCH_ALLIANCE_OUTPOST);
            rotation = (isInNeutralZone) ? 135_deg : 45_deg;
        }
    }

    // calculate the center of the bump by averaging the hub and trench center poses in the Y direction
    // and using the hub center for the X direction
    centerY = (trenchCenterPose.Y() + hubCenterPose.Y()) / 2.0;
    centerX = hubCenterPose.X();
    return frc::Pose2d(centerX, centerY, rotation);
}
