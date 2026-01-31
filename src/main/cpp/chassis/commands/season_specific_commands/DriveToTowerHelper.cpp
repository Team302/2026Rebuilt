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

#include "chassis/commands/season_specific_commands/DriveToTowerHelper.h"
#include "chassis/ChassisConfigMgr.h"
#include "frc/geometry/Pose2d.h"
#include "utils/PoseUtils.h"

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
}

//------------------------------------------------------------------
/// @brief      Determines which Tower (red or blue) is nearest to the robot
/// @return     bool - true if the red Tower is nearest, false if blue Tower is nearest
/// @details    Calculates the distance from the robot's current pose to both
///             the blue and red Tower neutral sides, then compares them
//------------------------------------------------------------------
bool DriveToTowerHelper::IsNearestTowerRed() const
{
    if (m_chassis == nullptr || m_fieldConstants == nullptr)
    {
        return false;
    }

    auto currentPose = m_chassis->GetPose();

    auto blueTower = FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER;
    auto redTower = FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER;

    auto blueDistance = CalcDistanceToObject(blueTower, currentPose);
    auto redDistance = CalcDistanceToObject(redTower, currentPose);
    if (blueDistance < redDistance)
    {
        return false;
    }
    return true;
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

    // neutralPose X accounts for half the robot on the intake side + bumpers + agitator/intake being extended
    // neutralPose Y is center of the depot - no need to average with the side values
    // rotation is based on the color
    return frc::Pose2d(neutralPose.X(), neutralPose.Y(), isNearestTowerRed ? 0_deg : 180_deg);
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
                                                                frc::Pose2d currentPose) const
{
    if (m_fieldConstants == nullptr)
    {
        return units::length::meter_t(0.0);
    }
    return PoseUtils::GetDeltaBetweenPoses(currentPose, m_fieldConstants->GetFieldElementPose2d(element));
}