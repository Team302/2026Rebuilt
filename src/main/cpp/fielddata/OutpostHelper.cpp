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

#include "fielddata/OutpostHelper.h"
#include "chassis/ChassisConfigMgr.h"
#include "frc/geometry/Pose2d.h"

//------------------------------------------------------------------
/// @brief      Get the singleton instance of OutpostHelper
/// @return     OutpostHelper* - Pointer to the singleton instance
//------------------------------------------------------------------
OutpostHelper *OutpostHelper::m_instance = nullptr;
OutpostHelper *OutpostHelper::GetInstance()
{
    if (OutpostHelper::m_instance == nullptr)
    {
        OutpostHelper::m_instance = new OutpostHelper();
    }
    return OutpostHelper::m_instance;
}

//------------------------------------------------------------------
/// @brief      Constructor for OutpostHelper
/// @details    Initializes the chassis and field constants references
///             Used by GetInstance() to create the singleton
//------------------------------------------------------------------
OutpostHelper::OutpostHelper() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                                 m_fieldConstants(FieldConstants::GetInstance())
{
}

//------------------------------------------------------------------
/// @brief      Determines which Outpost (red or blue) is nearest to the robot
/// @return     bool - true if the red Outpost is nearest, false if blue Outpost is nearest
/// @details    Calculates the distance from the robot's current pose to both
///             the blue and red Outpost neutral sides, then compares them
//------------------------------------------------------------------
bool OutpostHelper::IsNearestOutpostRed() const
{
    auto currentPose = m_chassis->GetPose();

    auto blueOutpost = FieldConstants::FIELD_ELEMENT::BLUE_Outpost_NEUTRAL_SIDE;
    auto redOutpost = FieldConstants::FIELD_ELEMENT::RED_Outpost_NEUTRAL_SIDE;

    auto blueDistance = CalcDistanceToObject(blueOutpost, currentPose);
    auto redDistance = CalcDistanceToObject(redOutpost, currentPose);
    if (blueDistance < redDistance)
    {
        return false;
    }
    return true;
}

//------------------------------------------------------------------
/// @brief      Calculates the center pose of the nearest Outpost
/// @return     frc::Pose2d - The calculated center pose of the Outpost
/// @details    Determines which Outpost (red or blue) is nearest, then
///             calculates the center point by averaging the X and Y coordinates
///             of the left, right, and neutral side poses. Uses the neutral
///             side's rotation for the resulting pose orientation.
//------------------------------------------------------------------
frc::Pose2d OutpostHelper::CalcOutpostPose() const
{
    auto isNearestOutpostRed = IsNearestOutpostRed();
    auto leftPose = isNearestOutpostRed ? m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_Outpost_LEFT_SIDE)
                                        : m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_Outpost_LEFT_SIDE);
    auto rightPose = isNearestOutpostRed ? m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_Outpost_RIGHT_SIDE)
                                         : m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_Outpost_RIGHT_SIDE);
    auto neutralPose = isNearestOutpostRed ? m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_Outpost_NEUTRAL_SIDE)
                                           : m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_Outpost_NEUTRAL_SIDE);
    auto centerX = (leftPose.X() + rightPose.X() + neutralPose.X()) / 3.0;
    auto centerY = (leftPose.Y() + rightPose.Y() + neutralPose.Y()) / 3.0;
    return frc::Pose2d(units::meter_t(centerX), units::meter_t(centerY), neutralPose.Rotation());
}

//------------------------------------------------------------------
/// @brief      Calculates the distance from a given pose to a field element
/// @param[in]  element - The field element to measure distance to
/// @param[in]  currentPose - The pose to measure distance from
/// @return     units::length::meter_t - The distance in meters
/// @details    Uses the translation components of both poses to calculate
///             the Euclidean distance between them
//------------------------------------------------------------------
units::length::meter_t OutpostHelper::CalcDistanceToObject(FieldConstants::FIELD_ELEMENT element,
                                                           frc::Pose2d currentPose) const
{
    return currentPose.Translation().Distance(m_fieldConstants->GetFieldElementPose2d(element).Translation());
}