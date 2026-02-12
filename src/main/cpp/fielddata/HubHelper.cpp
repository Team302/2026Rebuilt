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

#include "fielddata/HubHelper.h"
#include "chassis/ChassisConfigMgr.h"
#include "fielddata/FieldOffsetValues.h"

#include "frc/geometry/Pose2d.h"

//------------------------------------------------------------------
/// @brief      Get the singleton instance of HubHelper
/// @return     HubHelper* - Pointer to the singleton instance
//------------------------------------------------------------------
HubHelper *HubHelper::m_instance = nullptr;
HubHelper *HubHelper::GetInstance()
{
    if (HubHelper::m_instance == nullptr)
    {
        HubHelper::m_instance = new HubHelper();
    }
    return HubHelper::m_instance;
}

//------------------------------------------------------------------
/// @brief      Private constructor for HubHelper
/// @details    Initializes the chassis and field constants references.
///             Called by GetInstance() to create the singleton instance.
///             The constructor initializes member pointers to the swerve
///             drivetrain subsystem and field constants.
//------------------------------------------------------------------
HubHelper::HubHelper() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                         m_fieldConstants(FieldConstants::GetInstance())
{
}

//------------------------------------------------------------------
/// @brief      Determines which Hub (red or blue) is nearest to the robot
/// @return     bool - true if the red Hub is nearest, false if blue Hub is nearest
/// @details    Calculates the distance from the robot's current pose to both
///             the blue and red Hub centers, then compares them to
///             determine which Hub is closest to the robot
//------------------------------------------------------------------
bool HubHelper::IsNearestHubRed() const
{
    if (m_chassis == nullptr || m_fieldConstants == nullptr)
    {
        return false;
    }

    auto currentPose = m_chassis->GetPose();

    auto blueDistance = CalcDistanceToObject(FieldConstants::FIELD_ELEMENT::BLUE_HUB_ALLIANCE_CENTER, currentPose);
    auto redDistance = CalcDistanceToObject(FieldConstants::FIELD_ELEMENT::RED_HUB_ALLIANCE_CENTER, currentPose);
    return (redDistance < blueDistance);
}

//------------------------------------------------------------------
/// @brief      Calculates the center pose of the nearest Hub
/// @return     frc::Pose2d - The calculated center pose of the Hub
/// @details    Determines which Hub (red or blue) is nearest to the robot.
///             Calculates the center pose using:
///             - The Hub center position from field constants
///             - Rotation based on the nearest alliance (0° for red, 180° for blue)
//------------------------------------------------------------------
frc::Pose2d HubHelper::CalcHubPose() const
{
    if (m_chassis == nullptr || m_fieldConstants == nullptr)
    {
        return frc::Pose2d();
    }

    auto isNearestHubRed = IsNearestHubRed();

    auto hubPose = isNearestHubRed ? m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_HUB_CENTER)
                                   : m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_HUB_CENTER);

    // Get the X position from the FieldOffsetValues based on nearest hub color
    // hubPose Y is center of the hub
    auto fieldOffsetValues = FieldOffsetValues::GetInstance();
    if (fieldOffsetValues == nullptr)
    {
        // Fallback: use the hubpose directly if FieldOffsetValues is unavailable
        return hubPose;
    }
    return frc::Pose2d(fieldOffsetValues->GetXValue(isNearestHubRed, FIELD_OFFSET_ITEMS::HUB_X), hubPose.Y(), isNearestHubRed ? 0_deg : 180_deg);
}

//------------------------------------------------------------------
/// @brief      Calculates the distance from a given pose to a field element
/// @param[in]  element - The field element to calculate distance to
/// @param[in]  currentPose - The pose to measure distance from
/// @return     units::length::meter_t - The distance in meters
/// @details    Computes the Euclidean distance between the translation
///             components of the current pose and the field element pose.
///             This is a helper method used internally for distance comparisons.
//------------------------------------------------------------------
units::length::meter_t HubHelper::CalcDistanceToObject(FieldConstants::FIELD_ELEMENT element,
                                                       frc::Pose2d currentPose) const
{
    if (m_fieldConstants == nullptr)
    {
        return units::length::meter_t(0.0);
    }
    return currentPose.Translation().Distance(m_fieldConstants->GetFieldElementPose2d(element).Translation());
}
