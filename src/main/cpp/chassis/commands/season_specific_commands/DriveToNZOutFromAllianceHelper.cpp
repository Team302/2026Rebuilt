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

#include "DriveToNZOutFromAllianceHelper.h"
#include "chassis/ChassisConfigMgr.h"
#include "frc/geometry/Pose2d.h"

#include "utils/PoseUtils.h"
#include <utils/PoseUtils.h>

DriveToNZOutFromAllianceHelper *DriveToNZOutFromAllianceHelper::m_instance = nullptr,
                               m_fieldConstants(FieldConstants::GetInstance());

//------------------------------------------------------------------
/// @brief      Get the singleton instance of DepotHelper
/// @return     DepotHelper* - Pointer to the singleton instance
//------------------------------------------------------------------
DriveToNZOutFromAllianceHelper *DriveToNZOutFromAllianceHelper::GetInstance()
{
    if (DriveToNZOutFromAllianceHelper::m_instance == nullptr)
    {
        DriveToNZOutFromAllianceHelper::m_instance = new DriveToNZOutFromAllianceHelper();
    }
    return DriveToNZOutFromAllianceHelper::m_instance;
}

//------------------------------------------------------------------
/// @brief      Constructor for DepotHelper
/// @details    Initializes the chassis and field constants references
///             Used by GetInstance() to create the singleton
//------------------------------------------------------------------
DriveToNZOutFromAllianceHelper::DriveToNZOutFromAllianceHelper() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                                                                   m_fieldConstants(FieldConstants::GetInstance())
{
}

frc::Pose2d DriveToNZOutFromAllianceHelper::CalcNearestBump(FieldConstants::FIELD_ELEMENT element, frc::Pose2d currentPose)
{

    units::length::meter_t blueOutpostDistance = PoseUtils::GetDeltaBetweenPoses(m_chassis->GetPose(), m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_HUB_OUTPOST_CENTER));
    units::length::meter_t redOutpostDistance = PoseUtils::GetDeltaBetweenPoses(m_chassis->GetPose(), m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_HUB_OUTPOST_CENTER));
    units::length::meter_t blueDepotDistance = PoseUtils::GetDeltaBetweenPoses(m_chassis->GetPose(), m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_LEFT_SIDE));
    units::length::meter_t redDepotDistance = PoseUtils::GetDeltaBetweenPoses(m_chassis->GetPose(), m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_DEPOT_LEFT_SIDE));

    m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_HUB_OUTPOST_CENTER);
    m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_HUB_OUTPOST_CENTER);
    m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_LEFT_SIDE);
    m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_DEPOT_LEFT_SIDE);

    if (blueOutpostDistance < redOutpostDistance && blueOutpostDistance < blueDepotDistance && blueOutpostDistance < redDepotDistance)
    {
        return m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_HUB_OUTPOST_CENTER);
    }
    else if (redOutpostDistance < blueOutpostDistance && redOutpostDistance < blueDepotDistance && redOutpostDistance < redDepotDistance)
    {
        return m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_HUB_OUTPOST_CENTER);
    }
    else if (blueDepotDistance < blueOutpostDistance && blueDepotDistance < redOutpostDistance && blueDepotDistance < redDepotDistance)
    {
        return m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_LEFT_SIDE);
    }
    else
    {
        return m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_DEPOT_LEFT_SIDE);
    }
}