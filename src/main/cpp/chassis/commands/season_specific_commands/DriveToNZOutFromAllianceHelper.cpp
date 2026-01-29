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

DriveToNZOutFromAllianceHelper *DriveToNZOutFromAllianceHelper::m_instance = nullptr;

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
