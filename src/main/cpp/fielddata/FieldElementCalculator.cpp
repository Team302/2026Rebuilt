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
#include "fielddata/FieldElementCalculator.h"

#include "RobotIdentifier.h"
#include "fielddata/FieldConstantsPoseLogger.h"
#include "frc/RobotController.h"
#include "utils/FMSData.h"
#include "utils/logging/debug/Logger.h"

void FieldElementCalculator::CalcPositionsForField(robin_hood::unordered_map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap)
{
    InitializeTransforms();

    // update all of the calculated values only
    for (auto &[key, translatedPose] : m_transformCalculatedMap)
    {
        fieldConstantsPoseMap[key] = fieldConstantsPoseMap[m_transformCalculatedMap[key].referencePose] + m_transformCalculatedMap[key].transform;
    }

    // after transform the tags, if the tags are transformed first it doubly transforms the calculated values
    for (auto &[key, unusedValue] : m_transformTagsMap)
    {
        fieldConstantsPoseMap[key] = fieldConstantsPoseMap[key];
    }

#ifdef INCLUDE_FIELD_ELEMENT_POSE_LOGGER
    FieldConstantsPoseLogger fpl;
    fpl.LogFieldElementPoses(fieldConstantsPoseMap);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("PoseLogger"), std::string("Logging"), std::string(""));
#endif
}

void FieldElementCalculator::InitializeTransforms()
{

    // no transforms for april tags
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_HUB_ALLIANCE_CENTER];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_HUB_OUTPOST_CENTER];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::RED_HUB_ALLIANCE_CENTER];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::RED_HUB_OUTPOST_CENTER];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER];
  

    // Blue Calculated Positions
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_HUB_CENTER] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_HUB_ALLIANCE_CENTER, m_calcHubCenter);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_TOWER_LEFT_STICK] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER, m_calcTowerLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_TOWER_RIGHT_STICK] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER, m_calcTowerRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_LEFT_SIDE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER, m_calcDepoOffsetLeft);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER, m_calcDepoOffsetCenter);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_RIGHT_SIDE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER, m_calcDepoOffsetRight);



    // Red Calculated Positions
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_HUB_CENTER] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_HUB_ALLIANCE_CENTER, m_calcHubCenter);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_TOWER_LEFT_STICK] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER, m_calcTowerLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_TOWER_RIGHT_STICK] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER, m_calcTowerRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_DEPOT_LEFT_SIDE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER, m_calcDepoOffsetLeft);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_DEPOT_NEUTRAL_SIDE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER, m_calcDepoOffsetCenter);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_DEPOT_RIGHT_SIDE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER, m_calcDepoOffsetRight);
}

