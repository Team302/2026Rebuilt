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
#include "fielddata/FieldConstants.h"

#include "fielddata/FieldAprilTagIDs.h"
#include "fielddata/FieldElementCalculator.h"
#include <utils/logging/debug/Logger.h>

FieldConstants *FieldConstants::m_instance = nullptr;
FieldConstants *FieldConstants::GetInstance()
{
    if (FieldConstants::m_instance == nullptr)
    {
        FieldConstants::m_instance = new FieldConstants();
    }
    return FieldConstants::m_instance;
}

FieldConstants::FieldConstants()
{
    ReadFieldCalibrationData();

    // Blue AprilTag locations
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_HUB_OUTPOST_CENTER] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_HUB_OUTPOST_CENTER_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_TOWER_CENTER] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_TOWER_CENTER_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_HUB_ALLIANCE_CENTER] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_HUB_ALLIANCE_CENTER_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_OUTPOST_CENTER] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_OUTPOST_CENTER_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_TRENCH_NEUTRAL_DEPOT] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_TRENCH_NEUTRAL_DEPOT_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_TRENCH_ALLIANCE_DEPOT] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_TRENCH_ALLIANCE_DEPOT_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_TRENCH_NEUTRAL_OUTPOST] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_TRENCH_NEUTRAL_OUTPOST_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_TRENCH_ALLIANCE_OUTPOST] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_TRENCH_ALLIANCE_OUTPOST_TAG));

    // Blue Calculated Positions
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_TOWER_LEFT_STICK] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_TOWER_RIGHT_STICK] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_TOWER_LEVEL_1_LEFT] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_TOWER_LEVEL_1_RIGHT] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_DEPOT_LEFT_SIDE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_DEPOT_RIGHT_SIDE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_HUB_CENTER] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_DEPOT_PASSING_TARGET] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_OUTPOST_PASSING_TARGET] = m_placeholder;

    // Red AprilTag locations
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_HUB_OUTPOST_CENTER] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_HUB_OUTPOST_CENTER_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_TOWER_CENTER] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_TOWER_CENTER_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_HUB_ALLIANCE_CENTER] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_HUB_ALLIANCE_CENTER_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_OUTPOST_CENTER] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_OUTPOST_CENTER_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_HUB_OUTPOST_CENTER] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_OUTPOST_CENTER_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_TRENCH_NEUTRAL_DEPOT] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_TRENCH_NEUTRAL_DEPOT_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_TRENCH_ALLIANCE_DEPOT] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_TRENCH_ALLIANCE_DEPOT_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_TRENCH_NEUTRAL_OUTPOST] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_TRENCH_NEUTRAL_OUTPOST_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_TRENCH_ALLIANCE_OUTPOST] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_TRENCH_ALLIANCE_OUTPOST_TAG));

    // Red Calculated Positions
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_TOWER_LEFT_STICK] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_TOWER_RIGHT_STICK] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_TOWER_LEVEL_1_LEFT] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_TOWER_LEVEL_1_RIGHT] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_DEPOT_NEUTRAL_SIDE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_DEPOT_LEFT_SIDE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_DEPOT_RIGHT_SIDE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_HUB_CENTER] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_DEPOT_PASSING_TARGET] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_OUTPOST_PASSING_TARGET] = m_placeholder;

    m_aprilTagPoseMap[1] = GetAprilTagPoseFromLayout(1);
    m_aprilTagPoseMap[2] = GetAprilTagPoseFromLayout(2);
    m_aprilTagPoseMap[3] = GetAprilTagPoseFromLayout(3);
    m_aprilTagPoseMap[4] = GetAprilTagPoseFromLayout(4);
    m_aprilTagPoseMap[5] = GetAprilTagPoseFromLayout(5);
    m_aprilTagPoseMap[6] = GetAprilTagPoseFromLayout(6);
    m_aprilTagPoseMap[7] = GetAprilTagPoseFromLayout(7);
    m_aprilTagPoseMap[8] = GetAprilTagPoseFromLayout(8);
    m_aprilTagPoseMap[9] = GetAprilTagPoseFromLayout(9);
    m_aprilTagPoseMap[10] = GetAprilTagPoseFromLayout(10);
    m_aprilTagPoseMap[11] = GetAprilTagPoseFromLayout(11);
    m_aprilTagPoseMap[12] = GetAprilTagPoseFromLayout(12);
    m_aprilTagPoseMap[13] = GetAprilTagPoseFromLayout(13);
    m_aprilTagPoseMap[14] = GetAprilTagPoseFromLayout(14);
    m_aprilTagPoseMap[15] = GetAprilTagPoseFromLayout(15);
    m_aprilTagPoseMap[16] = GetAprilTagPoseFromLayout(16);
    m_aprilTagPoseMap[17] = GetAprilTagPoseFromLayout(17);
    m_aprilTagPoseMap[18] = GetAprilTagPoseFromLayout(18);
    m_aprilTagPoseMap[19] = GetAprilTagPoseFromLayout(19);
    m_aprilTagPoseMap[20] = GetAprilTagPoseFromLayout(20);
    m_aprilTagPoseMap[21] = GetAprilTagPoseFromLayout(21);
    m_aprilTagPoseMap[22] = GetAprilTagPoseFromLayout(22);
    m_aprilTagPoseMap[23] = GetAprilTagPoseFromLayout(23);
    m_aprilTagPoseMap[24] = GetAprilTagPoseFromLayout(24);
    m_aprilTagPoseMap[25] = GetAprilTagPoseFromLayout(25);
    m_aprilTagPoseMap[26] = GetAprilTagPoseFromLayout(26);
    m_aprilTagPoseMap[27] = GetAprilTagPoseFromLayout(27);
    m_aprilTagPoseMap[28] = GetAprilTagPoseFromLayout(28);
    m_aprilTagPoseMap[29] = GetAprilTagPoseFromLayout(29);
    m_aprilTagPoseMap[30] = GetAprilTagPoseFromLayout(30);
    m_aprilTagPoseMap[31] = GetAprilTagPoseFromLayout(31);
    m_aprilTagPoseMap[32] = GetAprilTagPoseFromLayout(32);
    auto pose = m_aprilTagPoseMap[1];
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, "FieldConstants", "AprilTag1Pose", pose.X().value());

    for (const auto &pair : m_aprilTagPoseMap)
    {
        m_aprilTag2dPoses[pair.first] = pair.second.ToPose2d();
    }

    FieldElementCalculator fc;
    fc.CalcPositionsForField(fieldConstantsPoseMap);

    for (const auto &pair : fieldConstantsPoseMap)
    {
        m_fieldConst2dPoses[static_cast<unsigned int>(pair.first)] = pair.second.ToPose2d();
    }
}
frc::Pose3d FieldConstants::GetFieldElementPose(FIELD_ELEMENT element)
{
    return fieldConstantsPoseMap[element];
}

frc::Pose2d FieldConstants::GetFieldElementPose2d(FIELD_ELEMENT element)
{
    return m_fieldConst2dPoses[static_cast<unsigned int>(element)];
}

void FieldConstants::ReadFieldCalibrationData()
{
    if (std::filesystem::exists(m_fieldFilePath))
    {
        m_aprilTagVector = frc::AprilTagFieldLayout(m_fieldFilePath).GetTags();
        for (auto tag : m_aprilTagVector)
        {
            m_aprilTagPoseMap[tag.ID] = tag.pose;
        }
    }
}

frc::Pose3d FieldConstants::GetAprilTagPoseFromLayout(int tagID)
{
    if (!m_fieldLayout.GetTagPose(tagID).has_value())
    {
        return frc::Pose3d();
    }
    return m_fieldLayout.GetTagPose(tagID).value();
}

frc::Pose3d FieldConstants::GetAprilTagPose(FieldAprilTagIDs tag)
{
    return m_aprilTagPoseMap[static_cast<unsigned int>(tag)];
}

frc::Pose2d FieldConstants::GetAprilTagPose2d(FieldAprilTagIDs tag)
{
    return m_aprilTag2dPoses[static_cast<unsigned int>(tag)];
}
