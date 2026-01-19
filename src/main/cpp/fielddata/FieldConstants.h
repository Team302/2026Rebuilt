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
#pragma once
#include <filesystem>
#include <iostream>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>

#include "RobinHood/robin_hood.h"

#include "fielddata/FieldAprilTagIDs.h"
#include "frc/apriltag/AprilTagFieldLayout.h"

#include "units/angle.h"
#include "units/base.h"

class FieldConstants
{
public:
    static FieldConstants *GetInstance();
    enum class FIELD_ELEMENT
    {
        // 2026 - BLUE APRIL TAGS
        BLUE_HUB_ALLIANCE_CENTER,
        BLUE_HUB_OUTPOST_CENTER,
        BLUE_TOWER_CENTER,
        // 2026 - RED APRIL TAGS
        RED_HUB_ALLIANCE_CENTER,
        RED_HUB_OUTPOST_CENTER,
        RED_TOWER_CENTER,
        // 2026 - Blue Calculated Positions
        BLUE_HUB_CENTER,
        BLUE_TOWER_LEFT_STICK,
        BLUE_TOWER_RIGHT_STICK,
        BLUE_TOWER_LEVEL_1_LEFT,
        BLUE_TOWER_LEVEL_1_RIGHT,
        BLUE_DEPOT_NEUTRAL_SIDE,
        BLUE_DEPOT_LEFT_SIDE,
        BLUE_DEPOT_RIGHT_SIDE,

        // 2026 - Red Calculated Positions
        RED_HUB_CENTER,
        RED_TOWER_LEFT_STICK,
        RED_TOWER_RIGHT_STICK,
        RED_TOWER_LEVEL_1_LEFT,
        RED_TOWER_LEVEL_1_RIGHT,
        RED_DEPOT_NEUTRAL_SIDE,
        RED_DEPOT_LEFT_SIDE,
        RED_DEPOT_RIGHT_SIDE
    };

    enum class FIELD_ELEMENT_OFFSETS
    {
        LEFT_STICK,
        RIGHT_STICK
    };

    enum class CageLocation
    {
        LEFT,
        CENTER,
        RIGHT
    };

    frc::Pose3d GetFieldElementPose(FIELD_ELEMENT element);
    frc::Pose2d GetFieldElementPose2d(FIELD_ELEMENT element);

    frc::Pose3d GetAprilTagPose(FieldAprilTagIDs tag);
    frc::Pose2d GetAprilTagPose2d(FieldAprilTagIDs tag);

private:
    // make a singleton
    static FieldConstants *m_instance;
    std::vector<frc::AprilTag> m_aprilTagVector;
    const std::string m_fieldFilePath = "/home/lvuser/FieldData/output.json";
    // make constructor private
    FieldConstants();
    // make singleton copy constructor private
    FieldConstants(const FieldConstants &) = delete;
    FieldConstants &operator=(const FieldConstants &) = delete;
    frc::Pose3d GetAprilTagPoseFromLayout(int tagID);
    frc::AprilTagFieldLayout m_fieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded); // change this guy for andymark field

    void ReadFieldCalibrationData();

    frc::Pose3d m_placeholder = frc::Pose3d();

    robin_hood::unordered_map<FIELD_ELEMENT, frc::Pose3d> fieldConstantsPoseMap;
    std::array<frc::Pose2d, 68> m_fieldConst2dPoses;

    robin_hood::unordered_map<int, frc::Pose3d> m_aprilTagPoseMap;
    std::array<frc::Pose2d, 32> m_aprilTag2dPoses;
};