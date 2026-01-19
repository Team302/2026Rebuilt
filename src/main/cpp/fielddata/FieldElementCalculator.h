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

// c++ includes
#include <map>

#include <fielddata/FieldConstants.h>
#include <frc/geometry/Pose3d.h>

struct TransformToPose
{
    FieldConstants::FIELD_ELEMENT referencePose;
    frc::Transform3d transform;
};
enum OffsetEnums
{
    // Comp Red
    COMP_RIGHT_RED,
    COMP_LEFT_RED,
    // Comp Blue
    COMP_RIGHT_BLUE,
    COMP_LEFT_BLUE,
    // Practice Red
    PRACTICE_LEFT_RED,
    PRACTICE_RIGHT_RED,
    // Practice Blue
    PRACTICE_LEFT_BLUE,
    PRACTICE_RIGHT_BLUE
};

class FieldElementCalculator
{

public:
    void CalcPositionsForField(robin_hood::unordered_map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap);
    frc::Pose3d CalcOffsetPositionForElement(frc::Pose3d &poseOfFaceTag, FieldConstants::FIELD_ELEMENT_OFFSETS offset);

private:
    void InitializeTransforms();

    static constexpr units::length::inch_t m_xDistanceHubCenter{23.5};
    static constexpr units::length::inch_t m_xDistanceTowerStickOffsetLeft{45.0};
    static constexpr units::length::inch_t m_yDistanceTowerStickOffsetLeft{19.5};
    static constexpr units::length::inch_t m_xDistanceTowerStickOffsetRight{45.0};
    static constexpr units::length::inch_t m_yDistanceTowerStickOffsetRight{-19.5};
    static constexpr units::length::inch_t m_xDistanceTowerStickOffsetCenter{45.0};
    static constexpr units::length::inch_t m_xDistanceDepotOffsetCenter{27.0};
    static constexpr units::length::inch_t m_yDistanceDepotOffsetCenter{-87.31};
    static constexpr units::length::inch_t m_xDistanceDepotOffsetLeft{13.5};
    static constexpr units::length::inch_t m_yDistanceDepotOffsetLeft{-108.31};
    static constexpr units::length::inch_t m_xDistanceDepotOffsetRight{13.5};
    static constexpr units::length::inch_t m_yDistanceDepotOffsetRight{-66.31};
    static constexpr units::length::inch_t m_xNoOffset{0.0};
    static constexpr units::length::inch_t m_yNoOffset{0.0};
    static constexpr units::length::inch_t m_zNoOffset{0.0};

    // Robot is 34" from front to back
    frc::Transform3d m_halfRobotTransform = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(17), // 16
            units::length::inch_t(0.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    robin_hood::unordered_map<OffsetEnums, frc::Transform3d> m_reefBranchOffsetMap;

    // other transforms
    frc::Transform3d m_noTransform = frc::Transform3d(
        frc::Translation3d(
            m_xNoOffset,
            m_yNoOffset,
            m_zNoOffset),
        frc::Rotation3d());

    frc::Transform3d m_calcHubCenter = frc::Transform3d(
        frc::Translation3d(
            m_xDistanceHubCenter,
            m_yNoOffset,
            m_zNoOffset),
        frc::Rotation3d());

    frc::Transform3d m_calcTowerLeftStick = frc::Transform3d(
        frc::Translation3d(
            m_xDistanceTowerStickOffsetLeft,
            m_yDistanceTowerStickOffsetLeft,
            m_zNoOffset),
        frc::Rotation3d());

    frc::Transform3d m_calcTowerRightStick = frc::Transform3d(
        frc::Translation3d(
            m_xDistanceTowerStickOffsetRight,
            m_yDistanceTowerStickOffsetRight,
            m_zNoOffset),
        frc::Rotation3d());

    frc::Transform3d m_calcTowerCenter = frc::Transform3d(
        frc::Translation3d(
            m_xDistanceTowerStickOffsetCenter,
            m_yNoOffset,
            m_zNoOffset),
        frc::Rotation3d());

    frc::Transform3d m_calcDepoOffsetCenter = frc::Transform3d(
        frc::Translation3d(
            m_xDistanceDepotOffsetCenter,
            m_yDistanceDepotOffsetCenter,
            m_zNoOffset),
        frc::Rotation3d());

    frc::Transform3d m_calcDepoOffsetLeft = frc::Transform3d(
        frc::Translation3d(
            m_xDistanceDepotOffsetLeft,
            m_yDistanceDepotOffsetLeft,
            m_zNoOffset),
        frc::Rotation3d());

    frc::Transform3d m_calcDepoOffsetRight = frc::Transform3d(
        frc::Translation3d(
            m_xDistanceDepotOffsetRight,
            m_yDistanceDepotOffsetRight,
            m_zNoOffset),
        frc::Rotation3d());



    robin_hood::unordered_map<FieldConstants::FIELD_ELEMENT, TransformToPose> m_transformCalculatedMap;
    robin_hood::unordered_map<FieldConstants::FIELD_ELEMENT, TransformToPose> m_transformTagsMap;

};