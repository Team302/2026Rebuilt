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

#include <string>
#include <algorithm>

// FRC Includes
#include "frc/smartdashboard/SmartDashboard.h"
#include "utils/logging/debug/Logger.h"

// Team 302 Includes
#include "utils/DragonField.h"

using std::string;

DragonField *DragonField::m_instance = nullptr;

DragonField *DragonField::GetInstance()
{
    if (DragonField::m_instance == nullptr)
    {
        DragonField::m_instance = new DragonField();
    }
    return DragonField::m_instance;
}

DragonField::DragonField() : m_field(),
                             m_objects()
{
    frc::SmartDashboard::PutData(&m_field);
}

void DragonField::UpdateRobotPosition(frc::Pose2d robotPose)
{
    m_field.SetRobotPose(robotPose);
}

void DragonField::AddObject(std::string name, frc::Pose2d pose, bool defaultSelectorValue)
{
    if (!defaultSelectorValue)
        pose = frc::Pose2d(99_m, 99_m, frc::Rotation2d()); // place out of view when not enabled initially

    m_field.GetObject(name)->SetPose(pose);
    AddSelector(name, defaultSelectorValue);
}

void DragonField::AddTrajectory(std::string name, frc::Trajectory trajectory)
{
    m_objects.emplace_back(m_field.GetObject(name));
    m_field.GetObject(name)->SetTrajectory(trajectory);
}

void DragonField::ResetField()
{
    for (auto object : m_objects)
    {
        object->SetPoses(std::span<frc::Pose2d>());
    }
}

void DragonField::UpdateObject(std::string name, frc::Pose2d object)
{
    auto objectPair = std::find_if(m_objectNameEnabled.begin(), m_objectNameEnabled.end(),
                                   [&name](const std::pair<std::string, bool> &pair)
                                   { return pair.first == name; });

    if (objectPair != m_objectNameEnabled.end() && objectPair->second)
    {
        frc::FieldObject2d *fieldObject = m_field.GetObject(name);
        fieldObject->SetPose(object);
    }
}

void DragonField::UpdateEnabledStates()
{
    for (auto &objectPair : m_objectNameEnabled)
    {
        bool enabled = frc::SmartDashboard::GetBoolean(objectPair.first + " Enabled On Field", objectPair.second);
        objectPair.second = enabled;
    }
}

void DragonField::AddSelector(std::string name, bool defaultValue)
{
    frc::SmartDashboard::PutBoolean(name + " Enabled On Field", defaultValue);
    m_objectNameEnabled.emplace_back(name, defaultValue);
}

// void DragonField::UpdateObjectVisionPose(std::string name, std::optional<VisionPose> visionPose)
// {
//     frc::FieldObject2d *fieldObject = m_field.GetObject(name);
//     if (visionPose.has_value())
//     {
//         frc::Pose3d pose3d = visionPose.value().estimatedPose;
//         fieldObject->SetPose(pose3d.ToPose2d());
//     }
// }