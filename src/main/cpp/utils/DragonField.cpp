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

/// @file DragonField.cpp
/// @brief Implementation of the DragonField class for managing field visualization in SmartDashboard.
/// @details This class provides a singleton interface for displaying robot position, field objects,
///          and trajectories on the FRC SmartDashboard Field2d widget.

#include <string>

// FRC Includes
#include "frc/smartdashboard/SmartDashboard.h"

// Team 302 Includes
#include "utils/DragonField.h"

using std::string;

DragonField *DragonField::m_instance = nullptr;

/// @brief Get the singleton instance of DragonField.
/// @return Pointer to the singleton DragonField instance.
DragonField *DragonField::GetInstance()
{
    if (DragonField::m_instance == nullptr)
    {
        DragonField::m_instance = new DragonField();
    }
    return DragonField::m_instance;
}

/// @brief Constructor for DragonField.
/// @details Initializes the Field2d object and registers it with SmartDashboard.
DragonField::DragonField() : m_field(),
                             m_objects(),
                             m_objectNameEnabled()
{
    frc::SmartDashboard::PutData(&m_field);
}

/// @brief Update the robot's position on the field display.
/// @param robotPose The current pose of the robot (position and rotation).
void DragonField::UpdateRobotPosition(frc::Pose2d robotPose)
{
    m_field.SetRobotPose(robotPose);
}

/// @brief Add a field object with an initial pose and enable/disable selector.
/// @param name The name identifier for the object.
/// @param pose The initial pose of the object.
/// @param defaultSelectorValue Whether the object should be enabled by default on the field.
void DragonField::AddObject(std::string name, frc::Pose2d pose, bool defaultSelectorValue)
{
    if (!defaultSelectorValue)
        pose = frc::Pose2d(99_m, 99_m, frc::Rotation2d()); // place out of view when not enabled initially

    m_field.GetObject(name)->SetPose(pose);
    AddSelector(name, defaultSelectorValue);
}

/// @brief Add a trajectory to be displayed on the field.
/// @param name The name identifier for the trajectory.
/// @param trajectory The trajectory to display.
void DragonField::AddTrajectory(std::string name, frc::Trajectory trajectory)
{
    m_objects.emplace_back(m_field.GetObject(name));
    m_field.GetObject(name)->SetTrajectory(trajectory);
}

/// @brief Reset all field objects by clearing their poses.
/// @details This removes all displayed poses from field objects, effectively clearing the field display.
void DragonField::ResetField()
{
    for (auto object : m_objects)
    {
        object->SetPoses(std::span<frc::Pose2d>());
    }
}

/// @brief Update the pose of a specific field object if it is enabled.
/// @param name The name identifier of the object to update.
/// @param object The new pose for the object.
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

/// @brief Update the enabled/disabled state of all field objects based on SmartDashboard values.
/// @details Reads boolean values from SmartDashboard for each registered object to determine
///          whether they should be visible on the field display.
void DragonField::UpdateEnabledStates()
{
    for (auto &objectPair : m_objectNameEnabled)
    {
        bool enabled = frc::SmartDashboard::GetBoolean(objectPair.first + " Enabled On Field", objectPair.second);
        objectPair.second = enabled;
    }
}

/// @brief Add a boolean selector to SmartDashboard for enabling/disabling a field object.
/// @param name The name identifier for the object.
/// @param defaultValue The default enabled state for the selector.
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