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

// FRC Include
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/FieldObject2d.h>
#include <frc/trajectory/Trajectory.h>

/// @class DragonField
/// @brief Singleton class for managing field visualization in SmartDashboard.
/// @details This class provides a centralized interface for displaying robot position, field objects,
///          and trajectories on the FRC SmartDashboard Field2d widget. It manages the lifecycle of
///          field objects and provides methods to update their positions and visibility.
class DragonField
{
public:
    /// @brief Constructor for DragonField.
    /// @details Initializes the Field2d object and registers it with SmartDashboard.
    DragonField();

    /// @brief Destructor for DragonField.
    ~DragonField() = default;

    /// @brief Update the robot's position on the field display.
    /// @param robotPose The current pose of the robot (position and rotation).
    void UpdateRobotPosition(frc::Pose2d robotPose);

    /// @brief Add a field object with an initial pose and enable/disable selector.
    /// @param name The name identifier for the object.
    /// @param pose The initial pose of the object.
    /// @param defaultSelectorValue Whether the object should be enabled by default on the field.
    void AddObject(std::string name, frc::Pose2d pose, bool defaultSelectorValue = false);

    /// @brief Add a trajectory to be displayed on the field.
    /// @param name The name identifier for the trajectory.
    /// @param trajectory The trajectory to display.
    void AddTrajectory(std::string name, frc::Trajectory trajectory);

    /// @brief Update the pose of a specific field object if it is enabled.
    /// @param name The name identifier of the object to update.
    /// @param pose The new pose for the object.
    void UpdateObject(std::string name, frc::Pose2d pose);

    /// @brief Update the enabled/disabled state of all field objects based on SmartDashboard values.
    /// @details Reads boolean values from SmartDashboard for each registered object to determine
    ///          whether they should be visible on the field display.
    void UpdateEnabledStates();

    // void UpdateObjectVisionPose(std::string name, std::optional<VisionPose> visionPose);

    /// @brief Get the singleton instance of DragonField.
    /// @return Pointer to the singleton DragonField instance.
    static DragonField *GetInstance();

    /// @brief Reset all field objects by clearing their poses.
    /// @details This removes all displayed poses from field objects, effectively clearing the field display.
    void ResetField();

private:
    frc::Field2d m_field;                                          ///< The Field2d object used for visualization
    std::vector<frc::FieldObject2d *> m_objects;                   ///< List of field objects for trajectories
    std::vector<std::pair<std::string, bool>> m_objectNameEnabled; ///< Map of object names to their enabled state
    static DragonField *m_instance;                                ///< Singleton instance pointer

    /// @brief Add a boolean selector to SmartDashboard for enabling/disabling a field object.
    /// @param name The name identifier for the object.
    /// @param defaultValue The default enabled state for the selector.
    void AddSelector(std::string name, bool defaultValue);
};