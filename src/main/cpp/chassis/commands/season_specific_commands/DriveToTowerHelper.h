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

// C++ includes

#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "fielddata/FieldConstants.h"
#include "frc/geometry/Pose2d.h"

//====================================================================================================================================================
/// @class DRiveToTowerHelper
/// @brief Helper class for Tower-related calculations and navigation
///
/// This singleton class provides utilities for interacting with Towers on the field, including:
/// - Determining which Tower (red or blue) is closest to the robot
/// - Calculating the center pose of the nearest Tower
/// - Computing distances to field elements
///
/// The class uses the robot's current pose and field constants to make alliance-aware decisions
/// about Tower locations and navigation targets.
//====================================================================================================================================================
class DriveToTowerHelper
{
public:
    //------------------------------------------------------------------
    /// @brief      Get the singleton instance of DriveToTowerHelper
    /// @return     DriveToTowerHelper* - Pointer to the singleton instance
    //------------------------------------------------------------------
    static DriveToTowerHelper *GetInstance();

    //------------------------------------------------------------------
    /// @brief      Calculates the center pose of the nearest Tower
    /// @return     frc::Pose2d - The calculated center pose of the Tower
    /// @details    Determines which Tower (red or blue) is nearest, then
    ///             calculates the center point by averaging the X and Y
    ///             coordinates of the left, right, and neutral side poses.
    //------------------------------------------------------------------
    frc::Pose2d CalcTowerPose() const;

private:
    //------------------------------------------------------------------
    /// @brief      Private constructor for singleton pattern
    /// @details    Initializes the chassis and field constants references
    //------------------------------------------------------------------
    DriveToTowerHelper();

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    //------------------------------------------------------------------
    ~DriveToTowerHelper() = default;

    /// @brief Singleton instance pointer
    static DriveToTowerHelper *m_instance;

    //------------------------------------------------------------------
    /// @brief      Determines which Tower (red or blue) is nearest to the robot
    /// @return     bool - true if the red Tower is nearest, false if blue Tower is nearest
    //------------------------------------------------------------------
    bool IsNearestTowerRed() const;

    //------------------------------------------------------------------
    /// @brief      Calculates the distance from a given pose to a field element
    /// @param[in]  element - The field element to measure distance to
    /// @param[in]  currentPose - The pose to measure distance from
    /// @return     units::length::meter_t - The distance in meters
    //------------------------------------------------------------------
    units::length::meter_t CalcDistanceToObject(FieldConstants::FIELD_ELEMENT element, frc::Pose2d currentPose) const;

    /// @brief Pointer to the swerve drivetrain subsystem
    subsystems::CommandSwerveDrivetrain *m_chassis;

    /// @brief Pointer to the field constants singleton
    FieldConstants *m_fieldConstants;
};
