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
/// @class DepotHelper
/// @brief Helper class for depot-related calculations and navigation
///
/// This singleton class provides utilities for interacting with depots on the field, including:
/// - Determining which depot (red or blue) is closest to the robot
/// - Calculating the center pose of the nearest depot
/// - Computing distances to field elements
///
/// The class uses the robot's current pose and field constants to make alliance-aware decisions
/// about depot locations and navigation targets.
//====================================================================================================================================================
class DriveToNZOutFromAllianceHelper
{
public:
    //------------------------------------------------------------------
    /// @brief      Get the singleton instance of DepotHelper
    /// @return     DepotHelper* - Pointer to the singleton instance
    //------------------------------------------------------------------
    static DriveToNZOutFromAllianceHelper *GetInstance();
    units::length::meter_t CalcDistanceToObject(FieldConstants::FIELD_ELEMENT element, frc::Pose2d currentPose) const;

private:
    FieldConstants *m_fieldConstants;
    //------------------------------------------------------------------
    /// @brief      Private constructor for singleton pattern
    /// @details    Initializes the chassis and field constants references
    //------------------------------------------------------------------
    DriveToNZOutFromAllianceHelper();

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    //------------------------------------------------------------------
    ~DriveToNZOutFromAllianceHelper() = default;

    /// @brief Singleton instance pointer
    static DriveToNZOutFromAllianceHelper *m_instance;

    //------------------------------------------------------------------
    /// @brief      Calculates the distance from a given pose to a field element
    /// @param[in]  element - The field element to measure distance to
    /// @param[in]  currentPose - The pose to measure distance from
    /// @return     units::length::meter_t - The distance in meters
    //------------------------------------------------------------------
    frc::Pose2d CalcNearestBump(FieldConstants::FIELD_ELEMENT element, frc::Pose2d currentPose) const;

    /// @brief Pointer to the swerve drivetrain subsystem
    subsystems::CommandSwerveDrivetrain *m_chassis;

    /// @brief Pointer to the field constants singleton
};
