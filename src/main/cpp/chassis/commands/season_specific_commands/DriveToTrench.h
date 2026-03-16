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

#include "auton/AllianceZoneManager.h"
#include "auton/NeutralZoneManager.h"
#include "chassis/commands/DriveToPose.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "fielddata/FieldConstants.h"
#include "fielddata/TrenchHelper.h"
#include "units/angle.h"
#include "utils/FMSData.h"

//====================================================================================================================================================
/// @class DriveToTrench
/// @brief Command to autonomously drive the robot through a field trench using a two-stage navigation approach
///
/// This command extends DriveToPose to provide specialized functionality for navigating through the trenches
/// that separate the alliance zone from the neutral zone on the 2026 game field. The command uses
/// TrenchHelper to identify the nearest trench and obtain an ordered list of drive waypoints.
///
/// **Two-Stage Navigation:**
/// The command uses a mid-pose-to-end-pose strategy to align the robot before committing to the crossing:
/// 1. Mid pose  — robot's current X with the trench near-side Y (aligns laterally first)
/// 2. End pose  — the far-side trench position returned by TrenchHelper
///
/// **Rotation Handling:**
/// Waypoint rotations are supplied by TrenchHelper and orient the robot to face the appropriate
/// alliance wall while transiting the trench.
///
/// The command uses TrenchHelper for trench identification and waypoint generation, making it
/// fully autonomous and field-aware.
///
/// @see DriveToPose       Base class providing PID-controlled pose navigation
/// @see TrenchHelper      Utility for identifying the nearest trench and building drive waypoints
/// @see FieldOffsetValues Field coordinate management used internally by TrenchHelper
//====================================================================================================================================================
class DriveToTrench : public DriveToPose
{
public:
    //------------------------------------------------------------------
    /// @brief      Constructor for DriveToTrench command
    /// @param[in]  chassis - Pointer to the swerve drive subsystem that will execute the movement
    /// @details    Initializes the base DriveToPose functionality and configures the distance,
    ///             angle, and Y-transition tolerances for trench crossing completion detection.
    //------------------------------------------------------------------
    DriveToTrench(subsystems::CommandSwerveDrivetrain *chassis);

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    /// @details    No cleanup required as all resources are managed by parent class or are value types
    //------------------------------------------------------------------
    ~DriveToTrench() = default;

protected:
    //------------------------------------------------------------------
    /// @brief      Calculates target poses for two-stage trench crossing
    /// @return     DriveToPoses struct with mid pose (current X, near-trench Y) and
    ///             end pose (far side of trench) from TrenchHelper
    /// @details    Overrides base class to provide trench-specific navigation waypoints.
    ///             See DriveToTrench.cpp for full implementation details.
    /// @see        TrenchHelper::GetTrenchDrivePositions()
    //------------------------------------------------------------------
    struct DriveToPoses GetDriveToPoses() override;

    //------------------------------------------------------------------
    /// @brief      Returns the maximum translational velocity for trench crossing
    /// @return     units::velocity::meters_per_second_t - Maximum velocity (kMaxVelocityDriveToTrench)
    /// @details    Limits speed to ensure safe alignment and crossing of the field trench.
    //------------------------------------------------------------------
    units::velocity::meters_per_second_t GetMaxVelocity() const override { return kMaxVelocityDriveToTrench; }

    //------------------------------------------------------------------
    /// @brief      Returns the maximum translational acceleration for trench crossing
    /// @return     units::acceleration::meters_per_second_squared_t - Maximum acceleration (kMaxAccelerationDriveToTrench)
    /// @details    Limits acceleration to reduce jerk when approaching and clearing the trench.
    //------------------------------------------------------------------
    units::acceleration::meters_per_second_squared_t GetMaxAcceleration() const override { return kMaxAccelerationDriveToTrench; }

private:
    /// @brief Translational distance tolerance for pose completion (robot must be within 12 in of target)
    static constexpr units::length::inch_t kDistanceThreshold = 12_in;
    /// @brief Heading tolerance for pose completion (robot heading must be within 1°)
    static constexpr units::angle::degree_t kAngleTolerance = 1.0_deg;
    /// @brief Extra Y-axis tolerance applied during the transition from mid pose to end pose
    static constexpr units::length::inch_t kYTransitionToEndPointTolerance = 3.0_in;

    /// @brief Maximum translational velocity while crossing the trench (2.0 m/s)
    static constexpr units::velocity::meters_per_second_t kMaxVelocityDriveToTrench = 2.0_mps;
    /// @brief Maximum translational acceleration while crossing the trench (1.0 m/s²)
    static constexpr units::acceleration::meters_per_second_squared_t kMaxAccelerationDriveToTrench = 1.0_mps_sq;
};
