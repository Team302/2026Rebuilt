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
/// @brief Command to autonomously drive the robot over a field trench using a two-stage navigation approach
///
/// This command extends DriveToPose to provide specialized functionality for safely navigating over the trenches
/// that separate the alliance zone from the neutral zone on the 2026 game field. The command intelligently
/// determines which trench (Red/Blue, Depot/Outpost) is nearest and calculates an optimal path over it.
///
/// **Two-Stage Navigation:**
/// The command uses a midpoint-to-endpoint strategy to ensure the robot successfully crosses the trench:
/// 1. First, drive to the midpoint pose (near side of the trench)
/// 2. Then, drive to the endpoint pose (far side of the trench)
///
/// **Directional Intelligence:**
/// - If starting in the neutral zone: Drive toward the alliance zone
/// - If starting in the alliance zone: Drive toward the neutral zone
///
/// **Rotation Handling:**
/// Each bump has predefined rotation angles (45° or 315°) that orient the robot toward the hub center
/// while crossing, ensuring optimal positioning for game play after the transition.
///
/// The command uses BumpHelper for bump identification, FieldOffsetValues for coordinates, and
/// NeutralZoneManager for zone detection, making it fully autonomous and field-aware.
///
/// @see DriveToPose Base class providing PID-controlled pose navigation
/// @see BumpHelper Utility for identifying nearest bump
/// @see FieldOffsetValues Field coordinate management
//====================================================================================================================================================
class DriveToTrench : public DriveToPose
{
public:
    //------------------------------------------------------------------
    /// @brief      Constructor for DriveToTrench command
    /// @param[in]  chassis - Pointer to the swerve drive subsystem that will execute the movement
    /// @details    Initializes the command with the chassis reference for autonomous navigation.
    ///             The constructor sets up the base DriveToPose functionality.
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
    /// @return     DriveToPoses struct with midpoint (trench side) and endpoint (opposite side)
    /// @details    Overrides base class to provide trench-specific navigation.
    ///             See implementation for detailed pose calculation logic.
    /// @see        DriveToTrench.cpp for full implementation details
    //------------------------------------------------------------------
    struct DriveToPoses GetDriveToPoses() override;

    //------------------------------------------------------------------
    /// @brief      Returns the maximum translational velocity for trench crossing
    /// @return     units::velocity::meters_per_second_t - Maximum velocity (2.0 m/s)
    /// @details    Limits speed to ensure safe crossing of the trench obstacle.
    //------------------------------------------------------------------
    units::velocity::meters_per_second_t GetMaxVelocity() const override { return kMaxVelocityDriveToTrench; }

    //------------------------------------------------------------------
    /// @brief      Returns the maximum translational acceleration for trench crossing
    /// @return     units::acceleration::meters_per_second_squared_t - Maximum acceleration (1.0 m/s²)
    /// @details    Limits acceleration to reduce jerk when approaching and leaving the trench.
    //------------------------------------------------------------------
    units::acceleration::meters_per_second_squared_t GetMaxAcceleration() const override { return kMaxAccelerationDriveToTrench; }

private:
    //------------------------------------------------------------------
    /// @brief      Determines the robot heading for the current trench crossing
    /// @return     units::angle::degree_t - Target rotation angle in degrees
    /// @details    Returns kTowardBlueAllianceWall (0°) for blue alliance or
    ///             kTowardRedAllianceWall (180°) for red alliance, orienting the
    ///             robot toward its own alliance wall while crossing.
    //------------------------------------------------------------------
    units::angle::degree_t GetRotation() const;

    /// @brief Rotation to face the blue alliance wall (0°) — used when alliance is blue
    static constexpr units::degree_t kTowardBlueAllianceWall{0.0};
    /// @brief Rotation to face the red alliance wall (180°) — used when alliance is red
    static constexpr units::degree_t kTowardRedAllianceWall{180.0};

    /// @brief Maximum lateral distance from the target pose before the command considers itself complete
    static constexpr units::length::inch_t kDistanceThreshold = 12_in;
    /// @brief Maximum heading error before the command considers the rotation complete
    static constexpr units::angle::degree_t kAngleTolerance = 1.0_deg;
    /// @brief Additional Y-axis tolerance applied during the transition from midpoint to endpoint
    static constexpr units::length::inch_t kYTransitionToEndPointTolerance = 3.0_in;

    /// @brief Maximum translational velocity allowed while crossing the trench
    static constexpr units::velocity::meters_per_second_t kMaxVelocityDriveToTrench = 2.0_mps;
    /// @brief Maximum translational acceleration allowed while crossing the trench
    static constexpr units::acceleration::meters_per_second_squared_t kMaxAccelerationDriveToTrench = 1.0_mps_sq;
};
