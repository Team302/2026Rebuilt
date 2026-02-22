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

#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>

struct DriveToPoses
{
    frc::Pose2d endPose{};
    bool hasMidPose = false;
    frc::Pose2d midPose{};
};

//====================================================================================================================================================
/// @class DriveToPose
/// @brief Base command class for autonomous navigation to field poses using hybrid feedforward+PID control
///
/// This command implements a sophisticated control strategy for driving a swerve drivetrain to specific
/// field poses with high accuracy and smooth motion profiles. It serves as a base class that can be
/// extended for navigation to specific field elements or game pieces.
///
/// **Control Architecture:**
/// The command uses a hybrid control approach combining:
/// - **Feedforward Control:** Distance-based velocity scaling for efficient long-range travel
/// - **PID Feedback:** Separate ProfiledPIDControllers for X and Y translation for precise positioning
/// - **Heading Control:** Independent rotation control to face target direction
/// - **Adaptive Reset:** Automatic PID reset when error exceeds threshold to prevent windup
///
/// **Motion Profile:**
/// - Maximum velocity: 4 m/s
/// - Maximum acceleration: 4 m/s² (trapezoidal profile)
/// - Feedforward ramp: 0-1.25m from target (linear scaling)
/// - PID active when error < 0.25m
/// - Completion threshold: 0.25 inches (customizable)
///
/// **Extensibility:**
/// Derived classes override GetDriveToPoses() to provide specific target calculations:
/// - DriveToDepot: Navigate to depot zones
/// - DriveOverBump: Multi-stage bump crossing
/// - DriveToGamePiece: Vision-based game piece targeting
///
/// **State Management:**
/// The command publishes state changes to RobotState for coordination with other subsystems
/// and provides logging for debugging and performance analysis.
///
/// @see DriveOverBump Example of multi-waypoint navigation using SetTargetPose()
/// @see CommandSwerveDrivetrain The swerve drive subsystem controlled by this command
//====================================================================================================================================================
class DriveToPose : public frc2::CommandHelper<frc2::Command, DriveToPose>
{
public:
    //------------------------------------------------------------------
    /// @brief      Constructs a DriveToPose command for autonomous navigation
    /// @param[in]  chassis - Pointer to the swerve drive subsystem that will execute the motion
    /// @details    Initializes the command with ProfiledPID controllers for X and Y translation,
    ///             configures I-Zone to prevent integral windup, and captures initial robot state.
    ///             The command requires exclusive access to the chassis subsystem during execution.
    ///
    ///             **Controller Configuration:**
    ///             - X/Y PID gains: kP, kI, kD
    ///             - Trapezoidal constraints: max velocity, max acceleration
    ///             - I-Zone: (prevents integral accumulation when far from target)
    ///
    /// @note       Derived classes should call this constructor via initializer list
    /// @see        Initialize() for per-execution setup
    //------------------------------------------------------------------
    DriveToPose(subsystems::CommandSwerveDrivetrain *chassis);

    //------------------------------------------------------------------
    // FRC Command Lifecycle Methods
    //------------------------------------------------------------------

    //------------------------------------------------------------------
    /// @brief      Initializes the command when scheduled
    /// @details    Called once when the command starts. Gets the target pose,
    ///             resets movement detection, configures controllers, and publishes
    ///             state changes to notify other subsystems that navigation is active.
    /// @see        GetDriveToPoses() for target determination
    //------------------------------------------------------------------
    void Initialize() override;

    //------------------------------------------------------------------
    /// @brief      Executes the command periodically (every 20ms)
    /// @details    Updates robot pose, calculates feedforward and PID corrections,
    ///             applies velocity clamping, and sends control requests to the drivetrain.
    ///             Implements adaptive PID reset when error exceeds threshold.
    /// @see        CalculateFeedForward() for velocity calculation
    /// @see        IsFinished() for completion detection
    //------------------------------------------------------------------
    void Execute() override;

    //------------------------------------------------------------------
    /// @brief      Checks if navigation is complete
    /// @return     true if target reached or robot is stuck, false to continue
    /// @details    Returns true when robot is within distance threshold of target
    ///             or when robot has stopped moving (stuck detection).
    /// @see        SetDistanceThreshold() to adjust completion tolerance
    //------------------------------------------------------------------
    bool IsFinished() override;

    //------------------------------------------------------------------
    /// @brief      Cleans up when command ends
    /// @param[in]  interrupted - true if interrupted, false if finished normally
    /// @details    Stops the robot with brake control and publishes state changes
    ///             to notify other subsystems that navigation has ended.
    //------------------------------------------------------------------
    void End(bool interrupted) override;

protected:
    //------------------------------------------------------------------
    /// @brief      Gets the target pose(s) for navigation
    /// @return     DriveToPoses struct containing endpoint and optional midpoint
    /// @details    Virtual method that derived classes override to provide
    ///             specific target pose calculations. The base implementation
    ///             returns an empty DriveToPoses struct.
    ///
    ///             **Usage Pattern:**
    ///             - Override in derived classes to provide field element targets
    ///             - Set hasMidPose=true for two-stage navigation
    ///             - Called during Initialize() to determine navigation goals
    ///
    /// @note       Derived classes should implement this to specify their targets
    /// @see        DriveOverBump::GetDriveToPoses() for multi-waypoint example
    /// @see        DriveToDepot::GetDriveToPoses() for single-target example
    //------------------------------------------------------------------
    virtual struct DriveToPoses GetDriveToPoses() { return DriveToPoses{}; }

    //------------------------------------------------------------------
    /// @brief      Updates the target pose during execution
    /// @param[in]  endPose - The new target pose to navigate to
    /// @details    Allows dynamic target changes during command execution.
    ///             Resets PID controllers with current state to ensure smooth
    ///             transition to the new target. Used for multi-stage navigation
    ///             or reactive path planning.
    ///
    /// @note       PID controllers are reset to prevent control discontinuities
    /// @see        DriveOverBump::IsFinished() for example usage
    //------------------------------------------------------------------
    void SetTargetPose(const frc::Pose2d &endPose);

    //------------------------------------------------------------------
    /// @brief      Sets the distance threshold for completion detection
    /// @param[in]  distanceThreshold - Distance tolerance in inches for considering target reached
    /// @details    Customizes how close the robot must be to the target before
    ///             IsFinished() returns true. Smaller values provide higher precision
    ///             but may increase settling time.
    ///
    /// @note       Derived classes should call this in their constructors or
    ///  initialization methods
    //------------------------------------------------------------------
    void SetDistanceThreshold(const units::length::inch_t &distanceThreshold) { m_distanceThreshold = distanceThreshold; }

    //------------------------------------------------------------------
    /// @brief      Sets the angle tolerance for completion detection
    /// @param[in]  angleTolerance - Angular tolerance in degrees for considering target rotation reached
    /// @details    Customizes how close the robot's heading must be to the target angle before
    ///             IsFinished() returns true. Smaller values provide higher rotational precision
    ///             but may increase settling time.
    ///
    /// @note       Derived classes should call this in their constructors or
    ///             initialization methods to override the default tolerance
    //------------------------------------------------------------------
    void SetAngleTolerance(const units::angle::degree_t &angleTolerance) { m_angleTolerance = angleTolerance; }
    void SetXTransitionToEndPointTolerance(const units::length::inch_t &xTolerance) { m_xtoleranceForTransitionToEndPoint = xTolerance; }
    void SetYTransitionToEndPointTolerance(const units::length::inch_t &yTolerance) { m_yToleranceForTransitionToEndPoint = yTolerance; }

    //------------------------------------------------------------------
    /// @brief      Gets the chassis subsystem pointer
    /// @return     Pointer to the swerve drive subsystem
    /// @details    Provides protected access to the chassis for derived classes
    ///             that need to query robot state or modify control behavior.
    //------------------------------------------------------------------
    subsystems::CommandSwerveDrivetrain *GetChassis() const { return m_chassis; }

private:
    bool ShouldSkipMidPoint() const;

    //------------------------------------------------------------------
    /// @brief      Calculates feedforward velocity component toward target
    /// @param[out] chassisSpeeds - Reference to populate with feedforward velocities
    /// @details    Implements distance-based velocity scaling with three zones
    /// @see        Execute() for how feedforward combines with PID
    //------------------------------------------------------------------
    void CalculateFeedForward(frc::ChassisSpeeds &chassisSpeeds);

    //------------------------------------------------------------------
    // Subsystem and Control Request Members
    //------------------------------------------------------------------

    /// @brief Pointer to the swerve drive subsystem controlled by this command
    subsystems::CommandSwerveDrivetrain *m_chassis;

    /// @brief Field-centric drive request with heading control capability
    swerve::requests::FieldCentricFacingAngle m_driveRequest;

    //------------------------------------------------------------------
    // Pose Tracking Members
    //------------------------------------------------------------------

    /// @brief Flag indicating if robot pose hasn't changed (for stuck detection)
    bool m_isSamePose = false;

    /// @brief Robot pose from previous cycle for movement detection
    frc::Pose2d m_prevPose;

    /// @brief Current robot pose from odometry/vision fusion
    frc::Pose2d m_currentPose;

    /// @brief Target pose to navigate to
    frc::Pose2d m_targetPose;
    frc::Pose2d m_endPose;
    frc::Pose2d m_midPose;
    bool m_hasMidPose = false;
    bool m_beforeMidPose = true;
    units::angle::degree_t m_angleTolerance{20.0};
    units::length::inch_t m_xtoleranceForTransitionToEndPoint{0.25};
    units::length::inch_t m_yToleranceForTransitionToEndPoint{0.25};

    //------------------------------------------------------------------
    // Threshold and Range Constants
    //------------------------------------------------------------------

    /// @brief Distance threshold for considering target reached (default 0.25 inches)
    units::length::inch_t m_distanceThreshold{0.25};

    /// @brief Distance threshold for target regeneration (unused, legacy member)
    const units::length::inch_t m_regenerationDistanceThreshold{2.0};

    /// @brief Minimum radius for feedforward activation (0m = always active)
    const units::length::meter_t m_ffMinRadius{0.0};
    const units::length::meter_t m_ffMaxRadius{1.65};

    //------------------------------------------------------------------
    // Velocity and Acceleration Limits
    //------------------------------------------------------------------

    /// @brief Maximum translational velocity for the robot (4 m/s)
    const units::velocity::meters_per_second_t kMaxVelocity = 4_mps;
    const units::acceleration::meters_per_second_squared_t kMaxAcceleration = 3_mps_sq;

    /// @brief Maximum rotational velocity (540°/s, not actively used in current implementation)
    const units::angular_velocity::degrees_per_second_t kMaxAngularVelocity = 540_deg_per_s;

    //------------------------------------------------------------------
    // NetworkTables Keys (Legacy)
    //------------------------------------------------------------------

    /// @brief NetworkTables key for integral gain tuning (not actively used)
    std::string m_iGainKey = "I_Gain";

    /// @brief NetworkTables key for proportional gain tuning (not actively used)
    std::string m_pGainKey = "P_Gain";

    /// @brief Latch for run-once operations (not actively used)
    bool runOnceLatch = false;

    //------------------------------------------------------------------
    // Translation PID Gains
    //------------------------------------------------------------------

    /// @brief Proportional gain for X and Y translation controllers
    const double m_translationKP = 4.5;

    /// @brief Integral gain for X and Y translation controllers (disabled)
    const double m_translationKI = 0.0;

    /// @brief Derivative gain for X and Y translation controllers
    const double m_translationKD = 0.5;

    //------------------------------------------------------------------
    // Rotation PID Gains
    //------------------------------------------------------------------

    /// @brief Proportional gain for heading controller
    const double m_rotationKP = 6.0;

    /// @brief Integral gain for heading controller (disabled)
    const double m_rotationKI = 0.0;

    /// @brief Derivative gain for heading controller (disabled)
    const double m_rotationKD = 0.0;

    //------------------------------------------------------------------
    // Sweep and Error Tracking (Legacy/Unused)
    //------------------------------------------------------------------

    /// @brief Angular sweep delta for vision search patterns (not actively used)
    units::angle::degree_t m_sweepDelta{90.0};

    //------------------------------------------------------------------
    // Runtime State Variables
    //------------------------------------------------------------------

    /// @brief Current distance error from robot to target (updated in CalculateFeedForward)
    units::length::meter_t m_distanceError{0.0};

    /// @brief Threshold for PID reset - when error exceeds this, PID resets to prevent windup
    units::length::meter_t m_pidResetThreshold{0.25};

    /// @brief Precomputed feedforward range (m_ffMaxRadius - m_ffMinRadius) for scaling calculations
    units::length::meter_t m_feedForwardRange;

    //------------------------------------------------------------------
    // PID Controllers
    //------------------------------------------------------------------

    /// @brief Trapezoidal motion profile constraints for both X and Y controllers
    frc::TrapezoidProfile<units::length::meters>::Constraints m_translationConstraints{kMaxVelocity, kMaxAcceleration};

    /// @brief Profiled PID controller for X-axis translation with trapezoidal velocity profiles
    frc::ProfiledPIDController<units::length::meters> m_translationPIDX{m_translationKP, m_translationKI, m_translationKD, m_translationConstraints, 20_ms};

    /// @brief Profiled PID controller for Y-axis translation with trapezoidal velocity profiles
    frc::ProfiledPIDController<units::length::meters> m_translationPIDY{m_translationKP, m_translationKI, m_translationKD, m_translationConstraints, 20_ms};
};
