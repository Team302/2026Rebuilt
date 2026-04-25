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

//====================================================================================================================================================
/// @file DriveToPose.cpp
/// @brief Implementation of DriveToPose command for autonomous pose-based navigation
/// @details This command provides a base class for driving the robot to specific field poses using a combination
///          of feedforward and PID control. The control system uses:
///          - Separate ProfiledPIDControllers for X and Y translation
///          - Feedforward velocity scaling based on distance to target
///          - Heading control to face a target direction
///          - Adaptive PID reset to handle large position errors
///
///          The command is designed to be extended by derived classes that provide specific target poses
///          for different field elements or game-specific locations.
//====================================================================================================================================================

#include "chassis/commands/DriveToPose.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "state/RobotState.h"
#include "utils/AngleUtils.h"
#include "utils/PoseUtils.h"
#include "utils/logging/debug/Logger.h"

//------------------------------------------------------------------
/// @brief      Constructor for DriveToPose command
/// @param[in]  chassis - Pointer to the swerve drive subsystem used for navigation
/// @details    Initializes the command with the following configuration:
///
///             **PID Controller Setup:**
///             - Creates separate ProfiledPIDControllers for X and Y axes
///             - Configures I-Zone (integral zone) to prevent integral windup
///             - Uses trapezoidal motion profiles for smooth acceleration/deceleration
///
///             **Initial State:**
///             - Captures current robot pose as the previous pose reference
///             - Calculates feedforward range (max radius - min radius) for velocity scaling
///             - Registers chassis as a required subsystem
///
///             **Control Strategy:**
///             The command combines feedforward and feedback control:
///             - Feedforward: Provides base velocity towards target (scaled by distance)
///             - PID Feedback: Applies corrections for precise positioning
///             - Heading Control: Rotates robot to face target direction
///
/// @note       The command requires exclusive access to the chassis subsystem
/// @see        Initialize() for per-run setup when command is scheduled
//------------------------------------------------------------------
DriveToPose::DriveToPose(subsystems::CommandSwerveDrivetrain *chassis) : m_chassis(chassis)
{
    AddRequirements(m_chassis);

    // Store initial pose for movement detection
    m_prevPose = m_chassis != nullptr ? m_chassis->GetPose() : frc::Pose2d();

}

//------------------------------------------------------------------
/// @brief      Initializes the command when it starts execution
/// @details    Called once by the command scheduler when the command is first scheduled.
///             Performs the following initialization steps:
///
///             **Target Acquisition:**
///             - Calls GetDriveToPoses() to determine the target pose(s) (overridable by derived classes)
///             - Sets the end pose and optional mid pose as navigation goals
///
///             **State Preparation:**
///             - Resets the "same pose" tracker to detect when robot stops moving
///             - Updates the target pose for the control system
///
///             **Robot State Publishing:**
///             - Publishes DriveToFieldElement_Bool = true (navigation active)
///             - Publishes DriveToFinished_Bool = false (not yet complete)
///
///             These state changes notify other subsystems that autonomous navigation is in progress,
///             which may trigger coordinated behaviors or safety interlocks.
///
/// @note       Derived classes can override GetDriveToPoses() to provide custom target calculations
/// @see        GetDriveToPoses() for target pose determination
/// @see        SetTargetPose() for controller configuration
//------------------------------------------------------------------
void DriveToPose::Initialize()
{
    // Configure integral zones to prevent windup when far from target
    m_translationPIDX.SetIZone(0.5);
    m_translationPIDY.SetIZone(0.5);

    // Re-read velocity/acceleration limits here (not in constructor) because virtual dispatch
    // does not work during base-class construction — derived overrides (e.g. DriveOverBump)
    kMaxVelocity = GetMaxVelocity();
    kMaxAcceleration = GetMaxAcceleration();
    m_translationConstraints = frc::TrapezoidProfile<units::length::meters>::Constraints(kMaxVelocity, kMaxAcceleration);
    m_translationPIDX = frc::ProfiledPIDController<units::length::meters>(m_translationKP, m_translationKI, m_translationKD, m_translationConstraints, 20_ms);
    m_translationPIDY = frc::ProfiledPIDController<units::length::meters>(m_translationKP, m_translationKI, m_translationKD, m_translationConstraints, 20_ms);

    // Calculate the range over which feedforward velocity is ramped
    m_feedForwardRange = m_ffMaxRadius - m_ffMinRadius;
    
    // Get the target pose (may be overridden by derived classes)
    auto poses = GetDriveToPoses();
    m_midPointSpeed = poses.midPointSpeed;
    m_endPointSpeed = poses.endPointSpeed;
    m_endPose = poses.endPose;
    m_hasMidPose = poses.hasMidPose;

    if (poses.hasMidPose)
    {
        m_midPose = poses.midPose;
        m_beforeMidPose = ShouldSkipMidPoint() ? false : true; // Skip mid pose if angle to target is small
    }
    else
    {
        m_beforeMidPose = false;
    }

    // Reset movement detection to start fresh
    m_chassis->ResetSamePose();

    // Reset completion flag so each run starts clean
    m_isFinished = false;

    // Set current target speed based on which pose we're navigating to
    m_currentTargetSpeed = m_beforeMidPose ? m_midPointSpeed : m_endPointSpeed;

    // Configure controllers with the target pose
    SetTargetPose(m_beforeMidPose ? m_midPose : m_endPose);

    // Notify robot state that navigation has started
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFieldElement_Bool, true);
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFinished_Bool, false);
}

//------------------------------------------------------------------
/// @brief      Updates the target pose and resets controllers
/// @param[in]  endPose - The new target pose to navigate to
/// @details    This method allows the target pose to be changed during command execution,
///             useful for multi-stage navigation or dynamic target updates.
///
///             **Controller Reset:**
///             - Resets both X and Y PID controllers with current position and velocity
///             - Prevents control discontinuities when changing targets
///             - Maintains smooth motion by starting from current state
///
///             **Goal Update:**
///             - Sets new X and Y position goals for the PID controllers
///             - Controllers will now navigate toward the new target
///
///             This is used internally during Initialize() and can be called by derived classes
///             to implement multi-waypoint navigation (e.g., DriveOverBump's two-stage approach).
///
/// @note       Calling this during execution will cause the robot to smoothly redirect to the new target
/// @see        DriveOverBump::IsFinished() for an example of dynamic target switching
//------------------------------------------------------------------
void DriveToPose::SetTargetPose(const frc::Pose2d &pose)
{
    m_targetPose = pose;
    if (m_chassis != nullptr)
    {

        // Get current robot velocities for smooth controller reset
        auto speeds = m_chassis->GetState().Speeds;

        // Reset PID controllers with current state to prevent discontinuities
        m_translationPIDX.Reset(m_currentPose.X(), speeds.vx);
        m_translationPIDY.Reset(m_currentPose.Y(), speeds.vy);

        // Update current pose and set new controller goals
        m_currentPose = m_chassis->GetPose();
        m_translationPIDX.SetGoal(m_targetPose.X());
        m_translationPIDY.SetGoal(m_targetPose.Y());
    }
}

//------------------------------------------------------------------
/// @brief      Executes the command periodically during autonomous navigation
/// @details    Called repeatedly by the command scheduler (typically every 20ms) while the command is active.
///             Implements a hybrid control strategy combining feedforward and feedback control:
///
///             **Control Flow:**
///             1. Update current robot pose from odometry/vision fusion
///             2. Calculate feedforward velocities based on distance to target
///             3. Determine if PID reset is needed (large position error)
///             4. Apply PID corrections for fine positioning
///             5. Clamp velocities to maximum limits
///             6. Send combined control to drivetrain with heading control
///             7. Log error and status for debugging
///
///             **Adaptive PID Reset:**
///             When distance error exceeds m_pidResetThreshold, the PID controllers are reset
///             to prevent integral windup. This happens when:
///             - Target is changed dynamically (multi-stage navigation)
///             - Robot is bumped or pushed off course
///             - Initial error is large at command start
///
///             **Control Modes:**
///             - Large error: Feedforward only, PID reset
///             - Small error: Feedforward + PID corrections
///
///             **Heading Control:**
///             Robot rotation is controlled separately to face m_targetPose.Rotation() using
///             a heading PID controller with gains.
///
/// @note       All movements are field-centric relative to blue alliance perspective
/// @see        CalculateFeedForward() for feedforward velocity calculation
/// @see        IsFinished() for completion detection
//------------------------------------------------------------------
void DriveToPose::Execute()
{
    frc::ChassisSpeeds chassisSpeeds{};
    if (m_chassis != nullptr)
    {
        // Update current position from odometry
        m_currentPose = m_chassis->GetPose();

        // Calculate base feedforward velocities toward target
        CalculateFeedForward(chassisSpeeds);

        // Adaptive control: Reset PID when error is large, apply corrections when error is small
        if (m_distanceError > m_pidResetThreshold)
        {
            // Large error: Reset PIDs to prevent windup, rely on feedforward
            m_translationPIDX.Reset(m_currentPose.X(), chassisSpeeds.vx);
            m_translationPIDY.Reset(m_currentPose.Y(), chassisSpeeds.vy);
        }
        else
        {
            // Small error: Add PID corrections for precise positioning
            chassisSpeeds.vx += units::velocity::meters_per_second_t(m_translationPIDX.Calculate(m_currentPose.X(), m_targetPose.X()));
            chassisSpeeds.vy += units::velocity::meters_per_second_t(m_translationPIDY.Calculate(m_currentPose.Y(), m_targetPose.Y()));

            // Clamp velocities to safe maximum
            chassisSpeeds.vx = std::clamp(chassisSpeeds.vx, -kMaxVelocity, kMaxVelocity);
            chassisSpeeds.vy = std::clamp(chassisSpeeds.vy, -kMaxVelocity, kMaxVelocity);
        }

        // Send control request to drivetrain with heading control
        m_chassis->SetControl(
            m_driveRequest.WithVelocityX(chassisSpeeds.vx)
                .WithVelocityY(chassisSpeeds.vy)
                .WithTargetDirection(m_targetPose.Rotation().Degrees())
                .WithHeadingPID(m_rotationKP, m_rotationKI, m_rotationKD)
                .WithForwardPerspective(ctre::phoenix6::swerve::requests::ForwardPerspectiveValue::BlueAlliance));
    }

    // Log current error for debugging and tuning (reuse m_distanceError computed in CalculateFeedForward)
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToPose", "Vx", units::math::abs(chassisSpeeds.vx).value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToPose", "Vy", units::math::abs(chassisSpeeds.vy).value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToPose", "Error", units::length::inch_t(m_distanceError).value());
}

//------------------------------------------------------------------
/// @brief      Determines if the command has completed navigation to the target pose
/// @return     bool - true if navigation is complete or robot is stuck, false to continue execution
/// @details    Implements a two-condition completion check to handle both successful completion
///             and stuck robot detection:
///
///             **Condition 1: Target is at the origin**
///             - Checks if the end pose is at the origin (0,0) with a small tolerance
///             - Returns true if end pose is at origin, indicating an error in target calculation (physically the robot
///               center cannot be at the origin of the field).
///
///             **Condition 2: Target Reached**
///             - Compares current pose to target pose using m_distanceThreshold
///             - Returns true if robot is within tolerance of target position
///             - Indicates successful navigation completion
///
///             **Condition 3: Robot Stopped Moving**
///             - Uses chassis IsSamePose() to detect if robot hasn't moved between cycles
///             - Returns true if robot is stuck or unable to make progress
///             - Prevents command from running indefinitely if blocked
///
///             **Debug Logging:**
///             - Logs "Is Done" status (target reached)
///             - Logs "Is SamePose" status (movement detection)
///             - Useful for tuning distance thresholds and diagnosing navigation issues
///
///             The stuck detection provides graceful failure handling - if the robot can't reach
///             the target (blocked by game pieces, defense, field obstacles), the command will
///             eventually timeout rather than running indefinitely.
///
/// @note       Updates m_prevPose each cycle for next iteration's comparison
/// @note       Distance threshold (m_distanceThreshold) can be customized via SetDistanceThreshold()
/// @see        SetDistanceThreshold() for adjusting the completion distance threshold
//------------------------------------------------------------------
bool DriveToPose::IsFinished()
{
    // Safety check: If end pose wasn't calculated properly, stop immediately
    if (PoseUtils::IsPoseAtOrigin(m_targetPose, units::length::centimeter_t{1.0}))
    {
        m_isFinished = true;
        return true;
    }

    // Determine which distance threshold to use based on whether we have speed targets
    auto distanceThreshold = (m_beforeMidPose && m_midPointSpeed > 0_mps) || (!m_beforeMidPose && m_endPointSpeed > 0_mps)
                                 ? m_distanceThresholdWithSpeed
                                 : m_distanceThreshold;

    // Check if we've reached the target pose within tolerance
    bool isDone = PoseUtils::IsSamePose(m_currentPose, m_targetPose, distanceThreshold);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToPose", "Is Done", isDone);

    // Handle transition from mid pose to end pose
    if (m_hasMidPose && m_beforeMidPose)
    {
        // Transition if mid pose reached or within tolerance to end pose
        if (isDone ||
            (units::math::abs(m_endPose.X() - m_currentPose.X()) < m_xtoleranceForTransitionToEndPoint &&
             units::math::abs(m_endPose.Y() - m_currentPose.Y()) < m_yToleranceForTransitionToEndPoint) ||
            ShouldSkipMidPoint())
        {
            m_currentTargetSpeed = m_endPointSpeed;
            SetTargetPose(m_endPose);
            m_beforeMidPose = false;
            m_isFinished = false;
            return false; // Continue execution to reach end pose
        }
    }

    // Check if robot has stopped moving (stuck or blocked)
    bool isSamePose = m_chassis->IsSamePose();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToPose", "Is SamePose", isSamePose);

    m_prevPose = m_currentPose; // Update for next cycle

    return (m_isFinished = isDone || isSamePose);
}

//------------------------------------------------------------------
/// @brief      Cleans up and stops the robot when the command ends
/// @param[in]  interrupted - true if the command was interrupted by another command,
///                          false if it finished normally via IsFinished()
/// @details    Called once by the command scheduler when the command terminates.
///             Performs cleanup operations:
///
///             **Robot Control:**
///             - Applies SwerveDriveBrake to immediately stop all wheel motion
///             - Ensures robot doesn't coast after command ends
///             - Important for safety and precise positioning
///
///             **State Publishing:**
///             - Updates DriveToFinished_Bool with completion status from IsFinished()
///             - Updates DriveToFieldElement_Bool to false (navigation no longer active)
///             - Notifies other subsystems that autonomous navigation has ended
///
///             The interrupted parameter indicates how the command ended:
///             - false: Command completed normally (reached target or detected stuck)
///             - true: Command was preempted by another command or manually cancelled
///
///             In both cases, the robot is stopped and state is updated to reflect
///             that navigation is no longer active.
///
/// @note       This is called automatically by the command scheduler, not manually
/// @see        Initialize() for the corresponding startup method
//------------------------------------------------------------------
void DriveToPose::End(bool interrupted)
{
    // Immediately stop the robot
    if (m_chassis != nullptr)
    {
        m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
    }

    // Publish completion status to robot state (use cached result to avoid redundant recomputation)
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFinished_Bool, m_isFinished);
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFieldElement_Bool, false);
}

//------------------------------------------------------------------
/// @brief      Calculates feedforward velocity component toward the target pose
/// @param[out] chassisSpeeds - Reference to chassis speeds structure that will be populated
///                             with calculated feedforward velocities (vx and vy)
/// @details    Implements a distance-based feedforward velocity profile that provides smooth
///             approach behavior and reduces overshoot:
///
///             **Distance-Based Scaling:**
///             The feedforward velocity is scaled based on distance to target using three zones:
///
///             1. **Near Target (distance ≤ m_ffMinRadius = 0.0m):**
///                - feedforwardSpeed = 0 m/s
///                - Relies entirely on PID for final positioning
///                - Prevents overshoot from feedforward
///
///             2. **Ramping Zone:**
///                - feedforwardSpeed linearly scaled from 0 to kMaxVelocity
///                - Scale factor = (distance - minRadius) / (maxRadius - minRadius)
///                - Provides smooth acceleration as robot approaches target
///
///             3. **Far From Target (distance > m_ffMaxRadius):**
///                - feedforwardSpeed = kMaxVelocity
///                - Full speed when target is distant
///                - Minimizes navigation time for long distances
///
///             **Velocity Decomposition:**
///             The calculated speed is directed along the angle to the target:
///             - vx = feedforwardSpeed × cos(angleToTarget)
///             - vy = feedforwardSpeed × sin(angleToTarget)
///
///             This provides the base velocity that PID corrections are added to in Execute().
///
/// @note       Distance error (m_distanceError) is updated and used by Execute() for PID reset logic
/// @note       Feedforward range (m_feedForwardRange) is precomputed in constructor
/// @see        Execute() for how feedforward combines with PID control
//------------------------------------------------------------------
void DriveToPose::CalculateFeedForward(frc::ChassisSpeeds &chassisSpeeds)
{
    if (m_chassis != nullptr)
    {
        // Calculate Euclidean distance from current position to target
        m_distanceError = m_currentPose.Translation().Distance(m_currentTargetSpeed > 0_mps ? CalculatePoseWithTargetSpeed(m_targetPose, m_currentTargetSpeed).Translation() : m_targetPose.Translation());

        // Determine feedforward speed based on distance using ramped profile
        units::velocity::meters_per_second_t feedforwardSpeed = 0.0_mps;
        if (m_distanceError > m_ffMinRadius)
        {
            // Scale feedforward linearly within the ramp range, clamp to [0.0, 1.0]
            double feedForwardScale = std::clamp(((m_distanceError - m_ffMinRadius) / (m_feedForwardRange)).value(), 0.0, 1.0);
            feedforwardSpeed = kMaxVelocity * feedForwardScale;
        }
        // else: Within minimum radius, feedforward = 0 (rely on PID only)

        // Calculate direction vector from current position to target
        frc::Translation2d translationError = m_targetPose.Translation() - m_currentPose.Translation();
        frc::Rotation2d angleToTarget = translationError.Angle();

        // Decompose feedforward velocity into X and Y components
        chassisSpeeds.vx = feedforwardSpeed * angleToTarget.Cos();
        chassisSpeeds.vy = feedforwardSpeed * angleToTarget.Sin();
    }
}

/**
 * @brief Determines whether the midpoint target should be skipped based on the robot's alignment with the end pose.
 *
 * This method calculates the angle between the robot's current position and the final target position.
 * If the robot is already aligned within the specified angle tolerance, the midpoint can be skipped
 * for a more direct path to the end pose.
 *
 * @return true if the angle to the end pose is within the angle tolerance and the midpoint should be skipped
 * @return false if the robot should navigate through the midpoint before reaching the end pose
 */
bool DriveToPose::ShouldSkipMidPoint() const
{
    auto currentPose = m_chassis->GetPose();
    auto rotationToEnd = (currentPose.Translation() - m_endPose.Translation()).Angle();
    auto angleToEnd = units::math::abs(AngleUtils::GetEquivAngle(rotationToEnd.Degrees()));
    angleToEnd = std::min(angleToEnd, 180.0_deg - angleToEnd);
    return angleToEnd < m_angleTolerance;
}

frc::Pose2d DriveToPose::CalculatePoseWithTargetSpeed(const frc::Pose2d &targetPose, const units::velocity::meters_per_second_t &speed) const
{
    units::length::meter_t requiredDistance = ((speed / kMaxVelocity) * m_feedForwardRange) + m_ffMinRadius;

    frc::Translation2d translationError = targetPose.Translation() - m_currentPose.Translation();
    frc::Rotation2d approachAngle = translationError.Angle();
    frc::Translation2d lookAheadOffset{requiredDistance, approachAngle};

    return frc::Pose2d(targetPose.Translation() + lookAheadOffset, targetPose.Rotation());
}