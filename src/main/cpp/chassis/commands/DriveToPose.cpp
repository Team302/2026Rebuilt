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
#include "chassis/commands/DriveToPose.h" // Update path if needed
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "state/RobotState.h"
#include "utils/AngleUtils.h"
#include "utils/PoseUtils.h"
#include "utils/logging/debug/Logger.h"

//------------------------------------------------------------------
/// @brief      Constructor for DriveToPose command
/// @param[in]  chassis - Pointer to the swerve drive subsystem
/// @details    Initializes the command with PID controllers for X and Y
///             translation. Sets up I-zone for integral control, captures
///             the initial pose, and calculates the feedforward range.
///             The command uses a combination of feedforward and PID
///             control for smooth and accurate pose targeting.
//------------------------------------------------------------------
DriveToPose::DriveToPose(
    subsystems::CommandSwerveDrivetrain *chassis) : m_chassis(chassis)

{
    AddRequirements(m_chassis);
    m_translationPIDX.SetIZone(0.20);
    m_translationPIDY.SetIZone(0.20);
    m_prevPose = m_chassis != nullptr ? m_chassis->GetPose() : frc::Pose2d();
    m_feedForwardRange = m_ffMaxRadius - m_ffMinRadius;
}

//------------------------------------------------------------------
/// @brief      Initializes the command when it starts
/// @details    Called once when the command is scheduled. Resets the
///             same pose tracker, retrieves current chassis speeds,
///             gets the target end pose, and resets both PID controllers
///             with the current position and velocity. Sets the PID
///             goals to the target end pose coordinates.
//------------------------------------------------------------------
void DriveToPose::Initialize()
{
    m_chassis->ResetSamePose();
    auto speeds = m_chassis->GetState().Speeds;

    m_endPose = GetEndPose();

    m_translationPIDX.Reset(m_currentPose.X(), speeds.vx);
    m_translationPIDY.Reset(m_currentPose.Y(), speeds.vy);

    if (m_chassis != nullptr)
    {
        m_currentPose = m_chassis->GetPose();
        m_translationPIDX.SetGoal(m_endPose.X());
        m_translationPIDY.SetGoal(m_endPose.Y());
    }
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFieldElement_Bool, true);
}

//------------------------------------------------------------------
/// @brief      Executes the command periodically
/// @details    Called repeatedly while the command is scheduled (typically
///             every 20ms). Updates the current pose, calculates feedforward
///             velocity based on distance to target, and applies PID control
///             for fine adjustments. If the distance error exceeds the reset
///             threshold, the PID controllers are reset to prevent windup.
///             Otherwise, PID corrections are added to feedforward speeds
///             and clamped to maximum velocity. The resulting chassis speeds
///             are sent to the drivetrain with heading control to face the
///             target rotation. Logs error and completion status.
//------------------------------------------------------------------
void DriveToPose::Execute()
{
    frc::ChassisSpeeds chassisSpeeds{};
    if (m_chassis != nullptr)
    {

        m_currentPose = m_chassis->GetPose();
        CalculateFeedForward(chassisSpeeds);

        if (m_distanceError > m_pidResetThreshold)
        {
            m_translationPIDX.Reset(m_currentPose.X(), chassisSpeeds.vx);
            m_translationPIDY.Reset(m_currentPose.Y(), chassisSpeeds.vy);
        }
        else
        {
            chassisSpeeds.vx += units::velocity::meters_per_second_t(m_translationPIDX.Calculate(m_currentPose.X(), m_endPose.X()));
            chassisSpeeds.vy += units::velocity::meters_per_second_t(m_translationPIDY.Calculate(m_currentPose.Y(), m_endPose.Y()));
            chassisSpeeds.vx = std::clamp(chassisSpeeds.vx, -kMaxVelocity, kMaxVelocity);
            chassisSpeeds.vy = std::clamp(chassisSpeeds.vy, -kMaxVelocity, kMaxVelocity);
        }
    }
    m_chassis->SetControl(
        m_driveRequest.WithVelocityX(chassisSpeeds.vx)
            .WithVelocityY(chassisSpeeds.vy)
            .WithTargetDirection(m_endPose.Rotation().Degrees())
            .WithHeadingPID(m_rotationKP, m_rotationKI, m_rotationKD)
            .WithForwardPerspective(ctre::phoenix6::swerve::requests::ForwardPerspectiveValue::BlueAlliance));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "Error", m_endPose.Translation().Distance(m_currentPose.Translation()).value());
}

//------------------------------------------------------------------
/// @brief      Determines if the command has finished
/// @return     bool - true if the robot has reached the target pose or
///             stopped moving, false otherwise
/// @details    Checks two completion conditions:
///             1. Robot pose matches target pose within distance threshold
///             2. Robot has stopped moving (same pose as previous cycle)
///             Logs the completion status and same-pose status for debugging.
///             Updates the previous pose for next cycle comparison.
//------------------------------------------------------------------
bool DriveToPose::IsFinished()
{
    bool isDone = PoseUtils::IsSamePose(m_currentPose, m_endPose, m_distanceThreshold);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "Is Done", isDone);
    if (isDone)
    {
        return true;
    }

    auto isSamePose = m_chassis->IsSamePose();
    m_prevPose = m_currentPose;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "Is SamePose", isSamePose);
    return isSamePose;
}

//------------------------------------------------------------------
/// @brief      Ends the command
/// @param[in]  interrupted - true if the command was interrupted,
///             false if it finished normally
/// @details    Called once when the command ends, either by finishing
///             normally or being interrupted. Applies a brake request
///             to stop the robot's motion.
//------------------------------------------------------------------
void DriveToPose::End(bool interrupted)
{
    m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFieldElement_Bool, false);
}

//------------------------------------------------------------------
/// @brief      Calculates feedforward velocity towards the target
/// @param[out] chassisSpeeds - Chassis speeds structure to populate
///             with calculated feedforward velocities
/// @details    Computes the distance error to the target and applies
///             a scaled feedforward velocity based on the error magnitude.
///             Uses a ramping profile within the feedforward range:
///             - No feedforward if within minimum radius
///             - Linearly scaled feedforward between min and max radius
///             - Full feedforward speed beyond max radius
///             The feedforward velocity is directed along the angle
///             towards the target, decomposed into X and Y components.
///             This provides smooth approach behavior and reduces
///             overshoot near the target.
//------------------------------------------------------------------
void DriveToPose::CalculateFeedForward(frc::ChassisSpeeds &chassisSpeeds)
{
    if (m_chassis != nullptr)
    {
        m_distanceError = m_currentPose.Translation().Distance(m_endPose.Translation());

        units::velocity::meters_per_second_t feedforwardSpeed = 0.0_mps;
        if (m_distanceError > m_ffMinRadius)
        {
            double feedForwardScale = std::clamp(((m_distanceError - m_ffMinRadius) / (m_feedForwardRange)).value(), 0.0, 1.0);
            feedforwardSpeed = kMaxVelocity * feedForwardScale;
        }

        frc::Translation2d translationError = m_endPose.Translation() - m_currentPose.Translation();
        frc::Rotation2d angleToTarget = translationError.Angle();

        chassisSpeeds.vx = feedforwardSpeed * angleToTarget.Cos();
        chassisSpeeds.vy = feedforwardSpeed * angleToTarget.Sin();
    }
}
