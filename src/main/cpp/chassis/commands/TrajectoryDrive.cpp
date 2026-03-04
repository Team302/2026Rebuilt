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

// FRC Includes
#include "auton/drivePrimitives/AutonUtils.h"
#include "chassis/commands/TrajectoryDrive.h"
#include "choreo/Choreo.h"
#include "frc/DriverStation.h"
#include "frc/Timer.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "state/RobotState.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"
#include "utils/FMSData.h"
#include "utils/PoseUtils.h"
#include "utils/logging/debug/Logger.h"

TrajectoryDrive::TrajectoryDrive(
    subsystems::CommandSwerveDrivetrain *chassis) : m_chassis(chassis),
                                                    m_pathName(""),
                                                    m_trajectoryStates(),
                                                    m_prevPose(),
                                                    m_wasMoving(false),
                                                    m_timer(std::make_unique<frc::Timer>()),
                                                    m_whyDone("Trajectory isn't finished/Error"),
                                                    m_totalTrajectoryTime(0.0_s)
{
    // This command requires the chassis subsystem
    AddRequirements(m_chassis);
    // Enable continuous input for the heading controller for proper wrap-around
    m_headingController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
}

void TrajectoryDrive::Initialize()
{
    m_trajectory = AutonUtils::GetTrajectoryFromPathFile(m_pathName);
    if (m_trajectory.has_value())
    {
        auto trajectory = m_trajectory.value();
        m_trajectoryStates = trajectory.samples;
        m_totalTrajectoryTime = trajectory.GetTotalTime();
        m_thresholdTime = m_totalTrajectoryTime * kPercentComplete;
        auto finalPose = trajectory.GetFinalPose(FMSData::GetAllianceColor() == frc::DriverStation::Alliance::kRed);
        m_finalPose = finalPose.has_value() ? finalPose.value() : frc::Pose2d();
    }
    else
    {
        m_totalTrajectoryTime = 0_s;
        m_thresholdTime = 0_s;
        m_trajectoryStates.clear();
        m_finalPose = frc::Pose2d();
    }
    m_previousPose = frc::Pose2d();
    m_numberOfExecutions = 0;

    // Reset and start the timer when the command begins
    m_timer.get()->Reset();
    m_timer.get()->Start();

    // Reset PID controllers to clear any previous state
    m_xController.Reset();
    m_yController.Reset();
    m_headingController.Reset();
    m_chassisSpeeds.vx = 0_mps;
    m_chassisSpeeds.vy = 0_mps;
    m_chassisSpeeds.omega = units::angular_velocity::radians_per_second_t(0);

    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFinished_Bool, false);
}

void TrajectoryDrive::SetPath(const std::string &pathName)
{
    m_pathName = pathName;
}

void TrajectoryDrive::Execute()
{
    m_elapsedTime = m_timer->Get();  // cache once; reused by IsFinished() this cycle
    if (!m_trajectoryStates.empty()) // If we have a path parsed / have states to run
    {
        auto desiredState = m_trajectory.value().SampleAt(m_elapsedTime).value();
        if (m_chassis != nullptr)
        {
            auto currentPose = m_chassis->GetPose();

            units::meters_per_second_t xFeedback{m_xController.Calculate(currentPose.X().value(), desiredState.x.value())};
            units::meters_per_second_t yFeedback{m_yController.Calculate(currentPose.Y().value(), desiredState.y.value())};
            units::radians_per_second_t headingFeedback{m_headingController.Calculate(currentPose.Rotation().Radians().value(), desiredState.heading.value())};

            // Generate the next speeds for the robot
            m_chassisSpeeds.vx = desiredState.vx + xFeedback;
            m_chassisSpeeds.vy = desiredState.vy + yFeedback;
            m_chassisSpeeds.omega = desiredState.omega + headingFeedback;
        }
    }

    m_chassis->SetControl(
        m_driveRequest.WithVelocityX(m_chassisSpeeds.vx)
            .WithVelocityY(m_chassisSpeeds.vy)
            .WithRotationalRate(m_chassisSpeeds.omega)
            .WithForwardPerspective(ctre::phoenix6::swerve::requests::ForwardPerspectiveValue::BlueAlliance));
    m_numberOfExecutions++;
}

bool TrajectoryDrive::IsFinished()
{
    if (m_trajectoryStates.empty())
    {
        auto currentTime = m_timer.get()->Get();

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "current time", currentTime.value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "total time", m_totalTrajectoryTime.value());

        if ((currentTime) / m_totalTrajectoryTime > 0.9)
        {

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "current pose X", currentPose.X().value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "current pose Y", currentPose.Y().value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "current pose Rotation", currentPose.Rotation().Degrees().value());

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "target pose X", m_finalState.GetPose().X().value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "target pose Y", m_finalState.GetPose().Y().value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "target pose Rotation", m_finalState.GetPose().Rotation().Degrees().value());

            isDone = IsSamePose(currentPose, m_finalState.GetPose(), m_chassisSpeeds, 1.0, 3.0, 1.5); // TO DO verify these values
        }
        else if (m_chassis != nullptr)
        {
            isDone = m_chassis->IsSamePose();
        }
    }
    else
    {
        m_whyDone = "No states in trajectory";
        isDone = true;
        m_whyDone = "Trajectory states are empty";
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "", "why done", m_whyDone);
        return true;
    }

    if (m_chassis == nullptr)
    {
        m_whyDone = "Chassis is null";
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "", "why done", m_whyDone);
        return true;
    }

    auto currentPose = m_chassis->GetPose();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "current time", m_elapsedTime.value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "total time", m_totalTrajectoryTime.value());

    if (m_elapsedTime > m_thresholdTime && m_numberOfExecutions >= kMinExecutions) // avoids a division every loop
    {
        auto isSamePose = PoseUtils::IsSamePose(currentPose, m_finalPose, kPositionTolerance, kHeadingTolerance);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "is same pose?", isSamePose);
        if (isSamePose)
        {
            // previously also compared the robot's velocity to a tolerance of 1.5 m/s
            // right now skipping this because it doesn't make sense to me but here
            // is where to add it in (and if we do we should compare the squares to the
            // 1.5^2 to avoid the sqrt calculation)
            m_whyDone = "Robot is at the target pose";
            return true;
        }
        else if (PoseUtils::IsSamePose(currentPose, m_previousPose, kPositionTolerance, kHeadingTolerance))
        {
            m_whyDone = "Robot is not moving but is not at the target pose";
            return true;
        }
        m_whyDone = "Robot is not at the target pose";
        m_previousPose = currentPose; // update previous pose for next loop's comparison
    }
    return false;
}

void TrajectoryDrive::End(bool interrupted)
{
    // When the command ends (or is interrupted), stop the robot.
    m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
}
