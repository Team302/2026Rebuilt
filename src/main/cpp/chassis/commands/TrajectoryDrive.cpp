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
#include <cmath>

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
#include "utils/PoseUtils.h"

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
    m_startTimeOffset = 0.0_s;
    m_useSmartJoin = false;
    m_isApproachingPath = false;

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

void TrajectoryDrive::InitializeForTeleop(bool generateRedTrajectory, TrajectoryMatchStrategy matchStrategy, units::length::meter_t distanceTolerance)
{
    m_trajectory = AutonUtils::GetTrajectoryFromPathFile(m_pathName, generateRedTrajectory);
    if (m_trajectory.has_value())
    {
        auto trajectory = m_trajectory.value();
        m_trajectoryStates = trajectory.samples;
        m_totalTrajectoryTime = trajectory.GetTotalTime();
        m_thresholdTime = m_totalTrajectoryTime * kPercentComplete;
        auto finalPose = trajectory.GetFinalPose(generateRedTrajectory);
        m_finalPose = finalPose.has_value() ? finalPose.value() : frc::Pose2d();

        // Smart initialization: Find the closest point in the trajectory to start from
        if (m_chassis != nullptr && !m_trajectoryStates.empty())
        {
            auto currentPose = m_chassis->GetPose();
            size_t closestIndex = FindClosestTrajectoryPoint(currentPose, matchStrategy, distanceTolerance);

            // Calculate distance to the closest point
            const auto &closestSample = m_trajectoryStates[closestIndex];
            frc::Pose2d closestPose{closestSample.x, closestSample.y, closestSample.heading};
            units::length::meter_t distanceToPath = CalculateDistance(currentPose, closestPose, matchStrategy);

            // If we're not within tolerance, we need to approach the path first
            if (distanceToPath > distanceTolerance)
            {
                m_useSmartJoin = true;
                m_isApproachingPath = true;
                m_matchStrategy = matchStrategy;
                m_joinTolerance = distanceTolerance;
                m_targetJoinIndex = closestIndex;
                m_targetJoinPose = closestPose;
                m_startTimeOffset = 0.0_s; // Don't start trajectory timer yet
            }
            else
            {
                // We're already close enough, start following trajectory from this point
                m_useSmartJoin = false;
                m_isApproachingPath = false;
                m_startTimeOffset = closestSample.timestamp;
            }
        }
        else
        {
            m_useSmartJoin = false;
            m_isApproachingPath = false;
            m_startTimeOffset = 0.0_s;
        }
    }
    else
    {
        m_totalTrajectoryTime = 0_s;
        m_thresholdTime = 0_s;
        m_trajectoryStates.clear();
        m_finalPose = frc::Pose2d();
        m_startTimeOffset = 0.0_s;
        m_useSmartJoin = false;
        m_isApproachingPath = false;
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
    if (m_chassis == nullptr || m_trajectoryStates.empty())
    {
        return;
    }

    // Phase 1: Approaching the path (if enabled)
    if (m_useSmartJoin && m_isApproachingPath)
    {
        auto currentPose = m_chassis->GetPose();
        units::length::meter_t distanceToTarget = CalculateDistance(currentPose, m_targetJoinPose, m_matchStrategy);

        // Check if we've reached the target join point
        if (distanceToTarget <= m_joinTolerance)
        {
            m_isApproachingPath = false;
            m_startTimeOffset = m_trajectoryStates[m_targetJoinIndex].timestamp;
            m_timer.get()->Reset();
            m_timer.get()->Start();
        }
        else
        {
            // Drive toward the join point
            units::meters_per_second_t vx = 0_mps;
            units::meters_per_second_t vy = 0_mps;

            if (m_matchStrategy == TrajectoryMatchStrategy::MATCH_Y_ONLY)
            {
                units::meters_per_second_t yFeedback{m_yController.Calculate(currentPose.Y().value(), m_targetJoinPose.Y().value())};
                vy = yFeedback;
                vx = 0_mps;
            }
            else if (m_matchStrategy == TrajectoryMatchStrategy::MATCH_X_ONLY)
            {
                units::meters_per_second_t xFeedback{m_xController.Calculate(currentPose.X().value(), m_targetJoinPose.X().value())};
                vx = xFeedback;
                vy = 0_mps;
            }
            else // MATCH_XY
            {
                vx = units::meters_per_second_t{m_xController.Calculate(currentPose.X().value(), m_targetJoinPose.X().value())};
                vy = units::meters_per_second_t{m_yController.Calculate(currentPose.Y().value(), m_targetJoinPose.Y().value())};
            }

            units::radians_per_second_t omega{m_headingController.Calculate(currentPose.Rotation().Radians().value(), m_targetJoinPose.Rotation().Radians().value())};

            m_chassisSpeeds.vx = vx;
            m_chassisSpeeds.vy = vy;
            m_chassisSpeeds.omega = omega;

            m_chassis->SetControl(
                m_driveRequest.WithVelocityX(m_chassisSpeeds.vx)
                    .WithVelocityY(m_chassisSpeeds.vy)
                    .WithRotationalRate(m_chassisSpeeds.omega)
                    .WithForwardPerspective(ctre::phoenix6::swerve::requests::ForwardPerspectiveValue::BlueAlliance));

            return; // Don't follow trajectory yet
        }
    }

    // Phase 2: Following the trajectory
    m_elapsedTime = m_timer->Get() + m_startTimeOffset; // Add offset for mid-trajectory starts
    if (!m_trajectoryStates.empty())
    {
        auto desiredState = m_trajectory.value().SampleAt(m_elapsedTime).value();
        if (m_chassis != nullptr)
        {
            auto currentPose = m_chassis->GetPose();

            units::meters_per_second_t xFeedback{m_xController.Calculate(currentPose.X().value(), desiredState.x.value())};
            units::meters_per_second_t yFeedback{m_yController.Calculate(currentPose.Y().value(), desiredState.y.value())};
            units::radians_per_second_t headingFeedback{m_headingController.Calculate(currentPose.Rotation().Radians().value(), desiredState.heading.value())};

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
        m_whyDone = "Trajectory states are empty";
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "", "why done", m_whyDone);
        return true;
    }
    else if (m_chassis == nullptr)
    {
        m_whyDone = "Chassis is null";
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "", "why done", m_whyDone);
        return true;
    }
    else if (m_useSmartJoin && m_isApproachingPath)
    {
        return false;
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

//------------------------------------------------------------------
/// @brief Find the closest point in the trajectory to the robot's current position
/// @details Uses the specified match strategy to find the optimal starting point
///          in the trajectory. This allows the robot to join the trajectory at
///          the most appropriate point rather than always starting from the beginning.
///
///          For MATCH_Y_ONLY: Finds the trajectory point closest to robot's current X,
///                            then picks the one with closest Y at that X position
///          For MATCH_X_ONLY: Finds the trajectory point closest to robot's current Y,
///                            then picks the one with closest X at that Y position
///          For MATCH_XY: Finds the point with minimum Euclidean distance
//------------------------------------------------------------------
size_t TrajectoryDrive::FindClosestTrajectoryPoint(const frc::Pose2d &currentPose, TrajectoryMatchStrategy matchStrategy, units::length::meter_t tolerance) const
{
    if (m_trajectoryStates.empty())
    {
        return 0;
    }

    size_t closestIndex = 0;
    units::length::meter_t minDistance = std::numeric_limits<double>::max() * 1_m;

    if (matchStrategy == TrajectoryMatchStrategy::MATCH_Y_ONLY)
    {
        // For Y-only matching: find the point closest in X, then check Y distance at that X
        // This ensures we join the trajectory at our current X position (or closest to it)
        units::length::meter_t minXDistance = std::numeric_limits<double>::max() * 1_m;
        units::length::meter_t bestYDistance = std::numeric_limits<double>::max() * 1_m;

        for (size_t i = 0; i < m_trajectoryStates.size(); ++i)
        {
            const auto &sample = m_trajectoryStates[i];
            units::length::meter_t xDistance = units::math::abs(sample.x - currentPose.X());
            units::length::meter_t yDistance = units::math::abs(sample.y - currentPose.Y());

            // Find points that are close in X (within a reasonable window)
            // Then among those, pick the one closest in Y
            if (xDistance < minXDistance + 0.3_m) // 0.3m window for X matching
            {
                if (xDistance < minXDistance || (xDistance <= minXDistance + 0.1_m && yDistance < bestYDistance))
                {
                    minXDistance = xDistance;
                    bestYDistance = yDistance;
                    closestIndex = i;
                }
            }
        }
    }
    else if (matchStrategy == TrajectoryMatchStrategy::MATCH_X_ONLY)
    {
        // For X-only matching: find the point closest in Y, then check X distance at that Y
        units::length::meter_t minYDistance = std::numeric_limits<double>::max() * 1_m;
        units::length::meter_t bestXDistance = std::numeric_limits<double>::max() * 1_m;

        for (size_t i = 0; i < m_trajectoryStates.size(); ++i)
        {
            const auto &sample = m_trajectoryStates[i];
            units::length::meter_t xDistance = units::math::abs(sample.x - currentPose.X());
            units::length::meter_t yDistance = units::math::abs(sample.y - currentPose.Y());

            // Find points that are close in Y (within a reasonable window)
            // Then among those, pick the one closest in X
            if (yDistance < minYDistance + 0.3_m) // 0.3m window for Y matching
            {
                if (yDistance < minYDistance || (yDistance <= minYDistance + 0.1_m && xDistance < bestXDistance))
                {
                    minYDistance = yDistance;
                    bestXDistance = xDistance;
                    closestIndex = i;
                }
            }
        }
    }
    else // MATCH_XY
    {
        for (size_t i = 0; i < m_trajectoryStates.size(); ++i)
        {
            const auto &sample = m_trajectoryStates[i];
            frc::Pose2d trajectoryPose{sample.x, sample.y, sample.heading};

            units::length::meter_t distance = CalculateDistance(currentPose, trajectoryPose, matchStrategy);

            if (distance < minDistance)
            {
                minDistance = distance;
                closestIndex = i;
            }
        }
    }

    return closestIndex;
} //------------------------------------------------------------------
/// @brief Calculate distance between two poses based on match strategy
/// @details Supports three different matching strategies:
///          - MATCH_Y_ONLY: Only considers Y coordinate difference (for horizontal paths)
///          - MATCH_X_ONLY: Only considers X coordinate difference (for vertical paths)
///          - MATCH_XY_EUCLIDEAN: Uses full 2D Euclidean distance (default)
//------------------------------------------------------------------
units::length::meter_t TrajectoryDrive::CalculateDistance(const frc::Pose2d &pose1, const frc::Pose2d &pose2, TrajectoryMatchStrategy matchStrategy) const
{
    units::length::meter_t dx = pose1.X() - pose2.X();
    units::length::meter_t dy = pose1.Y() - pose2.Y();

    switch (matchStrategy)
    {
    case TrajectoryMatchStrategy::MATCH_Y_ONLY:
        return units::math::abs(dy);

    case TrajectoryMatchStrategy::MATCH_X_ONLY:
        return units::math::abs(dx);

    case TrajectoryMatchStrategy::MATCH_XY:
    default:
        return PoseUtils::GetDeltaBetweenPoses(pose1, pose2);
    }
}