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

#include "chassis/commands/PathfindToPose.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "pathplanner/lib/config/RobotConfig.h"
#include "state/RobotState.h"
#include "utils/DragonField.h"
#include "utils/PoseUtils.h"
#include "utils/logging/debug/Logger.h"

PathfindToPose::PathfindToPose(subsystems::CommandSwerveDrivetrain *chassis)
    : m_chassis(chassis),
      m_translationConstraints(kMaxVelocity, kMaxAcceleration),
      m_translationPIDX(3.0, 0.0, 0.0, m_translationConstraints, 20_ms),
      m_translationPIDY(3.0, 0.0, 0.0, m_translationConstraints, 20_ms)
{
    AddRequirements(m_chassis);
}

void PathfindToPose::Initialize()
{
    m_translationPIDX.SetIZone(0.5);
    m_translationPIDY.SetIZone(0.5);

    m_currentPose = m_chassis != nullptr ? m_chassis->GetPose() : frc::Pose2d();

    // Setup target pose (can be set dynamically later via SetTargetPose)
    m_targetPose = frc::Pose2d(6_m, 2_m, frc::Rotation2d(0_deg)); // Default dummy value

    // Make sure DragonField recognizes our custom field objects
    DragonField::GetInstance()->AddObject("Dynamic Obstacle (Robot)", m_currentPose, true);
    DragonField::GetInstance()->AddObject("Pathfind Predicted Path", m_currentPose, true);

    m_pathTimer.Reset();
    m_pathTimer.Start();

    m_replanTimer.Reset();
    m_replanTimer.Start();

    // Ensure Pathfinding is initialized (PathPlanner handles algorithms internally)
    pathplanner::Pathfinding::ensureInitialized();

    ReplanPath();
}

void PathfindToPose::SetTargetPose(const frc::Pose2d &targetPose)
{
    m_targetPose = targetPose;
    ReplanPath();
}

void PathfindToPose::Execute()
{
    if (m_chassis == nullptr)
        return;

    m_currentPose = m_chassis->GetPose();

    // 1. Get dynamic obstacles from derived classes (or empty if base class)
    std::vector<std::pair<frc::Translation2d, frc::Translation2d>> dynamicObstacles = GetDynamicObstacles(m_currentPose);

    // Send the detected obstacles to PathPlanner's A* Pathfinding system
    pathplanner::Pathfinding::setDynamicObstacles(dynamicObstacles, m_currentPose.Translation());

    // 2. Replan path periodically if the target or obstacle moved significantly
    if (m_replanTimer.HasElapsed(40_ms))
    {
        ReplanPath();
    }
    // 3. Follow the trajectory using our PIDs
    frc::ChassisSpeeds chassisSpeeds{};

    if (m_currentTrajectory.has_value())
    {
        auto targetState = m_currentTrajectory.value().sample(m_pathTimer.Get());

        chassisSpeeds.vx = targetState.linearVelocity * targetState.heading.Cos();
        chassisSpeeds.vy = targetState.linearVelocity * targetState.heading.Sin();

        // Add PID corrections for precise positioning
        chassisSpeeds.vx += units::velocity::meters_per_second_t(m_translationPIDX.Calculate(m_currentPose.X(), targetState.pose.X()));
        chassisSpeeds.vy += units::velocity::meters_per_second_t(m_translationPIDY.Calculate(m_currentPose.Y(), targetState.pose.Y()));

        chassisSpeeds.vx = std::clamp(chassisSpeeds.vx, -kMaxVelocity, kMaxVelocity);
        chassisSpeeds.vy = std::clamp(chassisSpeeds.vy, -kMaxVelocity, kMaxVelocity);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PathfindToPose", "Pathfinding status", "Valid path");
    }
    else
    {
        // No path could be found; brake
        chassisSpeeds.vx = 0_mps;
        chassisSpeeds.vy = 0_mps;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PathfindToPose", "Pathfinding status", "No valid path found to target pose");
    }

    // Send control request to drivetrain with heading control targeting the final pose heading
    m_chassis->SetControl(
        m_driveRequest.WithVelocityX(chassisSpeeds.vx)
            .WithVelocityY(chassisSpeeds.vy)
            .WithTargetDirection(m_targetPose.Rotation().Degrees())
            .WithHeadingPID(8, 0.0, 0.0) // Taken from DriveToPose m_rotationKP
            .WithForwardPerspective(ctre::phoenix6::swerve::requests::ForwardPerspectiveValue::BlueAlliance));
}

void PathfindToPose::ReplanPath()
{
    if (m_chassis == nullptr)
        return;

    try
    {
        // For PathPlanner 2026, we don't have GetPathWaypoints directly exposed like before.
        // Instead, PathPlanner handles the pathfinding via getCurrentPath from the pathfinder.
        pathplanner::PathConstraints constraints(kMaxVelocity, kMaxAcceleration, 360_deg_per_s, 720_deg_per_s_sq);
        pathplanner::GoalEndState goalState(0_mps, m_targetPose.Rotation());

        pathplanner::Pathfinding::setStartPosition(m_currentPose.Translation());
        pathplanner::Pathfinding::setGoalPosition(m_targetPose.Translation());

        m_currentPath = pathplanner::Pathfinding::getCurrentPath(constraints, goalState);

        if (!m_currentPath)
        {
            m_currentTrajectory = std::nullopt;
            return;
        }

        // Generate Trajectory
        frc::ChassisSpeeds currentSpeeds = m_chassis->GetState().Speeds;
        pathplanner::RobotConfig config;
        try
        {
            config = pathplanner::RobotConfig::fromGUISettings();
        }
        catch (...)
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, "PathfindToPose", "RobotConfig", "Failed to load PathPlanner GUI settings");
            m_currentTrajectory = std::nullopt;
            return;
        }
        m_currentTrajectory = pathplanner::PathPlannerTrajectory(m_currentPath, currentSpeeds, m_currentPose.Rotation(), config);
        m_pathTimer.Restart();

        // Map trajectory to poses and log to Field2d
        std::vector<frc::Pose2d> trajectoryPoses;
        if (m_currentTrajectory.has_value())
        {
            for (auto state : m_currentTrajectory.value().getStates())
            {
                trajectoryPoses.push_back(state.pose);
            }
        }
        DragonField::GetInstance()->UpdateObject("Pathfind Predicted Path", trajectoryPoses);

        m_replanTimer.Restart();
    }
    catch (...)
    {
        // Catch exceptions if path generation fails
        m_currentTrajectory = std::nullopt;
    }
}

bool PathfindToPose::IsFinished()
{
    // Check if we've reached the target pose within a 0.25 inch tolerance, similar to DriveToPose
    return PoseUtils::IsSamePose(m_currentPose, m_targetPose, 0.25_in);
}

std::vector<std::pair<frc::Translation2d, frc::Translation2d>> PathfindToPose::GetDynamicObstacles(const frc::Pose2d &currentPose)
{
    return std::vector<std::pair<frc::Translation2d, frc::Translation2d>>{};
}

void PathfindToPose::End(bool interrupted)
{
    if (m_chassis != nullptr)
    {
        m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
    }
}
