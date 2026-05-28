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
#include "state/RobotState.h"
#include "utils/PoseUtils.h"
#include "utils/logging/debug/Logger.h"
#include "vision/PoseOffsetUtils.h"

PathfindToPose::PathfindToPose(subsystems::CommandSwerveDrivetrain *chassis)
    : m_chassis(chassis),
      m_vision(DragonVision::GetDragonVision()),
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
    m_targetPose = frc::Pose2d(1_m, 1_m, frc::Rotation2d(0_deg)); // Default dummy value

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

    // 1. Detect dynamic obstacles (offending robots) via Vision
    if (m_vision != nullptr)
    {
        m_vision->SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE::OBJECT_DETECTION, DRAGON_LIMELIGHT_PIPELINE::BUMPERS);
        m_visionCache = m_vision->GetObjectDetectionTargetInfo(VisionTargetOption::CLOSEST_VALID_TARGET, std::vector<int>{});

        std::vector<std::pair<frc::Translation2d, frc::Translation2d>> dynamicObstacles;

        if (!m_visionCache.empty() && m_visionCache[0].get() != nullptr)
        {
            // Calculate distance to the robot, similar to AutoDefend
            auto targetinfo = PoseOffsetUtils::CalculateXYDistanceFromObject(*m_visionCache[0], 4_in);
            units::length::meter_t xDistRelative = -targetinfo.first;
            units::length::meter_t yDistRelative = -targetinfo.second;

            // Convert relative distance to field translation
            frc::Translation2d filterFieldTranslation = m_currentPose.Translation() + frc::Translation2d(xDistRelative, yDistRelative).RotateBy(m_currentPose.Rotation());

            // Assume 1m x 1m footprint for the offending robot obstacle
            frc::Translation2d obsSize{0.5_m, 0.5_m};
            dynamicObstacles.push_back({filterFieldTranslation - obsSize, filterFieldTranslation + obsSize});

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PathfindToPose", "Obstacle X", filterFieldTranslation.X().value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PathfindToPose", "Obstacle Y", filterFieldTranslation.Y().value());
        }

        // Send the detected obstacles to PathPlanner's A* Pathfinding system
        pathplanner::Pathfinding::setDynamicObstacles(dynamicObstacles, m_currentPose.Translation());
    }

    // 2. Replan path periodically if the target or obstacle moved significantly
    // PathPlanner's Pathfinding checks for path invalidation under the hood, but it's safe to poll
    if (m_pathTimer.Get() > 0.5_s && pathplanner::Pathfinding::isPathfindingTargetAccessible(m_targetPose.Translation()))
    {
        if (m_replanTimer.HasElapsed(0.5_s))
        {
            ReplanPath();
            m_replanTimer.Restart();
        }
    }

    // 3. Follow the trajectory using our PIDs
    frc::ChassisSpeeds chassisSpeeds{};

    if (m_currentTrajectory.has_value())
    {
        auto targetState = m_currentTrajectory.value().sample(m_pathTimer.Get());

        chassisSpeeds.vx = targetState.velocity * targetState.heading.Cos();
        chassisSpeeds.vy = targetState.velocity * targetState.heading.Sin();

        // Add PID corrections for precise positioning
        chassisSpeeds.vx += units::velocity::meters_per_second_t(m_translationPIDX.Calculate(m_currentPose.X(), targetState.position.X()));
        chassisSpeeds.vy += units::velocity::meters_per_second_t(m_translationPIDY.Calculate(m_currentPose.Y(), targetState.position.Y()));

        chassisSpeeds.vx = std::clamp(chassisSpeeds.vx, -kMaxVelocity, kMaxVelocity);
        chassisSpeeds.vy = std::clamp(chassisSpeeds.vy, -kMaxVelocity, kMaxVelocity);
    }
    else
    {
        // No path could be found; brake
        chassisSpeeds.vx = 0_mps;
        chassisSpeeds.vy = 0_mps;
    }

    // Send control request to drivetrain with heading control targeting the final pose heading
    m_chassis->SetControl(
        m_driveRequest.WithVelocityX(chassisSpeeds.vx)
            .WithVelocityY(chassisSpeeds.vy)
            .WithTargetDirection(m_targetPose.Rotation().Degrees())
            .WithHeadingPID(4.5, 0.0, 0.0) // Taken from DriveToPose m_rotationKP
            .WithForwardPerspective(ctre::phoenix6::swerve::requests::ForwardPerspectiveValue::BlueAlliance));
}

void PathfindToPose::ReplanPath()
{
    if (m_chassis == nullptr)
        return;

    try
    {
        auto pathPoints = pathplanner::Pathfinding::getPathWaypoints(m_currentPose.Translation(), m_targetPose.Translation());
        if (pathPoints.empty())
        {
            m_currentTrajectory = std::nullopt;
            return;
        }

        // Generate PathPlannerPath from points to adhere to constraints
        pathplanner::PathConstraints constraints(kMaxVelocity, kMaxAcceleration, 360_deg_per_s, 720_deg_per_s_sq);
        pathplanner::GoalEndState goalState(0_mps, m_targetPose.Rotation());

        m_currentPath = pathplanner::PathPlannerPath::fromPathPoints(pathPoints, constraints, goalState);

        // Generate Trajectory
        frc::ChassisSpeeds currentSpeeds = m_chassis->GetState().Speeds;
        m_currentTrajectory = m_currentPath->getTrajectory(currentSpeeds, m_currentPose.Rotation());

        m_pathTimer.Restart();
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

void PathfindToPose::End(bool interrupted)
{
    if (m_chassis != nullptr)
    {
        m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
    }
}
