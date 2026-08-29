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

#include "chassis/commands/season_specific_commands/PathfindToPoseWithVisionAvoidance.h"
#include "utils/DragonField.h"
#include "utils/logging/debug/Logger.h"
#include "vision/PoseOffsetUtils.h"

PathfindToPoseWithVisionAvoidance::PathfindToPoseWithVisionAvoidance(subsystems::CommandSwerveDrivetrain *chassis)
    : PathfindToPose(chassis)
{
    m_vision = DragonVision::GetDragonVision();
    if (m_vision != nullptr)
    {
        m_vision->SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE::OBJECT_DETECTION, DRAGON_LIMELIGHT_PIPELINE::BUMPERS);
    }
}

std::pair<frc::Translation2d, frc::Translation2d> PathfindToPoseWithVisionAvoidance::GetObstacleSize()
{
    // Expand the bounding box to account for both the opposing robot (approx 0.5m radius)
    // AND our robot's physical size/bumpers (approx 0.45m radius) + safety margin.
    // Total footprint effectively prevents our robot's center from entering collision zones.
    return {frc::Translation2d{1.5_m, 1.5_m}, frc::Translation2d{-1.5_m, -1.5_m}};
}

std::vector<std::pair<frc::Translation2d, frc::Translation2d>> PathfindToPoseWithVisionAvoidance::GetDynamicObstacles(const frc::Pose2d &currentPose)
{
    std::vector<std::pair<frc::Translation2d, frc::Translation2d>> dynamicObstacles;
    dynamicObstacles.push_back({frc::Translation2d{2_m, 2_m}, frc::Translation2d{2.5_m, 2.5_m}}); // Default dummy value to avoid empty vector issues

    if (m_vision == nullptr)
        return dynamicObstacles;

    m_vision->SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE::OBJECT_DETECTION, DRAGON_LIMELIGHT_PIPELINE::BUMPERS);
    m_visionCache = m_vision->GetObjectDetectionTargetInfo(VisionTargetOption::CLOSEST_VALID_TARGET, std::vector<int>{});

    if (!m_visionCache.empty() && m_visionCache[0].get() != nullptr)
    {
        // Calculate distance to the robot, similar to AutoDefend
        auto targetinfo = PoseOffsetUtils::CalculateXYDistanceFromObject(*m_visionCache[0], 4_in);
        units::length::meter_t xDistRelative = -targetinfo.first;
        units::length::meter_t yDistRelative = -targetinfo.second;

        // Convert relative distance to field translation
        frc::Translation2d filterFieldTranslation = currentPose.Translation() + frc::Translation2d(xDistRelative, yDistRelative).RotateBy(currentPose.Rotation());

        auto obsSize = GetObstacleSize();
        // The first is positive offsets, second is negative offsets
        dynamicObstacles.push_back({filterFieldTranslation + obsSize.second, filterFieldTranslation + obsSize.first});

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PathfindToPose", "Obstacle X", filterFieldTranslation.X().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PathfindToPose", "Obstacle Y", filterFieldTranslation.Y().value());
    }

    // Add logging to show dynamic obstacle bounding box on Field2d
    std::vector<frc::Pose2d> obstaclePoses;
    for (const auto &obs : dynamicObstacles)
    {
        frc::Translation2d bottomLeft = obs.first;
        frc::Translation2d topRight = obs.second;

        obstaclePoses.push_back(frc::Pose2d(bottomLeft, frc::Rotation2d(0_deg)));                   // Bottom Left
        obstaclePoses.push_back(frc::Pose2d(bottomLeft.X(), topRight.Y(), frc::Rotation2d(0_deg))); // Top Left
        obstaclePoses.push_back(frc::Pose2d(topRight, frc::Rotation2d(0_deg)));                     // Top Right
        obstaclePoses.push_back(frc::Pose2d(topRight.X(), bottomLeft.Y(), frc::Rotation2d(0_deg))); // Bottom Right
        obstaclePoses.push_back(frc::Pose2d(bottomLeft, frc::Rotation2d(0_deg)));                   // Back to Bottom Left
    }

    DragonField::GetInstance()->UpdateObject("Dynamic Obstacle (Robot)", obstaclePoses);

    return dynamicObstacles;
}