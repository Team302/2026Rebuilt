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

#include "chassis/commands/season_specific_commands/DriveAlongNearestWall.h"
#include "auton/drivePrimitives/AutonUtils.h"
#include "fielddata/BumpHelper.h"
#include "state/RobotState.h"
#include "utils/PoseUtils.h"
#include "utils/logging/debug/Logger.h"

//------------------------------------------------------------------
/// @brief      Constructor for DriveAlongNearestWall command
/// @param[in]  chassis - Pointer to the swerve drive subsystem
/// @details    Initializes the base TrajectoryDrive command with the chassis
///             and retrieves the BumpHelper singleton for wall proximity detection.
//------------------------------------------------------------------
DriveAlongNearestWall::DriveAlongNearestWall(subsystems::CommandSwerveDrivetrain *chassis)
    : TrajectoryDrive(chassis),
      m_bumpHelper(BumpHelper::GetInstance())
{
    m_blueDepotAllianceSweepTrajectory = AutonUtils::GetTrajectoryFromPathFile(kDepotAllianceSweepPath, false);
    m_redDepotAllianceSweepTrajectory = AutonUtils::GetTrajectoryFromPathFile(kDepotAllianceSweepPath, true);
    m_blueOutpostAllianceSweepTrajectory = AutonUtils::GetTrajectoryFromPathFile(kOutpostAllianceSweepPath, false);
    m_redOutpostAllianceSweepTrajectory = AutonUtils::GetTrajectoryFromPathFile(kOutpostAllianceSweepPath, true);
    m_blueDepotAllianceSweepTrajectoryReverse = AutonUtils::GetTrajectoryFromPathFile(kDepotAllianceSweepPathReverse, false);
    m_redDepotAllianceSweepTrajectoryReverse = AutonUtils::GetTrajectoryFromPathFile(kDepotAllianceSweepPathReverse, true);
    m_blueOutpostAllianceSweepTrajectoryReverse = AutonUtils::GetTrajectoryFromPathFile(kOutpostAllianceSweepPathReverse, false);
    m_redOutpostAllianceSweepTrajectoryReverse = AutonUtils::GetTrajectoryFromPathFile(kOutpostAllianceSweepPathReverse, true);

    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::IsLaunching_Bool);
}

//------------------------------------------------------------------
/// @brief      Initialize the command and select appropriate trajectory
/// @details    Determines which wall is nearest to the robot and selects
///             the corresponding trajectory path. The logic works as follows:
///             1. Calls BumpHelper::CalcNearestBump() to identify the closest bump (wall)
///             2. Uses SelectBestTrajectory to choose between forward and reverse paths
///                (this also caches the closest point for efficient initialization)
///             3. Calls InitializeWithTrajectory() with the selected trajectory
//------------------------------------------------------------------
void DriveAlongNearestWall::Initialize()
{
    // Determine which bump/wall is nearest to the robot's current position
    auto nearestBump = m_bumpHelper->CalcNearestBump();
    auto currentPose = GetChassis()->GetPose();

    std::optional<choreo::Trajectory<choreo::SwerveSample>> selectedTrajectory;
    bool isRedAlliance = false;
    bool isForward = true;

    switch (nearestBump)
    {
    case BUMP_ID::BLUE_DEPOT_BUMP:
    {
        auto [trajectory, isForward] = SelectBestTrajectory(m_blueDepotAllianceSweepTrajectory,
                                                            m_blueDepotAllianceSweepTrajectoryReverse,
                                                            currentPose,
                                                            TrajectoryMatchStrategy::MATCH_XY,
                                                            m_maxPercentToJoinForwardPath, m_maxPercentToJoinReversePath);
        selectedTrajectory = trajectory;
        isRedAlliance = false;
        break;
    }
    case BUMP_ID::RED_DEPOT_BUMP:
    {
        auto [trajectory, isForward] = SelectBestTrajectory(m_redDepotAllianceSweepTrajectory,
                                                            m_redDepotAllianceSweepTrajectoryReverse,
                                                            currentPose,
                                                            TrajectoryMatchStrategy::MATCH_XY,
                                                            m_maxPercentToJoinForwardPath, m_maxPercentToJoinReversePath);
        selectedTrajectory = trajectory;
        isRedAlliance = true;
        break;
    }
    case BUMP_ID::BLUE_OUTPOST_BUMP:
    {
        auto [trajectory, isForward] = SelectBestTrajectory(m_blueOutpostAllianceSweepTrajectory,
                                                            m_blueOutpostAllianceSweepTrajectoryReverse,
                                                            currentPose,
                                                            TrajectoryMatchStrategy::MATCH_XY,
                                                            m_maxPercentToJoinForwardPath, m_maxPercentToJoinReversePath);
        selectedTrajectory = trajectory;
        isRedAlliance = false;
        break;
    }
    case BUMP_ID::RED_OUTPOST_BUMP:
    {
        auto [trajectory, isForward] = SelectBestTrajectory(m_redOutpostAllianceSweepTrajectory,
                                                            m_redOutpostAllianceSweepTrajectoryReverse,
                                                            currentPose,
                                                            TrajectoryMatchStrategy::MATCH_XY,
                                                            m_maxPercentToJoinForwardPath, m_maxPercentToJoinReversePath);
        selectedTrajectory = trajectory;
        isRedAlliance = true;
        break;
    }
    default:
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, "DriveAlongNearestWall", "Unknown nearest bump detected", static_cast<int>(nearestBump));
        return;
    }

    // Initialize with the selected trajectory (uses cached closest point from SelectBestTrajectory)
    InitializeWithTrajectory(selectedTrajectory,
                             isRedAlliance,
                             TrajectoryMatchStrategy::MATCH_XY,
                             m_yDistanceThreshold,
                             isForward ? m_maxPercentToJoinForwardPath : m_maxPercentToJoinReversePath);
}

void DriveAlongNearestWall::NotifyStateUpdate(RobotStateChanges::StateChange change, bool value)
{
    if (change == RobotStateChanges::StateChange::IsLaunching_Bool)
    {
        m_isLaunching = value;
    }
}
