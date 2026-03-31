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

#include "chassis/commands/TrajectoryDrive.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "fielddata/BumpHelper.h"
#include "state/IRobotStateChangeSubscriber.h"

//====================================================================================================================================================
/// @class DriveAlongNearestWall
/// @brief Command to autonomously drive along the nearest alliance wall using a pre-defined trajectory
///
/// This command extends TrajectoryDrive to provide specialized functionality for navigating along the alliance
/// wall. The command intelligently determines which wall (Depot or Outpost) is nearest to the robot's current
/// position and selects the appropriate trajectory to follow.
///
/// **Path Selection:**
/// The command uses BumpHelper to determine proximity and selects between two trajectories:
/// - DepotAllianceSweep: Used when the depot wall is nearest
/// - OutpostAllianceSweep: Used when the outpost wall is nearest
///
/// **Alliance-Aware:**
/// The selected trajectory is automatically flipped based on the current alliance zone
///
/// **Smart Trajectory Entry:**
/// Uses Y-coordinate matching to find the closest point on the trajectory to the robot's current
/// position. This prevents the robot from driving backward if it's already on or past part of the
/// path. The robot will:
/// - Find the closest Y-coordinate match on the trajectory
/// - Start trajectory following from that point
/// - Only use this optimization if within 0.5m tolerance in Y-direction
///
/// **Usage:**
/// ```cpp
/// auto driveAlongWall = std::make_unique<DriveAlongNearestWall>(chassis);
/// // Command will automatically select and follow the appropriate trajectory
/// ```
///
/// @see TrajectoryDrive Base class providing trajectory following functionality
/// @see BumpHelper Utility for identifying nearest bump/wall
//====================================================================================================================================================
class DriveAlongNearestWall : public TrajectoryDrive, public IRobotStateChangeSubscriber
{
public:
    //------------------------------------------------------------------
    /// @brief      Constructor for DriveAlongNearestWall command
    /// @param[in]  chassis - Pointer to the swerve drive subsystem
    /// @details    Initializes the command with the specified chassis
    ///            and retrieves the BumpHelper singleton for wall proximity detection.
    //------------------------------------------------------------------
    explicit DriveAlongNearestWall(subsystems::CommandSwerveDrivetrain *chassis);

    //------------------------------------------------------------------
    /// @brief      Initialize the command and select appropriate trajectory
    /// @details    Overrides the base Initialize() to:
    ///             1. Determine the nearest wall (Depot or Outpost) using BumpHelper
    ///             2. Select the corresponding trajectory path name
    ///             3. Call InitializeWithTrajectory() with Y-only matching strategy
    ///             4. Find the closest Y-coordinate on the trajectory and start from there
    ///
    ///             The trajectory will be automatically flipped for red alliance.
    ///             Uses a 0.5m tolerance for Y-matching to allow intelligent path joining.
    //------------------------------------------------------------------
    void Initialize() override;

private:
    //------------------------------------------------------------------
    /// @brief      Pointer to BumpHelper singleton for wall proximity determination
    //------------------------------------------------------------------
    BumpHelper *m_bumpHelper;

    //------------------------------------------------------------------
    // Path name constants
    //------------------------------------------------------------------
    static constexpr const char *kDepotAllianceSweepPath = "DepotAllianceSweep";
    static constexpr const char *kOutpostAllianceSweepPath = "OutpostAllianceSweep";
    static constexpr const char *kDepotAllianceSweepPathReverse = "DepotAllianceSweepReverse";
    static constexpr const char *kOutpostAllianceSweepPathReverse = "OutpostAllianceSweepReverse";

    //------------------------------------------------------------------
    // Choreo Trajectories
    //------------------------------------------------------------------
    std::optional<choreo::Trajectory<choreo::SwerveSample>> m_blueDepotAllianceSweepTrajectory;
    std::optional<choreo::Trajectory<choreo::SwerveSample>> m_blueDepotAllianceSweepTrajectoryReverse;
    std::optional<choreo::Trajectory<choreo::SwerveSample>> m_redDepotAllianceSweepTrajectory;
    std::optional<choreo::Trajectory<choreo::SwerveSample>> m_redDepotAllianceSweepTrajectoryReverse;
    std::optional<choreo::Trajectory<choreo::SwerveSample>> m_blueOutpostAllianceSweepTrajectory;
    std::optional<choreo::Trajectory<choreo::SwerveSample>> m_blueOutpostAllianceSweepTrajectoryReverse;
    std::optional<choreo::Trajectory<choreo::SwerveSample>> m_redOutpostAllianceSweepTrajectory;
    std::optional<choreo::Trajectory<choreo::SwerveSample>> m_redOutpostAllianceSweepTrajectoryReverse;

    //------------------------------------------------------------------
    /// @brief      Tolerance for Y-coordinate matching when finding closest point on trajectory
    static constexpr units::meter_t m_yDistanceThreshold = 0.25_m;

    static constexpr double m_maxPercentToJoinReversePath = 0.3;
    static constexpr double m_maxPercentToJoinForwardPath = 0.5;

    void NotifyStateUpdate(RobotStateChanges::StateChange change, bool value) override;

    bool m_isLaunching = false;
};
