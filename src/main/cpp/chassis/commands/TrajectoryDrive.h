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

// FRC Includes
#include "frc/Timer.h"
#include "frc/controller/PIDController.h"
#include <choreo/trajectory/Trajectory.h>

//====================================================================================================================================================
/// @enum TrajectoryMatchStrategy
/// @brief Defines how to match robot position to trajectory when finding the closest starting point
///
/// Used by InitializeWithTrajectory to determine which axis/axes to consider when finding the optimal
/// point to begin trajectory following. This allows the robot to intelligently join a trajectory
/// at the most appropriate point rather than always starting from the beginning.
//====================================================================================================================================================
enum class TrajectoryMatchStrategy
{
    MATCH_Y_ONLY, ///< Match only the Y coordinate (for horizontal sweeps along alliance wall)
    MATCH_X_ONLY, ///< Match only the X coordinate (for vertical paths)
    MATCH_XY      ///< Match both X and Y using Euclidean distance (default behavior)
};

class TrajectoryDrive : public frc2::CommandHelper<frc2::Command, TrajectoryDrive>
{
public:
    /**
     * @brief Construct a new Trajectory Drive command
     *
     * @param chassis The swerve drive subsystem
     * @param trajectory The Choreo trajectory to follow
     */
    explicit TrajectoryDrive(subsystems::CommandSwerveDrivetrain *chassis);

    // FRC Command Lifecycle methods
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

    void InitializeWithTrajectory(std::optional<choreo::Trajectory<choreo::SwerveSample>> selectedTrajectory, bool generateRedTrajectory, TrajectoryMatchStrategy matchStrategy = TrajectoryMatchStrategy::MATCH_XY, units::length::meter_t distanceTolerance = 0.5_m, double maxPercentToJoinPath = 0.9);

    void SetPath(const std::string &pathName);

    std::string WhyDone() const { return m_whyDone; };

    units::time::second_t GetTotalTrajectoryTime() const { return m_totalTrajectoryTime; }

protected:
    subsystems::CommandSwerveDrivetrain *GetChassis() const { return m_chassis; }

    /// @brief Choose between forward and reverse trajectories based on closest entry point
    /// @param forwardTraj The forward trajectory option
    /// @param reverseTraj The reverse trajectory option
    /// @param currentPose The robot's current pose
    /// @param matchStrategy How to match positions (X, Y, or XY)
    /// @param maxPercentToJoinPath Maximum percentage of trajectory to search for entry point
    /// @return The selected trajectory (forward or reverse) and whether it's the forward one
    /// @details This method finds the closest point on both trajectories and returns the one
    ///          with the closer entry point. It caches the closest point information to avoid
    ///          redundant calculations in InitializeWithTrajectory.
    std::pair<std::optional<choreo::Trajectory<choreo::SwerveSample>>, bool> SelectBestTrajectory(
        const std::optional<choreo::Trajectory<choreo::SwerveSample>> &forwardTraj,
        const std::optional<choreo::Trajectory<choreo::SwerveSample>> &reverseTraj,
        const frc::Pose2d &currentPose,
        TrajectoryMatchStrategy matchStrategy,
        double maxPercentToJoinForForwardPath,
        double maxPercentToJoinForReversePath);

private:
    subsystems::CommandSwerveDrivetrain *m_chassis;
    std::string m_pathName;

    std::optional<choreo::Trajectory<choreo::SwerveSample>> m_trajectory;
    frc::Pose2d m_finalPose;
    frc::Pose2d m_previousPose;
    std::vector<choreo::SwerveSample> m_trajectoryStates;
    int m_numberOfExecutions{0};

    frc::Pose2d m_prevPose;
    bool m_wasMoving;
    frc::Transform2d m_delta;
    std::unique_ptr<frc::Timer> m_timer;

    std::string m_whyDone;
    units::time::second_t m_totalTrajectoryTime;
    units::time::second_t m_thresholdTime{0.0_s};   // pre-computed kPercentComplete * m_totalTrajectoryTime
    units::time::second_t m_elapsedTime{0.0_s};     // cached timer value, updated once per Execute() call
    units::time::second_t m_startTimeOffset{0.0_s}; // time offset when starting from a mid-trajectory point

    // Smart trajectory joining state
    bool m_useSmartJoin{false};                    // Whether to use smart trajectory joining
    bool m_isApproachingPath{false};               // True when driving to path, false when following trajectory
    TrajectoryMatchStrategy m_matchStrategy;       // Which axes to match when joining
    units::length::meter_t m_joinTolerance{0.5_m}; // Distance tolerance for joining the path
    frc::Pose2d m_targetJoinPose;                  // The pose on the trajectory we're driving to
    size_t m_targetJoinIndex{0};                   // Index of the trajectory point we're joining at

    // Cached closest point information from SelectBestTrajectory
    bool m_hasCachedClosestPoint{false};                   // Whether we have cached closest point data
    choreo::SwerveSample m_cachedClosestSample;            // The cached closest sample
    units::length::meter_t m_cachedClosestDistance{0.0_m}; // The cached distance to closest point

    static constexpr double kPDrive{3.5};
    static constexpr double kIDrive{0.0};
    static constexpr double kDDrive{0.0};

    static constexpr double kPHeading{6.0};
    static constexpr double kIHeading{0.0};
    static constexpr double kDHeading{0.0};

    frc::PIDController m_xController{kPDrive, kIDrive, kDDrive};
    frc::PIDController m_yController{kPDrive, kIDrive, kDDrive};
    frc::PIDController m_headingController{kPHeading, kIHeading, kDHeading};

    frc::ChassisSpeeds m_chassisSpeeds;

    swerve::requests::FieldCentric m_driveRequest;

    static constexpr units::length::centimeter_t kPositionTolerance{10.0_cm}; // this is what we had; should we lower it? This is probably why we stopped short on straight path test.
    static constexpr units::angle::degree_t kHeadingTolerance{3.0_deg};       // this is what we had; should we lower it?
    static constexpr double kPercentComplete{0.9};                            // this is what we had; should we up it so we don't compare poses so often?
    static constexpr int kMinExecutions{5};                                   // minimum number of executions before checking pose

    //------------------------------------------------------------------
    // Helper methods for smart trajectory initialization
    //------------------------------------------------------------------

    /// @brief Find the closest point in the trajectory to the robot's current position
    /// @param currentPose The robot's current pose
    /// @param matchStrategy How to match positions (X, Y, or XY)
    /// @param tolerance The tolerance for considering a match
    /// @param maxPercentToJoinPath The maximum percentage of the trajectory to consider for joining
    /// @return The index of the closest trajectory sample
    size_t FindClosestTrajectoryPoint(const frc::Pose2d &currentPose, TrajectoryMatchStrategy matchStrategy, units::length::meter_t tolerance, double maxPercentToJoinPath) const;

    /// @brief Find the closest point on a given trajectory to a specific pose
    /// @param trajectory The trajectory to search
    /// @param currentPose The pose to find the closest point to
    /// @param matchStrategy How to match positions (X, Y, or XY)
    /// @param maxPercentToJoinPath The maximum percentage of the trajectory to consider (0.0-1.0)
    /// @return A pair containing the closest sample and the distance to it
    std::pair<choreo::SwerveSample, units::length::meter_t> FindClosestPointOnTrajectory(
        const choreo::Trajectory<choreo::SwerveSample> &trajectory,
        const frc::Pose2d &currentPose,
        TrajectoryMatchStrategy matchStrategy,
        double maxPercentToJoinPath) const;

    /// @brief Calculate distance between two poses based on match strategy
    /// @param pose1 First pose
    /// @param pose2 Second pose
    /// @param matchStrategy How to calculate distance
    /// @return Distance value in meters
    units::length::meter_t CalculateDistance(const frc::Pose2d &pose1, const frc::Pose2d &pose2, TrajectoryMatchStrategy matchStrategy) const;
};
