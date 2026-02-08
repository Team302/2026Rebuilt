
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

#include <memory>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/smartdashboard/SendableChooser.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "state/IRobotStateChangeSubscriber.h"
#include "utils/DragonField.h"
#include "utils/logging/signals/DragonDataLogger.h"
#include "vision/DragonVisionPoseEstimatorStruct.h"
#include "vision/Questnavlib/QuestNav.h"

class DragonQuest : public IRobotStateChangeSubscriber, public DragonDataLogger

{
public:
    DragonQuest(
        units::length::inch_t mountingXOffset, /// <I> x offset of Quest from robot center (forward relative to robot)
        units::length::inch_t mountingYOffset, /// <I> y offset of Quest from robot center (left relative to robot)
        units::length::inch_t mountingZOffset, /// <I> z offset of Quest from robot center (up relative to robot)
        units::angle::degree_t mountingPitch,  /// <I> - Pitch of Quest
        units::angle::degree_t mountingYaw,    /// <I> - Yaw of Quest
        units::angle::degree_t mountingRoll    /// <I> - Roll of Quest
    );
    void DataLog(uint64_t timestamp) override;

    bool HealthCheck() { return m_questNav.IsConnected(); };

    DragonVisionPoseEstimatorStruct GetPoseEstimate();

    void AttemptSetRobotPose(const frc::Pose2d &pose);

    void Periodic();

    void NotifyStateUpdate(RobotStateChanges::StateChange change, bool value) override;

private:
    DragonQuest() = delete;

    /// @brief Read new pose frames from QuestNav, apply mounting transform, and cache the latest 2D pose.
    void GetEstimatedPose();

    /// @brief Read dashboard choosers and update m_isQuestEnabled.
    void HandleDashboard();

    /// @brief Apply the robot-to-Quest mounting offset and send a pose-reset command via QuestNav.
    void SetRobotPose(const frc::Pose2d &pose);

    /// @brief Convert a Quest frc::Pose3d to a robot frc::Pose2d by applying the inverse mounting transform.
    frc::Pose2d QuestPoseToRobotPose2d(const frc::Pose3d &questPose) const;

    /// @brief Convert a robot frc::Pose2d to a Quest frc::Pose3d by applying the mounting transform.
    frc::Pose3d RobotPose2dToQuestPose(const frc::Pose2d &robotPose) const;

    // ── QuestNav library instance (handles all NT / protobuf communication) ──
    QuestNav m_questNav;

    // ── Mounting offsets ──
    units::length::inch_t m_mountingXOffset;
    units::length::inch_t m_mountingYOffset;
    units::length::inch_t m_mountingZOffset;
    units::angle::degree_t m_mountingPitch;
    units::angle::degree_t m_mountingYaw;
    units::angle::degree_t m_mountingRoll;

    /// 3D transform from robot centre to Quest mounting location.
    frc::Transform3d m_robotToQuestTransform;

    /// 2D transform from Quest to robot (kept for the simple 2D path).
    frc::Transform2d m_questToRobotTransform2d;

    // ── Dashboard choosers ──
    frc::SendableChooser<bool> m_questEnabledChooser;
    frc::SendableChooser<bool> m_questEndgameEnabledChooser;

    // ── State flags ──
    bool m_hasReset = false;
    bool m_isQuestEnabled = false;
    bool m_isClimbMode = false;

    // ── Standard deviations for pose estimator ──
    static constexpr double m_stdxy{0.02};
    static constexpr double m_stddeg{0.035};

    // ── Cached latest pose ──
    frc::Pose2d m_lastCalculatedPose;
    units::time::second_t m_lastPoseTimestamp{0.0};

    /// Pose2d to reset to (held until connection is available).
    frc::Pose2d m_poseReset;

    /// Counter for throttling pose resets (only set every 3rd call).
    int m_poseResetCounter = 0;

    // ── Field visualisation ──
    DragonField *m_field = nullptr;
};
