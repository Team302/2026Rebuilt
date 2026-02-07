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
#include "vision/DragonQuest.h"

#include <cmath>
#include <string>

#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/geometry/Translation3d.h"
#include "state/IRobotStateChangeSubscriber.h"
#include "state/RobotState.h"
#include "state/RobotStateChanges.h"
#include "units/length.h"
#include "units/time.h"
#include "utils/DragonField.h"
#include "utils/logging/debug/Logger.h"
#include "vision/Questnavlib/PoseFrame.h"

// Static strings to avoid repeated heap allocations in logging
static const std::string kQuestNavDebug = "questnavdebug";

// ──────────────────────────────────────────────────────────────────────────────
// Constructor
// ──────────────────────────────────────────────────────────────────────────────
DragonQuest::DragonQuest(
    units::length::inch_t mountingXOffset,
    units::length::inch_t mountingYOffset,
    units::length::inch_t mountingZOffset,
    units::angle::degree_t mountingPitch,
    units::angle::degree_t mountingYaw,
    units::angle::degree_t mountingRoll)
    : IRobotStateChangeSubscriber(),
      DragonDataLogger(),
      m_questNav(),
      m_mountingXOffset(mountingXOffset),
      m_mountingYOffset(mountingYOffset),
      m_mountingZOffset(mountingZOffset),
      m_mountingPitch(mountingPitch),
      m_mountingYaw(mountingYaw),
      m_mountingRoll(mountingRoll)
{
    // Build a 3D transform from robot centre to the Quest mounting location.
    m_robotToQuestTransform = frc::Transform3d{
        frc::Translation3d{m_mountingXOffset, m_mountingYOffset, m_mountingZOffset},
        frc::Rotation3d{m_mountingRoll, m_mountingPitch, m_mountingYaw}};

    // Keep a simple 2D version for the pose-to-robot conversion path.
    m_questToRobotTransform2d = frc::Transform2d{
        frc::Translation2d{m_mountingXOffset, m_mountingYOffset},
        frc::Rotation2d{m_mountingYaw}};

    // Dashboard choosers
    m_questEnabledChooser.AddOption("ON", true);
    m_questEnabledChooser.AddOption("OFF", false);
    m_questEnabledChooser.SetDefaultOption("ON", true);

    m_questEndgameEnabledChooser.AddOption("ENDGAME ONLY", true);
    m_questEndgameEnabledChooser.AddOption("FULL MATCH", false);
    m_questEndgameEnabledChooser.SetDefaultOption("FULL MATCH", false);

    frc::SmartDashboard::PutData("Quest ON/OFF", &m_questEnabledChooser);
    frc::SmartDashboard::PutData("Quest Endgame ONLY", &m_questEndgameEnabledChooser);

    // Subscribe to climb-mode state changes
    RobotState *robotStates = RobotState::GetInstance();
    robotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus_Bool);

    // Field visualisation object
    m_field = DragonField::GetInstance();
    m_field->AddObject("QuestRobotPose", frc::Pose2d{}, true);
    m_field->AddObject("QuestPose", frc::Pose2d{}, false);
}

// ──────────────────────────────────────────────────────────────────────────────
// Periodic – called every robot loop
// ──────────────────────────────────────────────────────────────────────────────
void DragonQuest::Periodic()
{
    // Let QuestNav process command responses & version checking
    m_questNav.CommandPeriodic();

    bool connected = m_questNav.IsConnected();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, std::string("m_isConnected"), connected);

    HandleDashboard();

    // If a deferred pose reset is pending and we now have a connection, send it
    if (m_poseResetRequested && connected)
    {
        SetRobotPose(m_poseReset);
        return; // Skip GetEstimatedPose this cycle to let reset propagate
    }

    if (connected)
    {
        GetEstimatedPose();
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// GetEstimatedPose – drain the QuestNav frame queue and cache the latest robot pose
// ──────────────────────────────────────────────────────────────────────────────
void DragonQuest::GetEstimatedPose()
{
    auto frames = m_questNav.GetAllUnreadPoseFrames();
    if (frames.empty())
    {
        return; // No new data – keep the previous cached pose
    }

    // Use the most recent frame
    const PoseFrame &latest = frames.back();
    frc::Pose2d robotPose = QuestPoseToRobotPose2d(latest.questPose3d);

    m_lastCalculatedPose = robotPose;
    m_lastPoseTimestamp = units::time::second_t{latest.dataTimestamp};
    m_field->UpdateObject("QuestRobotPose", robotPose);
}

// ──────────────────────────────────────────────────────────────────────────────
// DataLog
// ──────────────────────────────────────────────────────────────────────────────
void DragonQuest::DataLog(uint64_t timestamp)
{
    Log2DPoseData(timestamp, DragonDataLogger::PoseSingals::CURRENT_CHASSIS_QUEST_POSE2D, m_lastCalculatedPose);
}

// ──────────────────────────────────────────────────────────────────────────────
// AttemptSetRobotPose – external entry point (from DragonVision::SetRobotPose)
// ──────────────────────────────────────────────────────────────────────────────
void DragonQuest::AttemptSetRobotPose(const frc::Pose2d &pose)
{
    m_poseReset = pose;
    if (m_questNav.IsConnected())
    {
        SetRobotPose(m_poseReset);
    }
    else
    {
        m_poseResetRequested = true;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// SetRobotPose – convert robot Pose2d to Quest Pose3d and delegate to QuestNav
// ──────────────────────────────────────────────────────────────────────────────
void DragonQuest::SetRobotPose(const frc::Pose2d &pose)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, std::string("SetRobotPoseX"), pose.X().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, std::string("SetRobotPoseY"), pose.Y().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, std::string("SetRobotPoseRot"), pose.Rotation().Degrees().value());

    frc::Pose3d questPose = RobotPose2dToQuestPose(pose);
    m_questNav.SetPose(questPose);

    m_hasReset = true;
    m_poseResetRequested = false;
}

// ──────────────────────────────────────────────────────────────────────────────
// HandleDashboard
// ──────────────────────────────────────────────────────────────────────────────
void DragonQuest::HandleDashboard()
{
    if (m_questEnabledChooser.GetSelected() == true)
    {
        m_isQuestEnabled = true;
        if (m_questEndgameEnabledChooser.GetSelected() == true && !m_isClimbMode)
        {
            m_isQuestEnabled = false;
        }
    }
    else
    {
        m_isQuestEnabled = false;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// NotifyStateUpdate
// ──────────────────────────────────────────────────────────────────────────────
void DragonQuest::NotifyStateUpdate(RobotStateChanges::StateChange change, bool value)
{
    if (RobotStateChanges::StateChange::ClimbModeStatus_Bool == change)
        m_isClimbMode = value;
}

// ──────────────────────────────────────────────────────────────────────────────
// GetPoseEstimate – build the struct consumed by the pose estimator
// ──────────────────────────────────────────────────────────────────────────────
DragonVisionPoseEstimatorStruct DragonQuest::GetPoseEstimate()
{
    bool connected = m_questNav.IsConnected();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, std::string("m_hasReset"), m_hasReset);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, std::string("m_isConnected"), connected);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, std::string("m_isQuestEnabled"), m_isQuestEnabled);

    DragonVisionPoseEstimatorStruct str;
    if (!m_hasReset || !connected || !m_isQuestEnabled)
    {
        str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::NONE;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, std::string("confidence"), std::string("NONE"));
    }
    else
    {
        // TODO: switch to HIGH once Quest pose is validated on the robot
        str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::NONE;
        str.m_visionPose = m_lastCalculatedPose;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, std::string("x"), str.m_visionPose.X().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, std::string("y"), str.m_visionPose.Y().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, std::string("rot"), str.m_visionPose.Rotation().Degrees().value());
        str.m_stds = wpi::array{m_stdxy, m_stdxy, m_stddeg};
        str.m_timeStamp = m_lastPoseTimestamp;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, std::string("confidence"), std::string("HIGH"));
    }
    return str;
}

// ──────────────────────────────────────────────────────────────────────────────
// QuestPoseToRobotPose2d – Quest Pose3d → Robot Pose2d (apply inverse mounting offset)
// ──────────────────────────────────────────────────────────────────────────────
frc::Pose2d DragonQuest::QuestPoseToRobotPose2d(const frc::Pose3d &questPose) const
{
    // Project the 3D Quest pose down to 2D, then apply the inverse mounting transform
    frc::Pose2d questPose2d{
        questPose.X(), questPose.Y(),
        frc::Rotation2d{questPose.Rotation().Z()}};
    m_field->UpdateObject("QuestPose", questPose2d);
    return questPose2d.TransformBy(m_questToRobotTransform2d);
}

// ──────────────────────────────────────────────────────────────────────────────
// RobotPose2dToQuestPose – Robot Pose2d → Quest Pose3d (apply mounting offset)
// ──────────────────────────────────────────────────────────────────────────────
frc::Pose3d DragonQuest::RobotPose2dToQuestPose(const frc::Pose2d &robotPose) const
{
    // Lift the 2D robot pose to 3D and apply the robot→Quest transform
    frc::Pose3d robotPose3d{robotPose};
    return robotPose3d.TransformBy(m_robotToQuestTransform.Inverse());
}
