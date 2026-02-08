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
static const std::string kLogIsConnected = "m_isConnected";
static const std::string kLogSetRobotPoseX = "SetRobotPoseX";
static const std::string kLogSetRobotPoseY = "SetRobotPoseY";
static const std::string kLogSetRobotPoseRot = "SetRobotPoseRot";
static const std::string kLogHasReset = "m_hasReset";
static const std::string kLogIsQuestEnabled = "m_isQuestEnabled";
static const std::string kLogConfidence = "confidence";
static const std::string kLogConfidenceNone = "NONE";
static const std::string kLogConfidenceHigh = "HIGH";
static const std::string kLogX = "x";
static const std::string kLogY = "y";
static const std::string kLogRot = "rot";

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
    m_field->AddObject("QuestPose", frc::Pose2d{}, true);
}

// ──────────────────────────────────────────────────────────────────────────────
// Periodic – called every robot loop
// ──────────────────────────────────────────────────────────────────────────────
void DragonQuest::Periodic()
{
    // Let QuestNav process command responses & version checking
    m_questNav.CommandPeriodic();

    bool connected = m_questNav.IsConnected() && m_questNav.IsTracking();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kLogIsConnected, connected);

    HandleDashboard();

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
    frc::Pose3d robotPose = QuestPoseToRobotPose3d(latest.questPose3d);

    m_lastCalculatedPose = robotPose;
    m_lastPoseTimestamp = units::time::second_t{latest.dataTimestamp};
    m_field->UpdateObject("QuestRobotPose", robotPose.ToPose2d());
}

// ──────────────────────────────────────────────────────────────────────────────
// DataLog
// ──────────────────────────────────────────────────────────────────────────────
void DragonQuest::DataLog(uint64_t timestamp)
{
    Log2DPoseData(timestamp, DragonDataLogger::PoseSingals::CURRENT_CHASSIS_QUEST_POSE2D, m_lastCalculatedPose.ToPose2d());
}

// ──────────────────────────────────────────────────────────────────────────────
// AttemptSetRobotPose – external entry point (from DragonVision::SetRobotPose)
// ──────────────────────────────────────────────────────────────────────────────
void DragonQuest::AttemptSetRobotPose(const frc::Pose2d &pose)
{
    m_poseReset = pose;

    // Throttle pose resets: only set every 3rd call
    m_poseResetCounter++;
    if (m_poseResetCounter < 3)
    {
        return; // Skip this reset
    }

    // Reset the counter
    m_poseResetCounter = 0;

    if (m_questNav.IsConnected())
    {
        SetRobotPose(m_poseReset);
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// SetRobotPose – convert robot Pose2d to Quest Pose3d and delegate to QuestNav
// ──────────────────────────────────────────────────────────────────────────────
void DragonQuest::SetRobotPose(const frc::Pose2d &pose)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kLogSetRobotPoseX, pose.X().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kLogSetRobotPoseY, pose.Y().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kLogSetRobotPoseRot, pose.Rotation().Degrees().value());

    frc::Pose3d questPose = RobotPose3dToQuestPose(frc::Pose3d{pose});
    m_questNav.SetPose(questPose);

    m_hasReset = true;
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

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kLogHasReset, m_hasReset);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kLogIsConnected, connected);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kLogIsQuestEnabled, m_isQuestEnabled);

    DragonVisionPoseEstimatorStruct str;
    if (!m_hasReset || !connected || !m_isQuestEnabled)
    {
        str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::NONE;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kLogConfidence, kLogConfidenceNone);
    }
    else
    {
        str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::HIGH;
        str.m_visionPose = m_lastCalculatedPose.ToPose2d();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kLogX, str.m_visionPose.X().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kLogY, str.m_visionPose.Y().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kLogRot, str.m_visionPose.Rotation().Degrees().value());
        str.m_stds = wpi::array{m_stdxy, m_stdxy, m_stddeg};
        str.m_timeStamp = m_lastPoseTimestamp;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kLogConfidence, kLogConfidenceHigh);
    }
    return str;
}

// ──────────────────────────────────────────────────────────────────────────────
// QuestPoseToRobotPose3d – Quest Pose3d → Robot Pose3d (apply inverse mounting offset)
// ──────────────────────────────────────────────────────────────────────────────
frc::Pose3d DragonQuest::QuestPoseToRobotPose3d(const frc::Pose3d &questPose) const
{
    m_field->UpdateObject("QuestPose", questPose.ToPose2d());
    return questPose.TransformBy(m_robotToQuestTransform.Inverse());
}

// ──────────────────────────────────────────────────────────────────────────────
// RobotPose3dToQuestPose – Robot Pose3d → Quest Pose3d (apply mounting offset)
// ──────────────────────────────────────────────────────────────────────────────
frc::Pose3d DragonQuest::RobotPose3dToQuestPose(const frc::Pose3d &robotPose) const
{
    return robotPose.TransformBy(m_robotToQuestTransform);
}
