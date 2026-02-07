
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
#include "state/IRobotStateChangeSubscriber.h"
#include "state/RobotState.h"
#include "state/RobotStateChanges.h"
#include "units/time.h"
#include "utils/AngleUtils.h"
#include "utils/DragonField.h"
#include "utils/logging/debug/Logger.h"

DragonQuest::DragonQuest(
    units::length::inch_t mountingXOffset, /// <I> x offset of Quest from robot center (forward relative to robot)
    units::length::inch_t mountingYOffset, /// <I> y offset of Quest from robot center (left relative to robot)
    units::length::inch_t mountingZOffset, /// <I> z offset of Quest from robot center (up relative to robot)
    units::angle::degree_t mountingPitch,  /// <I> - Pitch of Quest
    units::angle::degree_t mountingYaw,    /// <I> - Yaw of Quest
    units::angle::degree_t mountingRoll    /// <I> - Roll of Quest
    ) : IRobotStateChangeSubscriber(),
        DragonDataLogger(),
        m_mountingXOffset(mountingXOffset),
        m_mountingYOffset(mountingYOffset),
        m_mountingZOffset(mountingZOffset),
        m_mountingPitch(mountingPitch),
        m_mountingYaw(mountingYaw),
        m_mountingRoll(mountingRoll)
{

    m_questToRobotTransform = frc::Transform2d{
        frc::Translation2d(m_mountingXOffset, m_mountingYOffset),
        frc::Rotation2d(m_mountingYaw)};

    m_questEnabledChooser.AddOption("ON", true);
    m_questEnabledChooser.AddOption("OFF", false);
    m_questEnabledChooser.SetDefaultOption("ON", true);

    m_questEndgameEnabledChooser.AddOption("ENDGAME ONLY", true);
    m_questEndgameEnabledChooser.AddOption("FULL MATCH", false);
    m_questEndgameEnabledChooser.SetDefaultOption("FULL MATCH", false);

    frc::SmartDashboard::PutData("Quest ON/OFF", &m_questEnabledChooser);
    frc::SmartDashboard::PutData("Quest Endgame ONLY", &m_questEndgameEnabledChooser);
    RobotState *RobotStates = RobotState::GetInstance();
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus_Bool);
    m_field = DragonField::GetInstance();
    m_field->AddObject("QuestPose", frc::Pose2d{}, true);
}

// Static strings to avoid repeated heap allocations in logging
static const std::string kQuestNavDebug = "questnavdebug";
static const std::string kIsConnected = "m_isConnected";
static const std::string kIsNTInitialized = "m_isNTInitialized";
static const std::string kInitNT = "InitNT";
static const std::string kNetworkTableObtained = "NetworkTable obtained";
static const std::string kFailedToGetNT = "Failed to get NetworkTable";
static const std::string kSuccessfullyInitialized = "Successfully initialized";
static const std::string kBadAlloc = "InitNT bad_alloc";
static const std::string kException = "InitNT exception";

void DragonQuest::Periodic()
{
    if (!m_isNTInitialized)
    {
        InitNT(); // Try to initialize if not already done
    }

    if (m_isNTInitialized)
    {
        SetIsConnected();
        HandleDashboard();
        if (m_poseResetRequested && m_isConnected)
        {
            SetRobotPose(m_poseReset);
        }
        if (m_isConnected)
        {
            GetEstimatedPose();
        }
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnavdebug"), string("Periodic_reset_requested"), m_poseResetRequested);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kIsConnected, m_isConnected);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kIsNTInitialized, m_isNTInitialized);
}

void DragonQuest::InitNT()
{
#ifdef __FRC_ROBORIO__
    // Initialize protobuf topics with error handling
    if (m_isNTInitialized)
    {
        return; // Already initialized
    }

    auto networktable = nt::NetworkTableInstance::GetDefault().GetTable(std::string("QuestNav"));
    if (networktable.get() == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, kQuestNavDebug, kInitNT, kFailedToGetNT);
        return;
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kNetworkTableObtained, true);

    // Create subscribers with reduced buffer sizes to minimize memory allocation
    nt::PubSubOptions options;
    options.keepDuplicates = false;
    options.pollStorage = 1; // Minimal buffer size

    m_frameDataSubscriber = networktable.get()->GetRawTopic("frameData").Subscribe("proto:questnav.protos.data.ProtobufQuestNavFrameData", {}, options);
    m_deviceDataSubscriber = networktable.get()->GetRawTopic("deviceData").Subscribe("proto:questnav.protos.data.ProtobufQuestNavDeviceData", {}, options);
    m_commandPublisher = networktable.get()->GetRawTopic("commands").Publish("proto:questnav.protos.commands.ProtobufQuestNavCommand");
    m_commandResponseSubscriber = networktable.get()->GetRawTopic("response").Subscribe("proto:questnav.protos.commands.ProtobufQuestNavCommandResponse", {}, options);

    m_isNTInitialized = true; // Mark as successfully initialized
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, kQuestNavDebug, kInitNT, kSuccessfullyInitialized);
#endif
}

void DragonQuest::GetEstimatedPose()
{
#ifdef __FRC_ROBORIO__
    if (!m_isNTInitialized)
    {
        m_lastCalculatedPose = frc::Pose2d{};
        return;
    }
    auto rawData = m_frameDataSubscriber.Get();
    if (rawData.empty())
    {
        m_lastCalculatedPose = frc::Pose2d{}; // Set the last pose to a default pose if no data is available
        return;
    }

    questnav::protos::data::ProtobufQuestNavFrameData frameData;
    if (!frameData.ParseFromArray(rawData.data(), rawData.size()))
    {
        m_lastCalculatedPose = frc::Pose2d{}; // Set the last pose to a default pose if no data is available
        return;
    }

    if (!frameData.has_pose3d())
    {
        m_lastCalculatedPose = frc::Pose2d{}; // Set the last pose to a default pose if no data is available
        return;
    }

    const auto &pose3d = frameData.pose3d();
    const auto &translation = pose3d.translation();
    const auto &rotation = pose3d.rotation();

    // Extract quaternion
    const auto &q = rotation.q();
    double qw = q.w();
    double qz = q.z();

    // Convert quaternion to yaw (rotation around Z-axis)
    // For a quaternion representing rotation around Z: yaw = 2 * atan2(qz, qw)
    double yawRadians = 2.0 * std::atan2(qz, qw);

    // Convert from Quest coordinates to robot coordinates
    units::length::meter_t x{translation.x()};
    units::length::meter_t y{translation.y()};
    units::angle::radian_t yaw{yawRadians};

    frc::Pose2d questPose{x, y, yaw};
    frc::Pose2d robotPose = questPose.TransformBy(m_questToRobotTransform.Inverse());

    m_lastCalculatedPose = robotPose;
    m_field->UpdateObject("QuestPose", robotPose);

#endif
}

void DragonQuest::SetIsConnected()
{
#ifdef __FRC_ROBORIO__
    if (!m_isNTInitialized)
    {
        m_isConnected = false;
        return;
    }
    auto rawData = m_frameDataSubscriber.Get();
    if (rawData.empty())
    {
        m_isConnected = false;
        return;
    }

    questnav::protos::data::ProtobufQuestNavFrameData frameData;
    if (!frameData.ParseFromArray(rawData.data(), rawData.size()))
    {
        m_isConnected = false;
        return;
    }

    int32_t currentFrameCount = frameData.frame_count();

    m_loopCounter++;
    if (m_loopCounter > 5)
    {
        if (currentFrameCount != m_prevFrameCount && frameData.istracking())
        {
            m_loopCounter = 0;
            m_isConnected = true;
        }
        else
        {
            m_isConnected = false;
        }
        m_prevFrameCount = currentFrameCount;
    }
#endif
}

void DragonQuest::DataLog(uint64_t timestamp)
{
    Log2DPoseData(timestamp, DragonDataLogger::PoseSingals::CURRENT_CHASSIS_QUEST_POSE2D, m_lastCalculatedPose);
}

void DragonQuest::AttemptSetRobotPose(const frc::Pose2d &pose)
{
#ifdef __FRC_ROBORIO__
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnavdebug"), string("AttemptSetRobotPose"), string("reached"));
    m_poseReset = pose;
    if (m_isConnected)
    {
        SetRobotPose(m_poseReset);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnavdebug"), string("AttemptSetRobotPose_m_isconnected"), string("true"));
    }
    else
    {
        m_poseResetRequested = true;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnavdebug"), string("AttemptSetRobotPose_m_isconnected"), string("false"));
    }
#endif
}

void DragonQuest::SetRobotPose(const frc::Pose2d &pose)
{
    frc::Pose2d questPose = pose.TransformBy(m_questToRobotTransform);

    // Create pose reset command
    questnav::protos::commands::ProtobufQuestNavCommand command;
    command.set_type(questnav::protos::commands::POSE_RESET);
    command.set_command_id(m_nextCommandId++);

    auto *payload = command.mutable_pose_reset_payload();
    auto *targetPose = payload->mutable_target_pose();

    // Set translation
    auto *translation = targetPose->mutable_translation();
    translation->set_x(questPose.X().value());
    translation->set_y(questPose.Y().value());
    translation->set_z(0.0);

    // Set rotation using quaternion
    auto *rotation = targetPose->mutable_rotation();
    auto *quaternion = rotation->mutable_q();

    // Convert 2D rotation to quaternion (rotation around Z axis)
    double yawRadians = questPose.Rotation().Radians().value();
    double halfYaw = yawRadians / 2.0;

    quaternion->set_w(std::cos(halfYaw)); // Real part
    quaternion->set_x(0.0);               // X axis
    quaternion->set_y(0.0);               // Y axis
    quaternion->set_z(std::sin(halfYaw)); // Z axis (yaw rotation)

    // Serialize and publish
    std::string serialized;
    command.SerializeToString(&serialized);
    m_commandPublisher.Set(std::span<const uint8_t>(
        reinterpret_cast<const uint8_t *>(serialized.data()),
        serialized.size()));

    m_hasReset = true;
    m_poseResetRequested = false;
}

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

void DragonQuest::NotifyStateUpdate(RobotStateChanges::StateChange change, bool value)
{
    if (RobotStateChanges::StateChange::ClimbModeStatus_Bool == change)
        m_isClimbMode = value;
}
DragonVisionPoseEstimatorStruct DragonQuest::GetPoseEstimate()
{
#ifdef __FRC_ROBORIO__
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnavdebug"), string("m_hasReset"), m_hasReset);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnavdebug"), string("m_isConnected"), m_isConnected);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnavdebug"), string("m_isQuestEnabled"), m_isQuestEnabled);

    DragonVisionPoseEstimatorStruct str;
    if (!m_hasReset || !m_isConnected || !m_isQuestEnabled)
    {
        str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::NONE;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnavdebug"), string("confidence"), string("NONE"));
    }
    else
    {
        // str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::HIGH;
        str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::NONE;
        str.m_visionPose = m_lastCalculatedPose;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnavdebug"), string("x"), str.m_visionPose.X().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnavdebug"), string("y"), str.m_visionPose.Y().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnavdebug"), string("rot"), str.m_visionPose.Rotation().Degrees().value());
        str.m_stds = wpi::array{m_stdxy, m_stdxy, m_stddeg};
        str.m_timeStamp = units::time::second_t(m_frameDataSubscriber.GetAtomic().serverTime);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnavdebug"), string("confidence"), string("HIGH"));
    }
    return str;
#else
    DragonVisionPoseEstimatorStruct str;
    str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::NONE;
    return str;
#endif
}
