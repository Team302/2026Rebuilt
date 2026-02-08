/*
 * QUESTNAV
 *   https://github.com/QuestNav
 * Copyright (C) 2025 QuestNav
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the MIT License as published.
 *
 * C++ port for Team 302 usage.
 */

#include "vision/Questnavlib/QuestNav.h"

#include <cmath>
#include <cstring>
#include <string>

#include "frc/DriverStation.h"
#include "frc/Timer.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Translation3d.h"
#include "networktables/NetworkTableInstance.h"
#include "units/time.h"
#include "utils/logging/debug/Logger.h"

#ifdef __FRC_ROBORIO__
#include "vision/Questnavlib/commands.pb.h"
#include "vision/Questnavlib/data.pb.h"
#include "vision/Questnavlib/geometry3d.pb.h"
#endif

// ──────────────────────────────────────────────────────────────────────────────
// Version – mirror the Java BuildConfig.APP_VERSION.
// Update this string whenever you update the QuestNav vendordep.
// ──────────────────────────────────────────────────────────────────────────────
static constexpr const char *kLibVersion = "2025.2.0";

// ──────────────────────────────────────────────────────────────────────────────
// Constructor
// ──────────────────────────────────────────────────────────────────────────────
QuestNav::QuestNav()
{
    auto inst = nt::NetworkTableInstance::GetDefault();
    m_questNavTable = inst.GetTable("QuestNav");

    // PubSub options – match the Java library's configuration
    nt::PubSubOptions frameOpts;
    frameOpts.periodic = 0.01;
    frameOpts.sendAll = true;
    frameOpts.pollStorage = 20;

    nt::PubSubOptions responseOpts;
    responseOpts.periodic = 0.05;
    responseOpts.sendAll = true;
    responseOpts.pollStorage = 20;

    // Subscribers
    m_frameDataSubscriber = m_questNavTable->GetRawTopic("frameData")
                                .Subscribe("proto:questnav.protos.data.ProtobufQuestNavFrameData", {}, frameOpts);

    m_deviceDataSubscriber = m_questNavTable->GetRawTopic("deviceData")
                                 .Subscribe("proto:questnav.protos.data.ProtobufQuestNavDeviceData", {});

    m_responseSubscriber = m_questNavTable->GetRawTopic("response")
                               .Subscribe("proto:questnav.protos.commands.ProtobufQuestNavCommandResponse", {}, responseOpts);

    // Publisher
    m_requestPublisher = m_questNavTable->GetRawTopic("request")
                             .Publish("proto:questnav.protos.commands.ProtobufQuestNavCommand");

    // Version subscriber
    m_versionSubscriber = m_questNavTable->GetStringTopic("version").Subscribe("unknown");
}

// ──────────────────────────────────────────────────────────────────────────────
// SetPose – send a pose-reset command to the Quest headset
// ──────────────────────────────────────────────────────────────────────────────
void QuestNav::SetPose(const frc::Pose3d &pose)
{
#ifdef __FRC_ROBORIO__
    questnav::protos::commands::ProtobufQuestNavCommand command;
    command.set_type(questnav::protos::commands::POSE_RESET);
    command.set_command_id(++m_lastSentRequestId);

    auto *payload = command.mutable_pose_reset_payload();
    auto *targetPose = payload->mutable_target_pose();

    Pose3dToProtobuf(pose, targetPose);

    // Serialize and publish
    std::string serialized;
    command.SerializeToString(&serialized);
    m_requestPublisher.Set(std::span<const uint8_t>(
        reinterpret_cast<const uint8_t *>(serialized.data()),
        serialized.size()));
#endif
}

// ──────────────────────────────────────────────────────────────────────────────
// GetBatteryPercent
// ──────────────────────────────────────────────────────────────────────────────
std::optional<int> QuestNav::GetBatteryPercent()
{
#ifdef __FRC_ROBORIO__
    auto rawData = m_deviceDataSubscriber.Get();
    if (rawData.empty())
    {
        return std::nullopt;
    }

    questnav::protos::data::ProtobufQuestNavDeviceData deviceData;
    if (!deviceData.ParseFromArray(rawData.data(), rawData.size()))
    {
        return std::nullopt;
    }

    return deviceData.battery_percent();
#else
    return std::nullopt;
#endif
}

// ──────────────────────────────────────────────────────────────────────────────
// GetFrameCount
// ──────────────────────────────────────────────────────────────────────────────
std::optional<int> QuestNav::GetFrameCount()
{
#ifdef __FRC_ROBORIO__
    auto rawData = m_frameDataSubscriber.Get();
    if (rawData.empty())
    {
        return std::nullopt;
    }

    questnav::protos::data::ProtobufQuestNavFrameData frameData;
    if (!frameData.ParseFromArray(rawData.data(), rawData.size()))
    {
        return std::nullopt;
    }

    return frameData.frame_count();
#else
    return std::nullopt;
#endif
}

// ──────────────────────────────────────────────────────────────────────────────
// GetTrackingLostCounter
// ──────────────────────────────────────────────────────────────────────────────
std::optional<int> QuestNav::GetTrackingLostCounter()
{
#ifdef __FRC_ROBORIO__
    auto rawData = m_deviceDataSubscriber.Get();
    if (rawData.empty())
    {
        return std::nullopt;
    }

    questnav::protos::data::ProtobufQuestNavDeviceData deviceData;
    if (!deviceData.ParseFromArray(rawData.data(), rawData.size()))
    {
        return std::nullopt;
    }

    return deviceData.tracking_lost_counter();
#else
    return std::nullopt;
#endif
}

// ──────────────────────────────────────────────────────────────────────────────
// IsConnected – true when the last frame arrived within 50 ms
// ──────────────────────────────────────────────────────────────────────────────
bool QuestNav::IsConnected()
{
    // getLastChange() returns microseconds; Timer::GetFPGATimestamp() returns seconds
    double nowSeconds = frc::Timer::GetFPGATimestamp().value();
    double lastChangeMicroseconds = static_cast<double>(m_frameDataSubscriber.GetLastChange());
    double lastChangeSeconds = lastChangeMicroseconds / 1'000'000.0;
    double staleness = nowSeconds - lastChangeSeconds;
    return staleness < 0.050; // 50 ms threshold
}

// ──────────────────────────────────────────────────────────────────────────────
// GetLatency
// ──────────────────────────────────────────────────────────────────────────────
double QuestNav::GetLatency()
{
    double nowSeconds = frc::Timer::GetFPGATimestamp().value();
    double lastChangeMicroseconds = static_cast<double>(m_frameDataSubscriber.GetLastChange());
    double lastChangeSeconds = lastChangeMicroseconds / 1'000'000.0;
    double stalenessMs = (nowSeconds - lastChangeSeconds) * 1000.0;
    return stalenessMs;
}

// ──────────────────────────────────────────────────────────────────────────────
// GetAppTimestamp
// ──────────────────────────────────────────────────────────────────────────────
std::optional<double> QuestNav::GetAppTimestamp()
{
#ifdef __FRC_ROBORIO__
    auto rawData = m_frameDataSubscriber.Get();
    if (rawData.empty())
    {
        return std::nullopt;
    }

    questnav::protos::data::ProtobufQuestNavFrameData frameData;
    if (!frameData.ParseFromArray(rawData.data(), rawData.size()))
    {
        return std::nullopt;
    }

    return frameData.timestamp();
#else
    return std::nullopt;
#endif
}

// ──────────────────────────────────────────────────────────────────────────────
// IsTracking
// ──────────────────────────────────────────────────────────────────────────────
bool QuestNav::IsTracking()
{
#ifdef __FRC_ROBORIO__
    auto rawData = m_frameDataSubscriber.Get();
    if (rawData.empty())
    {
        return false;
    }

    questnav::protos::data::ProtobufQuestNavFrameData frameData;
    if (!frameData.ParseFromArray(rawData.data(), rawData.size()))
    {
        return false;
    }

    return frameData.istracking();
#else
    return false;
#endif
}

// ──────────────────────────────────────────────────────────────────────────────
// GetAllUnreadPoseFrames
// ──────────────────────────────────────────────────────────────────────────────
std::vector<PoseFrame> QuestNav::GetAllUnreadPoseFrames()
{
    std::vector<PoseFrame> result;

#ifdef __FRC_ROBORIO__
    auto timestampedValues = m_frameDataSubscriber.ReadQueue();

    result.reserve(timestampedValues.size());

    for (auto &tv : timestampedValues)
    {
        questnav::protos::data::ProtobufQuestNavFrameData frameData;
        if (!frameData.ParseFromArray(tv.value.data(), tv.value.size()))
        {
            continue; // Skip frames that fail to parse
        }

        PoseFrame frame;

        // Convert protobuf pose to frc::Pose3d
        if (frameData.has_pose3d())
        {
            frame.questPose3d = ProtobufToPose3d(frameData.pose3d());
        }

        // serverTime is in microseconds – convert to seconds
        frame.dataTimestamp = static_cast<double>(tv.serverTime) / 1'000'000.0;
        frame.appTimestamp = frameData.timestamp();
        frame.frameCount = frameData.frame_count();
        frame.isTracking = frameData.istracking();

        result.push_back(frame);
    }
#endif

    return result;
}

// ──────────────────────────────────────────────────────────────────────────────
// CommandPeriodic – process command responses and log errors
// ──────────────────────────────────────────────────────────────────────────────
void QuestNav::CommandPeriodic()
{
    CheckVersionMatch();

#ifdef __FRC_ROBORIO__
    auto responses = m_responseSubscriber.ReadQueue();

    for (auto &tv : responses)
    {
        questnav::protos::commands::ProtobufQuestNavCommandResponse response;
        if (!response.ParseFromArray(tv.value.data(), tv.value.size()))
        {
            continue; // Skip unparseable responses
        }

        if (!response.success())
        {
            std::string msg = "QuestNav command failed!\n" + response.error_message();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, "QuestNav", "questnavCommandFailed", msg);
        }
    }
#endif
}

// ──────────────────────────────────────────────────────────────────────────────
// GetLibVersion
// ──────────────────────────────────────────────────────────────────────────────
std::string QuestNav::GetLibVersion() const
{
    return kLibVersion;
}

// ──────────────────────────────────────────────────────────────────────────────
// GetQuestNavVersion
// ──────────────────────────────────────────────────────────────────────────────
std::string QuestNav::GetQuestNavVersion()
{
    return m_versionSubscriber.Get();
}

// ──────────────────────────────────────────────────────────────────────────────
// SetVersionCheckEnabled
// ──────────────────────────────────────────────────────────────────────────────
void QuestNav::SetVersionCheckEnabled(bool enabled)
{
    m_versionCheckEnabled = enabled;
}

// ──────────────────────────────────────────────────────────────────────────────
// CheckVersionMatch (private)
// ──────────────────────────────────────────────────────────────────────────────
void QuestNav::CheckVersionMatch()
{
    if (!m_versionCheckEnabled || !IsConnected())
    {
        return;
    }

    double currentTime = frc::Timer::GetFPGATimestamp().value();
    if ((currentTime - m_lastVersionCheckTime) < kVersionCheckIntervalSeconds)
    {
        return;
    }
    m_lastVersionCheckTime = currentTime;

    auto libVersion = GetLibVersion();
    auto questNavVersion = GetQuestNavVersion();

    if (questNavVersion != libVersion)
    {
        static const std::string msg = std::string("WARNING FROM QUESTNAV: QuestNavLib version (") + libVersion + ") on your robot does not match QuestNav app version (" + questNavVersion + ") on your headset. This may cause compatibility issues. Check the version of your vendordep and the app running on your headset.";
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, "QuestNav", "questnavLibVersionMismatch", msg);
    }
}

#ifdef __FRC_ROBORIO__

// ──────────────────────────────────────────────────────────────────────────────
// ProtobufToPose3d – helper to unpack a wpi::proto::ProtobufPose3d
// ──────────────────────────────────────────────────────────────────────────────
frc::Pose3d QuestNav::ProtobufToPose3d(const wpi::proto::ProtobufPose3d &proto)
{
    double tx = 0.0, ty = 0.0, tz = 0.0;
    if (proto.has_translation())
    {
        tx = proto.translation().x();
        ty = proto.translation().y();
        tz = proto.translation().z();
    }

    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
    if (proto.has_rotation() && proto.rotation().has_q())
    {
        qw = proto.rotation().q().w();
        qx = proto.rotation().q().x();
        qy = proto.rotation().q().y();
        qz = proto.rotation().q().z();
    }

    return frc::Pose3d{
        frc::Translation3d{units::meter_t{tx}, units::meter_t{ty}, units::meter_t{tz}},
        frc::Rotation3d{frc::Quaternion{qw, qx, qy, qz}}};
}

// ──────────────────────────────────────────────────────────────────────────────
// Pose3dToProtobuf – helper to pack an frc::Pose3d into a wpi::proto::ProtobufPose3d
// ──────────────────────────────────────────────────────────────────────────────
void QuestNav::Pose3dToProtobuf(const frc::Pose3d &pose, wpi::proto::ProtobufPose3d *proto)
{

    auto *translation = proto->mutable_translation();
    translation->set_x(pose.X().value());
    translation->set_y(pose.Y().value());
    translation->set_z(pose.Z().value());

    auto *rotation = proto->mutable_rotation();
    auto *quaternion = rotation->mutable_q();
    auto q = pose.Rotation().GetQuaternion();
    quaternion->set_w(q.W());
    quaternion->set_x(q.X());
    quaternion->set_y(q.Y());
    quaternion->set_z(q.Z());
}
#endif
