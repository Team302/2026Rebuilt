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
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "frc/geometry/Pose3d.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/RawTopic.h"
#include "networktables/StringTopic.h"
#include "vision/Questnavlib/PoseFrame.h"

#ifdef __FRC_ROBORIO__
#include "vision/Questnavlib/commands.pb.h"
#include "vision/Questnavlib/data.pb.h"
#endif

/**
 * The QuestNav class provides a comprehensive interface for communicating with an Oculus/Meta
 * Quest VR headset for robot localization and tracking in FRC robotics applications.
 *
 * This class handles all aspects of Quest-robot communication including:
 *   - Real-time pose tracking and localization data
 *   - Command sending and response handling
 *   - Device status monitoring (battery, tracking state, connectivity)
 *   - NetworkTables-based communication protocol
 *
 * Basic Usage:
 *   // Create QuestNav instance
 *   QuestNav questNav;
 *
 *   // Set initial robot pose (required for field-relative tracking)
 *   questNav.SetPose(frc::Pose3d{...});
 *
 *   // In robot periodic methods
 *   questNav.CommandPeriodic(); // Process command responses
 *   auto frames = questNav.GetAllUnreadPoseFrames();
 *
 * Coordinate Systems:
 *   - X-axis: Forward direction (towards opposing alliance)
 *   - Y-axis: Left direction (when facing forward)
 *   - Rotation: Counter-clockwise positive (standard mathematical convention)
 *   - Units: Meters for translation, radians for rotation
 *
 * Threading and Performance:
 *   - All methods are thread-safe for typical FRC usage patterns
 *   - NetworkTables handles the underlying communication asynchronously
 *   - Call CommandPeriodic() regularly to process command responses
 *
 * Error Handling:
 *   - Methods return std::optional types when data might not be available
 *   - Connection status can be checked with IsConnected()
 *   - Tracking status can be monitored with IsTracking()
 *   - Command failures are reported through DriverStation error logging
 */
class QuestNav
{
public:
    /**
     * Creates a new QuestNav instance for communicating with a Quest headset.
     *
     * This constructor initializes all necessary NetworkTables subscribers and publishers for
     * communication with the Quest device. The instance is ready to use immediately, but you should
     * call SetPose() to establish field-relative tracking before relying on pose data.
     */
    QuestNav();
    ~QuestNav() = default;

    // Non-copyable, non-movable
    QuestNav(const QuestNav &) = delete;
    QuestNav &operator=(const QuestNav &) = delete;

    /**
     * Sets the field-relative pose of the Quest headset by commanding it to reset its tracking.
     *
     * This method sends a pose reset command to the Quest headset, telling it where the Quest is
     * currently located on the field. This is essential for establishing field-relative tracking.
     *
     * Important: This should be the Quest's pose, not the robot's pose. If you know the robot's
     * pose, you need to apply the mounting offset to get the Quest's pose before calling this method.
     *
     * The command is sent asynchronously. Monitor command success/failure by calling
     * CommandPeriodic() regularly, which will log any errors to the DriverStation.
     *
     * @param pose The Quest's current field-relative pose in WPILib coordinates (meters, radians)
     */
    void SetPose(const frc::Pose3d &pose);

    /**
     * Returns the Quest headset's current battery level as a percentage.
     *
     * @return An optional containing the battery percentage (0-100), or empty if
     *         no device data is available or Quest is disconnected
     */
    std::optional<int> GetBatteryPercent();

    /**
     * Gets the current frame count from the Quest headset.
     *
     * @return An optional containing the frame count, or empty if no frame data is available
     */
    std::optional<int> GetFrameCount();

    /**
     * Gets the number of tracking lost events since the Quest connected to the robot.
     *
     * @return An optional containing the tracking lost counter, or empty if no device data is available
     */
    std::optional<int> GetTrackingLostCounter();

    /**
     * Determines if the Quest headset is currently connected to the robot. Connection is determined
     * by how stale the last received frame from the Quest is.
     *
     * @return true if the Quest is connected, false otherwise
     */
    bool IsConnected();

    /**
     * Gets the latency of the Quest > Robot Connection. Returns the latency between the current time
     * and the last frame data update.
     *
     * @return The latency in milliseconds
     */
    double GetLatency();

    /**
     * Returns the Quest app's uptime timestamp for debugging and diagnostics.
     *
     * Important: For integration with a pose estimator, use the timestamp from
     * PoseFrame::dataTimestamp instead! This method provides the Quest's internal application
     * timestamp.
     *
     * @return An optional containing the Quest app uptime in seconds, or empty if no frame data is available
     */
    std::optional<double> GetAppTimestamp();

    /**
     * Gets the current tracking state of the Quest headset.
     *
     * @return true if the Quest is actively tracking and pose data is reliable, false otherwise
     */
    bool IsTracking();

    /**
     * Retrieves all new pose frames received from the Quest since the last call to this method.
     *
     * This is the primary method for integrating QuestNav with FRC pose estimation systems.
     * Each frame contains pose data and a timestamp that can be fed into a pose estimator.
     *
     * Important: This method consumes the frame queue, so each frame is only returned once.
     * Call this method regularly (every robot loop) to avoid missing frames.
     *
     * @return Vector of new PoseFrame objects received since the last call. Empty if no new frames
     *         are available or Quest is disconnected.
     */
    std::vector<PoseFrame> GetAllUnreadPoseFrames();

    /**
     * Processes command responses from the Quest headset and handles any errors.
     *
     * This method must be called regularly (typically in RobotPeriodic()) to:
     *   - Process responses to commands sent via SetPose()
     *   - Log command failures to the DriverStation for debugging
     *   - Maintain proper command/response synchronization
     *   - Prevent command response queue overflow
     */
    void CommandPeriodic();

    /**
     * Retrieves the QuestNav-lib version number.
     *
     * @return The version number as a string.
     */
    std::string GetLibVersion() const;

    /**
     * Retrieves the QuestNav app version running on the Quest headset.
     *
     * @return The version number as a string, or "unknown" if unable to retrieve.
     */
    std::string GetQuestNavVersion();

    /**
     * Turns the version check on or off. When on, a warning will be reported to the DriverStation if
     * the QuestNavLib and QuestNav app versions do not match.
     *
     * @param enabled true to enable version checking, false to disable it. Default is true.
     */
    void SetVersionCheckEnabled(bool enabled);

private:
    /**
     * Checks the version of QuestNavLib and compares it to the version of QuestNav on the headset.
     * If the headset is connected and the versions don't match, a warning will be sent to the
     * DriverStation at an interval.
     */
    void CheckVersionMatch();

    /**
     * Helper: convert a protobuf Pose3d into an frc::Pose3d.
     * Called from the roborio-only code paths.
     */
    frc::Pose3d ProtobufToPose3d(const wpi::proto::ProtobufPose3d &proto);

    /**
     * Helper: pack an frc::Pose3d into a protobuf Pose3d.
     */
    void Pose3dToProtobuf(const frc::Pose3d &pose, wpi::proto::ProtobufPose3d *proto);

    /// Interval at which to check and log if the QuestNavLib version matches the QuestNav app version
    static constexpr double kVersionCheckIntervalSeconds = 5.0;

    /// NetworkTable for Quest navigation data
    std::shared_ptr<nt::NetworkTable> m_questNavTable;

    /// Raw subscribers/publishers for protobuf data
    nt::RawSubscriber m_frameDataSubscriber;
    nt::RawSubscriber m_deviceDataSubscriber;
    nt::RawSubscriber m_responseSubscriber;
    nt::RawPublisher m_requestPublisher;

    /// String subscriber for version checking
    nt::StringSubscriber m_versionSubscriber;

    /// Last sent request id
    int m_lastSentRequestId = 0;

    /// True to check for QuestNavLib and QuestNav version match at an interval
    bool m_versionCheckEnabled = true;

    /// The last time QuestNavLib and QuestNav were checked for a match
    double m_lastVersionCheckTime = 0.0;
};
