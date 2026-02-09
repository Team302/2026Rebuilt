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

#include "frc/geometry/Pose3d.h"

/**
 * Represents a single frame of pose tracking data received from the Quest headset.
 *
 * This struct encapsulates all the information needed to integrate Quest tracking data with FRC
 * pose estimation systems. Each frame represents a single tracking measurement from the Quest's
 * visual-inertial odometry system.
 *
 * Two timestamps are provided for different use cases:
 *   - dataTimestamp: NetworkTables reception time — use this for pose estimation
 *   - appTimestamp:  Quest internal time — use only for debugging/diagnostics
 *
 * The pose data follows WPILib field coordinate conventions:
 *   - X-axis: Forward (towards opposing alliance)
 *   - Y-axis: Left (when facing forward)
 *   - Rotation: Counter-clockwise positive
 *   - Units: Meters for translation, radians for rotation
 */
struct PoseFrame
{
    /// The robot's pose on the field as measured by the Quest tracking system.
    frc::Pose3d questPose3d{};

    /// NetworkTables timestamp indicating when this frame data was received by the robot (seconds).
    double dataTimestamp{0.0};

    /// Quest application's internal timestamp indicating when this frame was generated (seconds).
    double appTimestamp{0.0};

    /// Sequential frame number from the Quest tracking system.
    int frameCount{0};

    /// Whether the Quest is currently tracking its position.
    bool isTracking{false};
};
