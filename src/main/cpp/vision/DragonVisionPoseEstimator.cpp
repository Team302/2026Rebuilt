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

/**
 * @file DragonVisionPoseEstimator.cpp
 * @brief Implements DragonVisionPoseEstimator: bridges the vision subsystem to the robot
 *        chassis pose estimator.
 *
 * Responsibilities:
 *  - Acquire and maintain a reference to the DragonVision subsystem.
 *  - Initialize the robot pose using reliable vision tag observations (prefers MegaTag1 then MegaTag2).
 *  - Periodically feed vision-derived pose measurements into the chassis's estimator.
 *  - Maintain DragonQuest heartbeat/NT refresh while active.
 *
 * Current behavior:
 *  - CalculateInitialPose(): attempts to set an initial pose only when MegaTag1 and MegaTag2
 *    provide consistent estimates (MegaTag1 is preferred first; MegaTag2 confirms).
 *  - AddVisionMeasurements(): currently uses MegaTag2 for ongoing vision updates and calls
 *    m_chassis->AddVisionMeasurement(...) with the measured pose, timestamp, and std deviations.
 *
 * Dependencies:
 *  - DragonVision: supplies tag-based robot pose estimates and timestamps.
 *  - DragonQuest: heartbeat/NT refresh calls are performed in Execute().
 *  - ChassisConfigMgr / SwerveChassis: receives vision measurements via AddVisionMeasurement.
 *
 * Known limitations & assumptions:
 *  - Not thread safe; intended to run on the main robot thread / scheduler.
 *  - Timestamp units: AddVisionMeasurements converts the DragonVision timestamp directly into
 *    units::second_t. Ensure DragonVision timestamps are in seconds (or adjust conversion accordingly).
 *  - Current continuous updates use MegaTag2; if additional estimators are added, AddVisionMeasurements
 *    should be extended to aggregate multiple vision sources and check confidence.
 *
 * Notes:
 *  - This file contains only implementation details — consult the header for public API.
 *  - Keep documentation near the implementation when behavior/assumptions change.
 */

#include <optional>

#include "chassis/ChassisConfigMgr.h"
#include "frc/RobotBase.h"

#include "utils/PeriodicLooper.h"
#include "vision/DragonQuest.h"
#include "vision/DragonVision.h"
#include "vision/DragonVisionPoseEstimator.h"
#include "vision/DragonVisionPoseEstimatorStruct.h"

DragonVisionPoseEstimator::DragonVisionPoseEstimator()
{
    m_vision = DragonVision::GetDragonVision();
    m_chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();
    PeriodicLooper::GetInstance()->RegisterAll(this);
}
void DragonVisionPoseEstimator::RunCommonTasks()
{
    // Nothing to do here yet
}

/**
 * @brief Main periodic execution.
 *
 * Ensures vision pointer availability, attempts to set an initial pose if needed,
 * maintains DragonQuest heartbeat/NT refresh, and pushes vision measurements to the chassis.
 */
void DragonVisionPoseEstimator::RunCurrentState()
{
    // Make sure we have a vision subsystem pointer.
    // If we don't, try to get one
    // If we still don't have one exit
    // Only calculate initial pose if robot hasn't been enabled yet
    if (frc::DriverStation::IsDisabled() && !m_hasBeenEnabled)
    {
        CalculateInitialPose();
    }
    // Update the latch - once enabled, stay latched
    if (!m_hasBeenEnabled && !frc::DriverStation::IsDisabled())
    {
        m_hasBeenEnabled = true;
    }
    if (m_vision == nullptr)
    {
        m_vision = DragonVision::GetDragonVision();
    }
    if (m_vision == nullptr)
    {
        return;
    }

    m_vision->RefreshQuestData();

    AddVisionMeasurements();
}

void DragonVisionPoseEstimator::SetCurrentState(int state, bool run)
{
    // Nothing to do here yet
}

/**
 * @brief Get the current chassis pose.
 * @returns the chassis pose if available, otherwise a default Pose2d.
 */
frc::Pose2d DragonVisionPoseEstimator::GetPose() const
{
    return (m_chassis != nullptr) ? m_chassis->GetPose() : frc::Pose2d{};
}

/**
 * @brief Reset the chassis position to the provided pose.
 *
 * Wrapper around chassis ResetPose to centralize pose reset logic used by vision.
 */
void DragonVisionPoseEstimator::ResetPosition(const frc::Pose2d &pose)
{
    if (m_chassis != nullptr)
    {
        m_chassis->ResetPose(pose);
    }
}

/**
 * @brief Attempt to compute and set the initial robot pose from vision tags.
 *
 * Strategy:
 *  - Prefer MegaTag1; only if MegaTag1 and MegaTag2 both provide estimates do we mark
 *    the initial pose as set. This reduces the chance of incorrect resets.
 */
void DragonVisionPoseEstimator::CalculateInitialPose()
{
    if (m_vision == nullptr)
    {
        m_vision = DragonVision::GetDragonVision();
    }
    if (m_vision == nullptr)
    {
        return;
    }

    auto megaTag2Positions = m_vision->GetRobotPositionMegaTag2();
    if (!megaTag2Positions.empty())
    {
        auto pose = megaTag2Positions[0].estimatedPose.ToPose2d();
        ResetPosition(pose);
        m_vision->ResetQuestRobotPose(pose);
        // m_initialPoseSet = true;
    }
}

/**
 * @brief Pull vision pose estimates and push them into the chassis estimator.
 *
 * Currently the method contains TODOs — implementation should iterate estimators
 * from the vision subsystem and call the chassis AddVisionMeasurement APIs.
 */
void DragonVisionPoseEstimator::AddVisionMeasurements()
{
    if (!frc::RobotBase::IsSimulation())
    {
        if (m_vision == nullptr || m_chassis == nullptr)
        {
            return;
        }

        auto poses = m_vision->GetRobotPositionMegaTag2();
        for (auto pose : poses)
        {
            m_chassis->AddVisionMeasurement(pose.estimatedPose.ToPose2d(), units::second_t{pose.timeStamp}, pose.visionMeasurementStdDevs);
        }

        auto questPose = m_vision->GetRobotPositionQuest();
        if (questPose.m_confidenceLevel == DragonVisionPoseEstimatorStruct::ConfidenceLevel::HIGH)
        {
            m_chassis->AddVisionMeasurement(questPose.m_visionPose, questPose.m_timeStamp, questPose.m_stds);
        }
    }
}
