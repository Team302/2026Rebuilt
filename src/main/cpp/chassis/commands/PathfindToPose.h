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
#include <vector>

#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include <frc/Timer.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>

#include "vision/DragonVision.h"

// Note: To compile this class, PathPlannerLib needs to be installed in vendordeps.
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/pathfinding/Pathfinding.h>
#include <pathplanner/lib/trajectory/PathPlannerTrajectory.h>

class PathfindToPose : public frc2::CommandHelper<frc2::Command, PathfindToPose>
{
public:
    PathfindToPose(subsystems::CommandSwerveDrivetrain *chassis);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

    void SetTargetPose(const frc::Pose2d &targetPose);

protected:
    subsystems::CommandSwerveDrivetrain *GetChassis() const { return m_chassis; }

private:
    subsystems::CommandSwerveDrivetrain *m_chassis;
    DragonVision *m_vision;

    // Controllers
    frc::TrapezoidProfile<units::length::meters>::Constraints m_translationConstraints;
    frc::ProfiledPIDController<units::length::meters> m_translationPIDX;
    frc::ProfiledPIDController<units::length::meters> m_translationPIDY;

    // Output Request
    swerve::requests::FieldCentricFacingAngle m_driveRequest;

    // Limits
    units::velocity::meters_per_second_t kMaxVelocity = 4_mps;
    units::acceleration::meters_per_second_squared_t kMaxAcceleration = 3_mps_sq;

    frc::Pose2d m_targetPose;
    frc::Pose2d m_currentPose;

    // PathPlanner state
    std::shared_ptr<pathplanner::PathPlannerPath> m_currentPath;
    std::optional<pathplanner::PathPlannerTrajectory> m_currentTrajectory;
    frc::Timer m_pathTimer;

    // Vision and Replanning
    std::vector<std::unique_ptr<DragonVisionStruct>> m_visionCache;
    frc::Timer m_replanTimer;

    void ReplanPath();
};
