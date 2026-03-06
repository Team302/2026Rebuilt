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

    void SetPath(const std::string &pathName);

    std::string WhyDone() const { return m_whyDone; };

    units::time::second_t GetTotalTrajectoryTime() const { return m_totalTrajectoryTime; }

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
    units::time::second_t m_thresholdTime{0.0_s}; // pre-computed kPercentComplete * m_totalTrajectoryTime
    units::time::second_t m_elapsedTime{0.0_s};   // cached timer value, updated once per Execute() call

    static constexpr double kPDrive{3.5};
    static constexpr double kIDrive{0.0};
    static constexpr double kDDrive{0.0};

    static constexpr double kPHeading{2.0};
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
};
