#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "utils/FMSData.h"
#include "utils/logging/debug/Logger.h"
#include <frc/RobotController.h>
using namespace subsystems;

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled())
    {
        auto const allianceColor = FMSData::GetAllianceColor();

        SetOperatorPerspectiveForward(
            allianceColor == frc::DriverStation::Alliance::kRed
                ? kRedAlliancePerspectiveRotation
                : kBlueAlliancePerspectiveRotation);
        m_hasAppliedOperatorPerspective = true;
    }
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();
    m_simNotifier = std::make_unique<frc::Notifier>([this]
                                                    {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage()); });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

bool CommandSwerveDrivetrain::IsSamePose()
{
    return IsSamePose(m_distanceThreshold);
}

bool CommandSwerveDrivetrain::IsSamePose(units::length::inch_t distanceThreshold)
{
    frc::Pose2d currentPose = GetPose();
    bool isCurrentlyStopped = currentPose.Translation().Distance(m_prevPose.Translation()) < distanceThreshold;

    if (isCurrentlyStopped)
    {
        if (!m_debounceTimer.IsRunning())
        {
            m_debounceTimer.Start();
        }
    }
    else
    {
        m_debounceTimer.Reset();
    }
    m_prevPose = currentPose;

    return m_debounceTimer.HasElapsed(m_samePoseTime);
}
bool CommandSwerveDrivetrain::IsMoving()
{
    return IsMoving(m_velocityThreshold, m_angularVelocityThreshold);
}
bool CommandSwerveDrivetrain::IsMoving(units::velocity::meters_per_second_t velocityThreshold, units::angular_velocity::degrees_per_second_t angularVelocityThreshold)
{
    return (units::math::abs(this->GetState().Speeds.vx) > velocityThreshold) || (units::math::abs(this->GetState().Speeds.vy) > velocityThreshold) || (units::math::abs(this->GetState().Speeds.omega) > angularVelocityThreshold);
}