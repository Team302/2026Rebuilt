#include "chassis/generated/Telemetry.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;

Telemetry::Telemetry()
{
    /* Set up the module state Mechanism2d telemetry */
    for (size_t i = 0; i < m_moduleSpeeds.size(); ++i)
    {
        frc::SmartDashboard::PutData("Module " + std::to_string(i), &m_moduleMechanisms[i]);
    }

    // Register this instance with DragonDataLoggerMgr for periodic logging
    DragonDataLoggerMgr::GetInstance()->RegisterItem(this);
}

void Telemetry::Telemeterize(subsystems::CommandSwerveDrivetrain::SwerveDriveState const &state)
{
    ChassisConfigMgr *configMgr = ChassisConfigMgr::GetInstance();
    MaxSpeed = configMgr->GetMaxSpeed();

    /* Telemeterize the swerve drive state */
    drivePose.Set(state.Pose);
    driveSpeeds.Set(state.Speeds);
    driveModuleStates.Set(state.ModuleStates);
    driveModuleTargets.Set(state.ModuleTargets);
    driveModulePositions.Set(state.ModulePositions);
    driveTimestamp.Set(state.Timestamp.value());
    driveOdometryFrequency.Set(1.0 / state.OdometryPeriod.value());

    // Cache state data for periodic logging
    m_cachedPose = state.Pose;
    m_cachedSpeeds = state.Speeds;
    for (size_t i = 0; i < state.ModuleStates.size(); ++i)
    {
        m_cachedModuleStates[i] = state.ModuleStates[i];
    }
    for (size_t i = 0; i < state.ModuleTargets.size(); ++i)
    {
        m_cachedModuleTargets[i] = state.ModuleTargets[i];
    }
    for (size_t i = 0; i < state.ModulePositions.size(); ++i)
    {
        m_cachedModulePositions[i] = state.ModulePositions[i];
    }
    m_cachedOdometryPeriod = state.OdometryPeriod;

    /* Telemeterize each module state to a Mechanism2d */
    for (size_t i = 0; i < m_moduleSpeeds.size(); ++i)
    {
        m_moduleDirections[i]->SetAngle(state.ModuleStates[i].angle.Degrees());
        m_moduleSpeeds[i]->SetAngle(state.ModuleStates[i].angle.Degrees());
        m_moduleSpeeds[i]->SetLength(state.ModuleStates[i].speed / (2 * MaxSpeed));
    }
}

void Telemetry::DataLog(uint64_t timestamp)
{
    /* Log the cached drive state data using DragonDataLogger methods */
    double x = m_cachedPose.X().value();
    double y = m_cachedPose.Y().value();
    double rot = m_cachedPose.Rotation().Radians().value();
    LogDoubleArrayData(timestamp, "/Chassis/Pose2d", {x, y, rot}, "X, Y, Rotation");

    LogChassisSpeedsData(timestamp,
                         "/Chassis/ActualSpeeds/Vx", "/Chassis/ActualSpeeds/Vy", "/Chassis/ActualSpeeds/Omega",
                         m_cachedSpeeds, "Vx, Vy, Omega");

    // Log module states
    LogSwerveModuleStateData(timestamp, "/Chassis/FrontLeftModule/ActualState/Speed", "/Chassis/FrontLeftModule/ActualState/Angle", m_cachedModuleStates[0], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, "/Chassis/FrontRightModule/ActualState/Speed", "/Chassis/FrontRightModule/ActualState/Angle", m_cachedModuleStates[1], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, "/Chassis/BackLeftModule/ActualState/Speed", "/Chassis/BackLeftModule/ActualState/Angle", m_cachedModuleStates[2], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, "/Chassis/BackRightModule/ActualState/Speed", "/Chassis/BackRightModule/ActualState/Angle", m_cachedModuleStates[3], "Speed, Angle");

    // Log module targets
    LogSwerveModuleStateData(timestamp, "/Chassis/FrontLeftModule/TargetState/Speed", "/Chassis/FrontLeftModule/TargetState/Angle", m_cachedModuleTargets[0], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, "/Chassis/FrontRightModule/TargetState/Speed", "/Chassis/FrontRightModule/TargetState/Angle", m_cachedModuleTargets[1], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, "/Chassis/BackLeftModule/TargetState/Speed", "/Chassis/BackLeftModule/TargetState/Angle", m_cachedModuleTargets[2], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, "/Chassis/BackRightModule/TargetState/Speed", "/Chassis/BackRightModule/TargetState/Angle", m_cachedModuleTargets[3], "Speed, Angle");

    // OdometryPeriod logged as a double
    LogDoubleData(timestamp, "/Chassis/Yaw", m_cachedOdometryPeriod.value(), "Degrees");
}
