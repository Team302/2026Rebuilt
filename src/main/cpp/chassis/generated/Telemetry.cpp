#include "chassis/generated/Telemetry.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;

Telemetry::Telemetry()
{
    SignalLogger::Start();

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
    Log2DPoseData(timestamp, DragonDataLogger::PoseSingals::CURRENT_CHASSIS_POSE2D, m_cachedPose);
    LogChassisSpeedsData(timestamp, DragonDataLogger::ChassisSpeedSignals::ACTUAL_SPEEDS, m_cachedSpeeds);

    // Log module states
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::ACTUAL_LEFT_FRONT_STATE, m_cachedModuleStates[0]);
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::ACTUAL_RIGHT_FRONT_STATE, m_cachedModuleStates[1]);
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::ACTUAL_LEFT_BACK_STATE, m_cachedModuleStates[2]);
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::ACTUAL_RIGHT_BACK_STATE, m_cachedModuleStates[3]);

    // Log module targets
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::TARGET_LEFT_FRONT_STATE, m_cachedModuleTargets[0]);
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::TARGET_RIGHT_FRONT_STATE, m_cachedModuleTargets[1]);
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::TARGET_LEFT_BACK_STATE, m_cachedModuleTargets[2]);
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::TARGET_RIGHT_BACK_STATE, m_cachedModuleTargets[3]);

    // Note: Module positions are not directly supported by DragonDataLogger's existing methods
    // OdometryPeriod can be logged as a double if needed
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::CHASSIS_YAW_DEGREES, m_cachedOdometryPeriod.value());
}
