#include "chassis/generated/Telemetry.h"
#include "utils/logging/debug/Logger.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;

Telemetry::Telemetry()
{
    /* Set up the module state Mechanism2d telemetry */
    for (size_t i = 0; i < m_moduleSpeeds.size(); ++i)
    {
        frc::SmartDashboard::PutData("Module " + std::to_string(i), &m_moduleMechanisms[i]);
    }
}

void Telemetry::Telemeterize(subsystems::CommandSwerveDrivetrain::SwerveDriveState const &state)
{
    auto logger = Logger::GetLogger();
    ChassisConfigMgr *configMgr = ChassisConfigMgr::GetInstance();
    MaxSpeed = configMgr->GetMaxSpeed();

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

    double x = m_cachedPose.X().value();
    double y = m_cachedPose.Y().value();
    double rot = m_cachedPose.Rotation().Radians().value();
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("Pose2dX"), x);
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("Pose2dY"), y);
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("Pose2dRotation"), rot);
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("FrontLeftActualStateSpeed"), m_cachedModuleStates[0].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("FrontLeftActualStateAngle"), m_cachedModuleStates[0].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("FrontRightActualStateSpeed"), m_cachedModuleStates[1].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("FrontRightActualStateAngle"), m_cachedModuleStates[1].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("BackLeftActualStateSpeed"), m_cachedModuleStates[2].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("BackLeftActualStateAngle"), m_cachedModuleStates[2].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("BackRightActualStateSpeed"), m_cachedModuleStates[3].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("BackRightActualStateAngle"), m_cachedModuleStates[3].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("FrontLeftTargetStateSpeed"), m_cachedModuleTargets[0].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("FrontLeftTargetStateAngle"), m_cachedModuleTargets[0].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("FrontRightTargetStateSpeed"), m_cachedModuleTargets[1].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("FrontRightTargetStateAngle"), m_cachedModuleTargets[1].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("BackLeftTargetStateSpeed"), m_cachedModuleTargets[2].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("BackLeftTargetStateAngle"), m_cachedModuleTargets[2].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("BackRightTargetStateSpeed"), m_cachedModuleTargets[3].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("BackRightTargetStateAngle"), m_cachedModuleTargets[3].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("SwerveTelemetry"), std::string("OdometryPeriod"), m_cachedOdometryPeriod.value());
}

void Telemetry::DataLog(uint64_t timestamp)
{
    /* Log the cached drive state data using DragonDataLogger methods */
    LogPose2dData(timestamp, "/Chassis/Pose2d", m_cachedPose);

    LogChassisSpeedsData(timestamp,
                         "/Chassis/ActualSpeeds/Vx", "/Chassis/ActualSpeeds/Vy", "/Chassis/ActualSpeeds/Omega",
                         m_cachedSpeeds, "Vx, Vy, Omega");

    // Log module states
    LogSwerveModuleStateData(timestamp, m_frontLeftModuleActualSpeed, m_frontLeftModuleActualAngle, m_cachedModuleStates[0], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, m_frontRightModuleActualSpeed, m_frontRightModuleActualAngle, m_cachedModuleStates[1], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, m_backLeftModuleActualSpeed, m_backLeftModuleActualAngle, m_cachedModuleStates[2], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, m_backRightModuleActualSpeed, m_backRightModuleActualAngle, m_cachedModuleStates[3], "Speed, Angle");

    // Log module targets
    LogSwerveModuleStateData(timestamp, m_frontLeftModuleTargetSpeed, m_frontLeftModuleTargetAngle, m_cachedModuleTargets[0], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, m_frontRightModuleTargetSpeed, m_frontRightModuleTargetAngle, m_cachedModuleTargets[1], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, m_backLeftModuleTargetSpeed, m_backLeftModuleTargetAngle, m_cachedModuleTargets[2], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, m_backRightModuleTargetSpeed, m_backRightModuleTargetAngle, m_cachedModuleTargets[3], "Speed, Angle");

    // OdometryPeriod logged as a double
    LogDoubleData(timestamp, "/Chassis/OdometryPeriod", m_cachedOdometryPeriod.value(), "Seconds");
}
