#include "chassis/generated/Telemetry.h"
#include "utils/logging/debug/Logger.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;

Telemetry::Telemetry()
{
}

void Telemetry::Telemeterize(subsystems::CommandSwerveDrivetrain::SwerveDriveState const &state)
{
}

void Telemetry::DataLog(uint64_t timestamp)
{
    auto logger = Logger::GetLogger();

    /* Log the cached drive state data using DragonDataLogger methods */
    double x = m_cachedPose.X().value();
    double y = m_cachedPose.Y().value();
    double rot = m_cachedPose.Rotation().Radians().value();
    LogDoubleArrayData(timestamp, "/Chassis/Pose2d", {x, y, rot}, "X, Y, Rotation");
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("Pose2dX"), x);
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("Pose2dY"), y);
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("Pose2dRotation"), rot);

    LogChassisSpeedsData(timestamp,
                         "/Chassis/ActualSpeeds/Vx", "/Chassis/ActualSpeeds/Vy", "/Chassis/ActualSpeeds/Omega",
                         m_cachedSpeeds, "Vx, Vy, Omega");
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("ActualSpeedsVx"), m_cachedSpeeds.vx.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("ActualSpeedsVy"), m_cachedSpeeds.vy.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("ActualSpeedsOmega"), m_cachedSpeeds.omega.value());

    // Log module states
    LogSwerveModuleStateData(timestamp, "/Chassis/FrontLeftModule/ActualState/Speed", "/Chassis/FrontLeftModule/ActualState/Angle", m_cachedModuleStates[0], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, "/Chassis/FrontRightModule/ActualState/Speed", "/Chassis/FrontRightModule/ActualState/Angle", m_cachedModuleStates[1], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, "/Chassis/BackLeftModule/ActualState/Speed", "/Chassis/BackLeftModule/ActualState/Angle", m_cachedModuleStates[2], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, "/Chassis/BackRightModule/ActualState/Speed", "/Chassis/BackRightModule/ActualState/Angle", m_cachedModuleStates[3], "Speed, Angle");
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("FrontLeftActualStateSpeed"), m_cachedModuleStates[0].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("FrontLeftActualStateAngle"), m_cachedModuleStates[0].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("FrontRightActualStateSpeed"), m_cachedModuleStates[1].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("FrontRightActualStateAngle"), m_cachedModuleStates[1].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("BackLeftActualStateSpeed"), m_cachedModuleStates[2].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("BackLeftActualStateAngle"), m_cachedModuleStates[2].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("BackRightActualStateSpeed"), m_cachedModuleStates[3].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("BackRightActualStateAngle"), m_cachedModuleStates[3].angle.Degrees().value());

    // Log module targets
    LogSwerveModuleStateData(timestamp, "/Chassis/FrontLeftModule/TargetState/Speed", "/Chassis/FrontLeftModule/TargetState/Angle", m_cachedModuleTargets[0], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, "/Chassis/FrontRightModule/TargetState/Speed", "/Chassis/FrontRightModule/TargetState/Angle", m_cachedModuleTargets[1], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, "/Chassis/BackLeftModule/TargetState/Speed", "/Chassis/BackLeftModule/TargetState/Angle", m_cachedModuleTargets[2], "Speed, Angle");
    LogSwerveModuleStateData(timestamp, "/Chassis/BackRightModule/TargetState/Speed", "/Chassis/BackRightModule/TargetState/Angle", m_cachedModuleTargets[3], "Speed, Angle");
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("FrontLeftTargetStateSpeed"), m_cachedModuleTargets[0].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("FrontLeftTargetStateAngle"), m_cachedModuleTargets[0].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("FrontRightTargetStateSpeed"), m_cachedModuleTargets[1].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("FrontRightTargetStateAngle"), m_cachedModuleTargets[1].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("BackLeftTargetStateSpeed"), m_cachedModuleTargets[2].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("BackLeftTargetStateAngle"), m_cachedModuleTargets[2].angle.Degrees().value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("BackRightTargetStateSpeed"), m_cachedModuleTargets[3].speed.value());
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("BackRightTargetStateAngle"), m_cachedModuleTargets[3].angle.Degrees().value());

    // OdometryPeriod logged as a double
    LogDoubleData(timestamp, "/Chassis/Yaw", m_cachedOdometryPeriod.value(), "Degrees");
    logger->LogData(LOGGER_LEVEL::PRINT, std::string("Telemetry"), std::string("Yaw"), m_cachedOdometryPeriod.value());
}
