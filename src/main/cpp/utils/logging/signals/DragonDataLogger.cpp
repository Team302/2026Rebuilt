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

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "utils/logging/signals/DragonDataLogger.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"
#include "utils/logging/signals/ISignalLogger.h"


DragonDataLogger::DragonDataLogger()
{
    DragonDataLoggerMgr::GetInstance()->RegisterItem(this);
}

void DragonDataLogger::LogBoolData(uint64_t timestamp, DragonDataLogger::BoolSignals signalID, bool value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }
    
    switch (signalID)
    {
    case DragonDataLogger::BoolSignals::IS_BROWNOUT:
        logger->WriteBoolean(m_brownOutPath, value, timestamp);
        break;
    case DragonDataLogger::DRIVE_TO_IS_DONE:
        logger->WriteBoolean(m_IsDonePath, value, timestamp);
        break;
    default:
        break;
    }
}

void DragonDataLogger::LogDoubleData(uint64_t timestamp, DragonDataLogger::DoubleSignals signalID, double value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }

    switch (signalID)
    {
    case DragonDataLogger::DoubleSignals::CHASSIS_STORED_HEADING_DEGREES:
        logger->WriteDouble(m_storedHeadingPath, value, m_storedHeadingUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::CHASSIS_YAW_DEGREES:
        logger->WriteDouble(m_ChassisYawPath, value, m_ChassisYawUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::ELECTRICAL_VOLTAGE:
        logger->WriteDouble(m_electricalVoltagePath, value, m_electricalVoltageUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::ELECTRICAL_POWER:
        logger->WriteDouble(m_electricalPowerPath, value, m_electricalCurrentUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::ELECTRICAL_CURRENT:
        logger->WriteDouble(m_electricalCurrentPath, value, m_electricalCurrentUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LIMELIGHT_TV_1:
        logger->WriteDouble(m_tvPath, value, m_tvUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LIMELIGHT_TX_1:
        logger->WriteDouble(m_txPath, value, m_txUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LIMELIGHT_TY_1:
        logger->WriteDouble(m_tyPath, value, m_tyUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LIMELIGHT_FIDUCIAL_ID_1:
        logger->WriteDouble(m_fiducialPath, value, m_fiducialUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::BATTERY_VOLTAGE:
        logger->WriteDouble(m_batteryVoltagePath, value, m_batteryVoltageUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::BROWNOUT_VOLTAGE:
        logger->WriteDouble(m_brownoutVoltagePath, value, m_brownoutVoltageUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::INPUT_VOLTAGE:
        logger->WriteDouble(m_inputVoltagePath, value, m_inputVoltageUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::INPUT_CURRENT:
        logger->WriteDouble(m_inputCurrentPath, value, m_inputCurrentUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::CPU_TEMP:
        logger->WriteDouble(m_cpuTempPath, value, m_cpuTempUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_POWER:
        logger->WriteDouble(m_lfSteerPowerPath, value, m_lfSteerPowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_CURRENT:
        logger->WriteDouble(m_lfSteerCurrentPath, value, m_lfSteerCurrentUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_TOTALPOWER:
        logger->WriteDouble(m_lfSteerTotalPowerPath, value, m_lfSteerTotalPowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_WATT_HOURS:
        logger->WriteDouble(m_lfSteerWattHoursPath, value, m_lfSteerWattHoursUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_POWER:
        logger->WriteDouble(m_lfDrivePowerPath, value, m_lfDrivePowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_CURRENT:
        logger->WriteDouble(m_lfDriveCurrentPath, value, m_lfDriveCurrentUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_TOTALPOWER:
        logger->WriteDouble(m_lfDriveTotalPowerPath, value, m_lfDriveTotalPowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_WATT_HOURS:
        logger->WriteDouble(m_lfDriveWattHoursPath, value, m_lfDriveWattHoursUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_POWER:
        logger->WriteDouble(m_rfSteerPowerPath, value, m_rfSteerPowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_CURRENT:
        logger->WriteDouble(m_rfSteerCurrentPath, value, m_rfSteerCurrentUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_TOTALPOWER:
        logger->WriteDouble(m_rfSteerTotalPowerPath, value, m_rfSteerTotalPowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_WATT_HOURS:
        logger->WriteDouble(m_rfSteerWattHoursPath, value, m_rfSteerWattHoursUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_POWER:
        logger->WriteDouble(m_rfDrivePowerPath, value, m_rfDrivePowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_CURRENT:
        logger->WriteDouble(m_rfDriveCurrentPath, value, m_rfDriveCurrentUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_TOTALPOWER:
        logger->WriteDouble(m_rfDriveTotalPowerPath, value, m_rfDriveTotalPowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_WATT_HOURS:
        logger->WriteDouble(m_rfDriveWattHoursPath, value, m_rfDriveWattHoursUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_POWER:
        logger->WriteDouble(m_lbSteerPowerPath, value, m_lbSteerPowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_CURRENT:
        logger->WriteDouble(m_lbSteerCurrentPath, value, m_lbSteerCurrentUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_TOTALPOWER:
        logger->WriteDouble(m_lbSteerTotalPowerPath, value, m_lbSteerTotalPowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_WATT_HOURS:
        logger->WriteDouble(m_lbSteerWattHoursPath, value, m_lbSteerWattHoursUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_POWER:
        logger->WriteDouble(m_lbDrivePowerPath, value, m_lbDrivePowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_CURRENT:
        logger->WriteDouble(m_lbDriveCurrentPath, value, m_lbDriveCurrentUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_TOTALPOWER:
        logger->WriteDouble(m_lbDriveTotalPowerPath, value, m_lbDriveTotalPowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_WATT_HOURS:
        logger->WriteDouble(m_lbDriveWattHoursPath, value, m_lbDriveWattHoursUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_POWER:
        logger->WriteDouble(m_rbSteerPowerPath, value, m_rbSteerPowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_CURRENT:
        logger->WriteDouble(m_rbSteerCurrentPath, value, m_rbSteerPowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_TOTALPOWER:
        logger->WriteDouble(m_rbSteerTotalPowerPath, value, m_rbSteerTotalPowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_WATT_HOURS:
        logger->WriteDouble(m_rbSteerWattHoursPath, value, m_rbSteerWattHoursUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_POWER:
        logger->WriteDouble(m_rbDrivePowerPath, value, m_rbDrivePowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_CURRENT:
        logger->WriteDouble(m_rbDriveCurrentPath, value, m_rbDriveCurrentUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_TOTALPOWER:
        logger->WriteDouble(m_rbDriveTotalPowerPath, value, m_rbDriveTotalPowerUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_WATT_HOURS:
        logger->WriteDouble(m_rbDriveWattHoursPath, value, m_rbDriveWattHoursUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::SWERVE_CHASSIS_TOTAL_CURRENT:
        logger->WriteDouble(m_swerveChassisTotalCurrentPath, value, m_swerveChassisTotalCurrentUnits, timestamp);
        break;

    case DragonDataLogger::DoubleSignals::SWERVE_CHASSIS_WATT_HOURS:
        logger->WriteDouble(m_swerveChassisWattHoursPath, value, m_swerveChassisWattHoursUnits, timestamp);
        break;
    case DragonDataLogger::DoubleSignals::LIMELIGHT1_NUMBER_OF_TAGS:
        logger->WriteDouble(m_limelight1NumberOfTagsPath, value, m_limelight1NumberOfTagsUnits, timestamp);
        break;
    default:
        break;
    }
}

void DragonDataLogger::LogStringData(uint64_t timestamp, DragonDataLogger::StringSignals signalID, std::string value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }

    switch (signalID)
    {
    case DragonDataLogger::StringSignals::CHASSIS_DRIVE_STATE:
        logger->WriteString(m_driveStatePath, value, timestamp);
        break;

    case DragonDataLogger::StringSignals::CHASSIS_HEADING_STATE:
        logger->WriteString(m_headingStatePath, value, timestamp);
        break;

    default:
        break;
    }
}
void DragonDataLogger::Log2DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose2d value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }

    switch (signalID)
    {
    case DragonDataLogger::PoseSingals::CURRENT_CHASSIS_POSE2D:
    {
        double x = value.X().value();
        double y = value.Y().value();
        double rot = value.Rotation().Radians().value();
        std::vector<double> pose = {x, y, rot};
        logger->WriteDoubleArray(m_chassisPose2dPath, pose, m_pose2dUnits, timestamp);
        break;
    }

    case DragonDataLogger::PoseSingals::CURRENT_CHASSIS_QUEST_POSE2D:
    {
        double x = value.X().value();
        double y = value.Y().value();
        double rot = value.Rotation().Radians().value();
        std::vector<double> pose = {x, y, rot};
        logger->WriteDoubleArray(m_questPose2dPath, pose, m_questPose2dUnits, timestamp);
        break;
    }
    default:
        break;
    }
}

void DragonDataLogger::Log3DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose3d value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }

    switch (signalID)
    {
    case DragonDataLogger::PoseSingals::CURRENT_CHASSIS_LIMELIGHT_POSE3D:
    {
        double x = value.ToPose2d().X().value();
        double y = value.ToPose2d().Y().value();
        double rot = value.ToPose2d().Rotation().Radians().value();
        std::vector<double> pose = {x, y, rot};
        logger->WriteDoubleArray(m_limelight1Pose3dPath, pose, m_limelight2Pose3dPath, timestamp);
        break;
    }
    case DragonDataLogger::PoseSingals::CURRENT_CHASSIS_LIMELIGHT2_POSE3D:
    {
        double x = value.ToPose2d().X().value();
        double y = value.ToPose2d().Y().value();
        double rot = value.ToPose2d().Rotation().Radians().value();
        std::vector<double> pose = {x, y, rot};
        logger->WriteDoubleArray(m_limelight2Pose3dPath, pose, m_limelight2Pose3dPath, timestamp);
        break;
    }
    default:
        break;
    }
}

void DragonDataLogger::LogSwerveModuleStateData(uint64_t timestamp, DragonDataLogger::SwerveStateSingals signalID, frc::SwerveModuleState value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }

    switch (signalID)
    {
    case DragonDataLogger::SwerveStateSingals::TARGET_LEFT_FRONT_STATE:
    {
        double speed = value.speed.value();
        double angle = value.angle.Radians().value();
        logger->WriteDouble(m_frontLeftTargetSpeedPath, speed, m_swerveModuleStateUnits, timestamp);
        logger->WriteDouble(m_frontLeftTargetAnglePath, angle, m_swerveModuleStateUnits, timestamp);
        break;
    }

    case DragonDataLogger::SwerveStateSingals::TARGET_LEFT_BACK_STATE:
    {
        double speed = value.speed.value();
        double angle = value.angle.Radians().value();
        logger->WriteDouble(m_backLeftTargetSpeedPath, speed, m_swerveModuleStateUnits, timestamp);
        logger->WriteDouble(m_backLeftTargetAnglePath, angle, m_swerveModuleStateUnits, timestamp);
        break;
    }

    case DragonDataLogger::SwerveStateSingals::TARGET_RIGHT_FRONT_STATE:
    {
        double speed = value.speed.value();
        double angle = value.angle.Radians().value();
        logger->WriteDouble(m_frontRightTargetSpeedPath, speed, m_swerveModuleStateUnits, timestamp);
        logger->WriteDouble(m_frontRightTargetAnglePath, angle, m_swerveModuleStateUnits, timestamp);
        break;
    }

    case DragonDataLogger::SwerveStateSingals::TARGET_RIGHT_BACK_STATE:
    {
        double speed = value.speed.value();
        double angle = value.angle.Radians().value();
        logger->WriteDouble(m_backRightTargetSpeedPath, speed, m_swerveModuleStateUnits, timestamp);
        logger->WriteDouble(m_backRightTargetAnglePath, angle, m_swerveModuleStateUnits, timestamp);
        break;
    }

    case DragonDataLogger::SwerveStateSingals::ACTUAL_LEFT_FRONT_STATE:
    {
        double speed = value.speed.value();
        double angle = value.angle.Radians().value();
        logger->WriteDouble(m_frontLeftActualSpeedPath, speed, m_swerveModuleStateUnits, timestamp);
        logger->WriteDouble(m_frontLeftActualAnglePath, angle, m_swerveModuleStateUnits, timestamp);
        break;
    }

    case DragonDataLogger::SwerveStateSingals::ACTUAL_LEFT_BACK_STATE:
    {
        double speed = value.speed.value();
        double angle = value.angle.Radians().value();
        logger->WriteDouble(m_backLeftActualSpeedPath, speed, m_swerveModuleStateUnits, timestamp);
        logger->WriteDouble(m_backLeftActualAnglePath, angle, m_swerveModuleStateUnits, timestamp);
        break;
    }

    case DragonDataLogger::SwerveStateSingals::ACTUAL_RIGHT_FRONT_STATE:
    {
        double speed = value.speed.value();
        double angle = value.angle.Radians().value();
        logger->WriteDouble(m_frontRightActualSpeedPath, speed, m_swerveModuleStateUnits, timestamp);
        logger->WriteDouble(m_frontRightActualAnglePath, angle, m_swerveModuleStateUnits, timestamp);
        break;
    }

    case DragonDataLogger::SwerveStateSingals::ACTUAL_RIGHT_BACK_STATE:
    {
        double speed = value.speed.value();
        double angle = value.angle.Radians().value();
        logger->WriteDouble(m_backRightActualSpeedPath, speed, m_swerveModuleStateUnits, timestamp);
        logger->WriteDouble(m_backRightActualAnglePath, angle, m_swerveModuleStateUnits, timestamp);
        break;
    }

    default:
        break;
    }
    
}

void DragonDataLogger::LogChassisSpeedsData(uint64_t timestamp, DragonDataLogger::ChassisSpeedSignals signalID, frc::ChassisSpeeds value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }
    // TODO:  need to compare/store; need to do element by element
    switch (signalID)
    {
    case DragonDataLogger::ChassisSpeedSignals::ACTUAL_SPEEDS:
    {
        double vx = value.vx.value();
        double vy = value.vy.value();
        double omega = value.omega.value();
        logger->WriteDouble(m_swerveActualvxPath, vx, m_swerveChassisSpeedUnits, timestamp);
        logger->WriteDouble(m_swerveActualvyPath, vy, m_swerveChassisSpeedUnits, timestamp);
        logger->WriteDouble(m_swerveActualOmegaPath, omega, m_swerveChassisSpeedUnits, timestamp);
        break;
    }
    case DragonDataLogger::ChassisSpeedSignals::TARGET_SPEEDS:
    {
        double vx = value.vx.value();
        double vy = value.vy.value();
        double omega = value.omega.value();
        logger->WriteDouble(m_swerveTargetvxPath, vx, m_swerveChassisSpeedUnits, timestamp);
        logger->WriteDouble(m_swerveTargetvyPath, vy, m_swerveChassisSpeedUnits, timestamp);
        logger->WriteDouble(m_swerveTargetOmegaPath, omega, m_swerveChassisSpeedUnits, timestamp);
        break;
    }
    default:
        break;
    }
}
