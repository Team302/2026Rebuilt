#pragma once

#include "ctre/phoenix6/SignalLogger.hpp"
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"

#include "chassis/ChassisConfigMgr.h"
class Telemetry : public DragonDataLogger
{
private:
    units::meters_per_second_t MaxSpeed = 0_mps; // Maximum speed of the robot, set by chassis configuration manager

    /* Mechanisms to represent the swerve module states */
    std::array<frc::Mechanism2d, 4> m_moduleMechanisms{
        frc::Mechanism2d{1, 1},
        frc::Mechanism2d{1, 1},
        frc::Mechanism2d{1, 1},
        frc::Mechanism2d{1, 1},
    };
    /* A direction and length changing ligament for speed representation */
    std::array<frc::MechanismLigament2d *, 4> m_moduleSpeeds{
        m_moduleMechanisms[0].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_deg),
        m_moduleMechanisms[1].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_deg),
        m_moduleMechanisms[2].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_deg),
        m_moduleMechanisms[3].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_deg),
    };
    /* A direction changing and length constant ligament for module direction */
    std::array<frc::MechanismLigament2d *, 4> m_moduleDirections{
        m_moduleMechanisms[0].GetRoot("RootDirection", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, frc::Color8Bit{frc::Color::kWhite}),
        m_moduleMechanisms[1].GetRoot("RootDirection", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, frc::Color8Bit{frc::Color::kWhite}),
        m_moduleMechanisms[2].GetRoot("RootDirection", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, frc::Color8Bit{frc::Color::kWhite}),
        m_moduleMechanisms[3].GetRoot("RootDirection", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, frc::Color8Bit{frc::Color::kWhite}),
    };

    /* Cached state data for periodic logging */
    frc::Pose2d m_cachedPose;
    frc::ChassisSpeeds m_cachedSpeeds;
    std::array<frc::SwerveModuleState, 4> m_cachedModuleStates;
    std::array<frc::SwerveModuleState, 4> m_cachedModuleTargets;
    std::array<frc::SwerveModulePosition, 4> m_cachedModulePositions;
    units::second_t m_cachedOdometryPeriod;

public:
    /**
     * Construct a telemetry object with the specified max speed of the robot.
     *
     * \param maxSpeed Maximum speed
     */
    Telemetry();

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    void Telemeterize(subsystems::CommandSwerveDrivetrain::SwerveDriveState const &state);

    /** Periodic data logging implementation */
    void DataLog(uint64_t timestamp) override;
};
