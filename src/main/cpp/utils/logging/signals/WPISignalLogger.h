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

#include <string>
#include <string_view>
#include <unordered_map>
#include <memory>

#include "utils/logging/signals/ISignalLogger.h"
#include "frc/DataLogManager.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "wpi/DataLog.h"
#include "units/time.h"

class WPISignalLogger : public ISignalLogger
{
public:
    WPISignalLogger() = default;
    ~WPISignalLogger() override = default;

    void WriteBoolean(std::string signalID, bool value, units::time::second_t latency) override;
    void WriteDouble(std::string signalID, double value, std::string_view units, units::time::second_t latency) override;
    void WriteInteger(std::string signalID, int64_t value, std::string_view units, units::time::second_t latency) override;
    void WriteString(std::string signalID, const std::string &value, units::time::second_t latency) override;
    void WriteDoubleArray(std::string signalID, const std::vector<double> &value, std::string_view units, units::time::second_t latency) override;

    void WritePose2d(std::string signalID, const frc::Pose2d &value, units::time::second_t latency) override;
    void WritePose3d(std::string signalID, const frc::Pose3d &value, units::time::second_t latency) override;
    void WriteChassisSpeeds(std::string signalID, const frc::ChassisSpeeds &value, units::time::second_t latency) override;
    void WriteSwerveModuleState(std::string signalID, const frc::SwerveModuleState &value, units::time::second_t latency) override;

    void Start() override;
    void Stop() override;

private:
    std::string CreateLogFileName();
    std::string GetLoggingDir();

    /// Get or create a BooleanLogEntry for the given signal path
    wpi::log::BooleanLogEntry &GetBooleanEntry(const std::string &signalID);

    /// Get or create a DoubleLogEntry for the given signal path
    wpi::log::DoubleLogEntry &GetDoubleEntry(const std::string &signalID);

    /// Get or create an IntegerLogEntry for the given signal path
    wpi::log::IntegerLogEntry &GetIntegerEntry(const std::string &signalID);

    /// Get or create a StringLogEntry for the given signal path
    wpi::log::StringLogEntry &GetStringEntry(const std::string &signalID);

    /// Get or create a DoubleArrayLogEntry for the given signal path
    wpi::log::DoubleArrayLogEntry &GetDoubleArrayEntry(const std::string &signalID);

    /// Get or create a StructLogEntry<Pose2d> for the given signal path
    wpi::log::StructLogEntry<frc::Pose2d> &GetPose2dEntry(const std::string &signalID);

    /// Get or create a StructLogEntry<Pose3d> for the given signal path
    wpi::log::StructLogEntry<frc::Pose3d> &GetPose3dEntry(const std::string &signalID);

    /// Get or create a StructLogEntry<ChassisSpeeds> for the given signal path
    wpi::log::StructLogEntry<frc::ChassisSpeeds> &GetChassisSpeedsEntry(const std::string &signalID);

    /// Get or create a StructLogEntry<SwerveModuleState> for the given signal path
    wpi::log::StructLogEntry<frc::SwerveModuleState> &GetSwerveModuleStateEntry(const std::string &signalID);

    std::unordered_map<std::string, std::unique_ptr<wpi::log::BooleanLogEntry>> m_boolEntries;
    std::unordered_map<std::string, std::unique_ptr<wpi::log::DoubleLogEntry>> m_doubleEntries;
    std::unordered_map<std::string, std::unique_ptr<wpi::log::IntegerLogEntry>> m_intEntries;
    std::unordered_map<std::string, std::unique_ptr<wpi::log::StringLogEntry>> m_stringEntries;
    std::unordered_map<std::string, std::unique_ptr<wpi::log::DoubleArrayLogEntry>> m_doubleArrayEntries;
    std::unordered_map<std::string, std::unique_ptr<wpi::log::StructLogEntry<frc::Pose2d>>> m_pose2dEntries;
    std::unordered_map<std::string, std::unique_ptr<wpi::log::StructLogEntry<frc::Pose3d>>> m_pose3dEntries;
    std::unordered_map<std::string, std::unique_ptr<wpi::log::StructLogEntry<frc::ChassisSpeeds>>> m_chassisSpeedsEntries;
    std::unordered_map<std::string, std::unique_ptr<wpi::log::StructLogEntry<frc::SwerveModuleState>>> m_swerveModuleStateEntries;
};
