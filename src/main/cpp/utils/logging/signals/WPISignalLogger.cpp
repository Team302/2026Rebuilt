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

#include "utils/logging/signals/WPISignalLogger.h"
#include "frc/DataLogManager.h"
#include "frc/RobotController.h"
#include "wpi/DataLog.h"
#include <filesystem>

void WPISignalLogger::WriteBoolean(std::string signalID, bool value, units::time::second_t latency)
{
    auto &entry = GetBooleanEntry(signalID);
    int64_t timestamp = frc::RobotController::GetFPGATime();
    entry.Append(value, timestamp);
}

void WPISignalLogger::WriteDouble(std::string signalID, double value, std::string_view units, units::time::second_t latency)
{
    auto &entry = GetDoubleEntry(signalID);
    int64_t timestamp = frc::RobotController::GetFPGATime();
    entry.Append(value, timestamp);
}

void WPISignalLogger::WriteInteger(std::string signalID, int64_t value, std::string_view units, units::time::second_t latency)
{
    auto &entry = GetIntegerEntry(signalID);
    int64_t timestamp = frc::RobotController::GetFPGATime();
    entry.Append(value, timestamp);
}

void WPISignalLogger::WriteString(std::string signalID, const std::string &value, units::time::second_t latency)
{
    auto &entry = GetStringEntry(signalID);
    int64_t timestamp = frc::RobotController::GetFPGATime();
    entry.Append(value, timestamp);
}

void WPISignalLogger::WriteDoubleArray(std::string signalID, const std::vector<double> &value, std::string_view units, units::time::second_t latency)
{
    auto &entry = GetDoubleArrayEntry(signalID);
    int64_t timestamp = frc::RobotController::GetFPGATime();
    entry.Append(value, timestamp);
}

void WPISignalLogger::WritePose2d(std::string signalID, const frc::Pose2d &value, units::time::second_t latency)
{
    auto &entry = GetPose2dEntry(signalID);
    int64_t timestamp = frc::RobotController::GetFPGATime();
    entry.Append(value, timestamp);
}

void WPISignalLogger::WritePose3d(std::string signalID, const frc::Pose3d &value, units::time::second_t latency)
{
    auto &entry = GetPose3dEntry(signalID);
    int64_t timestamp = frc::RobotController::GetFPGATime();
    entry.Append(value, timestamp);
}

void WPISignalLogger::WriteChassisSpeeds(std::string signalID, const frc::ChassisSpeeds &value, units::time::second_t latency)
{
    auto &entry = GetChassisSpeedsEntry(signalID);
    int64_t timestamp = frc::RobotController::GetFPGATime();
    entry.Append(value, timestamp);
}

void WPISignalLogger::WriteSwerveModuleState(std::string signalID, const frc::SwerveModuleState &value, units::time::second_t latency)
{
    auto &entry = GetSwerveModuleStateEntry(signalID);
    int64_t timestamp = frc::RobotController::GetFPGATime();
    entry.Append(value, timestamp);
}

void WPISignalLogger::Start()
{
    frc::DataLogManager::Start(GetLoggingDir(), CreateLogFileName(), 0.25);
}

void WPISignalLogger::Stop()
{
    frc::DataLogManager::Stop();
}

/**
 * @brief Create a log file name based on the current date and time
 */
std::string WPISignalLogger::CreateLogFileName()
{
    time_t now = time(0);
    tm *ltm = localtime(&now);
    char buffer[80];
    strftime(buffer, 80, "%Y%m%d-%H%M%S", ltm);
    std::string time(buffer);

    std::string filename = "frc302-" + time + ".wpilog";
    return filename;
}

std::string WPISignalLogger::GetLoggingDir()
{
    // check if usb log directory exists
    if (std::filesystem::exists("/media/sda1/logs/"))
    {
        return std::filesystem::path("/media/sda1/logs/").string();
    }
    else if (std::filesystem::exists("/home/lvuser/logs/"))
    {
        return std::filesystem::path("/home/lvuser/logs/").string();
    }
    else if (std::filesystem::exists("/home/systemcore/logs/"))
    {
        return std::filesystem::path("/home/systemcore/logs/").string();
    }

    return std::string("");
}

wpi::log::BooleanLogEntry &WPISignalLogger::GetBooleanEntry(const std::string &signalID)
{
    auto it = m_boolEntries.find(signalID);
    if (it == m_boolEntries.end())
    {
        auto &log = frc::DataLogManager::GetLog();
        auto entry = std::make_unique<wpi::log::BooleanLogEntry>(log, signalID);
        auto [inserted, success] = m_boolEntries.emplace(signalID, std::move(entry));
        return *inserted->second;
    }
    return *it->second;
}

wpi::log::DoubleLogEntry &WPISignalLogger::GetDoubleEntry(const std::string &signalID)
{
    auto it = m_doubleEntries.find(signalID);
    if (it == m_doubleEntries.end())
    {
        auto &log = frc::DataLogManager::GetLog();
        auto entry = std::make_unique<wpi::log::DoubleLogEntry>(log, signalID);
        auto [inserted, success] = m_doubleEntries.emplace(signalID, std::move(entry));
        return *inserted->second;
    }
    return *it->second;
}

wpi::log::IntegerLogEntry &WPISignalLogger::GetIntegerEntry(const std::string &signalID)
{
    auto it = m_intEntries.find(signalID);
    if (it == m_intEntries.end())
    {
        auto &log = frc::DataLogManager::GetLog();
        auto entry = std::make_unique<wpi::log::IntegerLogEntry>(log, signalID);
        auto [inserted, success] = m_intEntries.emplace(signalID, std::move(entry));
        return *inserted->second;
    }
    return *it->second;
}

wpi::log::StringLogEntry &WPISignalLogger::GetStringEntry(const std::string &signalID)
{
    auto it = m_stringEntries.find(signalID);
    if (it == m_stringEntries.end())
    {
        auto &log = frc::DataLogManager::GetLog();
        auto entry = std::make_unique<wpi::log::StringLogEntry>(log, signalID);
        auto [inserted, success] = m_stringEntries.emplace(signalID, std::move(entry));
        return *inserted->second;
    }
    return *it->second;
}

wpi::log::DoubleArrayLogEntry &WPISignalLogger::GetDoubleArrayEntry(const std::string &signalID)
{
    auto it = m_doubleArrayEntries.find(signalID);
    if (it == m_doubleArrayEntries.end())
    {
        auto &log = frc::DataLogManager::GetLog();
        auto entry = std::make_unique<wpi::log::DoubleArrayLogEntry>(log, signalID);
        auto [inserted, success] = m_doubleArrayEntries.emplace(signalID, std::move(entry));
        return *inserted->second;
    }
    return *it->second;
}

wpi::log::StructLogEntry<frc::Pose2d> &WPISignalLogger::GetPose2dEntry(const std::string &signalID)
{
    auto it = m_pose2dEntries.find(signalID);
    if (it == m_pose2dEntries.end())
    {
        auto &log = frc::DataLogManager::GetLog();
        auto entry = std::make_unique<wpi::log::StructLogEntry<frc::Pose2d>>(log, signalID);
        auto [inserted, success] = m_pose2dEntries.emplace(signalID, std::move(entry));
        return *inserted->second;
    }
    return *it->second;
}

wpi::log::StructLogEntry<frc::Pose3d> &WPISignalLogger::GetPose3dEntry(const std::string &signalID)
{
    auto it = m_pose3dEntries.find(signalID);
    if (it == m_pose3dEntries.end())
    {
        auto &log = frc::DataLogManager::GetLog();
        auto entry = std::make_unique<wpi::log::StructLogEntry<frc::Pose3d>>(log, signalID);
        auto [inserted, success] = m_pose3dEntries.emplace(signalID, std::move(entry));
        return *inserted->second;
    }
    return *it->second;
}

wpi::log::StructLogEntry<frc::ChassisSpeeds> &WPISignalLogger::GetChassisSpeedsEntry(const std::string &signalID)
{
    auto it = m_chassisSpeedsEntries.find(signalID);
    if (it == m_chassisSpeedsEntries.end())
    {
        auto &log = frc::DataLogManager::GetLog();
        auto entry = std::make_unique<wpi::log::StructLogEntry<frc::ChassisSpeeds>>(log, signalID);
        auto [inserted, success] = m_chassisSpeedsEntries.emplace(signalID, std::move(entry));
        return *inserted->second;
    }
    return *it->second;
}

wpi::log::StructLogEntry<frc::SwerveModuleState> &WPISignalLogger::GetSwerveModuleStateEntry(const std::string &signalID)
{
    auto it = m_swerveModuleStateEntries.find(signalID);
    if (it == m_swerveModuleStateEntries.end())
    {
        auto &log = frc::DataLogManager::GetLog();
        auto entry = std::make_unique<wpi::log::StructLogEntry<frc::SwerveModuleState>>(log, signalID);
        auto [inserted, success] = m_swerveModuleStateEntries.emplace(signalID, std::move(entry));
        return *inserted->second;
    }
    return *it->second;
}
