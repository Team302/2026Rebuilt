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
#include "frc/DriverStation.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"
#include "wpi/DataLog.h"
#include <filesystem>
#include <span>

void WPISignalLogger::WriteBoolean(std::string signalID, bool value, uint64_t timestamp)
{
    auto &entry = GetBooleanEntry(signalID);
    entry.Append(value, timestamp);
}

void WPISignalLogger::WriteDouble(std::string signalID, double value, std::string_view units, uint64_t timestamp)
{
    auto &entry = GetDoubleEntry(signalID + std::string(units));
    entry.Append(value, timestamp);
}

void WPISignalLogger::WriteInteger(std::string signalID, int64_t value, std::string_view units, uint64_t timestamp)
{
    auto &entry = GetIntegerEntry(signalID + std::string(units));
    entry.Append(value, timestamp);
}

void WPISignalLogger::WriteString(std::string signalID, const std::string &value, uint64_t timestamp)
{
    auto &entry = GetStringEntry(signalID);
    entry.Append(value, timestamp);
}

void WPISignalLogger::WriteDoubleArray(std::string signalID, const std::vector<double> &value, std::string_view units, uint64_t timestamp)
{
    auto &entry = GetDoubleArrayEntry(signalID + std::string(units));
    entry.Append(value, timestamp);
}

void WPISignalLogger::WritePose2d(std::string signalID, const frc::Pose2d &value, uint64_t timestamp)
{
    auto &entry = GetPose2dEntry(signalID);
    entry.Append(value, timestamp);
}

void WPISignalLogger::WritePose3d(std::string signalID, const frc::Pose3d &value, uint64_t timestamp)
{
    auto &entry = GetPose3dEntry(signalID);
    entry.Append(value, timestamp);
}

void WPISignalLogger::WriteChassisSpeeds(std::string signalID, const frc::ChassisSpeeds &value, uint64_t timestamp)
{
    auto &entry = GetChassisSpeedsEntry(signalID);
    entry.Append(value, timestamp);
}

void WPISignalLogger::WriteSwerveModuleState(std::string signalID, const frc::SwerveModuleState &value, uint64_t timestamp)
{
    auto &entry = GetSwerveModuleStateEntry(signalID);
    entry.Append(value, timestamp);
}

void WPISignalLogger::WriteGamePadState(std::string signalID, const std::array<double, 6> axes, const std::array<bool, 10> buttons, const std::array<int, 1> povs, uint64_t timestamp)
{
    // Log axes as float span (matching DriverStation format)
    {
        auto &entry = GetFloatArrayEntry(signalID + "/axes");
        std::array<float, 6> axesFloat;
        // Determine actual axis count (trim trailing zeros could be done, but log all 6 for safety)
        for (size_t i = 0; i < axes.size(); ++i)
        {
            axesFloat[i] = static_cast<float>(axes[i]);
        }
        entry.Append(std::span<const float>{axesFloat.data(), axesFloat.size()}, timestamp);
    }

    // Log buttons as int span (BooleanArrayLogEntry::Append requires span<const int>)
    {
        auto &entry = GetBoolArrayEntry(signalID + "/buttons");
        int buttonsArr[10];
        for (size_t i = 0; i < buttons.size(); ++i)
        {
            buttonsArr[i] = buttons[i] ? 1 : 0;
        }
        entry.Append(std::span<const int>{buttonsArr, buttons.size()}, timestamp);
    }

    // Log POVs as int64_t span (matching DriverStation format)
    {
        auto &entry = GetIntegerArrayEntry(signalID + "/povs");
        std::array<int64_t, 1> povs64 = {static_cast<int64_t>(povs[0])};
        entry.Append(std::span<const int64_t>{povs64.data(), povs64.size()}, timestamp);
    }
}

void WPISignalLogger::Start()
{
    frc::DataLogManager::Start(DragonDataLoggerMgr::GetInstance()->GetLoggingDirectory(), CreateLogFileName(), .5);
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

wpi::log::BooleanArrayLogEntry &WPISignalLogger::GetBoolArrayEntry(const std::string &signalID)
{
    auto it = m_boolArrayEntries.find(signalID);
    if (it == m_boolArrayEntries.end())
    {
        auto &log = frc::DataLogManager::GetLog();
        auto entry = std::make_unique<wpi::log::BooleanArrayLogEntry>(log, signalID);
        auto [inserted, success] = m_boolArrayEntries.emplace(signalID, std::move(entry));
        return *inserted->second;
    }
    return *it->second;
}

wpi::log::IntegerArrayLogEntry &WPISignalLogger::GetIntegerArrayEntry(const std::string &signalID)
{
    auto it = m_intArrayEntries.find(signalID);
    if (it == m_intArrayEntries.end())
    {
        auto &log = frc::DataLogManager::GetLog();
        auto entry = std::make_unique<wpi::log::IntegerArrayLogEntry>(log, signalID);
        auto [inserted, success] = m_intArrayEntries.emplace(signalID, std::move(entry));
        return *inserted->second;
    }
    return *it->second;
}

wpi::log::FloatArrayLogEntry &WPISignalLogger::GetFloatArrayEntry(const std::string &signalID)
{
    auto it = m_floatArrayEntries.find(signalID);
    if (it == m_floatArrayEntries.end())
    {
        auto &log = frc::DataLogManager::GetLog();
        auto entry = std::make_unique<wpi::log::FloatArrayLogEntry>(log, signalID);
        auto [inserted, success] = m_floatArrayEntries.emplace(signalID, std::move(entry));
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
