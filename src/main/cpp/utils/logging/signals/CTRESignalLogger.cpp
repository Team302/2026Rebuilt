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

#include "utils/logging/signals/CTRESignalLogger.h"
#include "utils/logging/debug/Logger.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"
#include <ctime>
#include <filesystem>
#include <iostream>
#include <string>
#include <span>

using ctre::phoenix6::SignalLogger;

void CTRESignalLogger::WriteBoolean(std::string_view signalID, bool value, uint64_t timestamp)
{
    SignalLogger::WriteBoolean(signalID, value, 0_s);
}

void CTRESignalLogger::WriteDouble(std::string_view signalID, double value, std::string_view units, uint64_t timestamp)
{
    SignalLogger::WriteDouble(signalID, value, units, 0_s);
}

void CTRESignalLogger::WriteInteger(std::string_view signalID, int64_t value, std::string_view units, uint64_t timestamp)
{
    SignalLogger::WriteInteger(signalID, value, units, 0_s);
}

void CTRESignalLogger::WriteString(std::string_view signalID, const std::string &value, uint64_t timestamp)
{
    SignalLogger::WriteString(signalID, value, 0_s);
}

void CTRESignalLogger::WriteDoubleArray(std::string_view signalID, const std::vector<double> &value, std::string_view units, uint64_t timestamp)
{
    SignalLogger::WriteDoubleArray(signalID, value, units, 0_s);
}

void CTRESignalLogger::WritePose2d(std::string_view signalID, const frc::Pose2d &value, uint64_t timestamp)
{
    SignalLogger::WriteStruct<frc::Pose2d>(signalID, value, 0_s);
}

void CTRESignalLogger::WritePose3d(std::string_view signalID, const frc::Pose3d &value, uint64_t timestamp)
{
    SignalLogger::WriteStruct<frc::Pose3d>(signalID, value, 0_s);
}

void CTRESignalLogger::WriteChassisSpeeds(std::string_view signalID, const frc::ChassisSpeeds &value, uint64_t timestamp)
{
    SignalLogger::WriteStruct<frc::ChassisSpeeds>(signalID, value, 0_s);
}

void CTRESignalLogger::WriteSwerveModuleState(std::string_view signalID, const std::array<frc::SwerveModuleState, 4> &value, uint64_t timestamp)
{
    SignalLogger::WriteStructArray<frc::SwerveModuleState>(signalID, value, 0_s);
}

void CTRESignalLogger::WriteGamePadState(std::string_view signalID, const std::array<double, 6> axes, const std::array<bool, 10> buttons, const std::array<int, 1> povs, uint64_t timestamp)
{
    const std::string id(signalID);
    // Log axes
    {
        std::vector<double> axesVec(axes.begin(), axes.end());
        SignalLogger::WriteDoubleArray(id + std::string(kSubpathAxes), axesVec, "", 0_s);
    }

    SignalLogger::WriteBooleanArray(id + std::string(kSubpathButtons), std::span(buttons), 0_s);

    // Log POVs
    {
        std::vector<double> povsVec;
        povsVec.reserve(povs.size());
        for (int p : povs)
            povsVec.push_back(static_cast<double>(p));
        SignalLogger::WriteDoubleArray(id + std::string(kSubpathPovs), povsVec, "", 0_s);
    }
}

void CTRESignalLogger::Start()
{
    SignalLogger::SetPath(DragonDataLoggerMgr::GetInstance()->GetLoggingDirectory().c_str());
    SignalLogger::Start();
    std::cout << "CTRE Signal Logger started with log file: " << CreateLogFileName() << std::endl;
}

void CTRESignalLogger::Stop()
{
    SignalLogger::Stop();
}

/**
 * @brief Create a log file name based on the current date and time
 */
std::string CTRESignalLogger::CreateLogFileName()
{
    time_t now = time(0);
    tm *ltm = localtime(&now);
    char buffer[80];
    strftime(buffer, 80, "%Y%m%d-%H%M%S", ltm);
    std::string time(buffer);

    std::string filename = "frc302-" + time + ".hoot";
    return filename;
}

void CTRESignalLogger::SetAutoLogging(bool enable)
{
    SignalLogger::EnableAutoLogging(enable);
}