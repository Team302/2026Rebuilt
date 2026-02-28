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
#include <filesystem>
#include <ctime>
#include <string>

using ctre::phoenix6::SignalLogger;

void CTRESignalLogger::WriteBoolean(std::string_view signalID, bool value, uint64_t timestamp)
{
    SignalLogger::WriteBoolean(signalID, value, units::time::microsecond_t(timestamp));
}

void CTRESignalLogger::WriteDouble(std::string_view signalID, double value, std::string_view units, uint64_t timestamp)
{
    SignalLogger::WriteDouble(signalID, value, units, units::time::microsecond_t(timestamp));
}

void CTRESignalLogger::WriteInteger(std::string_view signalID, int64_t value, std::string_view units, uint64_t timestamp)
{
    SignalLogger::WriteInteger(signalID, value, units, units::time::microsecond_t(timestamp));
}

void CTRESignalLogger::WriteString(std::string_view signalID, const std::string &value, uint64_t timestamp)
{
    SignalLogger::WriteString(signalID, value, units::time::microsecond_t(timestamp));
}

void CTRESignalLogger::WriteDoubleArray(std::string_view signalID, const std::vector<double> &value, std::string_view units, uint64_t timestamp)
{
    SignalLogger::WriteDoubleArray(signalID, value, units, units::time::microsecond_t(timestamp));
}

void CTRESignalLogger::WritePose2d(std::string_view signalID, const frc::Pose2d &value, uint64_t timestamp)
{
    std::vector<double> data = {value.X().value(), value.Y().value(), value.Rotation().Radians().value()};
    SignalLogger::WriteDoubleArray(signalID, data, kUnitsPose2d, units::time::microsecond_t(timestamp));
}

void CTRESignalLogger::WritePose3d(std::string_view signalID, const frc::Pose3d &value, uint64_t timestamp)
{
    std::vector<double> data = {value.X().value(), value.Y().value(), value.Z().value(),
                                value.Rotation().GetQuaternion().W(),
                                value.Rotation().GetQuaternion().X(),
                                value.Rotation().GetQuaternion().Y(),
                                value.Rotation().GetQuaternion().Z()};
    SignalLogger::WriteDoubleArray(signalID, data, kUnitsPose3d, units::time::microsecond_t(timestamp));
}

void CTRESignalLogger::WriteChassisSpeeds(std::string_view signalID, const frc::ChassisSpeeds &value, uint64_t timestamp)
{
    std::vector<double> data = {value.vx.value(), value.vy.value(), value.omega.value()};
    SignalLogger::WriteDoubleArray(signalID, data, kUnitsChassisSpeeds, units::time::microsecond_t(timestamp));
}

void CTRESignalLogger::WriteSwerveModuleState(std::string_view signalID, const frc::SwerveModuleState &value, uint64_t timestamp)
{
    std::vector<double> data = {value.speed.value(), value.angle.Radians().value()};
    SignalLogger::WriteDoubleArray(signalID, data, kUnitsSwerveState, units::time::microsecond_t(timestamp));
}

void CTRESignalLogger::WriteGamePadState(std::string_view signalID, const std::array<double, 6> axes, const std::array<bool, 10> buttons, const std::array<int, 1> povs, uint64_t timestamp)
{
    const std::string id(signalID);
    // Log axes
    {
        std::vector<double> axesVec(axes.begin(), axes.end());
        SignalLogger::WriteDoubleArray(id + std::string(kSubpathAxes), axesVec, "", units::time::microsecond_t(timestamp));
    }

    // Log buttons as doubles (0.0 or 1.0)
    {
        std::vector<double> buttonsVec;
        buttonsVec.reserve(buttons.size());
        for (bool b : buttons)
            buttonsVec.push_back(b ? 1.0 : 0.0);
        SignalLogger::WriteDoubleArray(id + std::string(kSubpathButtons), buttonsVec, "", units::time::microsecond_t(timestamp));
    }

    // Log POVs
    {
        std::vector<double> povsVec;
        povsVec.reserve(povs.size());
        for (int p : povs)
            povsVec.push_back(static_cast<double>(p));
        SignalLogger::WriteDoubleArray(id + std::string(kSubpathPovs), povsVec, "", units::time::microsecond_t(timestamp));
    }
}

void CTRESignalLogger::Start()
{
    SignalLogger::SetPath(DragonDataLoggerMgr::GetInstance()->GetLoggingDirectory().c_str());
    SignalLogger::EnableAutoLogging(true);
    SignalLogger::Start();
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

    std::string filename = "frc302-" + time + ".wpilog";
    return filename;
}

void CTRESignalLogger::SetAutoLogging(bool enable)
{
    SignalLogger::EnableAutoLogging(enable);
}