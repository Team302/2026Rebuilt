
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
#include <filesystem>
#include <ctime>
#include <string>

using ctre::phoenix6::SignalLogger;

void CTRESignalLogger::WriteBoolean(std::string signalID, bool value, units::time::millisecond_t timestamp)
{
    SignalLogger::WriteBoolean(signalID, value, timestamp);
}

void CTRESignalLogger::WriteDouble(std::string signalID, double value, std::string_view units, units::time::millisecond_t timestamp)
{
    SignalLogger::WriteDouble(signalID, value, units, timestamp);
}

void CTRESignalLogger::WriteInteger(std::string signalID, int64_t value, std::string_view units, units::time::millisecond_t timestamp)
{
    SignalLogger::WriteInteger(signalID, value, units, timestamp);
}

void CTRESignalLogger::WriteString(std::string signalID, const std::string &value, units::time::millisecond_t timestamp)
{
    SignalLogger::WriteString(signalID, value, timestamp);
}

void CTRESignalLogger::WriteDoubleArray(std::string signalID, const std::vector<double> &value, std::string_view units, units::time::millisecond_t timestamp)
{
    SignalLogger::WriteDoubleArray(signalID, value, units, timestamp);
}

void CTRESignalLogger::Start()
{
    SignalLogger::SetPath(GetLoggingDir().c_str());
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

std::string CTRESignalLogger::GetLoggingDir()
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