#include "utils/logging/signals/CTRESignalLogger.h"
#include <filesystem>
#include <ctime>
#include <string>

using ctre::phoenix6::SignalLogger;

void CTRESignalLogger::WriteBoolean(std::string signalID, bool value, units::time::second_t latency)
{
    SignalLogger::WriteBoolean(signalID, value, latency);
}

void CTRESignalLogger::WriteDouble(std::string signalID, double value, std::string_view units, units::time::second_t latency)
{
    SignalLogger::WriteDouble(signalID, value, units, latency);
}

void CTRESignalLogger::WriteInteger(std::string signalID, int64_t value, std::string_view units, units::time::second_t latency)
{
    SignalLogger::WriteInteger(signalID, value, units, latency);
}

void CTRESignalLogger::WriteString(std::string signalID, const std::string &value, units::time::second_t latency)
{
    SignalLogger::WriteString(signalID, value, latency);
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