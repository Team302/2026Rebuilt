#include "utils/logging/signals/CTRESignalLogger.h"

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
    SignalLogger::Start();
}

void CTRESignalLogger::Stop()
{
    SignalLogger::Stop();
}