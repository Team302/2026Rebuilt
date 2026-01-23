#pragma once

#include "utils/logging/signals/ISignalLogger.h"
#include "ctre/phoenix6/SignalLogger.hpp"
#include <units/time.h>

class CTRESignalLogger : public ISignalLogger
{
public:
    CTRESignalLogger() = default;
    ~CTRESignalLogger() override = default;

    void WriteBoolean(std::string signalID, bool value, units::time::second_t latency) override;
    void WriteDouble(std::string signalID, double value, std::string_view units, units::time::second_t latency) override;
    void WriteInteger(std::string signalID, int64_t value, std::string_view units, units::time::second_t latency) override;
    void WriteString(std::string signalID, const std::string &value, units::time::second_t latency) override;

    void Start() override;
    void Stop() override;
    std::string CreateLogFileName();
    std::string GetLoggingDir();
};