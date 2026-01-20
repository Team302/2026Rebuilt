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
    void WriteDouble(std::string signalI, double value, std::string_view units, units::time::second_t latency) override;
    void WriteInteger(std::string signalI, int64_t value, std::string_view units, units::time::second_t latency) override;
    void WriteString(std::string signalI, const std::string &value, units::time::second_t latency) override;

    void Start() override;
    void Stop() override;
};