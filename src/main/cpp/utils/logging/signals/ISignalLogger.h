#pragma once

#include <string>
#include <cstdint>
#include <units/time.h>

class ISignalLogger
{
public:
    virtual ~ISignalLogger() = default;

    virtual void WriteBoolean(std::string signalID, bool value, units::time::second_t timestamp) = 0;
    virtual void WriteDouble(std::string signalID, double value, std::string_view units, units::time::second_t timestamp) = 0;
    virtual void WriteInteger(std::string signalID, int64_t value, std::string_view units, units::time::second_t timestamp) = 0;
    virtual void WriteString(std::string signalID, const std::string &value, units::time::second_t timestamp) = 0;

    // Optional: methods for starting/stopping logging
    virtual void Start() = 0;
    virtual void Stop() = 0;
};