
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
    void WriteDoubleArray(std::string signalID, const std::vector<double> &value, std::string_view units, units::time::second_t latency) override;

    void Start() override;
    void Stop() override;
    void DisableAutoLogging();
    std::string CreateLogFileName();
    std::string GetLoggingDir();
};