
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

#include <string>
#include <string_view>
#include <cstdint>
#include <units/time.h>
#include <vector>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModuleState.h"

class ISignalLogger
{
public:
    virtual ~ISignalLogger() = default;

    virtual void WriteBoolean(std::string signalID, bool value, units::time::second_t latency) = 0;
    virtual void WriteDouble(std::string signalID, double value, std::string_view units, units::time::second_t latency) = 0;
    virtual void WriteInteger(std::string signalID, int64_t value, std::string_view units, units::time::second_t latency) = 0;
    virtual void WriteString(std::string signalID, const std::string &value, units::time::second_t latency) = 0;
    virtual void WriteDoubleArray(std::string signalID, const std::vector<double> &value, std::string_view units, units::time::second_t latency) = 0;

    virtual void WritePose2d(std::string signalID, const frc::Pose2d &value, units::time::second_t latency) = 0;
    virtual void WritePose3d(std::string signalID, const frc::Pose3d &value, units::time::second_t latency) = 0;
    virtual void WriteChassisSpeeds(std::string signalID, const frc::ChassisSpeeds &value, units::time::second_t latency) = 0;
    virtual void WriteSwerveModuleState(std::string signalID, const frc::SwerveModuleState &value, units::time::second_t latency) = 0;

    // Optional: methods for starting/stopping logging
    virtual void Start() = 0;
    virtual void Stop() = 0;
};