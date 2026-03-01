
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
    // Shared unit string constants for all logger implementations
    static constexpr std::string_view kUnitsPose2d = "X_m;Y_m;Rot_rad";
    static constexpr std::string_view kUnitsPose3d = "X_m;Y_m;Z_m;QW;QX;QY;QZ";
    static constexpr std::string_view kUnitsChassisSpeeds = "Vx_mps;Vy_mps;Omega_radps";
    static constexpr std::string_view kUnitsSwerveState = "Speed_mps;Angle_rad";
    static constexpr std::string_view kSubpathAxes = "/axes";
    static constexpr std::string_view kSubpathButtons = "/buttons";
    static constexpr std::string_view kSubpathPovs = "/povs";

    virtual ~ISignalLogger() = default;

    virtual void WriteBoolean(std::string_view signalID, bool value, uint64_t timestamp) = 0;
    virtual void WriteDouble(std::string_view signalID, double value, std::string_view units, uint64_t timestamp) = 0;
    virtual void WriteInteger(std::string_view signalID, int64_t value, std::string_view units, uint64_t timestamp) = 0;
    virtual void WriteString(std::string_view signalID, const std::string &value, uint64_t timestamp) = 0;
    virtual void WriteDoubleArray(std::string_view signalID, const std::vector<double> &value, std::string_view units, uint64_t timestamp) = 0;

    virtual void WritePose2d(std::string_view signalID, const frc::Pose2d &value, uint64_t timestamp) = 0;
    virtual void WritePose3d(std::string_view signalID, const frc::Pose3d &value, uint64_t timestamp) = 0;
    virtual void WriteChassisSpeeds(std::string_view signalID, const frc::ChassisSpeeds &value, uint64_t timestamp) = 0;
    virtual void WriteSwerveModuleState(std::string_view signalID, const std::array<frc::SwerveModuleState, 4> &value, uint64_t timestamp) = 0;
    virtual void WriteGamePadState(std::string_view signalID, const std::array<double, 6>, const std::array<bool, 10>, const std::array<int, 1>, uint64_t timestamp) = 0;
    virtual void Start() = 0;
    virtual void Stop() = 0;
};