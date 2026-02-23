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
#include <array>
#include <string>
#include <string_view>
#include <vector>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "units/time.h"
#include "frc/kinematics/SwerveModuleState.h"

class DragonDataLogger
{
public:
    DragonDataLogger();
    virtual ~DragonDataLogger() = default;

    virtual void DataLog(uint64_t timestamp) = 0;

    units::time::second_t m_latency = units::time::second_t(0);

protected:
    void LogBoolData(uint64_t timestamp, const std::string &path, bool value);
    void LogIntData(uint64_t timestamp, const std::string &path, int value, std::string units = "");
    void LogDoubleData(uint64_t timestamp, const std::string &path, double value, std::string_view units = "");
    void LogStringData(uint64_t timestamp, const std::string &path, const std::string &value);
    void LogDoubleArrayData(uint64_t timestamp, const std::string &path, const std::vector<double> &value, std::string_view units = "");
    void LogSwerveModuleStateData(uint64_t timestamp, const std::string &speedPath, const std::string &anglePath, frc::SwerveModuleState value, std::string_view units = "");
    void LogChassisSpeedsData(uint64_t timestamp, const std::string &vxPath, const std::string &vyPath, const std::string &omegaPath, frc::ChassisSpeeds value, std::string_view units = "");
    void LogGamePadData(uint64_t timestamp, const std::string &path, const std::array<double, 6> axes, const std::array<bool, 10> buttons, const std::array<int, 1> povs);
    void LogPose2dData(uint64_t timestamp, const std::string &path, const frc::Pose2d &value);
    void LogPose3dData(uint64_t timestamp, const std::string &path, const frc::Pose3d &value);

    const double m_doubleTolerance = 0.001;
};
