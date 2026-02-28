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

#include "utils/logging/signals/DragonDataLogger.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"
#include "utils/logging/signals/ISignalLogger.h"

DragonDataLogger::DragonDataLogger() : m_dataMgr(DragonDataLoggerMgr::GetInstance())
{
    m_dataMgr->RegisterItem(this);
}

ISignalLogger *DragonDataLogger::GetLogger() const
{
    return m_dataMgr != nullptr ? m_dataMgr->GetLogger() : nullptr;
}

void DragonDataLogger::LogBoolData(uint64_t timestamp, const std::string_view &path, bool value)
{
    auto logger = GetLogger();
    if (logger != nullptr)
    {
        logger->WriteBoolean(path, value, timestamp);
    }
}

void DragonDataLogger::LogIntData(uint64_t timestamp, const std::string_view &path, int value, std::string_view units)
{
    auto logger = GetLogger();
    if (logger != nullptr)
    {
        logger->WriteInteger(path, value, units, timestamp);
    }
}

void DragonDataLogger::LogDoubleData(uint64_t timestamp, const std::string_view &path, double value, std::string_view units)
{
    auto logger = GetLogger();
    if (logger != nullptr)
    {
        logger->WriteDouble(path, value, units, timestamp);
    }
}

void DragonDataLogger::LogStringData(uint64_t timestamp, const std::string_view &path, const std::string &value)
{
    auto logger = GetLogger();
    if (logger != nullptr)
    {
        logger->WriteString(path, value, timestamp);
    }
}

void DragonDataLogger::LogDoubleArrayData(uint64_t timestamp, const std::string_view &path, const std::vector<double> &value, std::string_view units)
{
    auto logger = GetLogger();
    if (logger != nullptr)
    {
        logger->WriteDoubleArray(path, value, units, timestamp);
    }
}

void DragonDataLogger::LogSwerveModuleStateData(uint64_t timestamp, const std::string_view &speedPath, const std::string_view &anglePath, const frc::SwerveModuleState &value, std::string_view units)
{
    auto logger = GetLogger();
    if (logger != nullptr)
    {
        logger->WriteDouble(speedPath, value.speed.value(), units, timestamp);
        logger->WriteDouble(anglePath, value.angle.Radians().value(), units, timestamp);
    }
}

void DragonDataLogger::LogChassisSpeedsData(uint64_t timestamp, const std::string_view &vxPath, const std::string_view &vyPath, const std::string_view &omegaPath, const frc::ChassisSpeeds &value, std::string_view units)
{
    auto logger = GetLogger();
    if (logger != nullptr)
    {
        logger->WriteDouble(vxPath, value.vx.value(), units, timestamp);
        logger->WriteDouble(vyPath, value.vy.value(), units, timestamp);
        logger->WriteDouble(omegaPath, value.omega.value(), units, timestamp);
    }
}

void DragonDataLogger::LogGamePadData(uint64_t timestamp, const std::string_view &path, const std::array<double, 6> &axes, const std::array<bool, 10> &buttons, const std::array<int, 1> &povs)
{
    auto logger = GetLogger();
    if (logger != nullptr)
    {
        logger->WriteGamePadState(path, axes, buttons, povs, timestamp);
    }
}

void DragonDataLogger::LogPose2dData(uint64_t timestamp, const std::string_view &path, const frc::Pose2d &value)
{
    auto logger = GetLogger();
    if (logger != nullptr)
    {
        logger->WritePose2d(path, value, timestamp);
    }
}

void DragonDataLogger::LogPose3dData(uint64_t timestamp, const std::string_view &path, const frc::Pose3d &value)
{
    auto logger = GetLogger();
    if (logger != nullptr)
    {
        logger->WritePose3d(path, value, timestamp);
    }
}