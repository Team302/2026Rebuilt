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

DragonDataLogger::DragonDataLogger()
{
    DragonDataLoggerMgr::GetInstance()->RegisterItem(this);
}

void DragonDataLogger::LogBoolData(uint64_t timestamp, const std::string &path, bool value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }

    logger->WriteBoolean(path, value, timestamp);
}

void DragonDataLogger::LogIntData(uint64_t timestamp, const std::string &path, int value, std::string units)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }

    logger->WriteInteger(path, value, units, timestamp);
}

void DragonDataLogger::LogDoubleData(uint64_t timestamp, const std::string &path, double value, std::string_view units)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }

    logger->WriteDouble(path, value, units, timestamp);
}

void DragonDataLogger::LogStringData(uint64_t timestamp, const std::string &path, const std::string &value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }

    logger->WriteString(path, value, timestamp);
}

void DragonDataLogger::LogDoubleArrayData(uint64_t timestamp, const std::string &path, const std::vector<double> &value, std::string_view units)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }

    logger->WriteDoubleArray(path, value, units, timestamp);
}

void DragonDataLogger::LogSwerveModuleStateData(uint64_t timestamp, const std::string &speedPath, const std::string &anglePath, frc::SwerveModuleState value, std::string_view units)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }

    double speed = value.speed.value();
    double angle = value.angle.Radians().value();
    logger->WriteDouble(speedPath, speed, units, timestamp);
    logger->WriteDouble(anglePath, angle, units, timestamp);
}

void DragonDataLogger::LogChassisSpeedsData(uint64_t timestamp, const std::string &vxPath, const std::string &vyPath, const std::string &omegaPath, frc::ChassisSpeeds value, std::string_view units)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }

    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }

    double vx = value.vx.value();
    double vy = value.vy.value();
    double omega = value.omega.value();
    logger->WriteDouble(vxPath, vx, units, timestamp);
    logger->WriteDouble(vyPath, vy, units, timestamp);
    logger->WriteDouble(omegaPath, omega, units, timestamp);
}

void DragonDataLogger::LogGamePadData(uint64_t timestamp, const std::string &path, const std::array<double, 6> axes, const std::array<bool, 10> buttons, const std::array<int, 1> povs)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }
    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }
    logger->WriteGamePadState(path, axes, buttons, povs, timestamp);
}

void DragonDataLogger::LogPose2dData(uint64_t timestamp, const std::string &path, const frc::Pose2d &value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }
    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }
    logger->WritePose2d(path, value, timestamp);
}

void DragonDataLogger::LogPose3dData(uint64_t timestamp, const std::string &path, const frc::Pose3d &value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr == nullptr)
    {
        return;
    }
    auto logger = dataMgr->GetLogger();
    if (logger == nullptr)
    {
        return;
    }
    logger->WritePose3d(path, value, timestamp);
}