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

#include <filesystem>

#include "ctre/phoenix6/SignalLogger.hpp"
#include "frc/DataLogManager.h"
#include "frc/DriverStation.h"
#include "frc/RobotController.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"
#include "utils/logging/signals/CTRESignalLogger.h"
#include "utils/logging/signals/UDPSignalLogger.h"

using namespace std;
using ctre::phoenix6::SignalLogger;

DragonDataLoggerMgr *DragonDataLoggerMgr::m_instance = nullptr;
DragonDataLoggerMgr *DragonDataLoggerMgr::GetInstance()
{
    if (DragonDataLoggerMgr::m_instance == nullptr)
    {
        DragonDataLoggerMgr::m_instance = new DragonDataLoggerMgr();
    }
    return DragonDataLoggerMgr::m_instance;
}

DragonDataLoggerMgr::DragonDataLoggerMgr() : m_items()
{
    // This line needs to be removed if we want to run hoot logs
    CTRESignalLogger *ctreLogger = new CTRESignalLogger();
    ctreLogger->SetAutoLogging(false);

    // m_logger = std::make_unique<CTRESignalLogger>();
    m_logger = std::make_unique<UDPSignalLogger>("127.0.0.1", 5900);

    m_logger->Start();
    m_timer.Start();
}

void DragonDataLoggerMgr::SetLogger(std::unique_ptr<ISignalLogger> logger)
{
    if (!logger)
    {
        return;
    }

    if (m_logger)
    {
        m_logger->Stop();
    }
    m_logger = std::move(logger);
    m_logger->Start();
}

void DragonDataLoggerMgr::SetLoggerType(LoggerType type, const std::string &ipAddress, int port)
{
    if (m_logger)
    {
        m_logger->Stop();
    }

    switch (type)
    {
    case LoggerType::CTRE_SIGNAL_LOGGER:
        m_logger = std::make_unique<CTRESignalLogger>();
        break;

    case LoggerType::UDP_LOGGER:
        if (!ipAddress.empty() && port > 0)
        {
            m_logger = std::make_unique<UDPSignalLogger>(ipAddress, port);
        }
        else
        {
            m_logger = std::make_unique<UDPSignalLogger>("127.0.0.1", 5800);
        }
        break;

    case LoggerType::NETWORK_TABLES_LOGGER:
        // Future implementation
        // m_logger = std::make_unique<NetworkTablesLogger>();
        break;

    default:
        m_logger = std::make_unique<UDPSignalLogger>("127.0.0.1", 5800);
        break;
    }

    if (m_logger)
    {
        m_logger->Start();
    }
}

DragonDataLoggerMgr::~DragonDataLoggerMgr()
{
    if (m_logger)
    {
        m_logger->Stop();
    }
}

void DragonDataLoggerMgr::RegisterItem(DragonDataLogger *item)
{
    m_items.emplace_back(item);
}

void DragonDataLoggerMgr::PeriodicDataLog()
{
    uint64_t timestamp = frc::RobotController::GetFPGATime();

    m_timer.Reset();
    while (m_timer.Get() < m_period)
    {
        auto item = m_items[m_lastIndex];
        item->DataLog(timestamp);
        m_lastIndex += ((m_lastIndex >= (m_items.size() - 1)) ? -m_lastIndex : 1);
    }
}
