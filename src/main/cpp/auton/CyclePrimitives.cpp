//====================================================================================================================================================
// Copyright 2025 Lake Orion Robotics FIRST Team 302
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

// C++ Includes
#include <memory>
#include <string>

// FRC includes
#include "frc/DriverStation.h"
#include "frc/Timer.h"

// Team 302 includes
#include "auton/AutonGrid.h"
#include "auton/AutonSelector.h"
#include "auton/CyclePrimitives.h"
#include "auton/PrimitiveEnums.h"
#include "auton/PrimitiveFactory.h"
#include "auton/PrimitiveParams.h"
#include "auton/PrimitiveParser.h"
#include "auton/drivePrimitives/IPrimitive.h"
#include "utils/logging/debug/Logger.h"

#include "chassis/ChassisConfigMgr.h"
#include "chassis/ChassisOptionEnums.h"

#include "mechanisms/Climber/Climber.h"
#include "mechanisms/Intake/Intake.h"
#include "mechanisms/Launcher/Launcher.h"
#include "mechanisms/MechanismTypes.h"
#include "mechanisms/configs/MechanismConfigMgr.h"

// Third Party Includes

using frc::DriverStation;
using frc::Timer;
using std::make_unique;
using std::string;

#include <pugixml/pugixml.hpp>
using namespace pugi;

CyclePrimitives::CyclePrimitives() : State(string("CyclePrimitives"), 0),
                                     m_primParams(),
                                     m_currentPrimSlot(0),
                                     m_currentPrim(nullptr),
                                     m_primFactory(PrimitiveFactory::GetInstance()),
                                     m_driveStop(nullptr),
                                     m_autonSelector(new AutonSelector()),
                                     m_timer(make_unique<Timer>()),
                                     m_maxTime(units::time::second_t(0.0)),
                                     m_isDone(false),
                                     m_chassis(),
                                     m_updatedHeadingOption(),
                                     m_cachedLauncher(nullptr),
                                     m_cachedIntake(nullptr),
                                     m_cachedClimber(nullptr)
{
    auto chassisConfig = ChassisConfigMgr::GetInstance();
    m_chassis = chassisConfig != nullptr ? chassisConfig->GetSwerveChassis() : nullptr;
    CacheMechanismPointers();
}

void CyclePrimitives::Init()
{
    m_primParams.clear();
    m_currentPrimSlot = 0; // Reset current prim
    m_currentPrim = nullptr;
    m_zones.clear();

    // Re-cache mechanism pointers in case config changed
    CacheMechanismPointers();

    auto selectedFile = m_autonSelector->GetSelectedAutoFile();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "CyclePrim", "About to parse XML file ", selectedFile.c_str());

    m_primParams = PrimitiveParser::ParseXML(selectedFile);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "CyclePrim", "nPrims", static_cast<double>(m_primParams.size()));

    if (!m_primParams.empty())
    {
        GetNextPrim();
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "CyclePrim", "end init", selectedFile.c_str());
}

void CyclePrimitives::Run()
{
    if (m_currentPrim != nullptr)
    {
        m_currentPrim->Run();

        if (m_chassis != nullptr && !m_zones.empty())
        {
            auto robotPose = m_chassis->GetPose(); // Get pose once per cycle instead of once per zone

            for (auto zone : m_zones)
            {
                if (zone.first != nullptr && zone.first->IsPoseInZone(robotPose) && !zone.second) // Check if pose is in zone and if we haven't already applied the zone actions
                {
                    zone.second = SetMechanismStatesFromZone(&zone); // Pass pointer to current zone pair

                    if (zone.first->GetChassisOption() != ChassisOptionEnums::AutonChassisOptions::NO_VISION)
                    {
                        // TODO:  plug in vision drive options
                    }

                    if (zone.first->GetAvoidOption() != ChassisOptionEnums::AutonAvoidOptions::NO_AVOID_OPTION)
                    {
                        // TODO:  plug in avoid options
                    }
                }
            }
        }

        if (m_currentPrim->IsDone())
        {
            GetNextPrim();
        }
    }
    else
    {
        m_isDone = true;
        m_primParams.clear();  // clear the primitive params vector
        m_currentPrimSlot = 0; // Reset current prim slot
        RunDriveStop();
    }
}

void CyclePrimitives::Exit()
{
}

bool CyclePrimitives::AtTarget()
{
    return m_isDone;
}

void CyclePrimitives::GetNextPrim()
{
    if (!m_primParams.empty())
    {
        PrimitiveParams *currentPrimParam = (m_currentPrimSlot < (int)m_primParams.size()) ? m_primParams[m_currentPrimSlot] : nullptr;

        m_currentPrim = (currentPrimParam != nullptr) ? m_primFactory->GetIPrimitive(currentPrimParam) : nullptr;
        if (m_currentPrim != nullptr)
        {
            m_currentPrim->Init(currentPrimParam);

            SetMechanismStatesFromParam(currentPrimParam);
            m_zones = currentPrimParam->GetZones();

            m_maxTime = currentPrimParam->GetTime();
            m_timer->Reset();
            m_timer->Start();
        }

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "CyclePrim", "Current Prim ", m_currentPrimSlot);

        m_currentPrimSlot++;
    }
}

void CyclePrimitives::RunDriveStop()
{
    if (m_driveStop == nullptr)
    {
        auto time = DriverStation::GetMatchType() != DriverStation::MatchType::kNone ? DriverStation::GetMatchTime() : units::time::second_t(15.0);
        auto params = new PrimitiveParams(DO_NOTHING, // identifier
                                          time,       // time
                                          ChassisOptionEnums::HeadingOption::MAINTAIN,
                                          0.0,      // heading
                                          string(), // ChoreoTrajectoryName
                                          ZoneParamsVector(),
                                          PrimitiveParams::VISION_ALIGNMENT::UNKNOWN,
                                          ChassisOptionEnums::DriveStateType::STOP_DRIVE,
                                          false, // launcherStateChanged
                                          false, // intakeStateChanged
                                          false, // climberStateChanged
                                          Launcher::STATE_OFF,
                                          Intake::STATE_OFF,
                                          Climber::STATE_OFF);

        m_driveStop = m_primFactory->GetIPrimitive(params);
        m_driveStop->Init(params);
    }
    m_driveStop->Run();
}

void CyclePrimitives::CacheMechanismPointers()
{
    // Always clear cached pointers first to avoid holding stale/dangling references
    m_cachedLauncher = nullptr;
    m_cachedIntake = nullptr;
    m_cachedClimber = nullptr;
    auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
    if (config != nullptr)
    {
        auto launcherStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::LAUNCHER);
        auto intakeStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::INTAKE);
        auto climberStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::CLIMBER);

        m_cachedLauncher = launcherStateMgr != nullptr ? dynamic_cast<Launcher *>(launcherStateMgr) : nullptr;
        m_cachedIntake = intakeStateMgr != nullptr ? dynamic_cast<Intake *>(intakeStateMgr) : nullptr;
        m_cachedClimber = climberStateMgr != nullptr ? dynamic_cast<Climber *>(climberStateMgr) : nullptr;
    }
}

void CyclePrimitives::SetMechanismStatesFromParam(PrimitiveParams *params)
{
    if (params != nullptr)
    {
        if (m_cachedLauncher != nullptr && params->IsLauncherStateChanging())
        {
            m_cachedLauncher->SetCurrentState(params->GetLauncherState(), true);
        }

        if (m_cachedIntake != nullptr && params->IsIntakeStateChanging())
        {
            m_cachedIntake->SetCurrentState(params->GetIntakeState(), true);
        }

        if (m_cachedClimber != nullptr && params->IsClimberStateChanging())
        {
            m_cachedClimber->SetCurrentState(params->GetClimberState(), true);
        }
    }
}
bool CyclePrimitives::SetMechanismStatesFromZone(std::pair<ZoneParams *, bool> *params)
{
    if (params != nullptr)
    {
        if (m_cachedLauncher != nullptr && params->first->IsLauncherStateChanging())
        {
            m_cachedLauncher->SetCurrentState(params->first->GetLauncherState(), true);
            return true; // Return true if we changed a mechanism state based on zone params
        }

        if (m_cachedIntake != nullptr && params->first->IsIntakeStateChanging())
        {
            m_cachedIntake->SetCurrentState(params->first->GetIntakeState(), true);
            return true; // Return true if we changed a mechanism state based on zone params
        }

        if (m_cachedClimber != nullptr && params->first->IsClimberStateChanging())
        {
            m_cachedClimber->SetCurrentState(params->first->GetClimberState(), true);
            return true; // Return true if we changed a mechanism state based on zone params
        }
    }
    return false; // Return false if no mechanism states were changed
}

void CyclePrimitives::DataLog(uint64_t timestamp)
{
    if (m_autonSelector != nullptr)
    {
        LogStringData(timestamp, "/Auton/PathName", m_autonSelector->GetSelectedAutoFile());
    }
}