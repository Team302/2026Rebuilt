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

#include "mechanisms/launcher/LauncherContainer.h"

// FRC Includes
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc2/command/button/Trigger.h>

// Team 302 Includes
#include "mechanisms/MechanismTypes.h"
#include "mechanisms/configs/MechanismConfig.h"
#include "mechanisms/configs/MechanismConfigMgr.h"
#include "mechanisms/launcher/Launcher.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/debug/Logger.h"

LauncherContainer *LauncherContainer::m_instance = nullptr;
LauncherContainer *LauncherContainer::GetInstance()
{
    if (LauncherContainer::m_instance == nullptr)
    {
        LauncherContainer::m_instance = new LauncherContainer();
    }
    return LauncherContainer::m_instance;
}

LauncherContainer::LauncherContainer()
{
}

void LauncherContainer::ConfigureBindings()
{
    auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
    if (config == nullptr)
    {
        return;
    }

    auto mech = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::LAUNCHER);
    m_launcher = (mech != nullptr) ? dynamic_cast<Launcher *>(mech) : nullptr;
    if (m_launcher == nullptr)
    {
        // Launcher not present on this robot (commented out in the per-robot config) - nothing to bind.
        return;
    }

    auto controller = TeleopControl::GetInstance();
    if (controller == nullptr)
    {
        return;
    }

    m_launcher->SetDefaultCommand(m_launcher->GetOffCommand());

    auto considerGamePadTransitions = frc2::RobotModeTriggers::Teleop();

    // Consider Gamepad Transitions (Telop)

    // Sensor Transitions (Auton + Telop)

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("LauncherContainer"), std::string("Configured"), std::string("Launcher"));
}