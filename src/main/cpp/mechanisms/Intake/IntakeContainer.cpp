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

#include "mechanisms/Intake/IntakeContainer.h"

// FRC Includes
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc2/command/button/Trigger.h>

// Team 302 Includes
#include "mechanisms/Intake/Intake.h"
#include "mechanisms/MechanismTypes.h"
#include "mechanisms/configs/MechanismConfig.h"
#include "mechanisms/configs/MechanismConfigMgr.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/debug/Logger.h"

IntakeContainer *IntakeContainer::m_instance = nullptr;
IntakeContainer *IntakeContainer::GetInstance()
{
    if (IntakeContainer::m_instance == nullptr)
    {
        IntakeContainer::m_instance = new IntakeContainer();
    }
    return IntakeContainer::m_instance;
}

IntakeContainer::IntakeContainer()
{
}

void IntakeContainer::ConfigureBindings()
{
    auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
    if (config == nullptr)
    {
        return;
    }

    auto mech = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::INTAKE);
    m_intake = (mech != nullptr) ? dynamic_cast<Intake *>(mech) : nullptr;
    if (m_intake == nullptr)
    {
        // Intake not present on this robot (commented out in the per-robot config) - nothing to bind.
        return;
    }

    auto controller = TeleopControl::GetInstance();
    if (controller == nullptr)
    {
        return;
    }

    m_intake->SetDefaultCommand(m_intake->GetOffCommand());

    auto considerGamePadTransitions = frc2::RobotModeTriggers::Teleop();

    auto intakeButton = controller->GetCommandTrigger(TeleopControlFunctions::INTAKE);
    auto expelButton = controller->GetCommandTrigger(TeleopControlFunctions::EXPEL);
    auto loadHopperButton = controller->GetCommandTrigger(TeleopControlFunctions::DRIVE_TO_OUTPOST);

    // Consider Gamepad Transitions (Telop)
    Intake *intake = m_intake;

    (intakeButton && considerGamePadTransitions).WhileTrue(m_intake->GetIntakeCommand());
    (expelButton && considerGamePadTransitions).WhileTrue(m_intake->GetExpelCommand());
    (loadHopperButton && considerGamePadTransitions).WhileTrue(m_intake->GetLoadHopperCommand());

    // Sensor Transitions (Auton + Telop)
    frc2::Trigger launching([intake, intakeButton]()
                            { return intake->IsLaunching() && intakeButton.Get(); });
    launching.WhileTrue(m_intake->GetLaunchCommand());

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("IntakeContainer"), std::string("Configured"), std::string("Intake"));
}
