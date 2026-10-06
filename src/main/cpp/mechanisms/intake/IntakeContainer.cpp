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

#include "mechanisms/intake/IntakeContainer.h"

// FRC Includes
#include "frc2/command/button/Trigger.h"
#include <frc2/command/button/RobotModeTriggers.h>

// Team 302 Includes
#include "mechanisms/MechanismTypes.h"
#include "mechanisms/configs/MechanismConfig.h"
#include "mechanisms/configs/MechanismConfigMgr.h"
#include "mechanisms/intake/Intake.h"
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

    //==============================================================================================
    // Binding table.
    //
    // One line per command. The transition logic for each command lives in its matching
    // Get<Command>Trigger() method further down, so this table stays a flat, readable list and the
    // per-command conditions can be regenerated/edited in isolation. Each command picks the binding
    // flavor that fits it:
    //
    //   .WhileTrue(cmd) - "held" command: scheduled on the condition's false->true edge, CANCELLED on
    //                     its true->false edge. Falls back to the default command the moment the
    //                     condition stops being true.
    //   .OnTrue(cmd)    - "self-governing" command: scheduled on the false->true edge, then left alone.
    //                     Runs until its OWN IsFinished() returns true (or another command interrupts by
    //                     requiring the subsystem).
    //
    // The resting state is bound as the subsystem's DEFAULT command - it runs whenever no other command
    // requires the subsystem.
    //==============================================================================================
    m_intake->SetDefaultCommand(m_intake->GetOffCommand());

    GetIntakeTrigger().WhileTrue(m_intake->GetIntakeCommand());
    GetExpelTrigger().WhileTrue(m_intake->GetExpelCommand());
    GetLoadHopperTrigger().WhileTrue(m_intake->GetLoadHopperCommand());
    GetLaunchTrigger().WhileTrue(m_intake->GetLaunchCommand());

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("IntakeContainer"), std::string("Configured"), std::string("Intake"));
}

//==================================================================================================
// Per-command transition triggers
//==================================================================================================

frc2::Trigger IntakeContainer::GetIntakeTrigger()
{
    // --- Intake -----------------------------------------------------------------------------------
    // Run while the INTAKE button is held (teleop only); fall back to Off on release.
    return frc2::RobotModeTriggers::Teleop() && TeleopControl::GetInstance()->GetCommandTrigger(TeleopControlFunctions::INTAKE);
}

frc2::Trigger IntakeContainer::GetExpelTrigger()
{
    // --- Expel ------------------------------------------------------------------------------------
    // Run while the EXPEL button is held (teleop only); fall back to Off on release.
    return frc2::RobotModeTriggers::Teleop() && TeleopControl::GetInstance()->GetCommandTrigger(TeleopControlFunctions::EXPEL);
}

frc2::Trigger IntakeContainer::GetLoadHopperTrigger()
{
    // --- Load hopper ------------------------------------------------------------------------------
    // Run while the DRIVE_TO_OUTPOST (load hopper) button is held (teleop only); fall back to Off on
    // release.
    return frc2::RobotModeTriggers::Teleop() && TeleopControl::GetInstance()->GetCommandTrigger(TeleopControlFunctions::DRIVE_TO_OUTPOST);
}

frc2::Trigger IntakeContainer::GetLaunchTrigger()
{
    // --- Launch -----------------------------------------------------------------------------------
    // Sensor transition (auton + teleop, so NOT gated on teleop): feed the launcher while it reports
    // it is launching AND the INTAKE button is held. Falls back to Off when either drops.
    Intake *intake = m_intake;
    return frc2::Trigger([intake]()
                         { return intake->IsLaunching() &&
                                  !TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::INTAKE); });
}
