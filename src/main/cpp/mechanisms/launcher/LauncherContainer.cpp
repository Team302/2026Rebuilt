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
#include "frc2/command/button/Trigger.h"
#include <frc2/command/button/RobotModeTriggers.h>

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

    if (m_launcher->IsTuningLauncherMode()) // Later can get rid of tuning mode and use opModes
    {
        m_launcher->SetDefaultCommand(m_launcher->GetLauncherTuningCommand().IgnoringDisable(true));
    }
    else
    {
        m_launcher->SetDefaultCommand(m_launcher->GetIdleCommand().IgnoringDisable(true));
    }

    GetManualLaunchTrigger().WhileTrue(m_launcher->GetManualLaunchCommand());

    GetPrepareToLaunchTrigger().WhileTrue(m_launcher->GetPrepareToLaunchCommand());
    GetLaunchTrigger().OnTrue(m_launcher->GetLaunchCommand());

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("LauncherContainer"), std::string("Configured"), std::string("Launcher"));
}

//==================================================================================================
// Per-command transition triggers
//==================================================================================================

frc2::Trigger LauncherContainer::GetLauncherOffTrigger()
{
    // --- Off (protected mode) ---------------------------------------------------------------------
    // Highest priority. Every other trigger is gated on !IsLauncherInProtectedMode() so Off wins.

    Launcher *launcher = m_launcher;
    return frc2::Trigger([launcher]()
                         { return launcher->IsLauncherInProtectedMode(); });
}

frc2::Trigger LauncherContainer::GetManualLaunchTrigger()
{
    // --- Manual launch ----------------------------------------------------------------------------
    // Old ManualLaunchState: run while the MANUAL_LAUNCH button is held; fall back to Idle on release.
    Launcher *launcher = m_launcher;
    return frc2::Trigger([launcher]()
                         { return !launcher->IsLauncherInProtectedMode() &&
                                  TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::MANUAL_LAUNCH); });
}

frc2::Trigger LauncherContainer::GetPrepareToLaunchTrigger()
{
    // --- Prepare to launch ------------------------------------------------------------------------
    // Normal operation (not tuning): hold the LAUNCH button to spin up in PrepareToLaunch until we are
    // at target. The !IsLauncherAtTarget() term is what sends us BACK to Prepare from Launch if we fall
    // off target while still holding LAUNCH (this rises again, interrupting Launch).
    Launcher *launcher = m_launcher;
    return frc2::Trigger([launcher]()
                         { return !launcher->IsLauncherInProtectedMode() &&
                                  !launcher->IsTuningLauncherMode() &&
                                  TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::LAUNCH) &&
                                  !launcher->IsLauncherAtTarget(); });
}

frc2::Trigger LauncherContainer::GetLaunchTrigger()
{
    // --- Launch -----------------------------------------------------------------------------------
    // Two ways in, matching the old LaunchState:
    //   1. Normal: holding LAUNCH (not tuning) once we ARE at target -> mutually exclusive with Prepare.
    //   2. Override: LAUNCH_OVERRIDE && !EXTENDER_MODIFIER -> the path out of LauncherTuning, also forces
    //      a launch from PrepareToLaunch regardless of at-target.
    // Bound with OnTrue (NOT WhileTrue): once scheduled, Launch is NOT cancelled when this condition goes
    // false. It is self-governing - it runs until LauncherLaunchCommand::IsDone() returns true (launch
    // buttons released for the debounce window, or auton launch detector) or until another command
    // (Prepare when we fall off target while still holding LAUNCH, or Off) interrupts by requiring the
    // subsystem. This is what prevents bouncing straight back to Idle the instant the condition drops.
    Launcher *launcher = m_launcher;
    return frc2::Trigger([launcher]()
                         {
        auto tc = TeleopControl::GetInstance();
        bool overrideToLaunch = tc->IsButtonPressed(TeleopControlFunctions::LAUNCH_OVERRIDE) &&
                                !tc->IsButtonPressed(TeleopControlFunctions::EXTENDER_MODIFIER);
        bool autoLaunch = !launcher->IsTuningLauncherMode() &&
                          tc->IsButtonPressed(TeleopControlFunctions::LAUNCH) &&
                          launcher->IsLauncherAtTarget();
        return !launcher->IsLauncherInProtectedMode() && (autoLaunch || overrideToLaunch); });
}
