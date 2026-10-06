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

#include "frc2/command/button/Trigger.h"

// Forward declares
class Launcher;

//====================================================================================================================================================
/// @class LauncherContainer
/// @brief Singleton that wires command-based mechanisms to driver/operator controls.
///
///
/// The mechanism objects themselves are still created per-robot by the
/// @c MechanismConfig (e.g. @c MechanismConfigCompBot_302). Each @c Configure*()
/// method asks the active config for its mechanism and returns early if that
/// mechanism is not present on the current robot, so commenting a mechanism out
/// in the per-robot config is enough - the container binds whatever exists.
///
/// @note @c ConfigureBindings() must be called after @c MechanismConfigMgr::InitRobot()
///       has created the mechanisms (see @c Robot::InitializeRobot()).
//====================================================================================================================================================
class LauncherContainer
{
public:
    /// @brief Get the singleton instance.
    /// @return LauncherContainer* - pointer to the singleton instance
    static LauncherContainer *GetInstance();

    /// @brief Bind every present command-based mechanism to its controls.
    ///        Comment out a single line here to drop a mechanism's bindings.
    void ConfigureBindings();

private:
    LauncherContainer();
    ~LauncherContainer() = default;

    //================================================================================================
    // Per-command transition triggers (code-generator friendly)
    //
    // There is one Get<Command>Trigger() per command. Each returns a wpi::cmd::Trigger whose condition
    // answers "should this command be running right now" - a HOLDING condition, NOT the old StateMgr
    // "enter this state" condition. Writing them as holding conditions makes the binding edges line up:
    // when the desired command changes, the outgoing command's condition falls (cancel) as the incoming
    // command's condition rises (schedule) in the same loop.
    //
    // Returning a Trigger (instead of a bool) lets ConfigureBindings() use the full command-based
    // binding vocabulary per command - WhileTrue / OnTrue / OnFalse / ToggleOnTrue / Debounce / boolean
    // composition (&&, ||, !) - rather than being limited to one hard-coded scheme.
    //
    // The generator can emit one stub per command - e.g. `return wpi::cmd::Trigger([this]() { return
    // false; });` - and the hand-written transition logic then lives only inside these methods. The
    // binding table in ConfigureBindings() stays hand-written, because only a human knows whether a
    // given command is WhileTrue vs OnTrue vs the default command, etc.
    //
    // A default command runs whenever no other command requires the subsystem, so making it a holding
    // trigger would fight the default command. Instead, the default command is bound directly to the
    // subsystem via SetDefaultCommand() and is not represented by a Get<Command>Trigger().
    //================================================================================================

    /// @brief Off has highest priority; every other trigger is gated on !off so Off always wins.
    frc2::Trigger GetLauncherOffTrigger();
    frc2::Trigger GetManualLaunchTrigger();
    frc2::Trigger GetPrepareToLaunchTrigger();
    frc2::Trigger GetLaunchTrigger();
    static LauncherContainer *m_instance;

    Launcher *m_launcher = nullptr;
};