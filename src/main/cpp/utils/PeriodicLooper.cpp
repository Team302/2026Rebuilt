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
//======================================================\==============================================================================================

#include "utils/PeriodicLooper.h"

#include "state/StateMgr.h"
#include <vector>

using std::vector;

/**
 * @file PeriodicLooper.cpp
 * @brief Implementation of the PeriodicLooper singleton.
 *
 * The PeriodicLooper manages collections of StateMgr objects for the
 * different robot operation modes (autonomous, teleop, simulation, disabled)
 * and runs their periodic processing each loop. It also ensures that
 * gamepad transition checking is initialized once per mode.
 *
 * This file contains the concrete implementations of the methods declared in
 * `PeriodicLooper.h`.
 */

PeriodicLooper *PeriodicLooper::m_instance = nullptr;

/**
 * @brief Get the singleton instance of the PeriodicLooper.
 *
 * If an instance has not yet been created, this function constructs it.
 * The caller owns no ownership of the pointer; the instance lives for the
 * lifetime of the process.
 *
 * @return PeriodicLooper* pointer to the singleton instance
 */
PeriodicLooper *PeriodicLooper::GetInstance()
{
    if (PeriodicLooper::m_instance == nullptr)
    {
        PeriodicLooper::m_instance = new PeriodicLooper();
    }
    return PeriodicLooper::m_instance;
}

/**
 * @brief Construct a new PeriodicLooper
 *
 * Initializes internal containers and flags used to ensure that
 * per-mode gamepad transition settings are applied once.
 */
PeriodicLooper::PeriodicLooper() : m_auton(),
                                   m_teleop(),
                                   m_simulation(),
                                   m_setGamepadForAuton(false),
                                   m_setGamepadForTeleop(false),
                                   m_setGamepadForSimulation(false)
{
}

/**
 * @brief Destroy the PeriodicLooper
 *
 * Clears internal vectors. The singleton pointer is not deleted here; the
 * destructor is provided for completeness and to allow clean teardown in
 * test environments.
 */
PeriodicLooper::~PeriodicLooper()
{
    m_auton.clear();
    m_teleop.clear();
    m_simulation.clear();
    m_disabled.clear();
}

/**
 * @brief Register a StateMgr to be run during autonomous mode loops.
 *
 * The provided StateMgr will have its periodic tasks and current state run
 * when AutonRunCurrentState() is called.
 */
void PeriodicLooper::RegisterAuton(StateMgr *mgr)
{
    m_auton.emplace_back(mgr);
}

/**
 * @brief Register a StateMgr to be run during teleop mode loops.
 */
void PeriodicLooper::RegisterTeleop(StateMgr *mgr)
{
    m_teleop.emplace_back(mgr);
}

/**
 * @brief Register a StateMgr to be run during simulation mode loops.
 */
void PeriodicLooper::RegisterSimulation(StateMgr *mgr)
{
    m_simulation.emplace_back(mgr);
}

/**
 * @brief Register a StateMgr to be run while robot is disabled.
 */
void PeriodicLooper::RegisterDisabled(StateMgr *mgr)
{
    m_disabled.emplace_back(mgr);
}

/**
 * @brief Convenience method to register a StateMgr for all modes.
 */
void PeriodicLooper::RegisterAll(StateMgr *mgr)
{
    m_auton.emplace_back(mgr);
    m_teleop.emplace_back(mgr);
    m_simulation.emplace_back(mgr);
    m_disabled.emplace_back(mgr);
}

/**
 * @brief Run periodic processing for all registered autonomous StateMgrs.
 *
 * On the first call after registration, ensure gamepad transition checking
 * is configured for the registered managers.
 */
void PeriodicLooper::AutonRunCurrentState()
{
    if (!m_setGamepadForAuton)
    {
        SetGamePadTransitions(m_auton, false);
        m_setGamepadForAuton = true;
    }
    RunCurrentStates(m_auton);
}

/**
 * @brief Run periodic processing for all registered teleop StateMgrs.
 */
void PeriodicLooper::TeleopRunCurrentState()
{
    if (!m_setGamepadForTeleop)
    {
        SetGamePadTransitions(m_teleop, true);
        m_setGamepadForTeleop = true;
    }
    RunCurrentStates(m_teleop);
}

/**
 * @brief Run periodic processing for all registered simulation StateMgrs.
 */
void PeriodicLooper::SimulationRunCurrentState()
{
    if (!m_setGamepadForSimulation)
    {
        SetGamePadTransitions(m_simulation, true);
        m_setGamepadForSimulation = true;
    }
    RunCurrentStates(m_simulation);
}

/**
 * @brief Run periodic processing for all registered disabled StateMgrs.
 */
void PeriodicLooper::DisabledRunCurrentState()
{
    RunCurrentStates(m_disabled); // No gamepad transitions in disabled mode
}

/**
 * @brief Configure whether gamepad-triggered state transitions should be checked
 * for the supplied StateMgr list.
 *
 * This sets the flag on each manager so that they will (or will not) consider
 * gamepad input when deciding to transition between states.
 *
 * @param mgrs list of StateMgr pointers to configure (copied by value)
 * @param checkSw true to enable gamepad transitions, false to disable
 */
void PeriodicLooper::SetGamePadTransitions(vector<StateMgr *> mgrs, bool checkSw)
{
    for (auto mgr : mgrs)
    {
        mgr->SetAreGamepadTransitionsChecked(checkSw);
    }
}

/**
 * @brief Run the periodic common tasks and current state for each manager.
 *
 * The managers are provided by value to keep the signature simple. Each
 * manager will first have its common periodic work run, followed by a call
 * to RunCurrentState() to execute the active state's logic.
 */
void PeriodicLooper::RunCurrentStates(vector<StateMgr *> mgrs)
{
    for (auto mgr : mgrs)
    {
        mgr->RunCommonTasks();
        mgr->RunCurrentState();
    }
}
