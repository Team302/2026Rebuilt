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
#include <vector>

class StateMgr;

/// @class PeriodicLooper
/// @brief Manages periodic execution of state machines across different robot operating modes.
///
/// The PeriodicLooper is a singleton class that orchestrates the execution of registered
/// StateMgr instances during different robot states: Autonomous, Teleop, Simulation, and Disabled.
/// It provides a centralized mechanism for registering state managers and invoking them at the
/// appropriate times during robot operation.
///
/// @note This class uses the singleton pattern to ensure only one instance exists.
class PeriodicLooper
{
public:
    /// @brief Gets the singleton instance of the PeriodicLooper.
    /// @return Pointer to the PeriodicLooper singleton instance.
    static PeriodicLooper *GetInstance();

    /// @brief Registers a StateMgr to run during autonomous mode.
    /// @param mgr Pointer to the StateMgr to register for autonomous operation.
    void RegisterAuton(StateMgr *mgr);

    /// @brief Registers a StateMgr to run during teleop mode.
    /// @param mgr Pointer to the StateMgr to register for teleop operation.
    void RegisterTeleop(StateMgr *mgr);

    /// @brief Registers a StateMgr to run during simulation mode.
    /// @param mgr Pointer to the StateMgr to register for simulation operation.
    void RegisterSimulation(StateMgr *mgr);

    /// @brief Registers a StateMgr to run during disabled mode.
    /// @param mgr Pointer to the StateMgr to register for disabled operation.
    void RegisterDisabled(StateMgr *mgr);

    /// @brief Registers a StateMgr to run in all operating modes.
    /// @param mgr Pointer to the StateMgr to register for all modes.
    void RegisterAll(StateMgr *mgr);

    /// @brief Executes the current state for all registered autonomous state managers.
    void AutonRunCurrentState();

    /// @brief Executes the current state for all registered teleop state managers.
    void TeleopRunCurrentState();

    /// @brief Executes the current state for all registered simulation state managers.
    void SimulationRunCurrentState();

    /// @brief Executes the current state for all registered disabled state managers.
    void DisabledRunCurrentState();

private:
    /// @brief Private constructor - use GetInstance() to get the singleton instance.
    PeriodicLooper();

    /// @brief Destructor.
    ~PeriodicLooper();

    /// @brief Sets up gamepad transitions for the given state managers.
    /// @param mgrs Vector of StateMgr pointers to configure gamepad transitions for.
    /// @param checkSw Whether to check switch states for transitions.
    void SetGamePadTransitions(std::vector<StateMgr *> mgrs, bool checkSw);

    /// @brief Executes the current state for all provided state managers.
    /// @param mgrs Vector of StateMgr pointers whose current states should be executed.
    void RunCurrentStates(std::vector<StateMgr *> mgrs);

    std::vector<StateMgr *> m_auton;      ///< State managers registered for autonomous mode.
    std::vector<StateMgr *> m_teleop;     ///< State managers registered for teleop mode.
    std::vector<StateMgr *> m_simulation; ///< State managers registered for simulation mode.
    std::vector<StateMgr *> m_disabled;   ///< State managers registered for disabled mode.

    bool m_setGamepadForAuton;      ///< Flag indicating if gamepad was set for autonomous mode.
    bool m_setGamepadForTeleop;     ///< Flag indicating if gamepad was set for teleop mode.
    bool m_setGamepadForSimulation; ///< Flag indicating if gamepad was set for simulation mode.

    static PeriodicLooper *m_instance; ///< Singleton instance of PeriodicLooper.
};
