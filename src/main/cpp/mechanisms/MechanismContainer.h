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

// Forward declares
class Intake;
// class Launcher;   // TODO: enable when the Launcher is converted to command-based

//====================================================================================================================================================
/// @class MechanismContainer
/// @brief Singleton that wires command-based mechanisms to driver/operator controls.
///
/// This is the mechanism analog of @c SwerveContainer. It owns the trigger/button
/// bindings for every command-based mechanism, with one @c Configure<Mechanism>()
/// method per mechanism so a mechanism can be added or removed simply by
/// commenting out a single line in @c ConfigureBindings().
///
/// The mechanism objects themselves are still created per-robot by the
/// @c MechanismConfig (e.g. @c MechanismConfigCompBot_302). Each @c Configure*()
/// method asks the active config for its mechanism and returns early if that
/// mechanism is not present on the current robot, so commenting a mechanism out
/// in the per-robot config is enough — the container binds whatever exists.
///
/// @note @c ConfigureBindings() must be called after @c MechanismConfigMgr::InitRobot()
///       has created the mechanisms (see @c Robot::InitializeRobot()).
//====================================================================================================================================================
class MechanismContainer
{
public:
    /// @brief Get the singleton instance.
    /// @return MechanismContainer* - pointer to the singleton instance
    static MechanismContainer *GetInstance();

    /// @brief Bind every present command-based mechanism to its controls.
    ///        Comment out a single line here to drop a mechanism's bindings.
    void ConfigureBindings();

private:
    MechanismContainer();
    ~MechanismContainer() = default;

    /// @brief Bind the Intake mechanism (default command + trigger bindings).
    ///        No-op if the Intake is not present on the current robot.
    void ConfigureIntake();
    // void ConfigureLauncher();   // TODO: enable when the Launcher is converted to command-based

    static MechanismContainer *m_instance;

    Intake *m_intake = nullptr;
    // Launcher *m_launcher = nullptr;   // TODO
};
