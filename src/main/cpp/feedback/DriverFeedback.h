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

#include <array>
#include <memory>
#include <string>

#include "chassis/ChassisOptionEnums.h"
#include "feedback/DragonCANdle.h"
#include "mechanisms/launcher/Launcher.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "state/IRobotStateChangeSubscriber.h"
#include "state/RobotStateChanges.h"
#include "teleopcontrol/TeleopControl.h"

class Intake;
class DragonVision;

/// @class DriverFeedback
/// @brief Singleton that manages all driver-facing feedback: LEDs, controller rumble, and diagnostic displays.
///
/// Subscribes to robot state changes (scoring mode, drive-to, climb mode, shift warnings) and
/// translates them into LED animations, rumble patterns, and NetworkTables controller status.
///
/// Performance notes:
///   - Expensive operations (vision health checks, CAN limit-switch reads) are throttled
///     via m_diagnosticUpdateInterval so they only run every Nth loop.
///   - Mechanism pointers (Launcher, Intake), DragonVision, TeleopControl, and the NetworkTable are cached
///     once (in the constructor or via in-class member initializers) to avoid repeated singleton lookups and dynamic_casts.

class DriverFeedback : public IRobotStateChangeSubscriber
{
public:
    /// @brief Top-level periodic call — updates LEDs, diagnostics (throttled), rumble, and controller status.
    void UpdateFeedback();

    /// @brief Returns the singleton instance, creating it on first call.
    static DriverFeedback *GetInstance();

    /// @brief Called by RobotState when an integer state value changes (scoring mode, drive state).
    void NotifyStateUpdate(RobotStateChanges::StateChange change, int value) override;
    /// @brief Called by RobotState when a boolean state value changes (drive-to, climb, shift warnings).
    void NotifyStateUpdate(RobotStateChanges::StateChange change, bool value) override;

    /// @brief Tells DriverFeedback whether the current autonomous file is valid (affects disabled LED color).
    void SetIsValidAutonFile(bool isValid) { m_isValidAutonFile = isValid; }

private:
    /// @brief Updates controller rumble based on shift-change warnings.
    void UpdateRumble();
    /// @brief Reads vision health and CAN limit-switch states; sets diagnostic LED indicators.
    ///        Only called every m_diagnosticUpdateInterval loops to avoid CAN bus overhead.
    void UpdateDiagnosticLEDs();
    /// @brief Publishes connected-controller info to NetworkTables while disabled (every ~25 loops).
    void CheckControllers();
    void DisplayPressure() const;
    /// @brief Determines the desired LED animation and color based on current robot state / launcher state.
    void UpdateLEDStates();
    void UpdateCompressorState();
    /// @brief Applies the desired animation/colors to the DragonCANdle only when they differ from the previous frame.
    void UpdateLEDs(DragonCANdle::AnimationMode desiredAnimation, frc::Color desiredPrimaryColor, frc::Color desiredSecondaryColor);
    DriverFeedback();
    ~DriverFeedback() = default;

    // --- Previous-frame state for change-detection in UpdateLEDs ---
    bool m_AutonomousEnabled = false;
    bool m_TeleopEnabled = false;

    frc::Color m_prevPrimaryColorState = frc::Color::kBlack;                        ///< Last primary color sent to LEDs
    frc::Color m_prevSecondaryColorState = frc::Color::kBlack;                      ///< Last secondary color sent to LEDs
    DragonCANdle::AnimationMode m_prevAnimaiton = DragonCANdle::AnimationMode::OFF; ///< Last animation mode sent

    enum DriverFeedbackStates
    {
        NONE
    };

    DragonCANdle *m_LEDStates = DragonCANdle::GetInstance(); ///< Cached LED controller singleton

    units::frequency::hertz_t m_blinkingFrequency = 5_Hz;      ///< Normal blink rate
    units::frequency::hertz_t m_shiftBlinkingFrequency = 1_Hz; ///< Slower blink rate for shift warnings

    static DriverFeedback *m_instance;
    RobotStateChanges::ScoringMode m_scoringMode = RobotStateChanges::ScoringMode::FUEL; ///< Current desired scoring mode

    // --- Robot state flags (updated via NotifyStateUpdate) ---
    bool m_isInDriveTo = false;           ///< True when chassis is auto-driving to a field element
    bool m_climbMode = false;             ///< True when robot is in climb mode
    bool m_isValidAutonFile = false;      ///< True when a valid autonomous file is loaded
    bool m_isIntakeIn = false;            ///< True when the intake extender reverse limit is tripped
    bool m_shiftChangeIn5Seconds = false; ///< True when a mode shift is approaching (~5 s)
    bool m_startLaunching = false;        ///< True when a mode shift is imminent (~1.5 s, triggers rumble)
    ChassisOptionEnums::DriveStateType m_driveStateType = ChassisOptionEnums::DriveStateType::ROBOT_DRIVE;

    // --- Cached pointers (resolved once in constructor to avoid per-loop lookups) ---
    TeleopControl *m_teleopControl = TeleopControl::GetInstance(); ///< Teleop controller for rumble
    Launcher *m_launcher = nullptr;                                ///< Cached launcher mechanism
    Intake *m_intake = nullptr;                                    ///< Cached intake mechanism
    DragonVision *m_dragonVision = nullptr;                        ///< Cached vision singleton
    std::shared_ptr<nt::NetworkTable> m_controllerTable = nullptr; ///< Cached NT table for controller status

    // --- Throttle counters (avoid expensive work every 20 ms loop) ---
    int m_diagnosticLoopCounter = 0;                      ///< Counts loops since last diagnostic update
    static constexpr int m_diagnosticUpdateInterval = 10; ///< Run diagnostics every 10 loops (~200 ms)
    int m_controllerLoopCounter = 0;                      ///< Counts loops since last controller check
    static constexpr int m_controllerUpdateInterval = 25; ///< Check controllers every 25 loops (~500 ms)

    // --- Pre-built NT key strings (avoid per-loop heap allocations) ---
    static constexpr int kMaxJoystickPorts = 6;                  ///< DriverStation::kJoystickPorts
    std::array<std::string, kMaxJoystickPorts> m_controllerKeys; ///< "Controller0" … "Controller5"

    // --- Previous rumble state (avoid redundant SetRumble writes) ---
    bool m_prevRumbleState = false;
    bool m_turretEnabled = false;

    // NetworkTable for Pi logger
    std::shared_ptr<nt::NetworkTable> m_loggerTable = nullptr;
};
