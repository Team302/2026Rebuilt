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

// C++ Includes

// FRC includes

// Team 302 includes
// #include <gamepad/IDragonGamepad.h>
#include <teleopcontrol/TeleopControlAxis.h>
#include <teleopcontrol/TeleopControlButton.h>
#include <teleopcontrol/TeleopControlFunctions.h>

#include <RobinHood/robin_hood.h>

constexpr TeleopControlButton driverAButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverBButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverXButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverYButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverLBumper = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverRBumper = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverSelectButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverStartButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverLStickPressed = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverRStickPressed = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverLTriggerPressed = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverRTriggerPressed = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverDPad0 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverDPad45 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverDPad90 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverDPad135 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverDPad180 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverDPad225 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverDPad270 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton driverDPad315 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

constexpr TeleopControlButton copilotAButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotBButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotXButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotYButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotLBumper = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotRBumper = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotSelectButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotStartButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotLStickPressed = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotRStickPressed = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotLTriggerPressed = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotRTriggerPressed = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotDPad0 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotDPad45 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotDPad90 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotDPad135 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotDPad180 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotDPad225 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotDPad270 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton copilotDPad315 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

constexpr TeleopControlButton extra1AButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1BButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1XButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1YButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1LBumper = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1RBumper = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1SelectButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1StartButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1LStickPressed = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1RStickPressed = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1LTriggerPressed = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1RTriggerPressed = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1DPad0 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1DPad45 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1DPad90 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1DPad135 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1DPad180 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1DPad225 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1DPad270 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra1DPad315 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

constexpr TeleopControlButton extra2AButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2BButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2XButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2YButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2LBumper = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2RBumper = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2SelectButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2StartButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2LStickPressed = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2RStickPressed = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2LTriggerPressed = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2RTriggerPressed = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2DPad0 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2DPad45 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2DPad90 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2DPad135 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2DPad180 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2DPad225 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2DPad270 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra2DPad315 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

constexpr TeleopControlButton extra3AButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3BButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3XButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3YButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3LBumper = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3RBumper = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3SelectButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3StartButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3LStickPressed = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3RStickPressed = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3LTriggerPressed = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3RTriggerPressed = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3DPad0 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3DPad45 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3DPad90 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3DPad135 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3DPad180 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3DPad225 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3DPad270 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra3DPad315 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

constexpr TeleopControlButton extra4AButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4BButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4XButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4YButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4LBumper = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4RBumper = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4SelectButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4StartButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4LStickPressed = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4RStickPressed = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4LTriggerPressed = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4RTriggerPressed = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4DPad0 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4DPad45 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4DPad90 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4DPad135 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4DPad180 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4DPad225 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4DPad270 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
constexpr TeleopControlButton extra4DPad315 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

robin_hood::unordered_map<TeleopControlFunctions::FUNCTION, const TeleopControlButton> teleopControlMapButtonMap{

    // Driver Controls
    {TeleopControlFunctions::RESET_POSITION, driverDPad90},
    {TeleopControlFunctions::ROBOT_ORIENTED_DRIVE, driverDPad0},
    {TeleopControlFunctions::SYSID_MODIFER, driverSelectButton},

    // Season Specific Driver Controls
    {TeleopControlFunctions::INTAKE, driverRTriggerPressed},
    {TeleopControlFunctions::EXPEL, driverLTriggerPressed},

    {TeleopControlFunctions::DRIVE_TO_FUEL, driverAButton},
    // {TeleopControlFunctions::DRIVE_TO_HUB, driverAButton},
    // {TeleopControlFunctions::SWEEP_BEHIND_BUMP, driverAButton},
    {TeleopControlFunctions::LEVEL1_CLIMB, driverAButton},

    {TeleopControlFunctions::DRIVE_TO_OUTPOST, driverBButton},
    {TeleopControlFunctions::DRIVE_TO_TOWER, driverBButton},

    {TeleopControlFunctions::DRIVE_ALONG_NEAREST_WALL, driverXButton},
    {TeleopControlFunctions::DRIVE_TO_TOWER, driverXButton},

    {TeleopControlFunctions::DRIVE_OVER_BUMP, driverYButton},
    {TeleopControlFunctions::LEVEL3_CLIMB, driverYButton},

    {TeleopControlFunctions::CLIMB_MODE, driverStartButton},

    // Season Specific Co-Pilot Controls
    {TeleopControlFunctions::UPDATE_TARGET_OFFSET_DOWN, copilotDPad180},
    {TeleopControlFunctions::UPDATE_TARGET_OFFSET_UP, copilotDPad0},
    {TeleopControlFunctions::UPDATE_TARGET_OFFSET_RIGHT, copilotDPad90},
    {TeleopControlFunctions::UPDATE_TARGET_OFFSET_LEFT, copilotDPad270},
    {TeleopControlFunctions::LAUNCH, copilotAButton},
    {TeleopControlFunctions::LAUNCHER_OFF, copilotStartButton},
    {TeleopControlFunctions::LAUNCH_OVERRIDE, copilotRBumper},
    {TeleopControlFunctions::MANUAL_LAUNCH, copilotXButton},

    {TeleopControlFunctions::INTAKE_IN, copilotLTriggerPressed},
    {TeleopControlFunctions::INTAKE_OUT, copilotRTriggerPressed},
    {TeleopControlFunctions::EXTENDER_MODIFIER, copilotSelectButton},
    {TeleopControlFunctions::TURRET_ENABLE, copilotLStickPressed}};

constexpr TeleopControlAxis driverLJoystickX = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::AXIS_PROFILE::LINEAR, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis driverLJoystickY = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::AXIS_PROFILE::LINEAR, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis driverRJoystickX = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::AXIS_PROFILE::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis driverRJoystickY = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::AXIS_PROFILE::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis driverLTrigger = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis driverRTrigger = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

constexpr TeleopControlAxis copilotLJoystickX = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis copilotLJoystickY = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis copilotRJoystickX = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis copilotRJoystickY = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis copilotLTrigger = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis copilotRTrigger = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

constexpr TeleopControlAxis extra1LJoystickX = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis extra1LJoystickY = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis extra1RJoystickX = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis extra1RJoystickY = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis extra1LTrigger = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis extra1RTrigger = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

constexpr TeleopControlAxis extra2LJoystickX = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis extra2LJoystickY = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis extra2RJoystickX = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis extra2RJoystickY = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis extra2LTrigger = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis extra2RTrigger = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

constexpr TeleopControlAxis extra3LJoystickX = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis extra3LJoystickY = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis extra3RJoystickX = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis extra3RJoystickY = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis extra3LTrigger = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis extra3RTrigger = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

constexpr TeleopControlAxis extra4LJoystickX = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis extra4LJoystickY = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis extra4RJoystickX = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis extra4RJoystickY = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
constexpr TeleopControlAxis extra4LTrigger = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
constexpr TeleopControlAxis extra4RTrigger = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

robin_hood::unordered_map<TeleopControlFunctions::FUNCTION, const TeleopControlAxis> teleopControlMapAxisMap{

    // Driver Controls
    {TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD, driverLJoystickY},
    {TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE, driverLJoystickX},
    {TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE, driverRJoystickX},
    {TeleopControlFunctions::CLIMB_MANUAL_ROTATE_DOWN, driverLTrigger},
    {TeleopControlFunctions::CLIMB_MANUAL_ROTATE_UP, driverRTrigger},

    // Season Specific Co-Pilot Controls
    {TeleopControlFunctions::UPDATE_DEPOT_PASSING_TARGET_X, copilotLJoystickY},
    {TeleopControlFunctions::UPDATE_DEPOT_PASSING_TARGET_Y, copilotLJoystickX},
    {TeleopControlFunctions::UPDATE_OUTPOST_PASSING_TARGET_X, copilotRJoystickY},
    {TeleopControlFunctions::UPDATE_OUTPOST_PASSING_TARGET_Y, copilotRJoystickX},
    {TeleopControlFunctions::MANUAL_INTAKE_OUT, copilotRTrigger},
    {TeleopControlFunctions::MANUAL_INTAKE_IN, copilotLTrigger}};
