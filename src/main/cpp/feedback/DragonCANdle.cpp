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

#include "feedback/DragonCANdle.h"
#include "utils/logging/debug/Logger.h"
#include "frc/RobotBase.h"

using namespace ctre::phoenix6;
using namespace ctre::phoenix6::controls;

DragonCANdle *DragonCANdle::m_instance = nullptr;

DragonCANdle *DragonCANdle::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new DragonCANdle();
    }
    return m_instance;
}

void DragonCANdle::Initialize(int canID, int stripSize, const std::string &canBus, StripTypeValue type)
{
    if (m_candle != nullptr)
    {
        Logger::GetLogger()->LogData(
            LOGGER_LEVEL::ERROR_ONCE,
            "DragonCANdle",
            "Already Initialized",
            "True");
        return;
    }

    m_candle = new hardware::CANdle(canID, canBus == "rio" ? CANBus::RoboRIO() : CANBus{canBus});

    configs::CANdleConfiguration configs{};
    configs.LED.BrightnessScalar = 1.0;
    configs.LED.StripType = type;
    configs.CANdleFeatures.VBatOutputMode = signals::VBatOutputModeValue::On;
    m_candle->GetConfigurator().Apply(configs);

    m_externalCount = stripSize;

    m_diagTimer.Start();
}

void DragonCANdle::Periodic()
{
    if (m_candle == nullptr)
        return;

    UpdateDiagnostics();
    UpdateAnimation();

    if (frc::RobotBase::IsSimulation() && m_candle != nullptr)
    {
        // Update simulated sensor states for testing
        auto &simState = m_candle->GetSimState();
        simState.SetSupplyVoltage(12_V);
        simState.GetLastStatusCode();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonCANdle", "AppliedControl", std::string(m_candle->GetAppliedControl().get()->GetName()));
    }
}

// ================= Animation =================

void DragonCANdle::SetAnimation(AnimationMode mode)
{
    m_animMode = mode;
}

void DragonCANdle::SetSolidColor(const frc::Color &color)
{
    m_primaryColor = color;
    m_animMode = AnimationMode::SOLID;
}

void DragonCANdle::SetAlternatingColors(const frc::Color &color1, const frc::Color &color2)
{
    m_primaryColor = color1;
    m_secondaryColor = color2;
    m_animMode = AnimationMode::ALTERNATING;
}

void DragonCANdle::SetBrightness(double brightness)
{
    if (m_candle == nullptr)
        return;

    m_brightness = std::clamp(brightness, 0.0, 1.0);

    configs::CANdleConfiguration configs{};
    configs.LED.BrightnessScalar = m_brightness;
    m_candle->GetConfigurator().Apply(configs);
}

void DragonCANdle::TurnOff()
{
    m_animMode = AnimationMode::OFF;
}

void DragonCANdle::UpdateAnimation()
{
    using RGBWColor = signals::RGBWColor;

    switch (m_animMode)
    {
    case AnimationMode::OFF:
    {
        // Turn off external LEDs
        m_candle->SetControl(controls::SolidColor{m_externalStart, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{frc::Color::kBlack}));
        break;
    }

    case AnimationMode::SOLID:
    {
        m_candle->SetControl(controls::SolidColor{m_externalStart, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{m_primaryColor}));
        break;
    }

    case AnimationMode::ALTERNATING: // Blinking pattern that swaps entire strip between two colors

    {
        if (m_alternatingTimer > 2 * m_alternatingPeriod)
            m_alternatingTimer = 0;

        int blinkState = (m_alternatingTimer / m_alternatingPeriod) % 2;
        frc::Color currentColor = (blinkState == 0) ? m_primaryColor : m_secondaryColor;

        m_candle->SetControl(controls::SolidColor{m_externalStart, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{currentColor}));

        m_alternatingTimer++;
        break;
    }

    case AnimationMode::RAINBOW:
    {
        m_candle->SetControl(controls::RainbowAnimation{m_externalStart, m_externalStart + m_externalCount - 1});
        break;
    }

    case AnimationMode::BREATHING:
    {
        m_candle->SetControl(controls::SingleFadeAnimation{m_externalStart, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{m_primaryColor})
                                 .WithFrameRate(m_breathingFrequency));
        break;
    }

    case AnimationMode::BLINKING:
    {
        m_candle->SetControl(controls::StrobeAnimation{m_externalStart, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{m_primaryColor})
                                 .WithFrameRate(m_blinkingFrequency));
        break;
    }

    case AnimationMode::CHASER:
    {
        m_candle->SetControl(controls::ColorFlowAnimation{m_externalStart, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{m_primaryColor}));
        break;
    }

    case AnimationMode::CLOSING_IN:
    {
        m_candle->SetControl(controls::LarsonAnimation{m_externalStart, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{m_primaryColor}));
        break;
    }
    default:
        break;
    }
}

// ================= Diagnostics =================

void DragonCANdle::UpdateDiagnostics()
{
    using RGBWColor = signals::RGBWColor;

    // Set individual onboard LEDs for diagnostics
    // Alliance
    if (m_alliance == frc::DriverStation::Alliance::kBlue)
        m_candle->SetControl(controls::SolidColor{0, 0}.WithColor(RGBWColor{frc::Color::kBlue}));
    else
        m_candle->SetControl(controls::SolidColor{0, 0}.WithColor(RGBWColor{frc::Color::kRed}));

    // Quest
    if (m_questOK)
        m_candle->SetControl(controls::SolidColor{1, 1}.WithColor(RGBWColor{frc::Color::kGreen}));
    else
        m_candle->SetControl(controls::SolidColor{1, 1}.WithColor(RGBWColor{frc::Color::kRed}));

    // Limelights (aggregated)
    int llCount = (m_ll1 ? 1 : 0) + (m_ll2 ? 1 : 0) + (m_ll3 ? 1 : 0);
    if (llCount == 3)
        m_candle->SetControl(controls::SolidColor{2, 2}.WithColor(RGBWColor{frc::Color::kGreen}));
    else if (llCount == 0)
        m_candle->SetControl(controls::SolidColor{2, 2}.WithColor(RGBWColor{frc::Color::kRed}));
    else
    {
        // Cycle through the 3 limelights, showing green for connected and red for missing
        // Each limelight is displayed for m_limelightBlinkPeriod frames
        m_limelightBlinkTimer++;
        if (m_limelightBlinkTimer >= 3 * m_limelightBlinkPeriod)
            m_limelightBlinkTimer = 0;

        int currentLL = (m_limelightBlinkTimer / m_limelightBlinkPeriod) % 3;

        frc::Color llColor = frc::Color::kRed;
        if (currentLL == 0 && m_ll1)
            llColor = frc::Color::kGreen;
        else if (currentLL == 1 && m_ll2)
            llColor = frc::Color::kGreen;
        else if (currentLL == 2 && m_ll3)
            llColor = frc::Color::kGreen;

        m_candle->SetControl(controls::SolidColor{2, 2}.WithColor(RGBWColor{llColor}));
    }

    // Data Logger
    if (m_dataLoggerOK)
        m_candle->SetControl(controls::SolidColor{3, 3}.WithColor(RGBWColor{frc::Color::kGreen}));
    else
        m_candle->SetControl(controls::SolidColor{3, 3}.WithColor(RGBWColor{frc::Color::kRed}));

    // Sensors - show yellow when triggered, black when not
    m_candle->SetControl(controls::SolidColor{4, 4}.WithColor(RGBWColor{m_intake ? frc::Color::kYellow : frc::Color::kBlack}));
    m_candle->SetControl(controls::SolidColor{5, 5}.WithColor(RGBWColor{m_hood ? frc::Color::kYellow : frc::Color::kBlack}));
    m_candle->SetControl(controls::SolidColor{6, 6}.WithColor(RGBWColor{m_turretZero ? frc::Color::kYellow : frc::Color::kBlack}));
    m_candle->SetControl(controls::SolidColor{7, 7}.WithColor(RGBWColor{m_turretEnd ? frc::Color::kYellow : frc::Color::kBlack}));
}