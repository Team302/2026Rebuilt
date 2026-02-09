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
}

// ================= Animation =================

void DragonCANdle::SetAnimation(AnimationMode mode)
{
    m_animMode = mode;
}

void DragonCANdle::SetSolidColor(const frc::Color &color)
{
    m_primaryColor = color;
    m_animMode = AnimationMode::Solid;
}

void DragonCANdle::SetAlternatingColors(const frc::Color &color1, const frc::Color &color2)
{
    m_primaryColor = color1;
    m_secondaryColor = color2;
    m_animMode = AnimationMode::Alternating;
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
    m_animMode = AnimationMode::Off;
}

void DragonCANdle::UpdateAnimation()
{
    using RGBWColor = signals::RGBWColor;

    switch (m_animMode)
    {
    case AnimationMode::Off:
    {
        // Turn off external LEDs
        m_candle->SetControl(controls::SolidColor{m_externalStart, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{frc::Color::kBlack}));
        break;
    }

    case AnimationMode::Solid:
    {
        m_candle->SetControl(controls::SolidColor{m_externalStart, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{m_primaryColor}));
        break;
    }

    case AnimationMode::Alternating:
    {
        int halfCount = m_externalCount / 2;
        m_candle->SetControl(controls::SolidColor{m_externalStart, m_externalStart + halfCount - 1}
                                 .WithColor(RGBWColor{m_primaryColor}));
        m_candle->SetControl(controls::SolidColor{m_externalStart + halfCount, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{m_secondaryColor}));
        break;
    }

    case AnimationMode::Rainbow:
    {
        m_candle->SetControl(controls::RainbowAnimation{m_externalStart, m_externalStart + m_externalCount - 1});
        break;
    }

    case AnimationMode::Breathing:
    {
        m_candle->SetControl(controls::SingleFadeAnimation{m_externalStart, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{m_primaryColor})
                                 .WithFrameRate(m_breathingFrequency));
        break;
    }

    case AnimationMode::Blinking:
    {
        m_candle->SetControl(controls::StrobeAnimation{m_externalStart, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{m_primaryColor})
                                 .WithFrameRate(m_blinkingFrequency));
        break;
    }

    case AnimationMode::Chaser:
    {
        m_candle->SetControl(controls::ColorFlowAnimation{m_externalStart, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{m_primaryColor}));
        break;
    }

    case AnimationMode::ClosingIn:
    {
        m_candle->SetControl(controls::LarsonAnimation{m_externalStart, m_externalStart + m_externalCount - 1}
                                 .WithColor(RGBWColor{m_primaryColor}));
        break;
    }
    }
}

// ================= Diagnostic Setters =================

void DragonCANdle::SetAlliance(frc::DriverStation::Alliance alliance)
{
    m_alliance = alliance;
}

void DragonCANdle::SetQuestStatus(bool connected)
{
    m_questOK = connected;
}

void DragonCANdle::SetDataLoggerStatus(bool connected)
{
    m_dataLoggerOK = connected;
}

void DragonCANdle::SetLimelightStatuses(bool ll1, bool ll2, bool ll3)
{
    m_ll1 = ll1;
    m_ll2 = ll2;
    m_ll3 = ll3;
}

void DragonCANdle::SetIntakeSensor(bool triggered)
{
    m_intake = triggered;
}

void DragonCANdle::SetHoodSwitch(bool triggered)
{
    m_hood = triggered;
}

void DragonCANdle::SetTurretZero(bool triggered)
{
    m_turretZero = triggered;
}

void DragonCANdle::SetTurretEnd(bool triggered)
{
    m_turretEnd = triggered;
}

// ================= Diagnostics =================

void DragonCANdle::UpdateDiagnostics()
{
    using RGBWColor = signals::RGBWColor;

    // Set individual onboard LEDs for diagnostics
    // Alliance
    if (m_alliance == frc::DriverStation::Alliance::kBlue)
        m_candle->SetControl(controls::SolidColor{0, 0}.WithColor(RGBWColor{0, 0, 255, 0}));
    else
        m_candle->SetControl(controls::SolidColor{0, 0}.WithColor(RGBWColor{255, 0, 0, 0}));

    // Quest
    if (m_questOK)
        m_candle->SetControl(controls::SolidColor{1, 1}.WithColor(RGBWColor{0, 255, 0, 0}));
    else
        m_candle->SetControl(controls::SolidColor{1, 1}.WithColor(RGBWColor{100, 0, 0, 0}));

    // Limelights (aggregated)
    int llCount = (m_ll1 ? 1 : 0) + (m_ll2 ? 1 : 0) + (m_ll3 ? 1 : 0);
    if (llCount == 3)
        m_candle->SetControl(controls::SolidColor{2, 2}.WithColor(RGBWColor{0, 255, 0, 0}));
    else if (llCount == 0)
        m_candle->SetControl(controls::SolidColor{2, 2}.WithColor(RGBWColor{255, 0, 0, 0}));
    else
    {
        // Blink yellow/blue for partial limelight connection
        if (static_cast<int>(m_diagTimer.Get().value()) % 2 == 0)
            m_candle->SetControl(controls::SolidColor{2, 2}.WithColor(RGBWColor{255, 255, 0, 0}));
        else
            m_candle->SetControl(controls::SolidColor{2, 2}.WithColor(RGBWColor{0, 0, 255, 0}));
    }

    // Data Logger
    if (m_dataLoggerOK)
        m_candle->SetControl(controls::SolidColor{3, 3}.WithColor(RGBWColor{0, 255, 0, 0}));
    else
        m_candle->SetControl(controls::SolidColor{3, 3}.WithColor(RGBWColor{100, 0, 0, 0}));

    // Sensors - show yellow when triggered, black when not
    m_candle->SetControl(controls::SolidColor{4, 4}.WithColor(RGBWColor{static_cast<uint8_t>(m_intake ? 255 : 0), static_cast<uint8_t>(m_intake ? 255 : 0), 0, 0}));
    m_candle->SetControl(controls::SolidColor{5, 5}.WithColor(RGBWColor{static_cast<uint8_t>(m_hood ? 255 : 0), static_cast<uint8_t>(m_hood ? 255 : 0), 0, 0}));
    m_candle->SetControl(controls::SolidColor{6, 6}.WithColor(RGBWColor{static_cast<uint8_t>(m_turretZero ? 255 : 0), static_cast<uint8_t>(m_turretZero ? 255 : 0), 0, 0}));
    m_candle->SetControl(controls::SolidColor{7, 7}.WithColor(RGBWColor{static_cast<uint8_t>(m_turretEnd ? 255 : 0), static_cast<uint8_t>(m_turretEnd ? 255 : 0), 0, 0}));
}