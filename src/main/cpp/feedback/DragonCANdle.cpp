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

void DragonCANdle::Initialize(int canID, const std::string &canBus, signals::StripTypeValue type)
{
    if (m_candle != nullptr)
    {
        Logger::GetLogger()->LogData(
            LOGGER_LEVEL::ERROR_ONCE,
            "DragonCANdle",
            "Already Initialized",
            "Only one CANdle allowed");
        return;
    }

    m_candle = new hardware::CANdle(canID, canBus);

    configs::CANdleConfiguration configs{};
    configs.LED.BrightnessScalar = 1.0;
    configs.LED.StripType = type;
    configs.CANdleFeatures.VBatOutputMode = signals::VBatOutputModeValue::On;
    m_candle->GetConfigurator().Apply(configs);

    m_diagTimer.Start();

    m_rainbow = controls::RainbowAnimation(1.0, 0.6, kExternalCount);
    m_strobe = StrobeAnimation(255, 255, 255, 0, 0.25, kExternalCount);
    m_breathe = SingleFadeAnimation(0, 255, 0, 0, 0.8, kExternalCount);
    m_chaser = ColorFlowAnimation(0, 255, 0, 0, 0.7, kExternalCount,
                                  ColorFlowAnimation::Direction::Forward);
    m_larson = LarsonAnimation(0, 255, 0, 0, 0.6, kExternalCount);
}

void DragonCANdle::Periodic()
{
    if (m_candle == nullptr)
        return;

    UpdateDiagnostics();
    UpdateAnimation();
}

// ================= Diagnostic Setters =================

void DragonCANdle::SetAlliance(frc::DriverStation::Alliance alliance)
{
    m_alliance = alliance;
}

void DragonCANdle::SetQuestStatus(bool ok)
{
    m_questOK = ok;
}

void DragonCANdle::SetDataLoggerStatus(bool ok)
{
    m_dataLoggerOK = ok;
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
    auto setLED = [&](int index, int r, int g, int b)
    {
        LEDControl ctrl{r, g, b, index, 1};
        m_candle->SetControl(ctrl);
    };

    // Alliance
    if (m_alliance == frc::DriverStation::Alliance::kBlue)
        setLED(0, 0, 0, 255);
    else
        setLED(0, 255, 0, 0);

    // Quest
    setLED(1, m_questOK ? 0 : 100, m_questOK ? 255 : 0, 0);

    // Limelights (aggregated)
    int llCount = (m_ll1 ? 1 : 0) + (m_ll2 ? 1 : 0) + (m_ll3 ? 1 : 0);
    if (llCount == 3)
        setLED(2, 0, 255, 0);
    else if (llCount == 0)
        setLED(2, 255, 0, 0);
    else
    {
        if (static_cast<int>(m_diagTimer.Get().value()) % 2 == 0)
            setLED(2, 255, 255, 0);
        else
            setLED(2, 0, 0, 255);
    }

    // Data Logger
    setLED(3, m_dataLoggerOK ? 0 : 100, m_dataLoggerOK ? 255 : 0, 0);

    // Sensors
    setLED(4, m_intake ? 255 : 0, m_intake ? 255 : 0, 0);
    setLED(5, m_hood ? 255 : 0, m_hood ? 255 : 0, 0);
    setLED(6, m_turretZero ? 255 : 0, m_turretZero ? 255 : 0, 0);
    setLED(7, m_turretEnd ? 255 : 0, m_turretEnd ? 255 : 0, 0);
}

// ================= Animation =================

void DragonCANdle::SetAnimation(AnimationMode mode)
{
    m_animMode = mode;
}

void DragonCANdle::SetSolidColor(int r, int g, int b)
{
    m_primaryR = r;
    m_primaryG = g;
    m_primaryB = b;
    m_animMode = AnimationMode::Solid;
}

void DragonCANdle::SetAlternatingColors(int r1, int g1, int b1,
                                        int r2, int g2, int b2)
{
    m_primaryR = r1;
    m_primaryG = g1;
    m_primaryB = b1;
    m_secondaryR = r2;
    m_secondaryG = g2;
    m_secondaryB = b2;
    m_animMode = AnimationMode::Alternating;
}

void DragonCANdle::UpdateAnimation()
{
    switch (m_animMode)
    {
    case AnimationMode::Off:
    {
        LEDControl off{0, 0, 0, kExternalStart, kExternalCount};
        m_candle->SetControl(off);
        break;
    }

    case AnimationMode::Solid:
    {
        LEDControl solid{
            m_primaryR, m_primaryG, m_primaryB,
            kExternalStart, kExternalCount};
        m_candle->SetControl(solid);
        break;
    }

    case AnimationMode::Alternating:
    {
        LEDControl first{
            m_primaryR, m_primaryG, m_primaryB,
            kExternalStart, kExternalCount / 2};

        LEDControl second{
            m_secondaryR, m_secondaryG, m_secondaryB,
            kExternalStart + kExternalCount / 2,
            kExternalCount / 2};

        m_candle->SetControl(first);
        m_candle->SetControl(second);
        break;
    }

    case AnimationMode::Rainbow:
    {
        AnimationControl ctrl{&m_rainbow};
        ctrl.StartIndex = kExternalStart;
        ctrl.NumLEDs = kExternalCount;
        m_candle->SetControl(ctrl);
        break;
    }

    case AnimationMode::Breathing:
    {
        m_breathe.Red = m_primaryR;
        m_breathe.Green = m_primaryG;
        m_breathe.Blue = m_primaryB;

        AnimationControl ctrl{&m_breathe};
        ctrl.StartIndex = kExternalStart;
        ctrl.NumLEDs = kExternalCount;
        m_candle->SetControl(ctrl);
        break;
    }

    case AnimationMode::Blinking:
    {
        m_strobe.Red = m_primaryR;
        m_strobe.Green = m_primaryG;
        m_strobe.Blue = m_primaryB;

        AnimationControl ctrl{&m_strobe};
        ctrl.StartIndex = kExternalStart;
        ctrl.NumLEDs = kExternalCount;
        m_candle->SetControl(ctrl);
        break;
    }

    case AnimationMode::Chaser:
    {
        m_chaser.Red = m_primaryR;
        m_chaser.Green = m_primaryG;
        m_chaser.Blue = m_primaryB;

        AnimationControl ctrl{&m_chaser};
        ctrl.StartIndex = kExternalStart;
        ctrl.NumLEDs = kExternalCount;
        m_candle->SetControl(ctrl);
        break;
    }

    case AnimationMode::ClosingIn:
    {
        m_larson.Red = m_primaryR;
        m_larson.Green = m_primaryG;
        m_larson.Blue = m_primaryB;

        AnimationControl ctrl{&m_larson};
        ctrl.StartIndex = kExternalStart;
        ctrl.NumLEDs = kExternalCount;
        m_candle->SetControl(ctrl);
        break;
    }
    }
}
