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
#include "frc/RobotBase.h"
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

void DragonCANdle::Initialize(int canID, int stripSize, const std::string &canBus, double brightness, StripTypeValue type)
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
    configs.LED.BrightnessScalar = brightness;
    configs.LED.StripType = type;
    configs.CANdleFeatures.VBatOutputMode = signals::VBatOutputModeValue::On;

    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int i = 0; i < m_attemptsToConfigCandle; ++i)
    {
        status = m_candle->GetConfigurator().Apply(configs, units::time::second_t(0.25));
        if (status.IsOK())
            break;
    }
    if (!status.IsOK())
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, "m_candle", "m_candle Status", status.GetName());

    m_externalCount = stripSize;

    m_diagTimer.Start();
}

void DragonCANdle::Periodic()
{
    if (m_candle == nullptr)
        return;

    UpdateAnimation();

    if (frc::RobotBase::IsSimulation() && m_candle != nullptr)
    {
        // Update simulated sensor states for testing
        auto &simState = m_candle->GetSimState();
        simState.SetSupplyVoltage(12_V);
        simState.GetLastStatusCode();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonCANdle", "AppliedControl", std::string(m_candle->GetAppliedControl().get()->GetName()));
    }
    else if (frc::DriverStation::IsDisabled())
    {
        UpdateDiagnostics();
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

void DragonCANdle::TurnOff()
{
    m_animMode = AnimationMode::OFF;
}

void DragonCANdle::UpdateAnimation()
{
    using RGBWColor = signals::RGBWColor;

    // Only proceed if the animation mode has changed
    if (m_animMode == m_prevAnimMode &&
        m_primaryColor == m_prevPrimaryColor &&
        m_secondaryColor == m_prevSecondaryColor &&
        m_animMode != AnimationMode::ALTERNATING) // Always update alternating (it has its own timer)
    {
        return;
    }

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
        if (m_alternatingTimer >= 2 * m_alternatingPeriod)
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

    // Cache the current state for next frame
    m_prevAnimMode = m_animMode;
    m_prevPrimaryColor = m_primaryColor;
    m_prevSecondaryColor = m_secondaryColor;
}

// ================= Diagnostics =================

void DragonCANdle::UpdateDiagnostics()
{
    using RGBWColor = signals::RGBWColor;

    // Set individual onboard LEDs for diagnostics
    // Only send CAN commands if state changed (reduces bus traffic significantly)

    // Alliance
    if (m_firstRun)
    {
        frc::Color allianceColor = (m_alliance == frc::DriverStation::Alliance::kRed) ? frc::Color::kRed : frc::Color::kBlue;
        frc::Color questColor    = m_questOK      ? frc::Color::kGreen  : frc::Color::kRed;
        frc::Color llColor       = m_limeLight    ? frc::Color::kGreen  : frc::Color::kRed;
        frc::Color loggerColor   = m_dataLoggerOK ? frc::Color::kGreen  : frc::Color::kRed;
        frc::Color intakeColor   = m_intake       ? frc::Color::kYellow : frc::Color::kBlack;
        frc::Color hoodColor     = m_hood         ? frc::Color::kBlack  : frc::Color::kYellow;
        frc::Color turretZColor  = m_turretZero   ? frc::Color::kBlack  : frc::Color::kYellow;
        frc::Color turretEColor  = m_turretEnd    ? frc::Color::kBlack  : frc::Color::kYellow;

        // Onboard LEDs (0–7)
        m_candle->SetControl(controls::SolidColor{0, 0}.WithColor(RGBWColor{allianceColor}));
        m_candle->SetControl(controls::SolidColor{1, 1}.WithColor(RGBWColor{questColor}));
        m_candle->SetControl(controls::SolidColor{2, 2}.WithColor(RGBWColor{llColor}));
        m_candle->SetControl(controls::SolidColor{3, 3}.WithColor(RGBWColor{loggerColor}));
        m_candle->SetControl(controls::SolidColor{4, 4}.WithColor(RGBWColor{intakeColor}));
        m_candle->SetControl(controls::SolidColor{5, 5}.WithColor(RGBWColor{hoodColor}));
        m_candle->SetControl(controls::SolidColor{6, 6}.WithColor(RGBWColor{turretZColor}));
        m_candle->SetControl(controls::SolidColor{7, 7}.WithColor(RGBWColor{turretEColor}));

        // External LEDs — 8 channels × 2 LEDs each (indices 9–16, 8 is MIA)
        m_candle->SetControl(controls::SolidColor{ 9,  9}.WithColor(RGBWColor{allianceColor}));
        m_candle->SetControl(controls::SolidColor{10, 10}.WithColor(RGBWColor{questColor}));
        m_candle->SetControl(controls::SolidColor{11, 11}.WithColor(RGBWColor{llColor}));
        m_candle->SetControl(controls::SolidColor{12, 12}.WithColor(RGBWColor{loggerColor}));
        m_candle->SetControl(controls::SolidColor{13, 13}.WithColor(RGBWColor{intakeColor}));
        m_candle->SetControl(controls::SolidColor{14, 14}.WithColor(RGBWColor{hoodColor}));
        m_candle->SetControl(controls::SolidColor{15, 15}.WithColor(RGBWColor{turretZColor}));
        m_candle->SetControl(controls::SolidColor{16, 16}.WithColor(RGBWColor{turretEColor}));

        m_firstRun = false;
    }
    else
    {
        // Alliance color
        if (m_alliance != m_prevAlliance)
        {
            frc::Color c = (m_alliance == frc::DriverStation::Alliance::kRed) ? frc::Color::kRed : frc::Color::kBlue;
            m_candle->SetControl(controls::SolidColor{0, 0}.WithColor(RGBWColor{c}));
            m_candle->SetControl(controls::SolidColor{ 9,  9}.WithColor(RGBWColor{c}));
            m_prevAlliance = m_alliance;
        }

        // Quest
        if (m_questOK != m_prevQuestOK)
        {
            frc::Color c = m_questOK ? frc::Color::kGreen : frc::Color::kRed;
            m_candle->SetControl(controls::SolidColor{ 1,  1}.WithColor(RGBWColor{c}));
            m_candle->SetControl(controls::SolidColor{10, 10}.WithColor(RGBWColor{c}));
            m_prevQuestOK = m_questOK;
        }

        // Limelight
        if (m_limeLight != m_prevLimeLight)
        {
            frc::Color c = m_limeLight ? frc::Color::kGreen : frc::Color::kRed;
            m_candle->SetControl(controls::SolidColor{ 2,  2}.WithColor(RGBWColor{c}));
            m_candle->SetControl(controls::SolidColor{11, 11}.WithColor(RGBWColor{c}));
            m_prevLimeLight = m_limeLight;
        }

        // Data Logger
        if (m_dataLoggerOK != m_prevDataLoggerOK)
        {
            frc::Color c = m_dataLoggerOK ? frc::Color::kGreen : frc::Color::kRed;
            m_candle->SetControl(controls::SolidColor{ 3,  3}.WithColor(RGBWColor{c}));
            m_candle->SetControl(controls::SolidColor{12, 12}.WithColor(RGBWColor{c}));
            m_prevDataLoggerOK = m_dataLoggerOK;
        }

        // Intake sensor
        if (m_intake != m_prevIntake)
        {
            frc::Color c = m_intake ? frc::Color::kYellow : frc::Color::kBlack;
            m_candle->SetControl(controls::SolidColor{ 4,  4}.WithColor(RGBWColor{c}));
            m_candle->SetControl(controls::SolidColor{13, 13}.WithColor(RGBWColor{c}));
            m_prevIntake = m_intake;
        }

        // Hood switch
        if (m_hood != m_prevHood)
        {
            frc::Color c = m_hood ? frc::Color::kBlack : frc::Color::kYellow;
            m_candle->SetControl(controls::SolidColor{ 5,  5}.WithColor(RGBWColor{c}));
            m_candle->SetControl(controls::SolidColor{14, 14}.WithColor(RGBWColor{c}));
            m_prevHood = m_hood;
        }

        // Turret zero
        if (m_turretZero != m_prevTurretZero)
        {
            frc::Color c = m_turretZero ? frc::Color::kBlack : frc::Color::kYellow;
            m_candle->SetControl(controls::SolidColor{ 6,  6}.WithColor(RGBWColor{c}));
            m_candle->SetControl(controls::SolidColor{15, 15}.WithColor(RGBWColor{c}));
            m_prevTurretZero = m_turretZero;
        }

        // Turret end
        if (m_turretEnd != m_prevTurretEnd)
        {
            frc::Color c = m_turretEnd ? frc::Color::kBlack : frc::Color::kYellow;
            m_candle->SetControl(controls::SolidColor{ 7,  7}.WithColor(RGBWColor{c}));
            m_candle->SetControl(controls::SolidColor{16, 16}.WithColor(RGBWColor{c}));
            m_prevTurretEnd = m_turretEnd;
        }
    }
}