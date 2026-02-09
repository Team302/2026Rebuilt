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

#include <string>
#include <vector>

#include <ctre/phoenix6/CANdle.hpp>

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/AddressableLED.h>
#include <frc/simulation/AddressableLEDSim.h>
#include <frc/util/Color.h>

class DragonCANdle
{
public:
	using RGBWColor = ctre::phoenix6::signals::RGBWColor;
	using StripTypeValue = ctre::phoenix6::signals::StripTypeValue;

	enum class AnimationMode
	{
		Off,
		Solid,
		Alternating,
		Rainbow,
		Breathing,
		Blinking,
		Chaser,
		ClosingIn
	};

	static DragonCANdle *GetInstance();

	void Initialize(int canID, int stripSize, const std::string &canBus = "rio", StripTypeValue type = StripTypeValue::GRB);
	void Periodic();

	// ===== Animation Control =====
	void SetAnimation(AnimationMode mode);
	void SetSolidColor(const frc::Color &color);
	void SetAlternatingColors(const frc::Color &color1, const frc::Color &color2);
	void SetBrightness(double brightness);
	void TurnOff();
	void SetBreathingFrequency(units::frequency::hertz_t frequency) { m_breathingFrequency = frequency; };
	void SetBlinkingFrequency(units::frequency::hertz_t frequency) { m_blinkingFrequency = frequency; };

	// ===== Diagnostic Inputs =====
	void SetAlliance(frc::DriverStation::Alliance alliance);
	void SetQuestStatus(bool connected);
	void SetDataLoggerStatus(bool connected);
	void SetLimelightStatuses(bool ll1, bool ll2, bool ll3);
	void SetIntakeSensor(bool triggered);
	void SetHoodSwitch(bool triggered);
	void SetTurretZero(bool triggered);
	void SetTurretEnd(bool triggered);

private:
	DragonCANdle() = default;
	DragonCANdle(const DragonCANdle &) = delete;
	DragonCANdle &operator=(const DragonCANdle &) = delete;

	void UpdateDiagnostics();
	void UpdateAnimation();

	static DragonCANdle *m_instance;

	ctre::phoenix6::hardware::CANdle *m_candle{nullptr};

	// LED Layout
	static constexpr int m_onboardStart = 0;
	static constexpr int m_onboardCount = 8;
	static constexpr int m_externalStart = 8;
	int m_externalCount;

	// Diagnostic State
	frc::DriverStation::Alliance m_alliance{frc::DriverStation::Alliance::kBlue};
	bool m_questOK{false};
	bool m_dataLoggerOK{false};
	bool m_ll1{false};
	bool m_ll2{false};
	bool m_ll3{false};
	bool m_intake{false};
	bool m_hood{false};
	bool m_turretZero{false};
	bool m_turretEnd{false};

	frc::Timer m_diagTimer;

	// Animation State
	AnimationMode m_animMode{AnimationMode::Off};

	frc::Color m_primaryColor{frc::Color::kGreen};
	frc::Color m_secondaryColor{frc::Color::kBlack};
	double m_brightness{1.0};
	units::frequency::hertz_t m_blinkingFrequency{0.5_Hz};
	units::frequency::hertz_t m_breathingFrequency{1_Hz};
};
