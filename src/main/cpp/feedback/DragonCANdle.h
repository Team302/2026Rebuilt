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

#include <ctre/phoenix6/CANdle.hpp>

#include <frc/DriverStation.h>
#include <frc/Timer.h>

class DragonCANdle
{
public:
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

	void Initialize(int canID, const std::string &canBus = "rio", signals::StripTypeValue type = signals::StripTypeValue::GRB);
	void Periodic();

	// ===== Diagnostic Inputs =====
	void SetAlliance(frc::DriverStation::Alliance alliance);
	void SetQuestStatus(bool ok);
	void SetDataLoggerStatus(bool ok);
	void SetLimelightStatuses(bool ll1, bool ll2, bool ll3);
	void SetIntakeSensor(bool triggered);
	void SetHoodSwitch(bool triggered);
	void SetTurretZero(bool triggered);
	void SetTurretEnd(bool triggered);

	// ===== Animation Control =====
	void SetAnimation(AnimationMode mode);
	void SetSolidColor(int r, int g, int b);
	void SetAlternatingColors(int r1, int g1, int b1,
							  int r2, int g2, int b2);

private:
	DragonCANdle() = default;

	void UpdateDiagnostics();
	void UpdateAnimation();

	static DragonCANdle *m_instance;

	ctre::phoenix6::hardware::CANdle *m_candle{nullptr};

	// LED Layout
	static constexpr int kOnboardStart = 0;
	static constexpr int kOnboardCount = 8;
	static constexpr int kExternalStart = 8;
	static constexpr int kExternalCount = 120;

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

	int m_primaryR{0}, m_primaryG{255}, m_primaryB{0};
	int m_secondaryR{0}, m_secondaryG{0}, m_secondaryB{0};

	// Animations
	ctre::phoenix6::controls::RainbowAnimation m_rainbow;
	ctre::phoenix6::controls::StrobeAnimation m_strobe;
	ctre::phoenix6::controls::SingleFadeAnimation m_breathe;
	ctre::phoenix6::controls::ColorFlowAnimation m_chaser;
	ctre::phoenix6::controls::LarsonAnimation m_larson;
};
