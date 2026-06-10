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

// FRC Includes
#include "units/angle.h"

// Team 302 Includes
#include "mechanisms/Intake/Intake.h"
#include "mechanisms/Intake/commands/IntakeLoadHopperCommand.h"

using namespace IntakeCommands;

IntakeLoadHopperCommand::IntakeLoadHopperCommand(Intake *intake) : m_intake(intake)
{
	AddRequirements(m_intake);
	SetName("IntakeLoadHopper");
}

void IntakeLoadHopperCommand::Initialize()
{
	m_intake->SetCurrentState(Intake::STATE_LOAD_HOPPER);
	m_intake->UpdateTargetIntakePercentOut(m_intakeTarget);
	m_intake->UpdateTargetExtenderPositionDeg(m_extenderTarget);
}

void IntakeLoadHopperCommand::Execute()
{
}

void IntakeLoadHopperCommand::End(bool interrupted)
{
}

bool IntakeLoadHopperCommand::IsFinished()
{
	// Holds the hopper-load on until the binding (WhileTrue / auton primitive) interrupts it.
	return false;
}
