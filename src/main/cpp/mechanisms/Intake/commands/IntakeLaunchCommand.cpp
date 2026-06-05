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
#include "frc/DriverStation.h"
#include "units/angle.h"

// Team 302 Includes
#include "mechanisms/Intake/Intake.h"
#include "mechanisms/Intake/commands/IntakeLaunchCommand.h"

using namespace IntakeCommands;

IntakeLaunchCommand::IntakeLaunchCommand(Intake *intake) : m_intake(intake)
{
    AddRequirements(m_intake);
    SetName("IntakeLaunch");
}

void IntakeLaunchCommand::Initialize()
{
    m_intake->SetCurrentMode(Intake::STATE_LAUNCH);
    m_bumpCounter = 0;
    m_currentExtenderBumpTarget = 0.0;
    m_intake->UpdateTargetIntakePercentOut(m_intakeTarget);
    m_intake->UpdateTargetExtenderPositionDeg(m_currentExtenderTarget);
    m_intake->PublishIntakeMode(false);
}

void IntakeLaunchCommand::Execute()
{
    BumpIntake();
}

void IntakeLaunchCommand::End(bool interrupted)
{
}

bool IntakeLaunchCommand::IsFinished()
{
    // Holds the launch feed on until the binding (WhileTrue / auton primitive) interrupts it.
    return false;
}

void IntakeLaunchCommand::BumpIntake()
{
    // Periodically "bump" the extender during autonomous launching (moved from LaunchState).
    if ((m_bumpCounter > m_counterMax) && frc::DriverStation::IsAutonomous())
    {
        m_currentExtenderBumpTarget = (m_currentExtenderBumpTarget > 0) ? m_extenderTargetDown : m_extenderTargetUp;
        m_intake->UpdateTargetExtenderPercentOut(m_currentExtenderBumpTarget);
        m_bumpCounter = 0;
    }
    else
    {
        m_bumpCounter++;
    }
}