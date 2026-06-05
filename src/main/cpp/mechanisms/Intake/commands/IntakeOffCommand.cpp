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
#include "mechanisms/Intake/commands/IntakeOffCommand.h"

using namespace IntakeCommands;

IntakeOffCommand::IntakeOffCommand(Intake *intake) : m_intake(intake)
{
    AddRequirements(m_intake);
    SetName("IntakeOff");
}

void IntakeOffCommand::Initialize()
{
    m_intake->SetCurrentMode(Intake::STATE_OFF);
    m_intake->UpdateTargetIntakePercentOut(m_intakeTarget);
    m_intake->PublishIntakeMode(false);
}

void IntakeOffCommand::Execute()
{
    // First time the robot is enabled, zero/reset the extender (was OffState::Run()).
    if (frc::DriverStation::IsEnabled() && !m_intake->HasBeenEnabled())
    {
        m_intake->SetHasBeenEnabled(true);
        if (frc::DriverStation::IsTeleop())
        {
            m_intake->UpdateTargetExtenderPercentOut(0.0);
            m_intake->GetExtender()->SetPosition(units::angle::turn_t(0.0));
        }
        else
        {
            m_intake->UpdateTargetExtenderPositionDeg(units::angle::turn_t(0.0));
        }
    }
}

void IntakeOffCommand::End(bool interrupted)
{
}

bool IntakeOffCommand::IsFinished()
{
    // Default command - runs until interrupted by another command requiring the Intake.
    return false;
}
