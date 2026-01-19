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

#include <frc/DriverStation.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "feedback/DriverFeedback.h"
#include "frc/DriverStation.h"
#include "state/IRobotStateChangeSubscriber.h"
#include "state/RobotState.h"
#include "state/RobotStateChanges.h"
#include "teleopcontrol/TeleopControl.h"
#include "utils/logging/debug/Logger.h"
#include "vision/DragonQuest.h"
#include "vision/DragonVision.h"

using frc::DriverStation;

DriverFeedback *DriverFeedback::m_instance = nullptr;

DriverFeedback *DriverFeedback::GetInstance()
{
    if (DriverFeedback::m_instance == nullptr)
    {
        DriverFeedback::m_instance = new DriverFeedback();
    }
    return DriverFeedback::m_instance;
}

void DriverFeedback::UpdateFeedback()
{
    UpdateLEDStates();
    UpdateDiagnosticLEDs();
    CheckControllers();
    m_LEDStates->commitLedData();
}

void DriverFeedback::UpdateRumble()
{
    auto controller = TeleopControl::GetInstance();
    if (!frc::DriverStation::IsTeleop() || m_climbMode)
    {
        controller->SetRumble(0, false, false);
        controller->SetRumble(1, false, false);
    }
    else
    {
        controller->SetRumble(1, m_driveToIsDone, m_driveToIsDone);
    }
}

void DriverFeedback::UpdateLEDStates()
{
    if (frc::DriverStation::IsDisabled())
    {
        m_LEDStates->SetChaserPattern(frc::Color::kAqua);
    }
    else
    {
    }
    if (m_oldState != m_currentState)
    {
        m_LEDStates->ResetVariables();
    }
}

void DriverFeedback::UpdateDiagnosticLEDs()
{
    bool questStatus = false;
    bool ll1Status = false;

    auto dragonVision = DragonVision::GetDragonVision();
    if (dragonVision != nullptr)
    {
        auto limelightRunning = dragonVision->HealthCheckAllLimelights();
        ll1Status = limelightRunning.empty() ? false : limelightRunning[0];
        questStatus = dragonVision->HealthCheckQuest();
    }

    m_LEDStates->DiagnosticPattern(FMSData::GetAllianceColor(), false, false, false, questStatus, ll1Status);
}

void DriverFeedback::ResetRequests(void)
{
}

DriverFeedback::DriverFeedback() : IRobotStateChangeSubscriber()
{

    RobotState *RobotStates = RobotState::GetInstance();
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredScoringMode_Int);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DriveToFieldElementIsDone_Bool);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DriveStateType_Int);
}
void DriverFeedback::NotifyStateUpdate(RobotStateChanges::StateChange change, int value)
{
    if (RobotStateChanges::StateChange::DesiredScoringMode_Int == change)
        m_scoringMode = static_cast<RobotStateChanges::ScoringMode>(value);

    else if (RobotStateChanges::StateChange::DriveStateType_Int == change)
        m_driveStateType = static_cast<ChassisOptionEnums::DriveStateType>(value);
}

void DriverFeedback::NotifyStateUpdate(RobotStateChanges::StateChange change, bool value)
{
    if (RobotStateChanges::StateChange::DriveToFieldElementIsDone_Bool == change)
        m_driveToIsDone = value;
}

void DriverFeedback::CheckControllers()
{
    if (m_controllerCounter == 0)
    {
        auto table = nt::NetworkTableInstance::GetDefault().GetTable("XBOX Controller");
        for (auto i = 0; i < DriverStation::kJoystickPorts; ++i)
        {
            table.get()->PutBoolean(std::string("Controller") + std::to_string(i), DriverStation::GetJoystickIsXbox(i));
        }
    }
    m_controllerCounter++;
    if (m_controllerCounter > 25)
    {
        m_controllerCounter = 0;
    }
}