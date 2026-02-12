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

// Season Specific Includes
#include "configs/MechanismConfigMgr.h"
#include "mechanisms/Intake/Intake.h"
#include "mechanisms/Launcher/Launcher.h"

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

DriverFeedback::DriverFeedback() : IRobotStateChangeSubscriber()
{

    RobotState *RobotStates = RobotState::GetInstance();
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredScoringMode_Int);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DriveToFieldElement_Bool);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DriveStateType_Int);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus_Bool);

    m_LEDStates->SetBlinkingFrequency(m_blinkingFrequency);
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
    if (RobotStateChanges::StateChange::DriveToFieldElement_Bool == change)
        m_isInDriveTo = value;
    else if (RobotStateChanges::StateChange::ClimbModeStatus_Bool == change)
        m_climbMode = value;
}

void DriverFeedback::UpdateFeedback()
{
    UpdateLEDStates();
    UpdateDiagnosticLEDs();
    CheckControllers();
    m_LEDStates->Periodic();
}

void DriverFeedback::UpdateRumble()
{
}

void DriverFeedback::UpdateLEDStates()
{
    DragonCANdle::AnimationMode desiredAnimation = m_prevAnimaiton;
    frc::Color desiredPrimaryColor = m_prevPrimaryColorState;
    frc::Color desiredSecondaryColor = m_prevSecondaryColorState;

    auto currentLauncherState = Launcher::STATE_OFF;
    bool isInLaunchZone = false;
    auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
    if (config != nullptr)
    {
        auto launcherStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::LAUNCHER);
        auto launcherMgr = launcherStateMgr != nullptr ? dynamic_cast<Launcher *>(launcherStateMgr) : nullptr;

        if (launcherMgr != nullptr)
        {
            currentLauncherState = static_cast<Launcher::STATE_NAMES>(launcherMgr->GetCurrentState());
            isInLaunchZone = launcherMgr->IsInLaunchZone();
        }
    }

    if (frc::DriverStation::IsDisabled())
    {
        desiredAnimation = DragonCANdle::AnimationMode::CHASER;
        desiredPrimaryColor = m_isValidAutonFile ? frc::Color::kDarkGreen : frc::Color::kRed;
    }
    else
    {
        if (m_isInDriveTo)
        {
            desiredAnimation = DragonCANdle::AnimationMode::RAINBOW;
        }
        else if (m_climbMode)
        {
            desiredPrimaryColor = frc::Color::kCrimson;

            if (m_isIntakeExtended)
            {
                desiredAnimation = DragonCANdle::AnimationMode::BLINKING;
            }
            else
            {
                desiredAnimation = DragonCANdle::AnimationMode::SOLID;
            }
        }
        else
        {
            switch (currentLauncherState)
            {
            case Launcher::STATE_OFF:
                desiredPrimaryColor = frc::Color::kBlack;
                desiredAnimation = DragonCANdle::AnimationMode::SOLID;
                break;

            case Launcher::STATE_INITIALIZE:
                desiredPrimaryColor = frc::Color::kGreen;
                desiredAnimation = DragonCANdle::AnimationMode::CLOSING_IN;
                break;

            case Launcher::STATE_IDLE:
            case Launcher::STATE_CLIMB:
            case Launcher::STATE_EMPTY_HOPPER:
            case Launcher::STATE_LAUNCHER_TUNING:
            case Launcher::STATE_MANUAL_LAUNCH:
                desiredPrimaryColor = frc::Color::kGreen;
                desiredAnimation = DragonCANdle::AnimationMode::SOLID;
                break;

            case Launcher::STATE_PREPARE_TO_LAUNCH:
                if (!isInLaunchZone)
                {
                    desiredPrimaryColor = frc::Color::kYellow;
                    desiredSecondaryColor = frc::Color::kCyan;
                    desiredAnimation = DragonCANdle::AnimationMode::ALTERNATING;
                }
                else
                {
                    desiredPrimaryColor = frc::Color::kYellow;
                    desiredAnimation = DragonCANdle::AnimationMode::CLOSING_IN;
                }
                break;

            case Launcher::STATE_LAUNCH:
                desiredPrimaryColor = frc::Color::kGreen;
                desiredAnimation = DragonCANdle::AnimationMode::BREATHING;
                break;
            }
        }
    }

    UpdateLEDs(desiredAnimation, desiredPrimaryColor, desiredSecondaryColor);
}

void DriverFeedback::UpdateLEDs(DragonCANdle::AnimationMode desiredAnimation, frc::Color desiredPrimaryColor, frc::Color desiredSecondaryColor)
{
    if (desiredAnimation != m_prevAnimaiton || desiredPrimaryColor != m_prevPrimaryColorState || desiredSecondaryColor != m_prevSecondaryColorState)
    {
        switch (desiredAnimation)
        {
        case DragonCANdle::AnimationMode::SOLID:
            m_LEDStates->SetSolidColor(desiredPrimaryColor);
            m_LEDStates->SetAnimation(desiredAnimation);
            break;

        case DragonCANdle::AnimationMode::ALTERNATING:
            m_LEDStates->SetAlternatingColors(desiredPrimaryColor, desiredSecondaryColor);
            m_LEDStates->SetAnimation(desiredAnimation);
            break;

        case DragonCANdle::AnimationMode::RAINBOW:
            m_LEDStates->SetAnimation(desiredAnimation);
            break;

        case DragonCANdle::AnimationMode::BREATHING:
            m_LEDStates->SetSolidColor(desiredPrimaryColor);
            m_LEDStates->SetAnimation(desiredAnimation);
            break;

        case DragonCANdle::AnimationMode::BLINKING:
            m_LEDStates->SetSolidColor(desiredPrimaryColor);
            m_LEDStates->SetAnimation(desiredAnimation);
            break;

        case DragonCANdle::AnimationMode::CHASER:
            m_LEDStates->SetSolidColor(desiredPrimaryColor);
            m_LEDStates->SetAnimation(desiredAnimation);
            break;

        case DragonCANdle::AnimationMode::CLOSING_IN:
            m_LEDStates->SetSolidColor(desiredPrimaryColor);
            m_LEDStates->SetAnimation(desiredAnimation);
            break;

        default:
            m_LEDStates->TurnOff();
            break;
        }
        m_prevAnimaiton = desiredAnimation;
        m_prevPrimaryColorState = desiredPrimaryColor;
        m_prevSecondaryColorState = desiredSecondaryColor;
    }
}

void DriverFeedback::UpdateDiagnosticLEDs()
{
    bool questStatus = false;
    bool backLeftLL = false;
    bool backRightLL = false;
    bool climberLL = false;

    bool dataLoggerConnected = false;

    bool hoodZeroSwitch = false;
    bool turretZero = false;
    bool turretEnd = false;

    auto dragonVision = DragonVision::GetDragonVision();
    if (dragonVision != nullptr)
    {
        auto limelightRunning = dragonVision->HealthCheckAllLimelights();
        if (limelightRunning.size() == 3)
        {
            backLeftLL = limelightRunning[0];
            backRightLL = limelightRunning[1];
            climberLL = limelightRunning[2];
        }

        questStatus = dragonVision->HealthCheckQuest();

        m_LEDStates->SetQuestStatus(questStatus);
        m_LEDStates->SetLimelightStatuses(backLeftLL, backRightLL, climberLL);
    }

    // Add Data Logger Connection Status dataLoggerConnected = ...
    m_LEDStates->SetDataLoggerStatus(dataLoggerConnected);

    auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
    if (config != nullptr)
    {
        auto launcherStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::LAUNCHER);
        auto intakeStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::INTAKE);

        auto launcherMgr = launcherStateMgr != nullptr ? dynamic_cast<Launcher *>(launcherStateMgr) : nullptr;
        auto intakeMgr = intakeStateMgr != nullptr ? dynamic_cast<Intake *>(intakeStateMgr) : nullptr;

        if (launcherMgr != nullptr)
        {
            hoodZeroSwitch = launcherMgr->GetHood()->GetReverseLimit().GetValue().value;
            turretZero = launcherMgr->GetTurret()->GetReverseLimit().GetValue().value;
            turretEnd = launcherMgr->GetTurret()->GetForwardLimit().GetValue().value;
        }

        if (intakeMgr != nullptr)
        {
            m_isIntakeExtended = intakeMgr->IsIntakeExtended();
        }
        m_LEDStates->SetIntakeSensor(m_isIntakeExtended);
        m_LEDStates->SetHoodSwitch(hoodZeroSwitch);
        m_LEDStates->SetTurretZero(turretZero);
        m_LEDStates->SetTurretEnd(turretEnd);
    }
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