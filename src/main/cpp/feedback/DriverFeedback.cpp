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

#include "frc/DriverStation.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

#include "feedback/DriverFeedback.h"
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

/// Singleton instance pointer.
DriverFeedback *DriverFeedback::m_instance = nullptr;

/// @brief Returns the singleton, creating it on first access.
DriverFeedback *DriverFeedback::GetInstance()
{
    if (DriverFeedback::m_instance == nullptr)
    {
        DriverFeedback::m_instance = new DriverFeedback();
    }
    return DriverFeedback::m_instance;
}

/// @brief Constructor — registers for state changes and caches pointers to avoid
///        repeated singleton lookups, map searches, and dynamic_casts in the periodic loop.
DriverFeedback::DriverFeedback() : IRobotStateChangeSubscriber()
{

    RobotState *RobotStates = RobotState::GetInstance();
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredScoringMode_Int);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DriveToFieldElement_Bool);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DriveStateType_Int);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus_Bool);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::ShiftChangeIn5Seconds_Bool);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::ShiftChangeIn3Seconds_Bool);

    auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
    if (config != nullptr)
    {
        auto launcherStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::LAUNCHER);
        m_launcher = launcherStateMgr != nullptr ? dynamic_cast<Launcher *>(launcherStateMgr) : nullptr;

        auto intakeStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::INTAKE);
        m_intake = intakeStateMgr != nullptr ? dynamic_cast<Intake *>(intakeStateMgr) : nullptr;
    }

    m_dragonVision = DragonVision::GetDragonVision();
    m_controllerTable = nt::NetworkTableInstance::GetDefault().GetTable("XBOX Controller");

    m_LEDStates->SetBlinkingFrequency(m_blinkingFrequency);

    m_LEDStates->SetBrightness(0.75);

    // Pre-build NT key strings once to avoid per-loop heap allocations
    for (int i = 0; i < kMaxJoystickPorts; ++i)
    {
        m_controllerKeys[i] = std::string("Controller") + std::to_string(i);
    }
}

/// @brief Handles integer state changes — scoring mode and drive state type.
void DriverFeedback::NotifyStateUpdate(RobotStateChanges::StateChange change, int value)
{
    if (RobotStateChanges::StateChange::DesiredScoringMode_Int == change)
        m_scoringMode = static_cast<RobotStateChanges::ScoringMode>(value);
    else if (RobotStateChanges::StateChange::DriveStateType_Int == change)
        m_driveStateType = static_cast<ChassisOptionEnums::DriveStateType>(value);
}

/// @brief Handles boolean state changes — drive-to, climb mode, shift warnings.
void DriverFeedback::NotifyStateUpdate(RobotStateChanges::StateChange change, bool value)
{
    if (RobotStateChanges::StateChange::DriveToFieldElement_Bool == change)
        m_isInDriveTo = value;
    else if (RobotStateChanges::StateChange::ClimbModeStatus_Bool == change)
        m_climbMode = value;
    else if (RobotStateChanges::StateChange::ShiftChangeIn5Seconds_Bool == change)
        m_shiftChangeIn5Seconds = value;
    else if (RobotStateChanges::StateChange::ShiftChangeIn3Seconds_Bool == change)
        m_shiftChangeIn3Seconds = value;
}

/// @brief Main periodic entry point. Runs every robot loop (~20 ms).
///        LED state logic runs every loop; diagnostic checks are throttled to every
///        m_diagnosticUpdateInterval loops to reduce CAN bus and network overhead.
void DriverFeedback::UpdateFeedback()
{
    UpdateLEDStates();

    if (DriverStation::IsDisabled())
    {
        if (++m_diagnosticLoopCounter >= m_diagnosticUpdateInterval)
        {
            m_diagnosticLoopCounter = 0;
            UpdateDiagnosticLEDs();
        }
    }

    UpdateRumble();
    CheckControllers();
    m_LEDStates->Periodic();
}

/// @brief Activates or deactivates controller rumble on both controllers.
///        Rumble is ON when a shift change is imminent (within 3 seconds).
///        Only writes to the controller when the rumble state actually changes.
void DriverFeedback::UpdateRumble()
{
    if (m_teleopControl == nullptr)
    {
        return;
    }

    if (m_shiftChangeIn3Seconds != m_prevRumbleState)
    {
        m_prevRumbleState = m_shiftChangeIn3Seconds;
        m_teleopControl->SetRumble(0, m_shiftChangeIn3Seconds, m_shiftChangeIn3Seconds);
        m_teleopControl->SetRumble(1, m_shiftChangeIn3Seconds, m_shiftChangeIn3Seconds);
    }
}

/// @brief Determines the desired LED animation and colors based on robot state priority:
///        1. Disabled → chaser (green if valid auton, red otherwise)
///        2. Drive-to active → rainbow
///        3. Climb mode → crimson solid/blinking depending on intake
///        4. Launcher state → maps each state to a specific animation/color
///        5. Shift-change warning overrides animation to slow blinking
void DriverFeedback::UpdateLEDStates()
{
    DragonCANdle::AnimationMode desiredAnimation = m_prevAnimaiton;
    frc::Color desiredPrimaryColor = m_prevPrimaryColorState;
    frc::Color desiredSecondaryColor = m_prevSecondaryColorState;

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

            if (m_isIntakeIn)
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
            // Only query launcher when we actually need the state (not disabled, not drive-to, not climb)
            auto currentLauncherState = Launcher::STATE_OFF;
            bool isInLaunchZone = false;

            if (m_launcher != nullptr)
            {
                currentLauncherState = static_cast<Launcher::STATE_NAMES>(m_launcher->GetCurrentState());
                isInLaunchZone = m_launcher->IsInLaunchZone();
            }

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

            if (m_shiftChangeIn5Seconds)
            {
                desiredAnimation = DragonCANdle::AnimationMode::BLINKING;
                m_LEDStates->SetBlinkingFrequency(m_shiftBlinkingFrequency);
            }
            else
            {
                m_LEDStates->SetBlinkingFrequency(m_blinkingFrequency);
            }
        }
    }

    UpdateLEDs(desiredAnimation, desiredPrimaryColor, desiredSecondaryColor);
}

/// @brief Sends animation/color changes to the DragonCANdle hardware only when the
///        desired state differs from the previously applied state (avoids redundant CAN writes).
/// @param desiredAnimation   The animation mode to apply.
/// @param desiredPrimaryColor   Primary LED color.
/// @param desiredSecondaryColor Secondary LED color (used by ALTERNATING mode).
void DriverFeedback::UpdateLEDs(DragonCANdle::AnimationMode desiredAnimation, frc::Color desiredPrimaryColor, frc::Color desiredSecondaryColor)
{
    if (desiredAnimation != m_prevAnimaiton || desiredPrimaryColor != m_prevPrimaryColorState || desiredSecondaryColor != m_prevSecondaryColorState)
    {
        switch (desiredAnimation)
        {
        case DragonCANdle::AnimationMode::ALTERNATING:
            m_LEDStates->SetAlternatingColors(desiredPrimaryColor, desiredSecondaryColor);
            m_LEDStates->SetAnimation(desiredAnimation);
            break;

        case DragonCANdle::AnimationMode::RAINBOW:
            m_LEDStates->SetAnimation(desiredAnimation);
            break;

        case DragonCANdle::AnimationMode::SOLID:
        case DragonCANdle::AnimationMode::BREATHING:
        case DragonCANdle::AnimationMode::BLINKING:
        case DragonCANdle::AnimationMode::CHASER:
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

/// @brief Reads hardware diagnostic inputs and pushes them to the DragonCANdle diagnostic LEDs.
///
///        This method is intentionally throttled (called every ~200 ms instead of every 20 ms)
///        because it performs expensive operations:
///          - Limelight health checks (3 network queries)
///          - Quest health check (1 network query)
///          - 3 CAN bus limit-switch reads (hood reverse, turret reverse, turret forward)
///          - 1 CAN bus limit-switch read for intake (via Intake::IsIntakeIn)
void DriverFeedback::UpdateDiagnosticLEDs()
{
    bool questStatus = false;
    bool backLeftLL = false;

    bool dataLoggerConnected = false;

    bool hoodZeroSwitch = false;
    bool turretZero = false;
    bool turretEnd = false;

    if (m_dragonVision != nullptr)
    {
        auto limelightRunning = m_dragonVision->HealthCheckAllLimelights();
        backLeftLL = limelightRunning[DragonVision::kBackLeftLimelightIndex];

        questStatus = m_dragonVision->HealthCheckQuest();

        m_LEDStates->SetQuestStatus(questStatus);
        m_LEDStates->SetLimelightStatuses(backLeftLL);
    }

    // Add Data Logger Connection Status dataLoggerConnected = ...
    m_LEDStates->SetDataLoggerStatus(dataLoggerConnected);

    if (m_launcher != nullptr)
    {
        hoodZeroSwitch = m_launcher->GetHood()->GetReverseLimit(false).GetValue().value;
        turretZero = m_launcher->GetTurret()->GetReverseLimit(false).GetValue().value;
        turretEnd = m_launcher->GetTurret()->GetForwardLimit(false).GetValue().value;
    }

    if (m_intake != nullptr)
    {
        m_isIntakeIn = m_intake->IsIntakeIn();
    }
    m_LEDStates->SetIntakeSensor(m_isIntakeIn);
    m_LEDStates->SetHoodSwitch(hoodZeroSwitch);
    m_LEDStates->SetTurretZero(turretZero);
    m_LEDStates->SetTurretEnd(turretEnd);
}

/// @brief While disabled, publishes Xbox controller connection status to NetworkTables
///        using pre-built key strings. Throttled to every ~25 loops (~500 ms).
void DriverFeedback::CheckControllers()
{
    if (frc::DriverStation::IsDisabled())
    {
        if (++m_controllerLoopCounter < m_controllerUpdateInterval)
        {
            return;
        }
        m_controllerLoopCounter = 0;

        for (int i = 0; i < kMaxJoystickPorts; ++i)
        {
            m_controllerTable->PutBoolean(m_controllerKeys[i], DriverStation::GetJoystickIsXbox(i));
        }
    }
}