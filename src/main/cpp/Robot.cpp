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

/*
  Robot.cpp — Developer Notes

  Overview
  - Implements the top-level robot entry point and lifecycle handlers (init/periodic for Disabled/Auton/Teleop/Test).
  - Wires up RobotContainer, subsystems, command scheduling, auton selection, and drive-team feedback.
  - Lightweight: avoid adding heavy computation in periodic methods; use PeriodicLooper for mode-specific loops.

  Key responsibilities
  - Global initialization (logger, configs, chassis, RobotContainer, RobotState).
  - Manage autorun/auton selection via CyclePrimitives and AutonPreviewer.
  - Run CommandScheduler and per-period updates (RobotState, vision heartbeats, drive-team HUD).
  - Provide extension points for new subsystems and auton sequences.

  Lifecycle summary (where to look)
  - Robot::Robot()          : startup wiring and warm-load resources (e.g., trajectories).
  - Robot::RobotPeriodic()  : runs CommandScheduler, logging (guarded by FMS), RobotState, vision heartbeats, and UpdateDriveTeamFeedback().
  - Robot::AutonomousInit() : raise thread priority, initialize cycle primitives, start auton PeriodicLooper state.
  - Robot::AutonomousPeriodic(): run cycle primitives and auton PeriodicLooper.
  - Robot::TeleopInit()/TeleopPeriodic(): start/run teleop PeriodicLooper state.
  - Robot::TestInit()      : cancel commands at test startup.

  Initialization order (high level)
  1. Logger selections -> dashboard
  2. FieldConstants, RoboRio singletons
  3. ChassisConfigMgr -> CreateDrivetrain()
  4. Instantiate RobotContainer (binds subsystems/commands)
  5. MechanismConfigMgr and RobotState initialization
  6. Create CyclePrimitives and AutonPreviewer for auton selection
  7. Initialize drive-team feedback (DragonField, DriverFeedback)

  Key collaborators / singletons
  - RobotContainer, ChassisConfigMgr, MechanismConfigMgr
  - RobotState, PeriodicLooper
  - CyclePrimitives, AutonPreviewer
  - DriverFeedback, DragonField
  - DragonQuest / DragonVision (vision heartbeats)
  - DragonDataLoggerMgr (optional data logger)

  Extension points
  - Add subsystems and bind them in RobotContainer.
  - Add new CyclePrimitives for auton routines and register them with AutonPreviewer.
  - Use PeriodicLooper for periodic publishing rather than adding heavy work to RobotPeriodic.

  Debugging & testing tips
  - Guard periodic debug logging with !frc::DriverStation::IsFMSAttached() to avoid FMS log flooding.
  - Warm-load trajectories in Robot constructor to avoid first-run stalls.
  - Test thread priority changes on non-competition hardware first.
  - Use PeriodicLooper to throttle heavy publishers and avoid overruns.

  TODO / Caveats
  - Consider re-enabling DragonDataLoggerMgr->PeriodicDataLog() with rate limiting.
  - Move DragonField initialization into a dedicated drive-team feedback manager.
  - Add more granular RobotState transition logging for diagnostics.

  Search tokens in this file
  - InitializeRobot(), InitializeAutonOptions(), InitializeDriveteamFeedback(), UpdateDriveTeamFeedback()
*/

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include "RobotContainer.h"
#include "RobotIdentifier.h"
#include "auton/AutonPreviewer.h"
#include "auton/CyclePrimitives.h"
// #include "auton/drivePrimitives/AutonUtils.h"
#include "chassis/ChassisConfigMgr.h"
#include "configs/MechanismConfigMgr.h"
#include "feedback/DriverFeedback.h"
#include "fielddata/FieldConstants.h"
#include "frc/DriverStation.h"
#include "frc/RobotController.h"
#include "frc/Threads.h"
#include "state/RobotState.h"
#include "utils/DragonField.h"
#include "utils/PeriodicLooper.h"
#include "utils/RoboRio.h"
#include "utils/logging/debug/Logger.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"
#include "vision/DragonVision.h"

#include "auton/NeutralZoneManager.h"

/// @brief Constructor for the Robot class.
/// Initializes all core subsystems, auton options, and drive-team feedback in sequence.
/// Also warm-loads the data logger to avoid first-run delays.
Robot::Robot()
{
    Logger::GetLogger()->PutLoggingSelectionsOnDashboard();

    InitializeRobot();
    InitializeAutonOptions();
    InitializeDriveteamFeedback();

    m_datalogger = DragonDataLoggerMgr::GetInstance();
    auto vision = DragonVision::GetDragonVision();

    if (frc::DriverStation::IsFMSAttached())
    {
        if (vision != nullptr)
        {
            vision->StartRewind();
        }
    }
    // auto path = AutonUtils::GetTrajectoryFromPathFile("BlueLeftInside_I"); // load choreo library so we don't get loop overruns during autonperiodic
}
/// @brief Called periodically while the robot is running, regardless of mode.
/// Runs the CommandScheduler, manages logging (guarded by FMS attachment),
/// updates RobotState, and refreshes drive-team feedback (vision, field position, HUD).
void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();

    isFMSAttached = frc::DriverStation::IsFMSAttached();
    if (!isFMSAttached)
    {
        Logger::GetLogger()->PeriodicLog();
    }

    if (m_datalogger != nullptr && !frc::DriverStation::IsDisabled())
    {
        m_datalogger->PeriodicDataLog();
    }

    if (m_robotState != nullptr)
    {
        m_robotState->Run();
    }

    UpdateDriveTeamFeedback();
}

/// @brief Called once while the robot is disabled.
void Robot::DisabledInit()
{
    if (frc::DriverStation::IsFMSAttached())
    {
        auto vision = DragonVision::GetDragonVision();
        if (vision != nullptr)
        {
            vision->SaveRewind(165.0); // Save full buffer (max 165 seconds)
            vision->StartRewind();
        }
    }
}

/// @brief Called periodically while the robot is disabled.
/// Updates the PeriodicLooper's disabled state for any mode-specific behavior.
void Robot::DisabledPeriodic()
{
    PeriodicLooper::GetInstance()->DisabledRunCurrentState();

    m_field->UpdateEnabledStates();
}

/// @brief Called once when autonomous mode begins.
/// Elevates thread priority to reduce jitter, initializes cycle primitives,
/// starts Rewind recording if FMS-attached (first time only), and transitions PeriodicLooper to autonomous state.
void Robot::AutonomousInit()
{
    frc::SetCurrentThreadPriority(true, 15);

    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Init();
    }
    PeriodicLooper::GetInstance()->AutonRunCurrentState();
}

/// @brief Called periodically while in autonomous mode.
/// Executes the current cycle primitives routine and updates the PeriodicLooper's autonomous state.
void Robot::AutonomousPeriodic()
{
    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Run();
    }
    PeriodicLooper::GetInstance()->AutonRunCurrentState();
}

/// @brief Called once when teleop mode begins.
/// Transitions PeriodicLooper to teleop state and cancels any outstanding commands.
void Robot::TeleopInit()
{
    PeriodicLooper::GetInstance()->TeleopRunCurrentState();
    frc2::CommandScheduler::GetInstance().CancelAll();
}

/// @brief Called periodically while in teleop mode.
/// Updates the PeriodicLooper's teleop state for mode-specific behavior.
void Robot::TeleopPeriodic() { PeriodicLooper::GetInstance()->TeleopRunCurrentState(); }

/// @brief Called once when test mode begins.
/// Cancels any outstanding commands to reset the robot state for testing.
void Robot::TestInit()
{
    frc2::CommandScheduler::GetInstance().CancelAll();
}

/// @brief Initializes all core robot subsystems in the correct order.
/// Sets up singletons (FieldConstants, RoboRio), creates the drivetrain via ChassisConfigMgr,
/// instantiates RobotContainer to bind commands and subsystems, configures mechanisms,
/// and initializes RobotState.
void Robot::InitializeRobot()
{
    FieldConstants::GetInstance();
    RoboRio::GetInstance();
    auto chassisConfig = ChassisConfigMgr::GetInstance();
    chassisConfig->CreateDrivetrain();

    new RobotContainer(); // instantiate RobotContainer to setup commands and subsystems

    int32_t teamNumber = frc::RobotController::GetTeamNumber();
    MechanismConfigMgr::GetInstance()->InitRobot((RobotIdentifier)teamNumber);

    m_robotState = RobotState::GetInstance();
    m_robotState->Init();
}

/// @brief Initializes autonomous options and routines.
/// Creates CyclePrimitives for managing auton sequences and AutonPreviewer for auton selection logic.
void Robot::InitializeAutonOptions()
{
    m_cyclePrims = new CyclePrimitives(); // intialize auton selections
    m_previewer = new AutonPreviewer(m_cyclePrims);
}
/// @brief Initializes drive-team feedback systems.
/// Sets up DragonField for field visualization and robot position tracking.
/// @todo Move DragonField initialization into a dedicated drive-team feedback manager class.
void Robot::InitializeDriveteamFeedback()
{
    m_field = DragonField::GetInstance(); // TODO: move to drive team feedback
}

/// @brief Updates all drive-team feedback and auton selection information.
/// Checks current auton selection via previewer, updates the field with the robot's current pose,
/// and refreshes driver feedback (dashboard HUD, camera feeds, etc.).
void Robot::UpdateDriveTeamFeedback()
{
    if (m_previewer != nullptr)
    {
        m_previewer->CheckCurrentAuton();
    }

    auto chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();
    if (m_field != nullptr && chassis != nullptr)
    {
        m_field->UpdateRobotPosition(chassis->GetPose());
    }
    auto feedback = DriverFeedback::GetInstance();
    if (feedback != nullptr)
    {
        feedback->UpdateFeedback();
    }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
