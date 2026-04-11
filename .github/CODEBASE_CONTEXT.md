```markdown
# Team 302 — 2026 Rebuilt Codebase Context

> **Purpose:** Give an AI assistant (or new team member) enough context to work effectively in this repository without reading every file.  
> **Season:** FRC 2026 — *Reefscape* → this codebase uses 2026 field elements (Hub, Outpost, Depot, Tower, Trench) and the corresponding game pieces.  
> **Team:** Lake Orion Robotics, FIRST Team 302  
> **Code generator version:** 20.26.00.xx — many files are auto-generated and carry a generation timestamp header.

---

## Table of Contents
1. [Project Structure](#1-project-structure)
2. [Build System](#2-build-system)
3. [Robot Entry Point & Lifecycle](#3-robot-entry-point--lifecycle)
4. [Core Singletons & Init Order](#4-core-singletons--init-order)
5. [Chassis & Drivetrain](#5-chassis--drivetrain) ⚠️ **Season-dependent**
6. [Mechanisms](#6-mechanisms) ⚠️ **Highly season-dependent**
7. [State Machine Architecture](#7-state-machine-architecture)
8. [Teleop Control System](#8-teleop-control-system) ⚠️ **Season-dependent**
9. [Autonomous System](#9-autonomous-system) ⚠️ **Season-dependent**
10. [RobotState & Pub/Sub Event System](#10-robotstate--pubsub-event-system)
11. [Vision System](#11-vision-system)
12. [Field Data & Target Calculation](#12-field-data--target-calculation) ⚠️ **Season-dependent**
13. [Logging & Telemetry](#13-logging--telemetry)
14. [Feedback & Dashboard](#14-feedback--dashboard)
15. [Utilities](#15-utilities)
16. [Robot Identifiers & Multi-Robot Support](#16-robot-identifiers--multi-robot-support)
17. [Key Design Patterns](#17-key-design-patterns)
18. [Season-Dependent Change Checklist](#18-season-dependent-change-checklist)

---

## 1. Project Structure

```
src/main/cpp/
├── Robot.cpp / Robot.h              # Top-level robot entry point
├── RobotContainer.cpp / .h          # Wires subsystems + commands
├── RobotIdentifier.h                # Enum for physical robot IDs
│
├── auton/                           # Autonomous framework ⚠️ SEASON-DEPENDENT
│   ├── CyclePrimitives              # Runs the auton sequence
│   ├── AutonPreviewer               # Dashboard auton selection
│   ├── AutonSelector                # Reads auton CSV/params
│   ├── PrimitiveEnums.h             # Available auton primitives
│   ├── PrimitiveFactory/Parser/Params
│   ├── drivePrimitives/             # Trajectory, reset, vision-align primitives
│   └── Zone managers (Alliance, Bump, Dead, Neutral) ⚠️ SEASON-DEPENDENT
│
├── chassis/                         # Drivetrain ⚠️ SEASON-DEPENDENT
│   ├── ChassisConfigMgr             # Creates + owns the swerve drivetrain
│   ├── SwerveContainer              # Subsystem container; binds drive commands
│   ├── ChassisOptionEnums.h         # HeadingOption, DriveStateType, etc.
│   ├── commands/                    # Drive commands
│   │   ├── TeleopFieldDrive / TeleopRobotDrive
│   │   ├── TrajectoryDrive / VisionDrive / DriveToPose
│   │   └── season_specific_commands/ ⚠️ ALL SEASON-DEPENDENT
│   │       ├── DriveToHub, DriveToOutpost, DriveToDepot
│   │       ├── DriveToTower, DriveOverBump
│   │       ├── DriveAlongNearestWall, SweepBehindBump
│   └── generated/                   # CTRE Swerve generated code (TunerConstants302, CommandSwerveDrivetrain)
│
├── mechanisms/                      # Robot game mechanisms ⚠️ HIGHLY SEASON-DEPENDENT
│   ├── MechanismTypes.h             # Enum: INTAKE, LAUNCHER, CLIMBER
│   ├── base/BaseMech                # Abstract base for all mechanisms
│   ├── configs/
│   │   ├── MechanismConfigMgr       # Singleton; selects config by RobotIdentifier
│   │   ├── MechanismConfig          # Abstract base config
│   │   ├── MechanismConfigCompBot_302  # Concrete 2026 comp-bot config ⚠️
│   │   └── RobotElementNames.h      # Motor/sensor usage enums ⚠️
│   ├── controllers/ControlData      # PIDF + control-mode configuration
│   ├── Intake/                      # ⚠️ 2026-specific: intake + hopper + extender
│   ├── Launcher/                    # ⚠️ 2026-specific: launcher + hood + turret + transfer + indexer
│   └── Climber/                     # ⚠️ 2026-specific: L1 / L3 climb
│
├── state/                           # Robot-wide state & pub/sub event bus
│   ├── RobotState                   # Central state manager + pub/sub broker
│   ├── RobotStateChanges.h          # Enum of all publishable state-change events
│   ├── StateMgr                     # Base class for mechanism state machines
│   ├── State                        # Individual state base class
│   └── IRobotStateChangeSubscriber  # Interface for event listeners
│
├── teleopcontrol/                   # Gamepad mappings ⚠️ SEASON-DEPENDENT
│   ├── TeleopControl                # Reads gamepad axes/buttons
│   ├── TeleopControlFunctions.h     # Enum of all robot functions
│   └── TeleopControlMap             # Maps functions → buttons/axes
│
├── fielddata/                       # Field geometry ⚠️ SEASON-DEPENDENT
│   ├── FieldConstants               # Singleton; 2026 field element poses
│   └── FieldAprilTagIDs             # April tag ID mapping
│
├── vision/
│   ├── DragonVision                 # Facade over Limelight + Quest
│   ├── DragonLimelight              # Limelight wrapper
│   ├── DragonQuest                  # Meta Quest (Questnav) wrapper
│   └── DragonVisionPoseEstimator    # Fuses vision into odometry
│
├── feedback/
│   ├── DriverFeedback               # Dashboard HUD, LEDs, rumble
│   └── GameDataHelper               # Parses FMS game-specific messages
│
├── utils/
│   ├── PeriodicLooper               # Throttled periodic runner for StateMgrs
│   ├── RebuiltTargetCalculator      # ⚠️ 2026-specific: launcher target calc
│   ├── TargetCalculator             # Base target calculator
│   ├── DragonField                  # Field2d visualization + robot pose
│   ├── RoboRio                      # RoboRIO hardware singleton
│   ├── FMSData                      # Alliance color helper
│   └── logging/                     # DragonDataLoggerMgr, DragonDataLogger
│
└── healthtests/                     # Hardware health checks
```

---

## 2. Build System

- **Gradle + GradleRIO** (`build.gradle`, `gradlew.bat`)
- `build302.bat` — standard build
- `build302v.bat` — verbose build
- `clean.bat` — clean build artifacts
- `codegen.bat` / `startCodeGen.bat` — run the Team 302 code generator
- `vendordeps/` — WPILib vendor dependencies (CTRE Phoenix 6, etc.)
- Target platform: **roboRIO 2** running FRC robot code

---

## 3. Robot Entry Point & Lifecycle

**File:** `src/main/cpp/Robot.cpp` / `Robot.h`

```
Robot constructor
  └── InitializeRobot()        → singletons, drivetrain, RobotContainer, mechanisms, RobotState
  └── InitializeAutonOptions() → CyclePrimitives, AutonPreviewer
  └── InitializeDriveteamFeedback() → DragonField, SwerveChassis ref, DriverFeedback, GameDataHelper
  └── DragonDataLoggerMgr warm-load

RobotPeriodic()
  └── CommandScheduler::Run()
  └── Logger::PeriodicLog() (non-FMS only)
  └── DragonDataLoggerMgr::PeriodicDataLog() (when enabled, not disabled)
  └── RobotState::Run()
  └── UpdateDriveTeamFeedback()

DisabledPeriodic()
  └── PeriodicLooper::DisabledRunCurrentState()
  └── DragonField::UpdateEnabledStates()
  └── FMSData::UpdateAllianceColor()

AutonomousInit()
  └── Set thread priority (real-time, 15)
  └── CyclePrimitives::StartAutonomous()
```

---

## 4. Core Singletons & Init Order

- **RobotState**: Central state manager for the robot.
- **DriverFeedback**: Manages feedback to the drive team.
- **GameDataHelper**: Parses FMS game-specific messages.
- **DragonDataLoggerMgr**: Handles logging of telemetry data.

---

## 5. Chassis & Drivetrain ⚠️ **Season-dependent**

- **Swerve Drivetrain**: Utilizes a swerve drive configuration.
- **ChassisConfigMgr**: Responsible for creating and managing the swerve drivetrain.
- **Commands**: Includes commands for teleop driving, trajectory following, and vision alignment.

---

## 6. Mechanisms ⚠️ **Highly season-dependent**

- **Intake**: Mechanism for collecting game pieces.
- **Launcher**: Mechanism for launching game pieces.
- **Climber**: Mechanism for climbing during the endgame.

---

## 7. State Machine Architecture

- **StateMgr**: Base class for all mechanism state machines.
- **State**: Represents an individual state within a state machine.
- **IRobotStateChangeSubscriber**: Interface for classes that need to respond to state changes.

---

## 8. Teleop Control System ⚠️ **Season-dependent**

- **TeleopControl**: Reads inputs from gamepads and maps them to robot functions.
- **TeleopControlMap**: Maps functions to specific buttons and axes on the gamepad.

---

## 9. Autonomous System ⚠️ **Season-dependent**

- **CyclePrimitives**: Manages the sequence of autonomous commands.
- **AutonPreviewer**: Provides a dashboard interface for selecting autonomous routines.
- **AutonSelector**: Reads autonomous parameters from configuration files.

---

## 10. RobotState & Pub/Sub Event System

- **RobotState**: Centralized state management for the robot.
- **Pub/Sub**: Event-driven architecture for handling state changes and notifications.

---

## 11. Vision System

- **DragonVision**: Interface for vision processing.
- **DragonLimelight**: Wrapper for Limelight camera integration.
- **DragonQuest**: Wrapper for Quest navigation system.

---

## 12. Field Data & Target Calculation ⚠️ **Season-dependent**

- **FieldConstants**: Contains constants for field element positions.
- **RebuiltTargetCalculator**: Calculates target positions based on field data.

---

## 13. Logging & Telemetry

- **DragonDataLogger**: Manages logging of telemetry data.
- **Telemetry**: Provides real-time feedback to the drive team.

---

## 14. Feedback & Dashboard

- **DriverFeedback**: Manages feedback to the drive team through HUD and LEDs.
- **GameDataHelper**: Parses FMS game-specific messages for display.

---

## 15. Utilities

- **PeriodicLooper**: Manages periodic tasks for state machines.
- **DragonField**: Visual representation of the field and robot position.

---

## 16. Robot Identifiers & Multi-Robot Support

- **RobotIdentifier**: Enum for identifying different robot configurations.

---

## 17. Key Design Patterns

- **Singleton**: Used for managing shared resources like RobotState and DriverFeedback.
- **Factory**: Used for creating instances of commands and mechanisms.

---

## 18. Season-Dependent Change Checklist

- [ ] Update autonomous routines in `src/main/deploy/auton/`.
- [ ] Verify mechanism configurations in `src/main/mechanisms/configs/`.
- [ ] Ensure teleop controls are mapped correctly in `src/main/teleopcontrol/`.
- [ ] Validate field data in `src/main/fielddata/`.
- [ ] Test all commands in `src/main/chassis/commands/` for season-specific behavior.
- [ ] Review and update logging and telemetry in `src/main/utils/logging/`.
- [ ] Confirm vision system integration in `src/main/vision/`.
- [ ] Check for any deprecated methods or classes from previous seasons.
```
