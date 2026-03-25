```markdown
# Team 302 — 2026 Rebuilt Codebase Context

> **Purpose:** Provide an AI assistant (or new team member) with sufficient context to work effectively in this repository without needing to read every file.  
> **Season:** FRC 2026 — *Reefscape* → this codebase is tailored for the 2026 game field (Hub, Outpost, Depot, Tower, Trench) and game pieces.  
> **Team:** Lake Orion Robotics, FIRST Team 302  
> **Code generator version:** 20.26.00.xx — many files are auto-generated and carry a generation timestamp header.

---

## Table of Contents
1. [Project Structure](#1-project-structure)
2. [Build System](#2-build-system)
3. [Robot Entry Point & Lifecycle](#3-robot-entry-point--lifecycle)
4. [Core Singletons & Init Order](#4-core-singletons--init-order)
5. [Chassis & Drivetrain](#5-chassis--drivetrain)
6. [Mechanisms](#6-mechanisms)
7. [State Machine Architecture](#7-state-machine-architecture)
8. [Teleop Control System](#8-teleop-control-system)
9. [Autonomous System](#9-autonomous-system)
10. [RobotState & Pub/Sub Event System](#10-robotstate--pubsub-event-system)
11. [Vision System](#11-vision-system)
12. [Field Data & Target Calculation](#12-field-data--target-calculation)
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
- **Build scripts:**
  - `build302.bat` — standard build
  - `build302v.bat` — verbose build
  - `clean.bat` — clean build artifacts
  - `codegen.bat` / `startCodeGen.bat` — run the Team 302 code generator
- **Dependencies:**
  - WPILib
  - CTRE Phoenix 6
- **Target platform:** roboRIO 2 running FRC robot code

---

## 3. Robot Entry Point & Lifecycle

**File:** `src/main/cpp/Robot.cpp` / `Robot.h`

### Lifecycle Overview

1. **Robot Initialization (`Robot::Robot`)**
   - Initialize singletons: `FieldConstants`, `RoboRio`, `ChassisConfigMgr`, `RobotContainer`, `MechanismConfigMgr`, `RobotState`, etc.
   - Warm-load `DragonDataLoggerMgr` and auton paths.
   - Initialize autonomous options and driver feedback.

2. **Periodic Methods**
   - `RobotPeriodic`: Runs `CommandScheduler`, logging, `RobotState`, and driver feedback updates.
   - `DisabledPeriodic`: Updates disabled state and FMS data.
   - `AutonomousPeriodic`: Runs autonomous primitives and state machines.
   - `TeleopPeriodic`: Runs teleop state machines and commands.

3. **Mode Transitions**
   - `AutonomousInit`: Initializes autonomous primitives and sets thread priority.
   - `TeleopInit`: Cancels all commands and reinitializes teleop state machines.
   - `TeleopExit`: Saves vision rewind data for post-match review.

---

## 4. Core Singletons & Init Order

All singletons use the `GetInstance()` pattern. Initialization order is critical:

| Singleton | File | Notes |
|---|---|---|
| `FieldConstants` | `fielddata/FieldConstants` | Must initialize first for field poses. |
| `RoboRio` | `utils/RoboRio` | Hardware abstraction. |
| `ChassisConfigMgr` | `chassis/ChassisConfigMgr` | Creates the swerve drivetrain. |
| `RobotContainer` | `RobotContainer` | Wires subsystems and commands. |
| `MechanismConfigMgr` | `mechanisms/configs/MechanismConfigMgr` | Selects per-robot mechanism config. |
| `RobotState` | `state/RobotState` | Central state manager and pub/sub broker. |
| `PeriodicLooper` | `utils/PeriodicLooper` | Registers state machines for periodic updates. |
| `DragonVision` | `vision/DragonVision` | Vision system facade. |
| `DragonField` | `utils/DragonField` | Field2d visualization. |
| `DriverFeedback` | `feedback/DriverFeedback` | Dashboard HUD and driver feedback. |
| `DragonDataLoggerMgr` | `utils/logging/DragonDataLoggerMgr` | High-frequency signal data logger. |

---

## 5. Chassis & Drivetrain

> ⚠️ **Season-dependent:** Physical swerve constants, module positions, drive/steer gear ratios, and CAN IDs live in `chassis/generated/TunerConstants302.h`. Season-specific drive commands change every year.

- **Type:** CTRE Swerve (Phoenix 6), field-centric by default.
- **Subsystem container:** `SwerveContainer` — owns all drive `frc2::Command` instances and button bindings.

### Standard Drive Commands
- `TeleopFieldDrive` — field-oriented swerve (default teleop).
- `TeleopRobotDrive` — robot-oriented swerve.
- `TrajectoryDrive` — follows Choreo paths (used by auton primitives).
- `VisionDrive` — vision-assisted alignment.
- `DriveToPose` — drives to an absolute field pose.

### ⚠️ Season-Specific Drive Commands
Located in `chassis/commands/season_specific_commands/`. For 2026:
- `DriveToHub`, `DriveToOutpost`, `DriveToDepot`, `DriveToTower`, `DriveOverBump`, etc.

---

## 6. Mechanisms

> ⚠️ **Highly season-dependent.** Mechanism types, states, motor assignments, and logic all change between seasons. Everything under `mechanisms/` other than `base/BaseMech` and `controllers/` should be treated as season-specific.

### Mechanism Overview
- All mechanisms extend `BaseMech` (type identity, logging).
- All mechanisms implement `StateMgr` (state machine) and `IRobotStateChangeSubscriber` (event listener).
- Mechanisms are instantiated in `MechanismConfig` subclasses (e.g., `MechanismConfigCompBot_302`).

### 2026 Mechanisms
- **`Intake`**: Handles intake roller, hopper, and extender.
- **`Launcher`**: Manages flywheel, hood, turret, transfer, and indexer.
- **`Climber`**: Controls L1/L3 climbing mechanisms.

---

## 7. State Machine Architecture

- **`StateMgr`**: Base class for all mechanism state machines.
- **`State`**: Represents individual states with `Init()`, `Run()`, and `Exit()` methods.
- **`PeriodicLooper`**: Registers and runs `StateMgr` instances for each robot mode (auton, teleop, disabled).

---

## 8. Teleop Control System

> ⚠️ **Season-dependent:** `TeleopControlFunctions` and `TeleopControlMap` must be updated each season to add/remove robot functions and re-map buttons.

- **`TeleopControlFunctions`**: Enum of all robot actions (e.g., `DRIVE_TO_HUB`, `LAUNCH`, `CLIMB_MODE`).
- **`TeleopControl`**: Singleton for reading gamepad inputs.

---

## 9. Autonomous System

> ⚠️ **Season-dependent:** Auton paths, zone managers, and auton scripts change every season.

- **`CyclePrimitives`**: Executes auton sequences defined in CSV files.
- **`AutonPreviewer`**: Displays selected auton path on the dashboard.
- **Zone Managers**: Determine robot behavior based on field position (e.g., `AllianceZoneManager`).

---

## 10. RobotState & Pub/Sub Event System

- **`RobotState`**: Singleton event bus for publishing and subscribing to robot-wide state changes.
- **Published Events**: Include `ClimbModeStatus_Bool`, `IsLaunching_Bool`, `DriveStateType_Int`, etc.

---

## 11. Vision System

- **`DragonVision`**: Facade for Limelight and Meta Quest vision systems.
- **`DragonVisionPoseEstimator`**: Fuses vision data into odometry.

---

## 12. Field Data & Target Calculation

> ⚠️ **Season-dependent:** `FieldConstants`, `FieldAprilTagIDs`, and `RebuiltTargetCalculator` are 2026-specific.

- **`FieldConstants`**: Provides 2026 field element poses.
- **`RebuiltTargetCalculator`**: Calculates launcher hood and turret targets based on field position.

---

## 13. Logging & Telemetry

- **`Logger`**: Debug logging to NetworkTables.
- **`DragonDataLoggerMgr`**: High-frequency signal data logging.

---

## 14. Feedback & Dashboard

- **`DriverFeedback`**: Manages LEDs, rumble, and dashboard HUD.
- **`DragonField`**: Field2d visualization for robot and field elements.

---

## 15. Utilities

| Class | Purpose |
|---|---|
| `PeriodicLooper` | Throttled StateMgr runner per robot mode. |
| `AngleUtils` | Angle wrap/normalize helpers. |
| `ConversionUtils` | Unit conversion helpers. |
| `PoseUtils` | Pose2d math helpers. |
| `FMSData` | Alliance color caching. |
| `RoboRio` | RoboRIO hardware info singleton. |

---

## 16. Robot Identifiers & Multi-Robot Support

**File:** `RobotIdentifier.h`

Defines robot IDs for multi-robot support. Example:
- `COMP_BOT_302`: Competition robot.
- `CHASSIS_BOT_9999`: Chassis test bot.

---

## 17. Key Design Patterns

| Pattern | Where Used |
|---|---|
| **Singleton** | Managers like `ChassisConfigMgr`, `RobotState`, etc. |
| **State Machine** | `StateMgr` + `State` for mechanisms and subsystems. |
| **Pub/Sub Event Bus** | `RobotState` for decoupled communication. |
| **Command-Based** | WPILib `frc2::Command` for drive commands. |
| **Factory Pattern** | `PrimitiveFactory` for auton primitives. |
| **Config/Strategy** | `MechanismConfig` for robot-specific hardware. |

---

## 18. Season-Dependent Change Checklist

### 🔴 Must Change Every Season
- [ ] Update `fielddata/FieldConstants` and `FieldAprilTagIDs`.
- [ ] Update auton zone managers and scripts.
- [ ] Update season-specific drive commands and enums.
- [ ] Replace mechanisms and states in `mechanisms/`.
- [ ] Update `TeleopControlFunctions` and `TeleopControlMap`.
- [ ] Update `RebuiltTargetCalculator` for new targets.
- [ ] Add new auton paths to `src/main/deploy/`.

### 🟡 Usually Changes Each Season
- [ ] Update swerve module constants in `TunerConstants302.h`.
- [ ] Add/remove state change events in `RobotStateChanges.h`.

### 🟢 Generally Stable
- Core framework files (`Robot.cpp`, `StateMgr`, `RobotState`, etc.).
- Utilities (`PeriodicLooper`, `RoboRio`, `DragonVision`, etc.).
- Logging infrastructure.
```
