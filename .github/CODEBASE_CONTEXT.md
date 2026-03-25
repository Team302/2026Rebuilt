```markdown
# Team 302 — 2026 Rebuilt Codebase Context

> **Purpose:** Provide an AI assistant (or new team member) with enough context to work effectively in this repository without reading every file.  
> **Season:** FRC 2026 — *Reefscape* → this codebase uses 2026 field elements (Hub, Outpost, Depot, Tower, Trench) and the corresponding game pieces.  
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
- `build302.bat` — standard build
- `build302v.bat` — verbose build
- `clean.bat` — clean build artifacts
- `codegen.bat` / `startCodeGen.bat` — run the Team 302 code generator
- `vendordeps/` — WPILib vendor dependencies (CTRE Phoenix 6, etc.)
- Target platform: **roboRIO 2** running FRC robot code

---

## 3. Robot Entry Point & Lifecycle

**File:** `src/main/cpp/Robot.cpp` / `Robot.h`

### Lifecycle Overview

1. **Initialization:**
   - `InitializeRobot()` — initializes singletons, drivetrain, mechanisms, and state.
   - `InitializeAutonOptions()` — sets up autonomous primitives and dashboard preview.
   - `InitializeDriveteamFeedback()` — initializes HUD, LEDs, and telemetry.
2. **Periodic Methods:**
   - `RobotPeriodic()` — runs the command scheduler, logger, and state updates.
   - `DisabledPeriodic()` — updates disabled state and FMS data.
   - `AutonomousPeriodic()` — executes autonomous primitives.
   - `TeleopPeriodic()` — runs teleop state machines and commands.
3. **Mode Transitions:**
   - `AutonomousInit()` — prepares for autonomous mode.
   - `TeleopInit()` — cancels auton commands and prepares for teleop.
   - `TeleopExit()` — saves vision rewind for post-match analysis.

---

## 4. Core Singletons & Init Order

All of these use the `GetInstance()` singleton pattern:

| Singleton | File | Notes |
|---|---|---|
| `FieldConstants` | `fielddata/FieldConstants` | Must init first; provides field poses |
| `RoboRio` | `utils/RoboRio` | Hardware abstraction |
| `ChassisConfigMgr` | `chassis/ChassisConfigMgr` | Creates swerve drivetrain |
| `RobotContainer` | `RobotContainer` | Wires subsystems and commands |
| `MechanismConfigMgr` | `mechanisms/configs/MechanismConfigMgr` | Selects per-robot config |
| `RobotState` | `state/RobotState` | Pub/sub event bus |
| `PeriodicLooper` | `utils/PeriodicLooper` | Registers StateMgrs per mode |
| `SwerveContainer` | `chassis/SwerveContainer` | Accessed after ChassisConfigMgr |
| `DragonVision` | `vision/DragonVision` | Vision facade |
| `DragonField` | `utils/DragonField` | Field2d visualization |
| `DriverFeedback` | `feedback/DriverFeedback` | Dashboard HUD |
| `DragonDataLoggerMgr` | `utils/logging/signals/DragonDataLoggerMgr` | Data logging |

---

## 5. Chassis & Drivetrain

> ⚠️ **Season-dependent:** Physical swerve constants, module positions, drive/steer gear ratios, and CAN IDs live in `chassis/generated/TunerConstants302.h`. Season-specific drive commands (see below) change every year.

- **Type:** CTRE Swerve (Phoenix 6), field-centric by default
- **Config:** `ChassisConfigMgr` creates a `CommandSwerveDrivetrain` (CTRE-generated) using `TunerConstants302`
- **Subsystem container:** `SwerveContainer` — owns all drive `frc2::Command` instances and button bindings
- **Standard drive commands:**
  - `TeleopFieldDrive` — field-oriented swerve (default teleop)
  - `TeleopRobotDrive` — robot-oriented swerve
  - `TrajectoryDrive` — follows Choreo paths (used by auton primitives)
  - `VisionDrive` — vision-assisted alignment
  - `DriveToPose` — drives to an absolute field pose

### ⚠️ Season-Specific Drive Commands (`chassis/commands/season_specific_commands/`)

These change every season to match the game field. **For 2026:**

| Command | Target |
|---|---|
| `DriveToHub` | Drives to alliance hub |
| `DriveToOutpost` | Drives to outpost scoring position |
| `DriveToDepot` | Drives to trench depot |
| `DriveToTower` | Drives to tower (climb prep) |
| `DriveOverBump` | Navigates over the bump obstacle |
| `DriveAlongNearestWall` | Hugs nearest field wall |
| `SweepBehindBump` | Sweeps around bump during auton |

These are exposed on `SwerveContainer` as `Get<Name>Command()` methods and bound to `TeleopControlFunctions` entries (e.g., `DRIVE_TO_HUB`, `DRIVE_TO_OUTPOST`).

### `ChassisOptionEnums`

- `HeadingOption`: `MAINTAIN`, `SPECIFIED_ANGLE`, `FACE_GAME_PIECE`, `IGNORE`
- `DriveStateType`: `ROBOT_DRIVE`, `FIELD_DRIVE`, `TRAJECTORY_DRIVE`, `HOLD_DRIVE`, `POLAR_DRIVE`, `STOP_DRIVE`, `DRIVE_TO_HUB`, `DRIVE_OVER_BUMP`, `DRIVE_TO_DEPOT`, `DRIVE_TO_OUTPOST`, `DRIVE_TO_TOWER`

---

## 6. Mechanisms

> ⚠️ **Highly season-dependent.** Mechanism types, states, motor assignments, and logic all change between seasons. Everything under `mechanisms/` other than `base/BaseMech` and `controllers/` should be treated as season-specific.

### Pattern

1. All mechanisms extend **`BaseMech`** (provides type identity, network table name, logging).
2. All mechanisms extend **`StateMgr`** (state machine; registered with `PeriodicLooper`).
3. All mechanisms extend **`IRobotStateChangeSubscriber`** (listen for robot-wide events).
4. All mechanisms extend **`DragonDataLogger`** (signal data logging).
5. Mechanisms are instantiated in the per-robot `MechanismConfig` class (e.g., `MechanismConfigCompBot_302`).

### Mechanism Config System

```
MechanismConfigMgr::InitRobot(RobotIdentifier)
  └── Selects concrete MechanismConfig (e.g., MechanismConfigCompBot_302)
      └── MechanismConfigCompBot_302::DefineMechanisms()
          └── Creates Intake, Launcher, Climber with RobotIdentifier
          └── Registers them in m_mechanismMap keyed by MechanismTypes::MECHANISM_TYPE
```

Access a mechanism: `MechanismConfigMgr::GetInstance()->GetCurrentConfig()->GetMechanism(MechanismTypes::INTAKE)`

### Motor Controller Names — `RobotElementNames::MOTOR_CONTROLLER_USAGE` ⚠️

```cpp
INTAKE_INTAKE        // Intake roller
INTAKE_HOPPER        // Hopper inside intake
LAUNCHER_LAUNCHER    // Launcher flywheel
LAUNCHER_HOOD        // Hood angle motor
LAUNCHER_TRANSFER    // Transfer belt
LAUNCHER_TURRET      // Turret rotation
LAUNCHER_INDEXER     // Indexer
CLIMBER_CLIMBER      // Climber arm
```

---

### 2026 Mechanisms

#### `Intake` (`mechanisms/Intake/`)
- **Motors:** `TalonFX` (intake roller) + `TalonFXS` (extender/hopper), `CANdi` sensor
- **States:**
  - `STATE_OFF` — idle, motors stopped
  - `STATE_INTAKE` — run intake roller, extend arm
  - `STATE_EXPEL` — reverse intake
  - `STATE_LAUNCH` — feed ball into launcher
  - `STATE_EMPTY_HOPPER` — purge hopper
  - `STATE_LOAD_HOPPER` — load from ground to hopper
- **Key methods:** `UpdateTargetIntakePercentOut()`, `UpdateTargetExtenderPositionDeg()`, `IsInClimbMode()`
- **Responds to:** `ClimbModeStatus_Bool` (disables intake in climb mode)

#### `Launcher` (`mechanisms/Launcher/`)
- **Motors:** `TalonFX` (flywheel), `TalonFXS` (hood, turret, transfer, indexer), `CANdi` sensors
- **States:**
  - `STATE_OFF`
  - `STATE_INITIALIZE` — boot-up calibration
  - `STATE_IDLE` — turret/hood at safe position
  - `STATE_PREPARE_TO_LAUNCH` — spin up flywheel, aim turret + hood
  - `STATE_LAUNCH` — fire
  - `STATE_EMPTY_HOPPER`
  - `STATE_CLIMB` — stow for climb
  - `STATE_LAUNCHER_TUNING` — PID tuning mode
  - `STATE_MANUAL_LAUNCH` — driver-controlled fire
- **Key methods:** `UpdateTargetLauncherVelocityRPS()`, `UpdateTargetHoodPositionDegreesHood()`, `UpdateTargetTurretPositionDegreesTurret()`
- **Uses:** `RebuiltTargetCalculator` for real-time hood/turret targeting
- **Hood range:** clamped between `m_minHoodAngle` and `m_maxHoodAngle` (91–267°)
- **Turret range:** clamped between `m_minTurretAngle` and `m_maxTurretAngle`

#### `Climber` (`mechanisms/Climber/`)
- **Motors:** `TalonFX` (arm), `CANcoder`, `CANdi` (limit switches), `Solenoid`
- **States:**
  - `STATE_OFF`
  - `STATE_WANT_TO_CLIMB` — climb mode armed
  - `STATE_PREPARE_TO_CLIMB` — drive-to-tower alignment
  - `STATE_L1CLIMB` — Level 1 (low) climb
  - `STATE_L3CLIMB` — Level 3 (high) climb
  - `STATE_EXIT` — abort climb
  - `STATE_AUTON_L1CLIMB` — auton-triggered L1 climb
- **Key methods:** `IsClimbMode()`, `IsAllowedToClimb()`, `IsDriveToDone()`, `IsClimberExtended()`
- **Integrates with:** `DriveToTower` command for autonomous approach

---

## 7. State Machine Architecture

`StateMgr` is the base for all mechanism and subsystem state machines.

```
StateMgr
  └── holds vector of State*
  └── SetCurrentState(int stateId, bool run)
  └── RunCurrentState()     ← called by PeriodicLooper
  └── Cyclic()              ← override in concrete mechanism for always-running tasks

State
  └── Init()
  └── Run()
  └── Exit()
```

`PeriodicLooper` calls `RunCurrentState()` on all registered `StateMgr` instances each period:
- `RegisterAuton(StateMgr*)` — runs only in auton
- `RegisterTeleop(StateMgr*)` — runs only in teleop
- `RegisterAll(StateMgr*)` — runs in all modes
- `RegisterDisabled(StateMgr*)` — runs when disabled

---

## 8. Teleop Control System

> ⚠️ **Season-dependent:** `TeleopControlFunctions` and `TeleopControlMap` must be updated each season to add/remove robot functions and re-map buttons.

- **`TeleopControlFunctions::FUNCTION`** — master enum of all robot actions:
  - General drive: `HOLONOMIC_DRIVE_FORWARD/ROTATE
