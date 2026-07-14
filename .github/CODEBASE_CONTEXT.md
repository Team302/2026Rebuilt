# Team 302 — 2026 Rebuilt Codebase Context

> **Purpose:** Give an AI assistant (or new team member) enough context to work effectively in this repository without reading every file.  
> **Season:** FRC 2026 — *Reefscape* → this codebase uses 2026 field elements (Hub, Outpost, Depot, Tower, Trench) and the corresponding game pieces.  
> **Team:** Lake Orion Robotics, FIRST Team 302  
> **Code generator version:** 20.26.00.xx — many files are auto-generated and carry a generation timestamp header.

---

## Table of Contents
1. [Project Structure](#1-project-structure)
   - 1.5 [Deployment, Build, & Support Files](#15-deployment-build--support-files)
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
│   │       ├── DriveToFuel, AutoDefend
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
│   ├── FieldElementCalculator       # Computes derived field element poses from transforms
│   ├── FieldAprilTagIDs             # April tag ID mapping
│   └── SweepLaneChanger             # Utility for managing sweep lane selection
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

## 1.5. Deployment, Build, & Support Files

```
src/main/
├── deploy/                          # Deployment package files (deployed to RoboRIO)
│   ├── 302/                         # Competition robot (COMP_BOT_302) configs
│   ├── 9999/                        # Test chassis configs (CHASSIS_BOT_9999, etc.)
│   ├── auton/                       # Autonomous routine XML files + DTDs
│   ├── chassis/                     # Chassis tuning and configuration files
│   ├── choreo/                      # Choreo trajectory files for path planning
│   │   └── NewPath.traj             # ✨ Recently added test/development path
│   └── mechanisms/                  # Mechanism-specific configuration files
│
└── thirdparty/                      # Third-party C++ libraries (non-vendor)

tools/                               # Code generation and utility scripts
├── DragonCodeGenerator/             # Team 302 code generator (v20.26.x)
│   └── templates/                   # Code generation templates
├── auton_diagram_gen.py             # Generates autonomous routine diagrams
├── auton_docs_html_gen.py           # Generates HTML documentation from auton XML
└── auton_zone_field_gen.py          # Generates field zone visualization

documents/                           # Design documentation and resources
├── auton/                           # Autonomous strategy documentation (Markdown)
├── AdvantageScopeAssets/            # AdvantageScope dashboard telemetry assets
├── Mechanism/                       # Mechanism design and specification docs
├── swerveprojects/                  # Swerve drive design and simulation files
└── vision/                          # Vision system documentation

elastic/                             # Elastic dashboard configuration
└── elastic-layout.json              # Dashboard layout and telemetry mappings

ctre_sim/                            # CTRE Phoenix simulation library files

vendordeps/                          # WPILib third-party vendor dependencies
├── ChoreoLib2026.json               # Choreo path planning library (2026)
├── Phoenix5-replay-*.json           # CTRE Phoenix 5 replay (legacy)
├── Phoenix6-replay-*.json           # CTRE Phoenix 6 replay + latest
└── WPILibNewCommands.json          # WPILib command framework
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
  └── CyclePrimitives::Init()
  └── PeriodicLooper::AutonRunCurrentState()
  └── DragonVision::StartRewind() (FMS + first-time latch)

AutonomousPeriodic()
  └── CyclePrimitives::Run()
  └── PeriodicLooper::AutonRunCurrentState()

TeleopInit()
  └── PeriodicLooper::TeleopRunCurrentState()
  └── CommandScheduler::CancelAll()
  └── DragonVision::StartRewind() (FMS + latch)

TeleopPeriodic()
  └── PeriodicLooper::TeleopRunCurrentState()

TeleopExit()
  └── DragonVision::SaveRewind(165s) + StartRewind() (FMS only)

TestInit()
  └── CommandScheduler::CancelAll()
```

---

## 4. Core Singletons & Init Order

All of these use the `GetInstance()` singleton pattern:

| Singleton | File | Notes |
|---|---|---|
| `FieldConstants` | `fielddata/FieldConstants` | Must init first; provides field poses |
| `RoboRio` | `utils/RoboRio` | Hardware abstraction |
| `ChassisConfigMgr` | `chassis/ChassisConfigMgr` | Creates swerve drivetrain |
| `RobotContainer` | `RobotContainer` | Heap-allocated with `new`; wires swerve + vision estimator |
| `MechanismConfigMgr` | `mechanisms/configs/MechanismConfigMgr` | `InitRobot(RobotIdentifier)` selects per-robot config |
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
- **Key methods:** `UpdateTargetLauncherVelocityLauncher()`, `UpdateTargetHoodPositionDegreesHood()`, `UpdateTargetTurretPositionDegreesTurret()`
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
  - General drive: `HOLONOMIC_DRIVE_FORWARD/ROTATE/STRAFE`, `ROBOT_ORIENTED_DRIVE`, `SLOW_MODE`, `RESET_POSITION`
  - Season-specific drive: `DRIVE_TO_HUB`, `DRIVE_TO_OUTPOST`, `DRIVE_TO_TOWER`, `DRIVE_OVER_BUMP`, `DRIVE_TO_DEPOT`, `DRIVE_ALONG_NEAREST_WALL`, `SWEEP_BEHIND_BUMP`
  - Intake: `INTAKE`, `EXPEL`, `INTAKE_OUT/IN`, `EXTENDER_MODIFIER`
  - Launcher: `LAUNCH`, `LAUNCH_OVERRIDE`, `MANUAL_LAUNCH`, `TURRET_ENABLE`, `LAUNCHER_OFF`
  - Climber: `CLIMB_MODE`, `LEVEL1_CLIMB`, `LEVEL3_CLIMB`, `ALIGN_TO_LEFT/RIGHT_TOWER`
  - SysID: `SYSID_QUASISTATIC/DYNAMIC_FORWARD/REVERSE`

- **`TeleopControl`** — singleton; call `GetAxisValue(FUNCTION)` or `IsButtonPressed(FUNCTION)` throughout the codebase.

---

## 9. Autonomous System

> ⚠️ **Season-dependent:** Auton paths (Choreo `.traj` files in `deploy/`), zone managers, and auton scripts change every season.

### Framework

```
CyclePrimitives
  └── Reads auton CSV/script (selected on dashboard via AutonPreviewer)
  └── PrimitiveParser → PrimitiveParams list
  └── PrimitiveFactory → instantiates IPrimitive per step
  └── Runs each primitive sequentially; calls Init() → Run() each period

AutonPreviewer
  └── Watches dashboard selection in disabled
  └── CheckCurrentAuton() previews path on DragonField
```

### Primitive Types (`PrimitiveEnums.h`)

```
DO_NOTHING
HOLD_POSITION
TRAJECTORY_DRIVE       ← follows a Choreo path via TrajectoryDrive command
RESET_POSITION
VISION_ALIGN
DO_NOTHING_MECHANISMS
```

### Zone Managers ⚠️ Season-dependent

Used to determine robot behavior based on field position:
- `AllianceZoneManager` — is robot in its alliance zone?
- `NeutralZoneManager` — is robot in neutral zone?
- `DeadZoneManager` — restricted movement zone
- `BumpZoneManager` — near the bump obstacle

`RebuiltTargetCalculator` uses `AllianceZoneManager` to decide whether to target hub vs. passing targets.

### Choreo Paths

Paths are `.traj` files loaded from `src/main/deploy/`. The `AutonUtils` helper loads them into `TrajectoryDrive`. Loading in the `Robot` constructor (warm-load) avoids loop overruns during the first autonomous period.

---

## 10. RobotState & Pub/Sub Event System

**File:** `state/RobotState.h`, `state/RobotStateChanges.h`

`RobotState` is a singleton that acts as an event bus. Subsystems **publish** state changes and other subsystems **subscribe** to them.

### Published Events (`RobotStateChanges::StateChange`)

| Event | Type | Description |
|---|---|---|
| `DesiredScoringMode_Int` | int | Current scoring mode |
| `IsLaunching_Bool` | bool | Flywheel spinning up / firing |
| `ClimbModeStatus_Bool` | bool | Climb mode armed |
| `AllowedToClimbStatus_Bool` | bool | Robot in position to climb |
| `ChassisTipStatus_Int` | int | Tip detection state |
| `DriveAssistMode_Int` | int | Drive assist on/off |
| `GameState_Int` | int | Auton / Teleop / Disabled |
| `ChassisPose_Pose2D` | Pose2d | Robot's estimated field pose |
| `DriveToFieldElement_Bool` | bool | Auto-drive to element active |
| `DriveToFinished_Bool` | bool | Auto-drive completed |
| `DriveStateType_Int` | int | Current `ChassisOptionEnums::DriveStateType` |
| `HubActive_Bool` | bool | Hub is the active target |
| `StartLaunching_Bool` | bool | Trigger launcher sequence |
| `TurretEnabled_Bool` | bool | Turret tracking on/off |

### Usage

```cpp
// Subscribe
RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::ClimbModeStatus_Bool);

// Receive (implement IRobotStateChangeSubscriber)
void MyClass::NotifyStateUpdate(RobotStateChanges::StateChange change, bool value) { ... }

// Publish
RobotState::GetInstance()->PublishStateChange(RobotStateChanges::IsLaunching_Bool, true);
```

---

## 11. Vision System

- **`DragonVision`** — singleton facade; use `DragonVision::GetDragonVision()`
  - `StartRewind()` / `SaveRewind(seconds)` — recording/playback via Quest
- **`DragonLimelight`** — Limelight 3/4 wrapper for AprilTag pose estimates
- **`DragonQuest`** — Meta Quest (Questnav) 6-DOF pose wrapper
- **`DragonVisionPoseEstimator`** — fuses Limelight + Quest measurements into `CommandSwerveDrivetrain` pose estimator; created in `RobotContainer`
- Vision rewind starts at match enable (FMS-attached) and saves on `TeleopExit` for post-match review

---

## 12. Field Data & Target Calculation

> ⚠️ **Season-dependent:** `FieldConstants`, `FieldAprilTagIDs`, and `RebuiltTargetCalculator` are 2026-specific.

### `FieldConstants` — 2026 Field Elements

Singleton; call `GetFieldElementPose(FIELD_ELEMENT)` or `GetFieldElementPose2d(FIELD_ELEMENT)`.

**2026 elements defined:**
- Blue/Red: `HUB_ALLIANCE_CENTER`, `HUB_OUTPOST_CENTER`, `TOWER_CENTER`, `OUTPOST_CENTER`
- Trenches: `TRENCH_NEUTRAL_DEPOT`, `TRENCH_ALLIANCE_DEPOT`, `TRENCH_NEUTRAL_OUTPOST`, `TRENCH_ALLIANCE_OUTPOST`
- Calculated: `HUB_CENTER`, `TOWER_DEPOT/OUTPOST_STICK`, `DEPOT_NEUTRAL/LEFT/RIGHT_SIDE`, `DEPOT/OUTPOST_PASSING_TARGET`

### `FieldElementCalculator` ⚠️

- Computes **derived** field element poses (those not directly from the field layout) by applying `Transform3d` offsets to reference poses.
- Called by `FieldConstants` during initialization via `CalcPositionsForField()`.
- Key offsets (2026-specific, will change each season):
  - Hub center: X = –23.5 in from hub alliance center
  - Tower stick offsets: left/right/center at ±19.5 in Y, 45 in X
  - Depot offsets: center (27 in X, 87.31 in Y), left (13.5 in X, 108.31 in Y)
- **Must update** when field geometry changes between seasons.

### `RebuiltTargetCalculator` ⚠️

- Extends `TargetCalculator`
- Provides real-time hood angle + turret target for the `Launcher`
- Target selection:
  - **In alliance zone** → target `HUB_CENTER`
  - **Outside alliance zone** → target nearest passing target (`OUTPOST_PASSING_TARGET` or `DEPOT_PASSING_TARGET`)
- Launcher offset from robot center: X = –3.333 in, Y = +4.604 in
- Supports driver X/Y offset adjustment via gamepad

---

## 13. Logging & Telemetry

- **`Logger`** (`utils/logging/debug/Logger`) — debug logging to NetworkTables; only runs when FMS is not attached
- **`DragonDataLoggerMgr`** — high-frequency signal data logger; warm-loaded in `Robot` constructor; called from `RobotPeriodic` (skips when disabled)
- **`DragonDataLogger`** — interface implemented by each mechanism; `DataLog(uint64_t timestamp)` is the per-signal callback

---

## 14. Feedback & Dashboard

- **`DriverFeedback`** — singleton; `UpdateFeedback()` called every `RobotPeriodic`; drives LEDs, controller rumble, and SmartDashboard HUD
- **`DragonField`** — wraps WPILib `Field2d`; `UpdateRobotPosition(pose)` called every `RobotPeriodic`; `UpdateEnabledStates()` in disabled
- **`GameDataHelper`** — parses FMS game-specific message string (e.g., alliance station, game data)
- **`AutonPreviewer`** — draws expected auton trajectory on field widget in Shuffleboard/Elastic during disabled

Dashboard layout: `elastic/elastic-layout.json`

---

## 15. Utilities

| Class | Purpose |
|---|---|
| `PeriodicLooper` | Throttled StateMgr runner per robot mode |
| `AngleUtils` | Angle wrap/normalize helpers |
| `ConversionUtils` | Unit conversion helpers |
| `PoseUtils` | Pose2d math helpers |
| `InterpolateUtils` | 1D/2D interpolation for lookup tables |
| `FMSData` | Alliance color caching (`UpdateAllianceColor()`) |
| `DragonPower` | Compressor/pneumatics control |
| `NetworkTableReader` | Generic NT value reader |
| `RoboRio` | RoboRIO hardware info singleton |
| `HardwareIDValidation` | CAN ID validation at startup |

---

## 16. Robot Identifiers & Multi-Robot Support

**File:** `RobotIdentifier.h`

The team number programmed on the RoboRIO is used as the robot identifier:

```cpp
enum class RobotIdentifier {
    CHASSIS_BOT_9999 = 9999,   // Chassis test bot
    CHASSIS_BOT_9998 = 9998,   // Chassis test bot 2
    CHASSIS_BOT_9997 = 9997,   // Chassis test bot 3
    COMP_BOT_302     = 302,    // Competition robot
    SIM_BOT_0        = 0,      // Simulator
};
```

`MechanismConfigMgr::InitRobot(id)` selects the appropriate `MechanismConfig` subclass. Currently only `MechanismConfigCompBot_302` is implemented for 302.

Each mechanism constructor receives the `RobotIdentifier` and internally calls `CreateCompBot302()` / `InitializeCompBot302()` to configure hardware.

---

## 17. Key Design Patterns

| Pattern | Where Used |
|---|---|
| **Singleton** | Nearly all managers: `ChassisConfigMgr`, `RobotState`, `PeriodicLooper`, `DragonVision`, `DriverFeedback`, etc. |
| **State Machine** | `StateMgr` + `State` — used by all three mechanisms and `SwerveContainer` |
| **Pub/Sub Event Bus** | `RobotState` + `IRobotStateChangeSubscriber` — decouples mechanisms from each other |
| **Command-Based (WPILib)** | All drive commands extend `frc2::Command`; scheduled by `CommandScheduler` |
| **Factory Pattern** | `PrimitiveFactory` creates auton primitives from parsed params |
| **Config/Strategy** | `MechanismConfig` / `MechanismConfigMgr` swaps full robot hardware config by ID |
| **Observer** | `DragonDataLogger` interface — mechanisms publish signals to logger |

---

## 18. Season-Dependent Change Checklist

When porting this framework to a **new season**, the following must be updated:

### 🔴 Must Change Every Season

- [ ] `fielddata/FieldConstants.h/.cpp` — all `FIELD_ELEMENT` enums and pose values
- [ ] `fielddata/FieldElementCalculator.h/.cpp` — derived pose transforms (offsets from reference poses)
- [ ] `fielddata/FieldAprilTagIDs.h` — April tag ID assignments
- [ ] `auton/` zone managers — `AllianceZoneManager`, `NeutralZoneManager`, `BumpZoneManager`, `DeadZoneManager`
- [ ] `chassis/commands/season_specific_commands/` — all game-specific drive commands (DriveToHub, etc.)
- [ ] `chassis/ChassisOptionEnums.h` — `DriveStateType` enum entries for new drive commands
- [ ] `mechanisms/MechanismTypes.h` — add/remove mechanism type enums
- [ ] `mechanisms/configs/RobotElementNames.h` — add/remove motor controller usage enums
- [ ] `mechanisms/configs/MechanismConfigCompBot_302` — create/delete mechanisms
- [ ] `mechanisms/Intake/`, `mechanisms/Launcher/`, `mechanisms/Climber/` — full replacement with new game mechanisms and states
- [ ] `teleopcontrol/TeleopControlFunctions.h` — add/remove robot functions
- [ ] `teleopcontrol/TeleopControlMap.h` — remap functions to controller buttons
- [ ] `utils/RebuiltTargetCalculator` — new target geometry for the season
- [ ] `src/main/deploy/` — new Choreo `.traj` auton path files
- [ ] `elastic/elastic-layout.json` — dashboard layout

### 🟡 Usually Changes Each Season

- [ ] `chassis/generated/TunerConstants302.h` — swerve module constants (if new chassis)
- [ ] `state/RobotStateChanges.h` — add/remove state change events
- [ ] `auton/PrimitiveEnums.h` — add new auton primitive types if needed
- [ ] `RobotIdentifier.h` — add new chassis/comp bot IDs

### 🟢 Generally Stable (Framework)

- `Robot.cpp` lifecycle structure
- `state/StateMgr`, `state/State`, `state/RobotState` event bus
- `utils/PeriodicLooper`
- `chassis/ChassisConfigMgr`, `chassis/SwerveContainer` (structure)
- `mechanisms/base/BaseMech`
- `mechanisms/controllers/ControlData`, `ControlModes`
- `teleopcontrol/TeleopControl`
- `vision/DragonVision`, `DragonVisionPoseEstimator`
- `utils/logging/` infrastructure
- Build system (`build.gradle`, `gradlew`)
