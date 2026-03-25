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
├── auton/                           # Autonomous framework (season-dependent)
│   ├── CyclePrimitives              # Runs the auton sequence
│   ├── AutonPreviewer               # Dashboard auton selection
│   ├── AutonSelector                # Reads auton CSV/params
│   ├── PrimitiveEnums.h             # Available auton primitives
│   ├── PrimitiveFactory/Parser/Params
│   ├── drivePrimitives/             # Trajectory, reset, vision-align primitives
│   └── Zone managers (Alliance, Bump, Dead, Neutral) (season-dependent)
│
├── chassis/                         # Drivetrain (season-dependent)
│   ├── ChassisConfigMgr             # Creates + owns the swerve drivetrain
│   ├── SwerveContainer              # Subsystem container; binds drive commands
│   ├── ChassisOptionEnums.h         # HeadingOption, DriveStateType, etc.
│   ├── commands/                    # Drive commands
│   │   ├── TeleopFieldDrive / TeleopRobotDrive
│   │   ├── TrajectoryDrive / VisionDrive / DriveToPose
│   │   └── season_specific_commands/ (season-dependent)
│   └── generated/                   # CTRE Swerve generated code
│
├── mechanisms/                      # Robot game mechanisms (season-dependent)
│   ├── MechanismTypes.h             # Enum: INTAKE, LAUNCHER, CLIMBER
│   ├── base/BaseMech                # Abstract base for all mechanisms
│   ├── configs/
│   │   ├── MechanismConfigMgr       # Singleton; selects config by RobotIdentifier
│   │   ├── MechanismConfig          # Abstract base config
│   │   ├── MechanismConfigCompBot_302  # Concrete 2026 comp-bot config
│   │   └── RobotElementNames.h      # Motor/sensor usage enums
│   ├── controllers/ControlData      # PIDF + control-mode configuration
│   ├── Intake/                      # 2026-specific: intake + hopper + extender
│   ├── Launcher/                    # 2026-specific: launcher + hood + turret + transfer
│   └── Climber/                     # 2026-specific: L1 / L3 climb
│
├── state/                           # Robot-wide state & pub/sub event bus
│   ├── RobotState                   # Central state manager + pub/sub broker
│   ├── RobotStateChanges.h          # Enum of all publishable state-change events
│   ├── StateMgr                     # Base class for mechanism state machines
│   ├── State                        # Individual state base class
│   └── IRobotStateChangeSubscriber  # Interface for event listeners
│
├── teleopcontrol/                   # Gamepad mappings (season-dependent)
│   ├── TeleopControl                # Reads gamepad axes/buttons
│   ├── TeleopControlFunctions.h     # Enum of all robot functions
│   └── TeleopControlMap             # Maps functions → buttons/axes
│
├── fielddata/                       # Field geometry (season-dependent)
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
│   ├── RebuiltTargetCalculator      # 2026-specific: launcher target calc
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

The robot lifecycle follows the standard WPILib structure, with key initialization steps for subsystems, autonomous routines, and telemetry.

### Key Lifecycle Methods
- **`RobotInit`**: Initializes singletons, subsystems, and autonomous options.
- **`RobotPeriodic`**: Runs the `CommandScheduler`, logs data, updates robot state, and provides feedback.
- **`AutonomousInit`**: Prepares the robot for autonomous mode, including setting thread priority and initializing vision rewind.
- **`TeleopInit`**: Cancels autonomous commands, resets state, and prepares for teleop control.
- **`DisabledPeriodic`**: Updates field visualization and FMS data.
- **`TestInit`**: Cancels all commands and prepares for testing.

---

## 4. Core Singletons & Init Order

| Singleton | File | Notes |
|---|---|---|
| `FieldConstants` | `fielddata/FieldConstants` | Must init first; provides field poses |
| `RoboRio` | `utils/RoboRio` | Hardware abstraction |
| `ChassisConfigMgr` | `chassis/ChassisConfigMgr` | Creates swerve drivetrain |
| `RobotContainer` | `RobotContainer` | Wires subsystems and commands |
| `MechanismConfigMgr` | `mechanisms/configs/MechanismConfigMgr` | Selects per-robot config |
| `RobotState` | `state/RobotState` | Pub/sub event bus |
| `PeriodicLooper` | `utils/PeriodicLooper` | Registers StateMgrs per mode |
| `DragonVision` | `vision/DragonVision` | Vision facade |
| `DragonField` | `utils/DragonField` | Field2d visualization |
| `DriverFeedback` | `feedback/DriverFeedback` | Dashboard HUD |
| `DragonDataLoggerMgr` | `utils/logging/DragonDataLoggerMgr` | Data logging |

---

## 5. Chassis & Drivetrain

> ⚠️ **Season-dependent:** Physical swerve constants, module positions, drive/steer gear ratios, and CAN IDs live in `chassis/generated/TunerConstants302.h`.

- **Type:** CTRE Swerve (Phoenix 6), field-centric by default
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

---

## 6. Mechanisms

> ⚠️ **Highly season-dependent.** Mechanism types, states, motor assignments, and logic all change between seasons.

### 2026 Mechanisms

#### `Intake`
- **Motors:** `TalonFX` (roller), `TalonFXS` (extender/hopper)
- **States:** `STATE_OFF`, `STATE_INTAKE`, `STATE_EXPEL`, `STATE_LAUNCH`, `STATE_EMPTY_HOPPER`, `STATE_LOAD_HOPPER`

#### `Launcher`
- **Motors:** `TalonFX` (flywheel), `TalonFXS` (hood, turret, transfer, indexer)
- **States:** `STATE_OFF`, `STATE_PREPARE_TO_LAUNCH`, `STATE_LAUNCH`, `STATE_IDLE`, `STATE_CLIMB`

#### `Climber`
- **Motors:** `TalonFX` (arm), `CANcoder`, `Solenoid`
- **States:** `STATE_OFF`, `STATE_L1CLIMB`, `STATE_L3CLIMB`, `STATE_PREPARE_TO_CLIMB`

---

## 7. State Machine Architecture

All mechanisms use the `StateMgr` base class for state management. States are updated via the `PeriodicLooper`.

---

## 8. Teleop Control System

> ⚠️ **Season-dependent:** `TeleopControlFunctions` and `TeleopControlMap` must be updated each season.

- **`TeleopControlFunctions`**: Enum of robot actions (e.g., `DRIVE_TO_HUB`, `LAUNCH`, `CLIMB_MODE`).
- **`TeleopControl`**: Singleton for reading gamepad inputs.

---

## 9. Autonomous System

> ⚠️ **Season-dependent:** Auton paths, zone managers, and auton scripts change every season.

### Framework
- **`CyclePrimitives`**: Executes auton sequences from CSV scripts.
- **`PrimitiveFactory`**: Creates primitives (e.g., `TRAJECTORY_DRIVE`, `VISION_ALIGN`).
- **Zone Managers**: Define field zones for auton behavior.

---

## 10. RobotState & Pub/Sub Event System

`RobotState` is a singleton event bus for robot-wide state changes. Subsystems publish and subscribe to events using `RobotStateChanges`.

---

## 11. Vision System

- **`DragonVision`**: Facade for Limelight and Meta Quest.
- **`DragonVisionPoseEstimator`**: Combines vision data with odometry for pose estimation.

---

## 12. Field Data & Target Calculation

> ⚠️ **Season-dependent:** `FieldConstants` and `RebuiltTargetCalculator` are specific to the 2026 game.

- **`FieldConstants`**: Provides field element poses.
- **`RebuiltTargetCalculator`**: Calculates launcher targets based on field position.

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

Key utilities include `PeriodicLooper` (state management), `RoboRio` (hardware abstraction), and `FMSData` (alliance color helper).

---

## 16. Robot Identifiers & Multi-Robot Support

**File:** `RobotIdentifier.h`

The robot identifier is determined by the RoboRIO team number. Example IDs:
- `COMP_BOT_302` — Competition robot
- `CHASSIS_BOT_9999` — Test chassis

---

## 17. Key Design Patterns

| Pattern | Where Used |
|---|---|
| **Singleton** | Managers like `RobotState`, `ChassisConfigMgr`, `PeriodicLooper` |
| **State Machine** | `StateMgr` + `State` for mechanisms |
| **Pub/Sub Event Bus** | `RobotState` for decoupled communication |
| **Command-Based** | WPILib `frc2::Command` framework for drive commands |
| **Factory Pattern** | `PrimitiveFactory` for auton primitives |
| **Config/Strategy** | `MechanismConfigMgr` for robot-specific configurations |

---

## 18. Season-Dependent Change Checklist

### 🔴 Must Change Every Season
- `fielddata/FieldConstants.h/.cpp` — field element poses
- `chassis/commands/season_specific_commands/` — game-specific drive commands
- `mechanisms/` — replace mechanisms and states
- `teleopcontrol/TeleopControlFunctions.h` — update robot functions
- `utils/RebuiltTargetCalculator` — update target geometry

### 🟡 Usually Changes Each Season
- `chassis/generated/TunerConstants302.h` — swerve module constants
- `auton/PrimitiveEnums.h` — auton primitive types
- `src/main/deploy/` — Choreo `.traj` files

### 🟢 Generally Stable
- Core framework: `Robot.cpp`, `StateMgr`, `RobotState`, `PeriodicLooper`, etc.
```
