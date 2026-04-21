# Team 302 — 2026 Rebuilt Codebase Context

> **Purpose:** Give an AI assistant (or new team member) enough context to work effectively in this repository without reading every file.  
> **Season:** FRC 2026 — *Reefscape* → this codebase uses 2026 field elements (Hub, Outpost, Depot, Tower, Trench, Bump) and the corresponding game pieces.  
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
│   ├── commands/
│   │   ├── TeleopFieldDrive         # Field-relative teleop drive
│   │   ├── TeleopRobotDrive         # Robot-relative teleop drive
│   │   ├── TrajectoryDrive          # Follows Choreo trajectories
│   │   ├── VisionDrive              # Vision-assisted driving
│   │   ├── DriveToPose              # Drives to a specific Pose2d
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
│   ├── FieldOffsetValues            # Per-element X/Y offset constants (FIELD_OFFSET_ITEMS enum)
│   ├── FieldAprilTagIDs             # April tag ID mapping
│   └── SweepLaneChanger             # Utility for managing sweep lane selection
│
├── vision/
│   ├── DragonVision                 # Facade over Limelight + Quest
│   ├── DragonLimelight              # Limelight wrapper (supports sim pose injection)
│   ├── DragonQuest                  # Meta Quest (Questnav) wrapper (supports sim pose injection)
│   └── DragonVisionPoseEstimator    # Fuses vision into odometry
│
├── feedback/
│   ├── DriverFeedback               # Dashboard HUD, LEDs, rumble
│   └── GameDataHelper               # Parses FMS game-specific messages
│
├── utils/
│   ├── PeriodicLooper               # Throttled periodic runner for StateMgrs
│   ├── RebuiltTargetCalculator      # ⚠️ 2026-specific: launcher target calc (singleton)
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
├── deploy/
│   ├── 302/                         # Competition robot (COMP_BOT_302) configs
│   ├── 9999/                        # Test chassis configs (CHASSIS_BOT_9999, etc.)
│   ├── auton/                       # Autonomous routine XML files + DTDs
│   │   ├── auton.dtd                # DTD for auton XML validation
│   │   ├── zone.dtd                 # DTD for zone XML validation
│   │   ├── Blue*.xml / Red*.xml     # Alliance-specific auton routines
│   │   ├── snippets/                # Reusable XML auton fragments
│   │   │   ├── CrossBumpFromNZ.xml  # Cross the bump from neutral zone
│   │   │   ├── GoToDepot.xml        # Drive to depot sequence
│   │   │   ├── LeftBumpInit.xml     # Left-side bump initialization
│   │   │   └── RightBumpInit.xml    # Right-side bump initialization
│   │   └── zones/                   # Zone-specific auton definitions
│   ├── chassis/                     # Chassis tuning and configuration files
│   ├── choreo/                      # Choreo trajectory files (.traj)
│   └── mechanisms/                  # Mechanism-specific configuration files
│
└── thirdparty/                      # Third-party C++ libraries (non-vendor)

tools/
├── DragonCodeGenerator/             # Team 302 code generator (v20.26.x)
│   └── templates/                   # Code generation templates
└── doxygen/                         # Doxygen documentation configuration
```

---

## 2. Build System

- **Build tool:** Gradle with `edu.wpi.first.GradleRIO` plugin (version 2026.x)
- **Language:** C++ (native executable via the `cpp` Gradle plugin)
- **Test framework:** Google Test
- **Vendor dependencies** (in `vendordeps/`):
  - `Phoenix5-replay` and `Phoenix6-replay-frc2026` — CTRE motor/sensor APIs
  - `ChoreoLib2026` — Choreo trajectory following
  - `WPILibNewCommands` — Command-based framework

### Common Commands
```bash
./gradlew build                  # Full build
./gradlew deploy                 # Deploy to RoboRIO
./gradlew simulateExternalCpp    # Run in simulation
```

---

## 3. Robot Entry Point & Lifecycle

- **`Robot.h` / `Robot.cpp`** — Inherits `frc::TimedRobot`. Calls into `RobotContainer` and dispatches all periodic methods.
- **`RobotContainer.h` / `RobotContainer.cpp`** — Constructs `SwerveContainer` and `DragonVisionPoseEstimator`. The primary wiring point for subsystems, commands, and button bindings.

**Periodic loop order (per mode):**
1. `Robot::XxxPeriodic()` called by WPILib scheduler
2. `SwerveContainer` runs the active drive command
3. `StateMgr::RunCurrentState()` called for each mechanism via `PeriodicLooper`
4. `DragonVisionPoseEstimator::RunCurrentState()` fuses vision into odometry

---

## 4. Core Singletons & Init Order

| Singleton | Header | Notes |
|---|---|---|
| `SwerveContainer` | `chassis/SwerveContainer.h` | Owns `CommandSwerveDrivetrain` |
| `MechanismConfigMgr` | `mechanisms/configs/MechanismConfigMgr.h` | Selects per-robot config |
| `AllianceZoneManager` | `auton/AllianceZoneManager.h` | Alliance + zone awareness |
| `DeadZoneManager` | `auton/DeadZoneManager.h` | Dead zone detection |
| `FieldConstants` | `fielddata/FieldConstants.h` | 2026 field element poses |
| `RebuiltTargetCalculator` | `utils/RebuiltTargetCalculator.h` | Launcher targeting singleton |
| `TeleopControl` | `teleopcontrol/TeleopControl.h` | Gamepad input mapping |
| `DriverFeedback` | `feedback/DriverFeedback.h` | LEDs, dashboard, rumble |
| `DragonVision` | `vision/DragonVision.h` | Vision facade |
| `AutonSelector` | `auton/AutonSelector.h` | Auton routine selection |

Singletons are created in `RobotContainer` constructor or lazily via `GetInstance()`. Destruction order is not explicitly managed — avoid cross-singleton teardown.

---

## 5. Chassis & Drivetrain

⚠️ **Season-dependent** — swerve module positions, gear ratios, and drive commands change every year.

- **`SwerveContainer`** — Singleton; owns the `CommandSwerveDrivetrain` instance, all drive `Command` objects, and the gamepad button bindings.
- **`CommandSwerveDrivetrain`** — CTRE-generated; manages the four swerve modules via Phoenix 6 API.
- **`TunerConstants302.h`** — Per-robot swerve tuning (wheel radius, gear ratios, PID, CAN IDs). One file per `RobotIdentifier`.

### Drive Commands (`chassis/commands/`)

| Command | Description |
|---|---|
| `TeleopFieldDrive` | Primary field-relative driver control |
| `TeleopRobotDrive` | Robot-relative driver control |
| `TrajectoryDrive` | Follows a Choreo `.traj` trajectory |
| `VisionDrive` | Drives to a vision target |
| `DriveToPose` | Drives to a specific `Pose2d` using PID |

### Season-Specific Commands (`chassis/commands/season_specific_commands/`) ⚠️

| Command | Description |
|---|---|
| `DriveToHub` | Drives to the 2026 alliance hub |
| `DriveToOutpost` | Drives to the outpost |
| `DriveToDepot` | Drives to the depot |
| `DriveToTower` | Drives to the tower |
| `DriveOverBump` | Drives over the field bump; uses `FIELD_OFFSET_ITEMS::BUMP_*` constants |
| `DriveAlongNearestWall` | Aligns with the nearest wall |
| `SweepBehindBump` | Sweeps game pieces from behind the bump |
| `DriveToFuel` | Drives to a fuel target |
| `AutoDefend` | Defensive autonomous behavior |

---

## 6. Mechanisms

⚠️ **Highly season-dependent** — all mechanism states, motor configs, and sensor logic are 2026-specific.

All mechanisms inherit: `BaseMech`, `StateMgr`, `IRobotStateChangeSubscriber`, `DragonDataLogger`.

### Intake (`mechanisms/Intake/`)
States: `OffState`, `IntakeState`, `ExpelState`, `LaunchState`, `EmptyHopperState`, `LoadHopperState`

### Launcher (`mechanisms/Launcher/`)
Auto-generated skeleton + hand-written extension for 2026.

**States:**

| Enum | Description |
|---|---|
| `STATE_OFF` | All motors stopped |
| `STATE_INITIALIZE` | Home hood + turret on startup |
| `STATE_IDLE` | Hold position, wheels at speed |
| `STATE_PREPARE_TO_LAUNCH` | Aim + spin up using `RebuiltTargetCalculator` |
| `STATE_LAUNCH` | Fire sequence |
| `STATE_EMPTY_HOPPER` | Clear hopper |
| `STATE_CLIMB` | Retract for climb |
| `STATE_LAUNCHER_TUNING` | Manual tuning mode |
| `STATE_MANUAL_LAUNCH` | Override fire |

**Key hardware:** hood (TalonFX), turret (TalonFX), launcher wheels (TalonFX), transfer (TalonFXS), indexer (TalonFXS), CANdi for game piece detection.

**Logged signals:** `/Launcher/HoodActualAngle` (deg), `/Launcher/TurretActualAngle` (deg), `/Launcher/TargetHoodAngle` (deg), `/Launcher/TargetTurretAngle` (deg).

### Climber (`mechanisms/Climber/`)
States: `OffState`, `WantToClimbState`, `PrepareToClimbState`, `L1ClimbState`, `L3ClimbState`, `AutonL1ClimbState`, `ExitState`

---

## 7. State Machine Architecture

- **`State`** (`state/State.h`) — Abstract base with lifecycle: `Init()`, `Run()`, `Exit()`, `AtTarget()`. Supports registered transition states.
- **`StateMgr`** (`state/StateMgr.h`) — Owns a `vector<State*>`, runs the active state each periodic cycle, checks sensor and gamepad transitions.
- Each mechanism's constructor registers all its state objects via `AddToStateVector()`.
- Transitions are registered on each state via `AddTransitionState()` with a sensor-check lambda.

---

## 8. Teleop Control System

⚠️ **Season-dependent** — button/axis mappings change with the robot's capabilities.

- **`TeleopControl`** (`teleopcontrol/TeleopControl.h`) — Singleton. Reads Xbox/hybrid controller inputs and maps them to `TeleopControlFunctions` enum values.
- **`TeleopControlFunctions.h`** — Enum of every driver/operator action (e.g. `DRIVE_INTAKE`, `DRIVE_LAUNCH`, `CLIMBER_PREPARE`).
- **`TeleopControlMap`** — Maps each function to a physical button or axis on a specific controller.
- Gamepad abstraction: `DragonXBox`, `DragonHybridController`, `DragonGamepad` all implement `IDragonGamepad`.

---

## 9. Autonomous System

⚠️ **Season-dependent** — routines, field zones, and trajectories change every year.

### Framework
- **`CyclePrimitives`** — Main auton executor. Reads a sequence of `PrimitiveParams` and executes them in order.
- **`PrimitiveParser`** / **`PrimitiveFactory`** — Parse XML auton files from `deploy/auton/` and construct `IPrimitive` objects.
- **`AutonSelector`** / **`AutonPreviewer`** — Dashboard widgets for routine selection and preview.

### XML Auton Files (`src/main/deploy/auton/`)
Naming convention encodes start position, game piece actions, zone traversal, and climb result:

| Token | Meaning |
|---|---|
| `Blue` / `Red` | Alliance color |
| `B` / `T` | Bump-side / Trench-side start |
| `Dep` / `Out` / `Hub` | Starting field position |
| `Keep3` / `Drop2` / `Launch0/1/2` | Game piece strategy |
| `DepNOutScore` | Score via Depot-to-NeutralZone-to-Outpost path |
| `NCli` / `Cli` | No Climb / Climb |
| `Win` / `Lose` / `Invert` | Tiebreaker variant |

**Current routines include:**
- `BlueBDepKeep3{Dep|N}DepNOutScoreNCli{Win|Lose|Invert}` — Blue bump-start, keep 3 game pieces, depot path variants
- `RedBDepKeep3{Dep|N}DepNOutScoreNCli{Win|Lose|Invert}` — Red mirror
- `BlueBOutKeep3NDepNOutScoreNCli{Win|Lose|Invert}` — Blue bump-out start variants
- `RedBOutKeep3NDepNOutScoreNCli{Win|Lose|Invert}` — Red mirror
- `Blue/RedBDep/OutLaunch{0|1|2}*` — Launch-focused routines
- `Blue/RedTDep/OutLaunch*` — Trench-start launch routines
- `SquareTest.xml` — Test/calibration routine

### Auton Snippets (`src/main/deploy/auton/snippets/`)
Reusable XML fragments included by full routines:

| File | Description |
|---|---|
| `CrossBumpFromNZ.xml` | Cross the bump from the neutral zone |
| `GoToDepot.xml` | Navigate to the depot |
| `LeftBumpInit.xml` | Initialize robot on the left side of the bump |
| `RightBumpInit.xml` | Initialize robot on the right side of the bump |

### Zone Managers
- `AllianceZoneManager` — Determines if robot is in alliance zone (triggers hub targeting)
- `NeutralZoneManager` — Neutral zone detection
- `DeadZoneManager` — Dead zone detection
- `BumpZoneManager` — Bump zone detection
- `AutonGrid` — Grid-based zone lookup

### Drive Primitives (`auton/drivePrimitives/`)
All implement `IPrimitive`:
- `AutonDrivePrimitive` — Follows a Choreo trajectory segment
- `VisionDrivePrimitive` — Vision-aligned drive step
- `ResetPositionTrajectory` — Resets odometry at trajectory start

### Choreo Trajectories (`src/main/deploy/choreo/`)
Trajectory files (`.traj`) generated by the Choreo path-planning tool.
Naming convention: `{Alliance}{Side}Drop{N}_{Leg}[_{Variant}].traj`

Key trajectory groups:
- `BlueMLDrop3_*` — Blue Middle-Left, 3-piece drop routine legs (A, C, C2, C_Back_copy1, C_Forth, D, D_copy1, F, Init)
- `BlueMLDrop2_*`, `BlueMRDrop2_*` — 2-piece drop variants
- `BlueMLDepotOutpost_*`, `BlueMDepotOutpost_*`, `BlueLDepotOutpost_*` — Depot-to-Outpost paths
- `BlueLeftBumpNZNZ_*` — Left-side bump neutral zone paths
- Red mirrors follow the same naming convention with `Red` prefix

---

## 10. RobotState & Pub/Sub Event System

- **`RobotState`** (`state/RobotState.h`) — Tracks global robot state (mode, game piece held, etc.)
- **`RobotStateChangeBroker`** — Publisher; mechanisms call `Publish(RobotStateChanges::STATE_CHANGE_XXX, value)` to notify subscribers.
- **`IRobotStateChangeSubscriber`** — Interface; implement `Update(RobotStateChanges::STATE_CHANGE_XXX, double value)` to react.
- `SwerveContainer`, all mechanisms, and `DriverFeedback` implement this interface.

---

## 11. Vision System

- **`DragonVision`** (`vision/DragonVision.h`) — Facade over all cameras. Provides `SetSimPoses(Pose2d)` to inject chassis pose into all cameras in simulation.
- **`DragonLimelight`** (`vision/DragonLimelight.h`) — Limelight wrapper. In sim, `SetSimPose()` makes `GetMegaTag1Pose()` / `GetMegaTag2Pose()` return the injected pose with 0.1m/0.1rad standard deviations.
- **`DragonQuest`** (`vision/DragonQuest.h`) — Meta Quest (Questnav) wrapper. In sim: always reports `IsQuestConnected=true`, `IsQuestTracking=true`, `QuestBatteryPercent=67`, and returns the injected pose with HIGH confidence.
- **`DragonVisionPoseEstimator`** — Calls `SetSimPoses(chassisPose)` before `AddVisionMeasurements()` in simulation. Vision measurements are fused unconditionally (not gated by `IsSimulation`).

---

## 12. Field Data & Target Calculation

⚠️ **Season-dependent** — all field element positions and offset tables are 2026-specific.

### FieldConstants
- **File:** `src/main/cpp/fielddata/FieldConstants.h`
- Singleton providing `frc::Pose2d` / `frc::Translation2d` for all 2026 field elements: Hub center, Outpost, Depot, Tower, Trench positions (alliance-mirrored).

### FieldOffsetValues ⚠️
- **File:** `src/main/cpp/fielddata/FieldOffsetValues.h`
- Provides tunable X/Y offsets for every field element approach via the `FIELD_OFFSET_ITEMS` enum.
- **`FIELD_OFFSET_ITEMS` values:**
  - `DEPOT_X` — Depot neutral-side X
  - `OUTPOST_X`, `OUTPOST_APPROACH_X` — Outpost X coordinates
  - `HUB_X` — Hub center with offset
  - `BUMP_ALLIANCE_X/Y`, `BUMP_NEUTRAL_X/Y` — Bump crossing coordinates
  - `BUMP_ALLIANCE_X_LANE_0…3`, `BUMP_NEUTRAL_X_LANE_0…3` — Per-lane bump X for sweep operations
  - `TOWER_OUTPOST_X/Y`, `TOWER_DEPOT_X` — Tower approach offsets
- Used by: `DriveOverBump`, `SweepBehindBump`, `DriveToHub`, `DriveToDepot`, `DriveToOutpost`, `DriveToTower`.

### RebuiltTargetCalculator ⚠️
- **File:** `src/main/cpp/utils/RebuiltTargetCalculator.h`
- Singleton. Extends `TargetCalculator` with 2026-specific launcher angle computation.
- **Target selection:** In alliance zone → hub center. Outside alliance zone → closest passing target (outpost or depot).
- **Features:** Alliance-aware caching, launcher angle calculation (mechanical range: 91–267°), driver offset adjustment via controller, field visualization updates.
- **Dependencies:** `AllianceZoneManager`, `FieldConstants`, `DragonField`, `TargetCalculator`

### TargetCalculator
- **File:** `src/main/cpp/utils/TargetCalculator.h`
- Base class extended by `RebuiltTargetCalculator`.
- **Dependencies:** `ChassisConfigMgr`, `CommandSwerveDrivetrain`

---

## 13. Logging & Telemetry

- **`DragonDataLogger`** (`utils/logging/`) — Base class. Mechanisms inherit this and override `DataLog()`.
- **`DragonDataLoggerMgr`** — Singleton manager; calls `DataLog()` on all registered loggers each cycle.
- Signals are written to WPILib's `DataLog` (`.wpilog` format, viewable in AdvantageScope).
- **`Telemetry`** (`chassis/generated/Telemetry.h`) — CTRE-generated swerve telemetry logger.

### Launcher Logged Signals

| Log Path | Units | Description |
|---|---|---|
| `/Launcher/HoodActualAngle` | degrees | Measured hood position |
| `/Launcher/TurretActualAngle` | degrees | Measured turret position |
| `/Launcher/TargetHoodAngle` | degrees | Commanded hood target |
| `/Launcher/TargetTurretAngle` | degrees | Commanded turret target |

---

## 14. Feedback & Dashboard

- **`DriverFeedback`** (`feedback/DriverFeedback.h`) — Singleton. Manages LEDs (via `DragonCANdle`), Shuffleboard/SmartDashboard values, and controller rumble.
- **`DragonCANdle`** / **`DragonLeds`** — LED control via CTRE CANdle.
- **`GameDataHelper`** — Reads and parses FMS game-specific message string.

---

## 15. Utilities

| File | Description |
|---|---|
| `utils/RebuiltTargetCalculator.h` | ⚠️ 2026-specific singleton launcher target calc — see Section 12 |
| `utils/TargetCalculator.h` | Base class for target calculation |
| `utils/PeriodicLooper.h` | Throttled periodic execution for `StateMgr` subclasses |
| `utils/DragonField.h` | `Field2d` visualization + robot pose publishing |
| `utils/FMSData.h` | Alliance color and FMS data helper |
| `utils/RoboRio.h` | RoboRIO hardware singleton |
| `utils/AngleUtils.h` | Angle normalization and conversion helpers |
| `utils/ConversionUtils.h` | Unit conversion helpers |
| `utils/InterpolateUtils.h` | Linear interpolation helpers |
| `utils/PoseUtils.h` | Pose math utilities |
| `utils/logging/` | `DragonDataLoggerMgr`, signal-based logging framework |

---

## 16. Robot Identifiers & Multi-Robot Support

**File:** `src/main/cpp/RobotIdentifier.h`

| Identifier | ID | Description |
|---|---|---|
| `COMP_BOT_302` | 302 | Competition robot |
| `CHASSIS_BOT_9999` | 9999 | Primary test chassis |
| `CHASSIS_BOT_9998` | 9998 | Secondary test chassis |
| `CHASSIS_BOT_9997` | 9997 | Tertiary test chassis |
| `SIM_BOT_0` | 0 | Simulation bot |

Robot identity is read from the RoboRIO at startup. `MechanismConfigMgr` and `TunerConstants` select robot-specific parameters based on this ID.

---

## 17. Key Design Patterns

### Adding a New Mechanism
1. Add type to `MechanismTypes::MECHANISM_TYPE` enum.
2. Create folder `mechanisms/<Name>/` with main class inheriting `BaseMech`, `StateMgr`, `IRobotStateChangeSubscriber`, `DragonDataLogger`.
3. Define `STATE_NAMES` enum; create individual state classes inheriting `State`.
4. Add robot-specific config in `mechanisms/configs/`.
5. Register in `CyclePrimitives` if used in auton.

### Adding a New State
1. Create `<StateName>State.h/.cpp` in the mechanism folder.
2. Implement `Init()`, `Run()`, `Exit()`, `AtTarget()`.
3. Add to the mechanism's `STATE_NAMES` enum.
4. Register via `AddToStateVector()` in the mechanism constructor.
5. Register transitions via `AddTransitionState()`.

### Adding a New Auton Routine
1. Create an XML file in `src/main/deploy/auton/` following `auton.dtd`.
2. Reference snippet files from `auton/snippets/` for reusable sequences.
3. Reference Choreo trajectory names from `src/main/deploy/choreo/`.
4. The file will appear in `AutonSelector` automatically.

### Adding a New Drive Command
1. Create the class in `chassis/commands/` (season-specific → `season_specific_commands/`).
2. Register it in `SwerveContainer` and bind to a gamepad control.

### Singleton Pattern
Used for all manager classes. Always access via `ClassName::GetInstance()`. Never store raw pointers across periodic boundaries.

---

## 18. Season-Dependent Change Checklist

> Use this list when porting to a new season or robot configuration.

- [ ] `RobotIdentifier.h` — Add new robot IDs
- [ ] `TunerConstants<RobotId>.h/.cpp` — New swerve constants per robot
- [ ] `MechanismConfigCompBot_302.h/.cpp` — Update motor/sensor CAN IDs
- [ ] `RobotElementNames.h` — Update motor/sensor usage enums
- [ ] `MechanismTypes.h` — Add/remove mechanism types
- [ ] `mechanisms/<Name>/` — Rewrite or update all mechanism + state classes
- [ ] `fielddata/FieldConstants.h` — New field element poses
- [ ] `fielddata/FieldOffsetValues.h` — New `FIELD_OFFSET_ITEMS` enum + offset values
- [ ] `teleopcontrol/TeleopControlFunctions.h` — Update driver/operator function list
- [ ] `teleopcontrol/TeleopControlMap` — Remap buttons/axes
- [ ] `chassis/commands/season_specific_commands/` — New/updated drive commands
- [ ] `src/main/deploy/auton/` — New auton XML routines
- [ ] `src/main/deploy/auton/snippets/` — New/updated reusable snippets
- [ ] `src/main/deploy/choreo/` — New Choreo trajectories
- [ ] `utils/RebuiltTargetCalculator.h` — Update target calc for new field
- [ ] `state/RobotStateChanges.h` — Add new state-change events
- [ ] `feedback/DriverFeedback` — Update LED patterns and dashboard values
- [ ] `vision/` — Update AprilTag layout, camera calibrations
