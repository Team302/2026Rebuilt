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
  └── CyclePrimitives::Start()
```

---

## 4. Core Singletons & Init Order

- **RobotState**: Central state manager for the robot.
- **FMSData**: Handles FMS-related data, including alliance color.
- **DragonDataLoggerMgr**: Manages logging of telemetry data.
- **ChassisConfigMgr**: Manages the configuration of the chassis and drivetrain.
- **MechanismConfigMgr**: Selects the appropriate mechanism configuration based on the robot identifier.

---

## 5. Chassis & Drivetrain ⚠️ **Season-dependent**

The chassis is based on a swerve drive system. The main components include:

- **CommandSwerveDrivetrain**: Extends the Phoenix 6 SwerveDrivetrain class and implements the Subsystem interface for command-based projects.
- **TeleopFieldDrive**: Command for driving the robot in teleoperated mode using field-oriented control.
- **DriveAlongNearestWall**: Command for driving along the nearest wall, utilizing sensors to maintain proximity.
- **TrajectoryDrive**: Command for following a predefined trajectory.
- **VisionDrive**: Command for driving based on vision processing.

### Important Notes:
- Ensure that the swerve modules are calibrated before competition.
- The robot's perspective changes based on the alliance color, which is handled in the `CommandSwerveDrivetrain` class.

---

## 6. Mechanisms ⚠️ **Highly season-dependent**

Mechanisms are specific to the game and may include:

- **Intake**: Mechanism for collecting game pieces.
- **Launcher**: Mechanism for launching game pieces.
- **Climber**: Mechanism for climbing to different levels.

### Important Notes:
- Each mechanism should be tested thoroughly to ensure reliability during matches.
- Mechanism configurations are managed by `MechanismConfigMgr`.

---

## 7. State Machine Architecture

The robot uses a state machine architecture to manage different states of operation, including:

- **Idle**: The robot is not performing any actions.
- **Autonomous**: The robot is executing autonomous commands.
- **Teleop**: The robot is under human control.
- **Disabled**: The robot is disabled and not operational.

### Important Notes:
- State transitions should be handled carefully to avoid unexpected behavior.
- Each state should have a clearly defined entry and exit process.

---

## 8. Teleop Control System ⚠️ **Season-dependent**

The teleop control system is designed to allow the driver to control the robot effectively. Key components include:

- **TeleopControl**: Reads inputs from the gamepad and maps them to robot functions.
- **TeleopControlMap**: Maps specific functions to buttons and axes on the gamepad.

### Important Notes:
- Ensure that the control mappings are intuitive for drivers.
- Test the control system thoroughly to ensure responsiveness.

---

## 9. Autonomous System ⚠️ **Season-dependent**

The autonomous system is responsible for executing pre-defined sequences during the autonomous period. Key components include:

- **CyclePrimitives**: Manages the execution of autonomous commands.
- **AutonPreviewer**: Provides a dashboard interface for selecting autonomous routines.

### Important Notes:
- Autonomous routines should be tested extensively to ensure reliability.
- Use the `AutonSelector` to configure and select the appropriate autonomous sequence.

---

## 10. RobotState & Pub/Sub Event System

The `RobotState` class manages the overall state of the robot and facilitates communication between different components through a publish/subscribe event system. This allows for decoupled communication between subsystems and commands.

### Important Notes:
- Ensure that all state changes are published correctly to avoid inconsistencies.
- Subsystems should subscribe to relevant state changes to react accordingly.

---

## 11. Vision System

The vision system integrates various components to provide the robot with visual feedback and target tracking capabilities. Key components include:

- **DragonVision**: A facade over the Limelight and Quest vision systems.
- **DragonVisionPoseEstimator**: Fuses vision data into the robot's odometry.

### Important Notes:
- Ensure that the vision system is calibrated for the competition field.
- Test the vision processing under various lighting conditions.

---

## 12. Field Data & Target Calculation ⚠️ **Season-dependent**

Field data is crucial for accurate navigation and targeting. Key components include:

- **FieldConstants**: Contains the poses of field elements for the 2026 season.
- **RebuiltTargetCalculator**: Calculates target positions based on the current field layout.

### Important Notes:
- Update field constants as necessary based on the competition field.
- Ensure that target calculations are accurate for effective gameplay.

---

## 13. Logging & Telemetry

The logging system captures telemetry data for analysis and debugging. Key components include:

- **DragonDataLoggerMgr**: Manages the logging of data during operation.
- **Logger**: Provides methods for logging various types of data.

### Important Notes:
- Enable logging during testing to capture important data.
- Review logs after matches to identify areas for improvement.

---

## 14. Feedback & Dashboard

The feedback system provides real-time information to drivers and operators through the dashboard. Key components include:

- **DriverFeedback**: Manages the display of information on the dashboard.
- **GameDataHelper**: Parses FMS messages for game-specific data.

### Important Notes:
- Ensure that feedback is clear and actionable for drivers.
- Test the dashboard interface for usability.

---

## 15. Utilities

Utility classes provide common functionality used throughout the codebase. Key components include:

- **PeriodicLooper**: Manages periodic tasks for state machines.
- **FMSData**: Provides access to FMS-related information.

### Important Notes:
- Utilize utility classes to avoid code duplication.
- Ensure that utility functions are well-documented.

---

## 16. Robot Identifiers & Multi-Robot Support

The robot identifier system allows for easy configuration of multiple robots. Key components include:

- **RobotIdentifier**: Enum that defines different robot configurations.

### Important Notes:
- Ensure that each robot is configured correctly based on its identifier.
- Test multi-robot scenarios to ensure compatibility.

---

## 17. Key Design Patterns

The codebase employs several design patterns to promote maintainability and scalability. Key patterns include:

- **Command Pattern**: Used for implementing commands that can be executed by the robot.
- **Singleton Pattern**: Used for managing global state and resources.

### Important Notes:
- Follow design patterns consistently throughout the codebase.
- Document any deviations from standard patterns.

---

## 18. Season-Dependent Change Checklist

- [ ] Update field constants and target calculations for the 2026 season.
- [ ] Test all season-specific commands and mechanisms.
- [ ] Review and update the teleop control mappings as necessary.
- [ ] Ensure that all autonomous routines are configured and tested.
- [ ] Validate the vision system's performance under competition conditions.
```
