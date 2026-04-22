```markdown
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
│   │   ├── ExpelState.cpp           # Handles the expelling of game pieces from the intake
│   │   ├── ExpelState.h             # Header for ExpelState
│   │   ├── Intake.cpp               # Main intake control logic
│   │   ├── Intake.h                 # Header for Intake
│   │   ├── IntakeState.cpp          # Manages the state of the intake mechanism
│   │   ├── LaunchState.cpp          # Controls the launching mechanism
│   │   ├── LaunchState.h            # Header for LaunchState
│   │   ├── OffState.cpp             # Represents the off state of the intake
│   │   ├── OffState.h               # Header for OffState
│   │   └── LoadHopperState.cpp      # Manages loading game pieces into the hopper
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
│   ├── 9999/                        #
│   └── ...
```

---

## 7. State Machine Architecture

The state machine architecture is designed to manage the various states of the robot and its subsystems. The `StateMgr` class serves as the base class for all mechanism state machines, while the `RobotState` class acts as the central state manager. The `RobotStateChanges.h` file defines an enumeration of all publishable state-change events, which are used to communicate state changes across the system.

### Key Components:
- **RobotState**: Central state manager that coordinates state changes.
- **StateMgr**: Base class for mechanism state machines, providing common functionality.
- **RobotStateChanges.h**: Contains the enumeration of state change events, facilitating communication between different parts of the robot code.

### Usage:
- Each mechanism should derive from `StateMgr` to implement its specific state management logic.
- State changes should be published using the events defined in `RobotStateChanges.h` to ensure all interested subscribers are notified.

---

## 10. RobotState & Pub/Sub Event System

The `RobotState` class is responsible for managing the overall state of the robot. It utilizes a publish/subscribe event system to notify various components of state changes. The `RobotStateChanges.h` file defines the events that can be published, allowing for a decoupled architecture where components can react to state changes without direct dependencies.

### Key Features:
- **Pub/Sub Mechanism**: Allows components to subscribe to state changes and react accordingly.
- **State Change Events**: Defined in `RobotStateChanges.h`, these events represent various state changes that can occur during operation.

### Implementation:
- Components interested in state changes should implement the `IRobotStateChangeSubscriber` interface to receive notifications.
- The `RobotState` class will manage the subscription and publication of state change events.

---

## 18. Season-Dependent Change Checklist

- [ ] Review and update `RobotStateChanges.h` for any new state change events relevant to the 2026 season.
- [ ] Ensure all mechanisms properly implement state management using the `StateMgr` base class.
- [ ] Verify that all state changes are published correctly and that subscribers are responding as expected.
- [ ] Test the overall state machine architecture to ensure it meets the requirements of the 2026 game.
```
