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
│   │   ├── TeleopFieldDrive         # Command for field-oriented teleop driving
│   │   ├── TeleopRobotDrive         # Command for robot-oriented teleop driving
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
│   │   ├── Intake.cpp               # Implementation of the intake mechanism
│   │   ├── Intake.h                 # Header for the intake mechanism
│   ├── Launcher/                    # ⚠️ 2026-specific: launcher + hood + turret + transfer + indexer
│   │   ├── IdleState.cpp            # State for the launcher when idle
│   │   ├── IdleState.h              # Header for the idle state of the launcher
│   │   ├── InitializeState.cpp      # State for initializing the launcher
│   │   ├── InitializeState.h        # Header for the initialization state of the launcher
│   │   ├── Launcher.cpp             # Implementation of the launcher mechanism
│   │   ├── Launcher.h               # Header for the launcher mechanism
│   │   ├── ManualLaunchState.cpp    # State for manual launching
│   │   ├── ManualLaunchState.h      # Header for the manual launch state
│   │   ├── OffState.cpp             # State for when the launcher is off
│   │   ├── OffState.h               # Header for the off state of the launcher
│   │   ├── PrepareToLaunchState.cpp # State for preparing to launch
│   │   ├── PrepareToLaunchState.h   # Header for the prepare to launch state
│   │   ├── LaunchState.cpp          # State for launching
│   │   ├── LaunchState.h            # Header for the launch state
│   │   ├── ClimbState.cpp           # State for climbing with the launcher
│   │   ├── ClimbState.h             # Header for the climb state of the launcher
│   │   ├── EmptyHopperState.cpp     # State for when the hopper is empty
│   │   ├── EmptyHopperState.h       # Header for the empty hopper state
│   │   ├── LauncherTuningState.cpp   # State for tuning the launcher
│   │   ├── LauncherTuningState.h     # Header for the tuning state of the launcher
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
│   ├── FieldElementCalculator        # ⚠️ 2026-specific: Calculates field element positions and relationships
│   └── SweepLaneChanger             # Utility for managing sweep lane selection
│
├── vision/
│   ├── DragonVision                 # Facade over Limelight + Quest
│   ├── DragonLimelight     
│   ├── DragonQuest.cpp              # Implementation of the DragonQuest vision processing
│   ├── DragonQuest.h                # Header for the DragonQuest vision processing
│   ├── DragonVision.cpp              # Implementation of the DragonVision facade
│   ├── DragonVision.h                # Header for the DragonVision facade
│   ├── DragonVisionEnums.h           # Enums for vision processing
│   ├── DragonVisionPoseEstimator.cpp # Implementation of the pose estimator for vision
│   ├── DragonVisionPoseEstimator.h   # Header for the pose estimator for vision
│   ├── DragonVisionPoseEstimatorStruct.h # Struct definitions for pose estimation
│   ├── DragonVisionStruct.h          # Struct definitions for vision processing
│   ├── PoseOffsetUtils.cpp           # Utility functions for pose offsets
│   ├── PoseOffsetUtils.h             # Header for pose offset utility functions
│   ├── Questnavlib/                  # Quest navigation library
│   │   ├── QuestNav.cpp              # Implementation of the Quest navigation
│   │   ├── QuestNav.h                # Header for the Quest navigation
│   │   ├── commands.pb.cc            # Protobuf commands for Quest navigation
│   │   ├── commands.pb.h             # Protobuf header for commands
│   │   ├── data.pb.cc                # Protobuf data for Quest navigation
│   │   ├── data.pb.h                 # Protobuf header for data
│   │   ├── geometry2d.pb.cc          # Protobuf for 2D geometry
│   │   ├── geometry2d.pb.h           # Protobuf header for 2D geometry
│   │   ├── geometry3d.pb.cc          # Protobuf for 3D geometry
│   │   └── geometry3d.pb.h           # Protobuf header for 3D geometry
│   └── definitions/                  # Vision definitions
│       ├── CameraConfig.cpp          # Implementation of camera configuration
│       ├── CameraConfig.h            # Header for camera configuration
│       ├── CameraConfigMgr.cpp       # Implementation of camera configuration manager
│       ├── CameraConfigMgr.h         # Header for camera configuration manager
│       ├── CameraConfig_302.cpp      # Implementation of camera configuration for comp-bot 302
│       ├── CameraConfig_302.h        # Header for camera configuration for comp-bot 302
│       ├── CameraConfig_9997.cpp     # Implementation of camera configuration for 9997
│       ├── CameraConfig_9997.h       # Header for camera configuration for 9997
│       ├── CameraConfig_9998.cpp     # Implementation of camera configuration for 9998
│       ├── CameraConfig_9998.h       # Header for camera configuration for 9998
│       └── CameraConfig_9999.cpp     # Implementation of camera configuration for 9999
│       └── CameraConfig_9999.h       # Header for camera configuration for 9999
│
├── utils/                           # Utility functions
│   ├── MathUtils.h                  # Math utility functions
│   ├── StringUtils.h                # String utility functions
│   └── TimeUtils.h                  # Time utility functions
│
└── README.md                        # Project overview and setup instructions
```

## 11. Vision System

The vision system is responsible for processing camera input and providing data to the robot for navigation and targeting. It utilizes the Limelight and Quest navigation libraries to achieve this.

### Key Components
- **DragonVision**: A facade that simplifies the interaction with the Limelight and Quest libraries.
- **DragonQuest**: Implements the vision processing logic, including target detection and pose estimation.
- **Pose Estimation**: Utilizes camera data to estimate the robot's position relative to field elements.

### Important Notes
- Ensure that camera configurations are set correctly in the `CameraConfig` files.
- The vision processing is highly dependent on the camera's calibration and the lighting conditions on the field.
- The vision system should be tested thoroughly during practice matches to ensure reliability during competition.

### Checklist
- [ ] Verify camera configurations.
- [ ] Test vision processing under various lighting conditions.
- [ ] Ensure integration with the robot's navigation system.
- [ ] Validate pose estimation accuracy with field elements.

### Manual Notes
- The vision system is a critical component for autonomous navigation and targeting.
- Any changes to the vision processing logic should be documented and tested extensively.

### Warnings
- Changes to the vision system may require adjustments in the autonomous routines.
- Ensure that the vision processing does not introduce latency that could affect robot performance.

```
