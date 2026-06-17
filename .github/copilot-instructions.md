# Copilot Instructions — Team 302 (2026 Rebuilt)

## Project Overview

This is a **FIRST Robotics Competition (FRC)** robot codebase for **Team 302 — Lake Orion Robotics**. It is a **C++ project** targeting the **RoboRIO** platform, built with **GradleRIO** (WPILib 2026). The robot uses a **swerve drivetrain** with CTRE Phoenix 6 hardware and includes mechanisms for Intake, Launcher, and Climber.

## Build System

- **Build tool:** Gradle with the `edu.wpi.first.GradleRIO` plugin (version 2026.2.1)
- **Language:** C++ (compiled as a native executable via the `cpp` Gradle plugin)
- **Test framework:** Google Test
- **Source directories:**
  - `src/main/cpp/` — All robot C++ source and headers
  - `src/main/thirdparty/` — Third-party C++ libraries
  - `src/main/deploy/` — Files deployed to the RoboRIO (auton XML, configs, Choreo trajectories)
- **Vendor dependencies** (in `vendordeps/`):
  - CTRE Phoenix 5 (replay) and Phoenix 6
  - ChoreoLib 2026
  - WPILib New Commands

### Common Build Commands

- `./gradlew build` — Full build
- `./gradlew deploy` — Deploy to the RoboRIO
- `./gradlew simulateExternalCpp` — Run in simulation

## Architecture & Code Organization

### Entry Point

- `Robot.h` / `Robot.cpp` — Inherits `frc::TimedRobot`. Orchestrates initialization, periodic loops for autonomous, teleop, and disabled modes.
- `RobotContainer.h` / `RobotContainer.cpp` — Initializes subsystems: `SwerveContainer` and `DragonVisionPoseEstimator`.

### Robot Identification

The codebase supports multiple robot configurations via `RobotIdentifier` enum:
- `COMP_BOT_302` (302) — Competition robot
- `CHASSIS_BOT_9999`, `CHASSIS_BOT_9998`, `CHASSIS_BOT_9997` — Test chassis bots
- `SIM_BOT_0` (0) — Simulation bot

Each robot variant may have different tuner constants and mechanism configurations.

### Chassis / Swerve Drivetrain

- **`chassis/`** — Swerve drive subsystem
  - `SwerveContainer` — Singleton that owns the swerve drivetrain, all drive commands, and button bindings. Implements `IRobotStateChangeSubscriber`.
  - `generated/CommandSwerveDrivetrain` — CTRE-generated swerve drivetrain class
  - `generated/TunerConstants*.h/.cpp` — Per-robot tuner constants (one per `RobotIdentifier`)
  - `generated/Telemetry` — Swerve telemetry logging
  - `commands/` — Drive commands:
    - `TeleopFieldDrive`, `TeleopRobotDrive` — Teleop drive modes
    - `TrajectoryDrive` — Follows Choreo trajectories for autonomous
    - `DriveToPose`, `VisionDrive` — Pose-targeting and vision-aided driving
    - `season_specific_commands/` — Game-specific commands (DriveToHub, DriveToDepot, DriveToTower, etc.)

### Mechanisms

Each mechanism inherits from **`BaseMech`**, **`StateMgr`**, **`IRobotStateChangeSubscriber`**, and **`DragonDataLogger`**.

- **`mechanisms/base/BaseMech`** — Base class providing mechanism type, network table name, logging, and control data reading.
- **`mechanisms/MechanismTypes.h`** — Enum of mechanism types: `INTAKE`, `LAUNCHER`, `CLIMBER`.
- **`mechanisms/configs/`** — Robot-specific mechanism configurations (`MechanismConfigMgr`, `MechanismConfig`, `MechanismConfigCompBot_302`, `RobotElementNames`).
- **`mechanisms/controllers/`** — `ControlData` and `ControlModes` for PID/motion-profile control.

#### Mechanism Implementations

Each mechanism has its own folder with a main class and individual state classes:

- **`mechanisms/Intake/`** — `Intake` class with states: `OffState`, `IntakeState`, `ExpelState`, `LaunchState`, `EmptyHopperState`, `LoadHopperState`.
- **`mechanisms/Launcher/`** — `Launcher` class with states: `OffState`, `IdleState`, `InitializeState`, `PrepareToLaunchState`, `LaunchState`, `ManualLaunchState`, `LauncherTuningState`, `ClimbState`, `EmptyHopperState`.
- **`mechanisms/Climber/`** — `Climber` class with states: `OffState`, `WantToClimbState`, `PrepareToClimbState`, `L1ClimbState`, `L3ClimbState`, `AutonL1ClimbState`, `ExitState`.

### State Machine Framework

- **`state/State`** — Abstract base class with `Init()`, `Run()`, `Exit()`, `AtTarget()` lifecycle. Supports registered transition states.
- **`state/StateMgr`** — Manages a vector of `State` objects, runs the current state, and checks for transitions (sensor-based and gamepad-based).
- **`state/RobotState`** — Tracks global robot state.
- **`state/RobotStateChangeBroker`** — Pub/sub broker for robot state changes.
- **`state/IRobotStateChangeSubscriber`** — Interface for classes that react to robot state changes.

### Autonomous System

- **`auton/CyclePrimitives`** — Main autonomous executor; inherits `State`. Sequences primitives.
- **`auton/AutonSelector`** / **`AutonPreviewer`** — Select and preview autonomous routines.
- **`auton/PrimitiveParser`** / **`PrimitiveFactory`** — Parse XML auton files and instantiate primitives.
- **`auton/PrimitiveParams`** — Parameters for each primitive step.
- **`auton/drivePrimitives/`** — Drive primitives: `AutonDrivePrimitive`, `VisionDrivePrimitive`, `ResetPositionTrajectory`. All implement `IPrimitive`.
- **`auton/Zone*`** — Zone-based auton logic (AllianceZoneManager, NeutralZoneManager, DeadZoneManager, BumpZoneManager, AutonGrid).
- **Auton XML files** are in `src/main/deploy/auton/` with DTD validation (`auton.dtd`, `zone.dtd`).

### Teleop Control / Gamepad

- **`teleopcontrol/TeleopControl`** — Maps gamepad inputs to robot functions. Defines button/axis enums and mappings.
- **`gamepad/`** — Gamepad abstraction layer: `IDragonGamepad` interface, `DragonXBox`, `DragonCommandController`, `DragonGamepad` implementations.
- **`gamepad/axis/`**, **`gamepad/button/`** — Axis and button abstraction.

### Vision

- **`vision/DragonVision`** — Main vision processing class.
- **`vision/DragonLimelight`** — Limelight camera interface.
- **`vision/DragonQuest`** — Meta Quest VR headset integration for odometry.
- **`vision/DragonVisionPoseEstimator`** — Fuses vision data with odometry for pose estimation.
- **`vision/Questnavlib/`** — Library for Quest navigation.

### Driver Feedback

- **`feedback/DriverFeedback`** — Manages driver feedback (LEDs, dashboard data).
- **`feedback/DragonCANdle`** / **`feedback/DragonLeds`** — LED control via CTRE CANdle.
- **`feedback/GameDataHelper`** — FMS/game data helper.

### Utilities

- **`utils/`** — Math, conversion, field, and logging utilities.
  - `AngleUtils`, `ConversionUtils`, `InterpolateUtils`, `PoseUtils` — Math helpers
  - `DragonField` — Field geometry/data
  - `FMSData` — FMS data wrapper
  - `TargetCalculator`, `RebuiltTargetCalculator` — Calculate targets on the field
  - `PeriodicLooper` — Periodic execution helper
  - `logging/` — Data logging framework (`DragonDataLoggerMgr`, signal-based logging)

### Health Tests

- **`healthtests/`** — `DragonTestCase` and `DragonTestSuiteManager` for runtime hardware health checks.

## Coding Conventions

### File Header

Every source file must include the Team 302 MIT license header:

```cpp
//====================================================================================================================================================
// Copyright 2026 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// ...
//====================================================================================================================================================
```

### Style Rules

- **Header guards:** Use `#pragma once` (not `#ifndef` guards).
- **Include order:** Standard library → WPILib/FRC → CTRE/vendor → Team 302 includes.
- **Forward declarations:** Prefer forward declarations in headers; include in `.cpp` files.
- **Naming:**
  - Classes: `PascalCase`
  - Methods/Functions: `PascalCase`
  - Member variables: `m_camelCase` prefix
  - Enums: `UPPER_SNAKE_CASE` values inside `PascalCase` enum classes or nested enums
  - Constants: `UPPER_SNAKE_CASE`
- **Singletons:** Used for manager classes (e.g., `SwerveContainer::GetInstance()`).
- **Inheritance:** Mechanisms inherit from `BaseMech`, `StateMgr`, `IRobotStateChangeSubscriber`, and `DragonDataLogger`.
- **State classes:** Each state has `Init()`, `Run()`, `Exit()`, `AtTarget()` methods.
- **Doxygen-style comments:** Use `///` for brief descriptions and `@param`, `@return`, `@brief`, `@class`, `@interface` tags.

### Code Generation

Some files are auto-generated by the **Team 302 code generator** (version 20.26.x). These files contain a comment like:

```cpp
// This file was automatically generated by the Team 302 code generator version 20.26.00.01
```

Be cautious when editing generated files — changes may be overwritten. Generated files include `MechanismTypes.h`, `RobotIdentifier.h`, mechanism main classes (e.g., `Intake.h`), and `RobotElementNames.h`.

## Key Patterns to Follow

### Adding a New Mechanism

1. Add the type to `MechanismTypes::MECHANISM_TYPE` enum.
2. Create a folder under `mechanisms/` with the mechanism name.
3. Create the main mechanism class inheriting `BaseMech`, `StateMgr`, `IRobotStateChangeSubscriber`, `DragonDataLogger`.
4. Define state enums (`STATE_NAMES`) and create individual state classes inheriting `State`.
5. Add robot-specific configuration in `mechanisms/configs/`.
6. Register in `CyclePrimitives` if used in autonomous.

### Adding a New Autonomous Routine

1. Create an XML file in `src/main/deploy/auton/` following the `auton.dtd` schema.
2. The file will be parsed by `PrimitiveParser` and executed by `CyclePrimitives`.

### Adding a New Drive Command

1. Create the command class in `chassis/commands/` (or `season_specific_commands/` for game-specific).
2. Register it in `SwerveContainer` and bind to appropriate gamepad controls.

### Adding a New State to a Mechanism

1. Create `<StateName>State.h` and `<StateName>State.cpp` in the mechanism's folder.
2. Inherit from `State` and implement `Init()`, `Run()`, `Exit()`, `AtTarget()`.
3. Add the state to the mechanism's `STATE_NAMES` enum.
4. Register the state in the mechanism's constructor via `AddToStateVector()`.
5. Register transition states as needed.

## Hardware & Vendor Libraries

- **Motors:** CTRE TalonFX, TalonFXS (Phoenix 6 API)
- **Sensors:** CTRE CANdi, Limelight cameras, Meta Quest VR headset
- **LEDs:** CTRE CANdle
- **Swerve:** CTRE swerve modules with Phoenix 6 CommandSwerveDrivetrain
- **Trajectory:** Choreo for path planning and trajectory generation

## Deploy Directory Structure

- `src/main/deploy/302/` — Competition bot config files
- `src/main/deploy/9999/` — Test chassis bot config files
- `src/main/deploy/auton/` — Autonomous XML routines and zone definitions
- `src/main/deploy/chassis/` — Chassis configuration files
- `src/main/deploy/choreo/` — Choreo trajectory files
- `src/main/deploy/mechanisms/` — Mechanism configuration files
