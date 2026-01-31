# Target Calculator Documentation

## Overview

The Target Calculator system provides a flexible framework for calculating aiming angles and distances to game targets. It handles complex mathematical operations including:

- **Virtual target compensation**: Accounts for robot movement during projectile flight
- **Mechanism offset transformation**: Converts between robot center and mechanism position
- **Field frame conversions**: Transforms between world coordinates and robot-relative coordinates

The system is designed with a base class (`TargetCalculator`) that handles common calculations, which can be extended with season-specific implementations (e.g., `RebuiltTargetCalculator` for 2026).

## Architecture

### Class Hierarchy

```
TargetCalculator (abstract base class)
    └── RebuiltTargetCalculator (2026-specific implementation)
```

### Key Components

#### TargetCalculator (Base Class)
- **Location**: `src/main/cpp/utils/TargetCalculator.h` and `TargetCalculator.cpp`
- **Purpose**: Provides core target calculation algorithms that are season-independent
- **Key Responsibilities**:
  - Calculate distances and angles to targets
  - Compute virtual target positions based on robot velocity
  - Transform between coordinate frames
  - Manage mechanism offset from robot center

#### RebuiltTargetCalculator (Child Class)
- **Location**: `src/main/cpp/utils/RebuiltTargetCalculator.h` and `RebuiltTargetCalculator.cpp`
- **Purpose**: Implements 2026-specific target selection and configuration
- **Key Responsibilities**:
  - Define target positions (currently hardcoded hub target)
  - Configure mechanism offsets for the 2026 robot
  - Select appropriate targets based on game state
  - Manage launcher angle constraints

## Core Concepts

### 1. Coordinate Frames

The system uses two primary coordinate frames:

**World Frame (Field Frame)**
- Origin at the center of the field
- X-axis points in a consistent direction across the field
- Y-axis perpendicular to X
- All target positions are defined in world coordinates

**Robot Frame (Local Frame)**
- Origin at robot center
- X-axis points forward (direction robot is facing)
- Y-axis points left
- Relative angles and velocities are in robot frame

### 2. Mechanism Offset

The mechanism offset accounts for the fact that shooting mechanisms are not located at the robot's center. This offset is defined in robot frame coordinates:

```cpp
frc::Translation2d m_mechanismOffset{-0.1397_m, 0_m};
```

- **X offset**: -0.1397 m means the mechanism is 0.1397 m behind the robot center
- **Y offset**: 0 m means the mechanism is centered left-right

The offset is transformed to world coordinates based on the robot's current rotation using:
```cpp
robotPose.Translation() + m_mechanismOffset.RotateBy(robotPose.Rotation());
```

### 3. Virtual Target Compensation

The virtual target adjusts the aim point to compensate for the robot's movement during projectile flight. This is essential for accurate long-distance shots.

**Formula**:
```
VirtualTarget = RealTarget - (VelocityVector × LookaheadTime)
```

**Example**:
- Robot moving toward target at 2 m/s
- Projectile flight time: 0.5 seconds
- Virtual target moves 1 meter further away from robot
- This causes the shooter to aim higher/further, compensating for forward movement

### 4. Lookahead Time

The lookahead time represents how long the projectile will be in flight. This is used to calculate how far the robot will move during that time and adjust the aim point accordingly.

**Typical Values**:
- 0 seconds: No compensation (instant hit or stationary target)
- 0.1-0.5 seconds: Typical projectile flight times for game pieces

## How It Works: Detailed Flow

### Getting the Launcher Target Angle

```
GetLauncherTarget()
    ↓
Get current target position: GetTargetPosition()
    ↓
Calculate virtual target: CalculateVirtualTarget(realTarget, lookaheadTime)
    ↓
Get mechanism world position: GetMechanismWorldPosition()
    ↓
Calculate angle from mechanism to virtual target
    ↓
Convert to robot-relative angle
    ↓
Find best angle within launcher constraints (90° to 270°)
    ↓
Return launcher angle setpoint
```

### Step-by-Step Calculation in RebuiltTargetCalculator

1. **Get Real Target**: `GetTargetPosition()` returns the hub target at (4.625 m, 4.025 m)

2. **Calculate Virtual Target**: 
   - Get robot velocity from chassis
   - Multiply by lookahead time
   - Subtract from real target

3. **Get Mechanism Position**:
   - Start with mechanism offset in robot frame: (-0.1397 m, 0 m)
   - Rotate by robot orientation
   - Add to robot center position

4. **Calculate Vector to Target**:
   - Subtract mechanism position from virtual target
   - Get angle using `atan2` (handled by WPILib)

5. **Convert to Robot-Relative**:
   - Subtract robot rotation from field angle
   - Result is the direction to target relative to robot forward

6. **Constrain to Launcher Limits**:
   - Check if angle is within 90° to 270°
   - If not, find the closest valid angle
   - Handle wraparound at 0°/360° boundary

## Implementation Guide

### Creating a New Target Calculator

Follow these steps to implement a season-specific target calculator for a new year:

#### Step 1: Create Header File

Create `src/main/cpp/utils/NewYearTargetCalculator.h`:

```cpp
#pragma once

#include "utils/TargetCalculator.h"
#include "utils/DragonField.h"

class NewYearTargetCalculator : public TargetCalculator
{
public:
    static NewYearTargetCalculator *GetInstance();
    
    frc::Translation2d GetTargetPosition() override;
    units::angle::degree_t GetLauncherTarget(
        units::time::second_t lookaheadTime, 
        units::angle::degree_t currentLauncherAngle);

private:
    NewYearTargetCalculator();
    
    static NewYearTargetCalculator *m_instance;
    
    // Season-specific targets
    frc::Translation2d m_hubTarget{...};
    frc::Translation2d m_otherTarget{...};
    
    // Mechanism offset from robot center (in robot frame)
    frc::Translation2d m_mechanismOffset{...};
    
    // Launcher angle constraints
    const units::degree_t m_minLauncherAngle = ...;
    const units::degree_t m_maxLauncherAngle = ...;
    
    DragonField *m_field;
};
```

#### Step 2: Implement the Class

Create `src/main/cpp/utils/NewYearTargetCalculator.cpp`:

```cpp
#include "utils/NewYearTargetCalculator.h"

NewYearTargetCalculator::NewYearTargetCalculator() : TargetCalculator()
{
    SetMechanismOffset(m_mechanismOffset);
    
    m_field = DragonField::GetInstance();
    m_field->AddObject("TargetPosition", frc::Pose2d(GetTargetPosition(), frc::Rotation2d()));
    m_field->AddObject("LauncherPosition", frc::Pose2d());
}

NewYearTargetCalculator *NewYearTargetCalculator::m_instance = nullptr;

NewYearTargetCalculator *NewYearTargetCalculator::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new NewYearTargetCalculator();
    }
    return m_instance;
}

frc::Translation2d NewYearTargetCalculator::GetTargetPosition()
{
    // TODO: Replace with dynamic target selection
    // For now, return hardcoded target for testing
    return m_hubTarget;
}

units::angle::degree_t NewYearTargetCalculator::GetLauncherTarget(
    units::time::second_t lookaheadTime, 
    units::angle::degree_t currentLauncherAngle)
{
    // Update field visualization
    m_field->UpdateObject("TargetPosition", GetVirtualTargetPose(lookaheadTime));
    
    // Calculate angle from mechanism to target
    units::degree_t fieldAngleToTarget = CalculateMechanismAngleToTarget(lookaheadTime);
    auto robotPose = GetChassisPose();
    
    // Convert to robot-relative angle
    frc::Rotation2d relativeRot = frc::Rotation2d(fieldAngleToTarget) - robotPose.Rotation();
    units::degree_t robotRelativeGoal = relativeRot.Degrees();
    
    // Find the best angle within constraints
    units::degree_t bestAngle = 0_deg;
    bool hasFoundValidAngle = false;
    units::degree_t minError = 360_deg;
    
    for (int i = -1; i <= 1; i++)
    {
        units::degree_t potentialSetpoint = robotRelativeGoal + (360_deg * i);
        
        if (potentialSetpoint >= m_minLauncherAngle && potentialSetpoint <= m_maxLauncherAngle)
        {
            auto error = units::math::abs(potentialSetpoint - currentLauncherAngle);
            if (error < minError)
            {
                bestAngle = potentialSetpoint;
                minError = error;
                hasFoundValidAngle = true;
            }
        }
    }
    
    // If no valid angle found, clamp to limits
    if (!hasFoundValidAngle)
    {
        units::degree_t normalizedGoal = relativeRot.Degrees();
        if (normalizedGoal < 0_deg)
            normalizedGoal += 360_deg;
        bestAngle = std::clamp(normalizedGoal, m_minLauncherAngle, m_maxLauncherAngle);
    }
    
    // Update field visualization
    auto robotPose_var = GetChassisPose();
    m_field->UpdateObject("LauncherPosition", 
        frc::Pose2d(GetMechanismWorldPosition(), robotPose_var.Rotation() + frc::Rotation2d(bestAngle)));
    
    return bestAngle;
}
```

#### Step 3: Configure for Your Season

When implementing your child class, you must configure:

1. **Target Positions**
   ```cpp
   frc::Translation2d m_hubTarget{4.625_m, 4.025_m};  // Field coordinates
   ```

2. **Mechanism Offset** (measure from robot CAD or physical robot)
   ```cpp
   frc::Translation2d m_mechanismOffset{-0.1397_m, 0_m};  // 5.5 inches back, centered
   ```

3. **Launcher Angle Constraints** (based on mechanism design)
   ```cpp
   const units::degree_t m_minLauncherAngle = 90_deg;  // Minimum reachable angle
   const units::degree_t m_maxLauncherAngle = 270_deg; // Maximum reachable angle
   ```

4. **Lookahead Time** (based on projectile flight time)
   ```cpp
   // Pass this when calling GetLauncherTarget()
   units::time::second_t lookaheadTime = 0.25_s;  // Typical value
   ```

## RebuiltTargetCalculator Example

The `RebuiltTargetCalculator` is the 2026-specific implementation with the following configuration:

### Configuration Values

- **Hub Target Position**: (4.625 m, 4.025 m) - Approximate field center
- **Mechanism Offset**: (-0.1397 m, 0 m) - 5.5 inches behind robot center
- **Launcher Angle Range**: 90° to 270° (back half of rotation)

### Hardcoded Values (To Be Updated)

These are currently hardcoded for testing and should be replaced with dynamic logic:

```cpp
// TODO: Replace with FieldElementCalculator and ZoneCalculator integration
frc::Translation2d m_hubTarget{4.625_m, 4.025_m};
```

## Future Enhancements

### FieldElementCalculator Integration
Replace hardcoded targets with dynamic lookup based on field elements.

### ZoneCalculator Integration
Select targets based on which zone the robot is in.

### Multi-Target Support
Support multiple targets and select the optimal one based on game state.

### Lookahead Time Calibration
Automatically determine optimal lookahead time based on shooter characteristics.

## Debugging and Visualization

The Target Calculator integrates with `DragonField` for visualization:

```cpp
// These are updated in real-time
m_field->UpdateObject("TargetPosition", GetVirtualTargetPose(lookaheadTime));
m_field->UpdateObject("LauncherPosition", ...);
```

Monitor these in the Field2d widget in Shuffleboard to verify:
- Target positions are correct
- Virtual target compensation is working
- Launcher is pointing at the target

## Common Issues and Solutions

### Issue: Launcher angle jumps between angles
**Cause**: Wraparound at 0°/360° boundary
**Solution**: The code handles this with the -1, 0, +1 iteration to find the closest valid angle

### Issue: Target is off-field
**Cause**: Incorrect target coordinates or coordinate frame mismatch
**Solution**: Verify target positions are in meters and use field visualization to debug

### Issue: Virtual target not working
**Cause**: Lookahead time is 0 or chassis velocity is not updating
**Solution**: Ensure lookahead time is non-zero and chassis is providing velocity updates

### Issue: Mechanism offset incorrect
**Cause**: Offset not measured properly or in wrong coordinate frame
**Solution**: Measure from robot CAD, verify sign convention (+ = forward, + = left)

## References

- WPILib Geometry: `frc::Translation2d`, `frc::Pose2d`, `frc::Rotation2d`
- Units Library: `units::meter_t`, `units::degree_t`, `units::time::second_t`
- Coordinate Frames: Right-hand rule (X forward, Y left, Z up)
