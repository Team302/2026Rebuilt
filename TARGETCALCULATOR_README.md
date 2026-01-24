# TargetCalculator Implementation Summary

## Overview

I've created a complete target calculation system that translates the Python simulation logic into a robust C++ framework for FRC robot targeting calculations. The system supports dynamic target selection and movement compensation.

## Files Created/Modified

### 1. **TargetCalculator.h** (Modified)
   - **Purpose**: Base class for all target calculations
   - **Key Methods**:
     - `CalculateDistanceToTarget()` - Distance from chassis center to target
     - `CalculateAngleToTarget()` - Angle in robot frame (0° = forward)
     - `CalculateLauncherDistanceToTarget()` - Distance from launcher mechanism to target
     - `CalculateLauncherAngleToTarget()` - Angle from launcher to target
     - `CalculateVirtualTarget()` - Compensate for robot movement during flight
     - `SetLauncherOffset()` / `GetLauncherOffset()` - Configure mechanism position
   - **Protected Methods**:
     - `GetChassisPose()` - Query current robot pose
     - `GetChassisVelocity()` - Query current robot velocity
   - **Virtual Method**:
     - `GetTargetPosition()` - Subclasses override to define their target

### 2. **TargetCalculator.cpp** (Modified)
   - Full implementation of base target calculations
   - Coordinate transformations from robot frame to world frame
   - Virtual target compensation mathematics
   - Angle normalization and unit conversions

### 3. **RebuiltTargetCalculator.h** (New)
   - Season-specific extension for 2026 Rebuilt
   - **Key Methods**:
     - `GetTargetPosition()` - Returns hardcoded hub target (for testing)
     - `SetLauncherOffset()` - Configure launcher position
     - `GetLauncherOffset()` - Query launcher position
   - **TODO**: Integrate with FieldElementCalculator and ZoneCalculator when available

### 4. **RebuiltTargetCalculator.cpp** (New)
   - Implementation of season-specific calculator
   - Hardcoded hub target for testing
   - Launcher offset management with season-specific configuration

### 5. **TargetCalculator_USAGE.txt** (New)
   - Comprehensive usage guide with examples
   - Architecture overview
   - Coordinate system documentation
   - Virtual target compensation explanation
   - Launcher offset configuration guide
   - Performance considerations

## Key Features

### Coordinate System Handling
- **World Frame**: Field coordinates (meters)
- **Robot Frame**: Angles relative to robot orientation (degrees)
- Automatic transformation between frames

### Virtual Target Compensation
Based on your Python simulation's `calculate_virtual_goal()` method:
```
VirtualTarget = RealTarget - (RobotVelocity × LookaheadTime)
```
Compensates for robot movement during projectile flight to maintain accuracy while moving.

### Launcher Offset Support
- Configurable launcher/mechanism position relative to robot center
- Default: 5.5 inches back (0.1397m), centered
- Accurate distance/angle calculations from actual launch point

### Dynamic Zone Selection
The `RebuiltTargetCalculator` provides a foundation for:
1. Hardcoded hub target (for immediate testing)
2. Launcher offset configuration for the shooting mechanism
3. Future integration with FieldElementCalculator and ZoneCalculator
   - Will allow dynamic target selection based on zone
   - Will query field element positions from game data

## Usage Examples

### Basic Distance and Angle
```cpp
auto targetCalc = RebuiltTargetCalculator::GetInstance();
auto distance = targetCalc->CalculateDistanceToTarget();
auto angle = targetCalc->CalculateAngleToTarget();
```

### Configure Launcher Position
```cpp
auto targetCalc = RebuiltTargetCalculator::GetInstance();
targetCalc->SetLauncherOffset(-0.1397_m, 0_m);  // 5.5" back, centered
```

### Launcher-Specific Calculations
```cpp
auto launcherDistance = targetCalc->CalculateLauncherDistanceToTarget();
auto launcherAngle = targetCalc->CalculateLauncherAngleToTarget();
```

## Design Principles

### Extensibility
- Base `TargetCalculator` contains all math
- Subclasses override only `GetTargetPosition()` for custom behavior
- Easy to create calculators for different mechanisms

### Robustness
- Proper angle normalization to [-π, π]
- Unit-safe with WPILib units library
- Null pointer checks for chassis reference

### Performance
- All calculations are O(1) time
- No dynamic allocations in critical paths
- Safe to call every control loop iteration

## Integration Notes

### Required Includes
```cpp
#include "utils/GamePieceTargetCalculator.h"
#include <units/length.h>
#include <units/angle.h>
```

### Chassis Integration
- Automatically connects to singleton chassis via `ChassisConfigMgr`
- Uses `GetPose()` for position/rotation
- Uses `GetState().Speeds` for velocity

### Unit System
- Uses WPILib `units` library (type-safe)
- Distances in meters
- Angles in degrees (output) or radians (internal)
- Velocities in m/s

## Testing Recommendations

1. **Stationary Robot**: Verify angles are correct at different chassis rotations
2. **Moving Robot**: Test virtual target compensation at various speeds
3. **Zone Switching**: Verify targets change correctly
4. **Boundary Cases**: Test angle calculations near ±180°
5. **Launcher Offsets**: Verify offset distances are calculated correctly

## Future Enhancements

Once FieldElementCalculator and ZoneCalculator are implemented:

1. **Dynamic Target Selection**: Update `GetTargetPosition()` to query field element calculator
2. **Zone-Based Targets**: Detect robot zone and return appropriate target
3. **Multiple Game Pieces**: Support different targets for different game piece types
4. **Target Priority**: Select between multiple valid targets
5. **Trajectory Prediction**: Calculate optimal shooting angle based on distance
6. **Dynamic Lookahead**: Adjust lookahead time based on shot type and distance
7. **Filtering/Smoothing**: Apply filtering to calculated values for stability
8. **Logging**: Add debug logging for trajectory analysis
