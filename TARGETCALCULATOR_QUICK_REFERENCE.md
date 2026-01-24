# TargetCalculator Quick Reference

## Files
- `TargetCalculator.h` / `TargetCalculator.cpp` - Base class with all math
- `RebuiltTargetCalculator.h` / `RebuiltTargetCalculator.cpp` - 2026 season-specific implementation

## Getting Started

### 1. Initialize in Your Code
```cpp
#include "utils/RebuiltTargetCalculator.h"

// In robot initialization or command
auto targetCalc = RebuiltTargetCalculator::GetInstance();

// Configure launcher position (one time, or per mechanism change)
targetCalc->SetLauncherOffset(-0.1397_m, 0_m);  // 5.5" back, centered
```

### 2. Get Distance and Angle to Target
```cpp
// Distance from chassis center
auto distance = targetCalc->CalculateDistanceToTarget();

// Distance from launcher
auto launcherDistance = targetCalc->CalculateLauncherDistanceToTarget();

// Angle in robot frame (0° = forward, 90° = left, -90° = right)
auto angle = targetCalc->CalculateAngleToTarget();

// Angle from launcher
auto launcherAngle = targetCalc->CalculateLauncherAngleToTarget();
```

### 3. Compensate for Robot Movement
```cpp
auto realTarget = targetCalc->GetTargetPosition();
auto virtualTarget = targetCalc->CalculateVirtualTarget(
    realTarget,
    1.0_s  // Lookahead time in seconds (adjust to your shot type)
);

// Get angle to virtual target for aiming while moving
auto compensatedAngle = targetCalc->CalculateAngleToTarget(&virtualTarget);
```

## Understanding the Angle Output

The angle is in **robot frame**, not world frame:
- **0°** = Forward (robot's forward direction)
- **90°** = Left (relative to robot)
- **-90°** = Right (relative to robot)
- **180°** = Backward

This means you can use the angle directly for:
- Turret aim calculation
- Heading correction
- Relative positioning

## Understanding Virtual Target

Virtual target compensates for robot movement while the projectile is in flight:
- If moving **toward** target → virtual target is **closer** (aim short)
- If moving **away** from target → virtual target is **farther** (aim long)
- If **stationary** → virtual target = real target

Adjust lookahead time based on projectile speed:
- Fast/direct shots: 0.3-0.5 seconds
- Medium shots: 0.7-1.0 seconds  
- Slow/arcing shots: 1.5-2.0 seconds

## Current Limitations (TODO)

The `GetTargetPosition()` currently returns hardcoded hub position (4.625m, 4.025m).

Future versions will integrate with:
- **FieldElementCalculator** - Actual target positions from field data
- **ZoneCalculator** - Zone detection for automatic target selection

For now, this is suitable for testing and development.

## Common Issues

### "Angle is always the same"
- Check that `GetChassisPose()` is being called correctly
- Verify robot pose is updating from odometry
- Try logging the chassis pose to confirm it changes

### "Distance doesn't match expected"
- Verify launcher offset is set correctly
- Check that target coordinates are in meters
- Confirm coordinate system (should be field frame)

### "Virtual target compensation isn't working"
- Verify `GetChassisVelocity()` returns actual robot velocity
- Try different lookahead times
- Log the virtual target position to debug offset calculation

## Methods Summary

| Method | Purpose | Returns | Notes |
|--------|---------|---------|-------|
| `GetTargetPosition()` | Current target in field frame | Translation2d | Non-const, can be dynamic |
| `CalculateDistanceToTarget()` | Distance from chassis center | meter_t | Optional target parameter |
| `CalculateLauncherDistanceToTarget()` | Distance from launcher | meter_t | More accurate for offset mechanisms |
| `CalculateAngleToTarget()` | Angle in robot frame | degree_t | Use for turret/shooter aim |
| `CalculateLauncherAngleToTarget()` | Angle from launcher in robot frame | degree_t | More accurate for offset mechanisms |
| `CalculateVirtualTarget()` | Compensate for movement | Translation2d | Pass result to angle/distance methods |
| `SetLauncherOffset()` | Configure mechanism position | void | In robot frame coordinates |
| `GetLauncherOffset()` | Query launcher offset | Translation2d | Useful for validation |
