# Target Calculator - Implementation Guide

## Quick Start: Creating Your Target Calculator

This guide walks you through creating a season-specific target calculator for your robot. We'll use the `RebuiltTargetCalculator` as our reference implementation.

## Phase 1: Preparation

Before writing code, gather the following information:

### 1. Target Positions

Measure or determine the coordinates of all targets in your game:

```
FRC 2026 Rebuilt:
- Hub: (4.625 m, 4.025 m)

Your Game:
- Target 1: (X m, Y m)
- Target 2: (X m, Y m)
- Target 3: (X m, Y m)
```

**Coordinate System**: Origin at field center, measure in meters. Consult your field diagram in meters.

### 2. Mechanism Offset

Measure the distance from your robot's center to the launching mechanism:

```
Example (2026):
- Distance back: 0.1397 m (5.5 inches)
- Distance left/right: 0 m (centered)
```

**How to measure**:
1. Find robot center (geometric center between wheels or specified in CAD)
2. Find mechanism center (shooting mechanism, climber, intake, etc.)
3. Calculate X offset (+ = forward, - = back)
4. Calculate Y offset (+ = left, - = right)

### 3. Launcher Angle Constraints

Determine the minimum and maximum angles your mechanism can reach:

```cpp
const units::degree_t m_minLauncherAngle = 90_deg;   // Minimum angle
const units::degree_t m_maxLauncherAngle = 270_deg;  // Maximum angle
```

**Coordinate System**: 
- 0° = Forward (robot +X)
- 90° = Left (robot +Y)
- 180° = Back (robot -X)
- 270° = Right (robot -Y)

### 4. Lookahead Time

Estimate projectile flight time:

```cpp
units::time::second_t lookaheadTime = 0.25_s;  // Adjust based on testing
```

**How to determine**:
- Measure projectile distance
- Measure projectile velocity
- Time = Distance / Velocity
- Example: 5 m at 20 m/s = 0.25 seconds

## Phase 2: File Creation

### Step 1: Create Header File

Create `src/main/cpp/utils/YourTargetCalculator.h`:

```cpp
#pragma once

#include "utils/TargetCalculator.h"
#include "utils/DragonField.h"

#include <frc/geometry/Translation2d.h>
#include <units/length.h>

/**
 * \class YourTargetCalculator
 * \brief [YEAR]-specific target calculator
 *
 * This class extends TargetCalculator with [YEAR]-specific configuration:
 * - [Game] target positions
 * - Mechanism offset configuration for the [YEAR] robot
 *
 * TODO: [Add any season-specific integration notes]
 */
class YourTargetCalculator : public TargetCalculator
{
public:
    /**
     * \brief Get singleton instance
     * \return Pointer to the YourTargetCalculator singleton
     */
    static YourTargetCalculator *GetInstance();

    /**
     * \brief Get the current target position
     * \return Translation2d with target position in meters (world frame)
     */
    frc::Translation2d GetTargetPosition() override;

    /**
     * \brief Get the launcher angle target
     * \param lookaheadTime Time for projectile flight compensation
     * \param currentLauncherAngle Current launcher angle for minimum movement
     * \return Angle in degrees
     */
    units::angle::degree_t GetLauncherTarget(
        units::time::second_t lookaheadTime, 
        units::angle::degree_t currentLauncherAngle);

private:
    /**
     * \brief Constructor - initializes with default mechanism offset
     */
    YourTargetCalculator();

    static YourTargetCalculator *m_instance;

    // ===== SEASON-SPECIFIC CONFIGURATION =====
    
    // Target positions (measure from field diagram in meters)
    frc::Translation2d m_primaryTarget{0.0_m, 0.0_m};    // TODO: Update
    frc::Translation2d m_alternateTarget{0.0_m, 0.0_m};  // TODO: Update
    
    // Mechanism offset from robot center (measure from robot CAD)
    // Positive X = forward, Positive Y = left
    frc::Translation2d m_mechanismOffset{0.0_m, 0.0_m};  // TODO: Update
    
    // Launcher angle constraints (degrees, based on mechanism design)
    // 0° = forward, 90° = left, 180° = back, 270° = right
    const units::degree_t m_minLauncherAngle = 0_deg;    // TODO: Update
    const units::degree_t m_maxLauncherAngle = 360_deg;  // TODO: Update
    
    // ===== END SEASON-SPECIFIC CONFIGURATION =====

    DragonField *m_field;
};
```

### Step 2: Create Implementation File

Create `src/main/cpp/utils/YourTargetCalculator.cpp`:

```cpp
#include "utils/YourTargetCalculator.h"

YourTargetCalculator::YourTargetCalculator() : TargetCalculator()
{
    // Initialize base class with mechanism offset
    SetMechanismOffset(m_mechanismOffset);

    // Initialize field visualization
    m_field = DragonField::GetInstance();
    m_field->AddPose("TargetPosition", frc::Pose2d(GetTargetPosition(), frc::Rotation2d()));
    m_field->AddPose("LauncherPosition", frc::Pose2d());
}

YourTargetCalculator *YourTargetCalculator::m_instance = nullptr;

YourTargetCalculator *YourTargetCalculator::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new YourTargetCalculator();
    }
    return m_instance;
}

frc::Translation2d YourTargetCalculator::GetTargetPosition()
{
    // TODO: Replace with dynamic target selection based on:
    // - Game state
    // - Robot position
    // - FieldElementCalculator (if available)
    
    // For now, return primary target for testing
    return m_primaryTarget;
}

units::angle::degree_t YourTargetCalculator::GetLauncherTarget(
    units::time::second_t lookaheadTime, 
    units::angle::degree_t currentLauncherAngle)
{
    // Update field visualization with virtual target
    m_field->UpdateObject("TargetPosition", GetVirtualTargetPose(lookaheadTime));

    // Calculate angle from mechanism to target in field frame
    units::degree_t fieldAngleToTarget = CalculateMechanismAngleToTarget(lookaheadTime);
    auto robotPose = GetChassisPose();

    // Convert to robot-relative angle
    frc::Rotation2d relativeRot = frc::Rotation2d(fieldAngleToTarget) - robotPose.Rotation();
    units::degree_t robotRelativeGoal = relativeRot.Degrees();

    // Find best angle within constraints, prioritizing minimal movement
    units::degree_t bestAngle = 0_deg;
    bool hasFoundValidAngle = false;
    units::degree_t minError = 360_deg; // Large initial value

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

    // If no valid angle found, clamp to nearest limit
    if (!hasFoundValidAngle)
    {
        units::degree_t normalizedGoal = relativeRot.Degrees();
        if (normalizedGoal < 0_deg)
            normalizedGoal += 360_deg;
        bestAngle = std::clamp(normalizedGoal, m_minLauncherAngle, m_maxLauncherAngle);
    }

    // Update field visualization with launcher position
    auto robotPose_var = GetChassisPose();
    m_field->UpdateObject("LauncherPosition", 
        frc::Pose2d(GetMechanismWorldPosition(), robotPose_var.Rotation() + frc::Rotation2d(bestAngle)));

    return bestAngle;
}
```

## Phase 3: Configuration

### Step 1: Measure and Input Target Positions

1. **Get Field Coordinates**: Refer to your game's field diagram (measurements in meters)
2. **Update Header File**: Replace TODO values with actual measurements
3. **Verify Units**: Ensure all measurements are in meters

```cpp
// Example for 2026:
frc::Translation2d m_primaryTarget{4.625_m, 4.025_m};
```

### Step 2: Calculate Mechanism Offset

**Procedure**:

1. Open your robot CAD or physically measure:
   - Robot center position
   - Mechanism (shooter, climber, etc.) center position

2. Calculate offset in robot frame:
   ```
   X offset = Mechanism X - Robot Center X
   Y offset = Mechanism Y - Robot Center Y
   ```

3. Update header file:
   ```cpp
   frc::Translation2d m_mechanismOffset{-0.1397_m, 0_m};
   ```

**Sign Convention**:
- X: Positive = forward, Negative = back
- Y: Positive = left, Negative = right

### Step 3: Determine Angle Constraints

1. **Physical Limits**: Check your mechanism's range of motion
   - Minimum angle: Lowest it can reach
   - Maximum angle: Highest it can reach

2. **Update Constants**:
   ```cpp
   const units::degree_t m_minLauncherAngle = 90_deg;
   const units::degree_t m_maxLauncherAngle = 270_deg;
   ```

**Remember**: Angle coordinate system
- 0° = robot forward (+X)
- 90° = robot left (+Y)
- 180° = robot back (-X)
- 270° = robot right (-Y)

## Phase 4: Integration

### Step 1: Update Build Files

1. Open `build.gradle`
2. Ensure your new `.cpp` and `.h` files are included in the build

### Step 2: Update Header Includes

In your command/subsystem files that use the calculator:

```cpp
#include "utils/YourTargetCalculator.h"

// Use in a command or subsystem:
YourTargetCalculator *calculator = YourTargetCalculator::GetInstance();
units::angle::degree_t launcherAngle = calculator->GetLauncherTarget(0.25_s, currentAngle);
```

### Step 3: Test Basic Functionality

```cpp
// Simple test code
auto calculator = YourTargetCalculator::GetInstance();

// Test 1: Get target position
auto target = calculator->GetTargetPosition();
printf("Target: (%f, %f)\n", target.X().value(), target.Y().value());

// Test 2: Get launcher angle with no velocity
auto angle = calculator->GetLauncherTarget(0_s, 180_deg);
printf("Launcher angle (no compensation): %f\n", angle.value());

// Test 3: Get launcher angle with velocity compensation
auto angleCompensated = calculator->GetLauncherTarget(0.25_s, 180_deg);
printf("Launcher angle (with compensation): %f\n", angleCompensated.value());
```

## Phase 5: Testing and Tuning

### Test 1: Verify Target Position

1. Enable robot code
2. Open Shuffleboard
3. Add "Field2d" widget
4. Check that "TargetPosition" appears on the field at the correct location
5. If incorrect, verify target coordinates

### Test 2: Verify Mechanism Offset

1. Drive robot to a known position
2. Rotate robot to a known heading
3. Check Field2d visualization for "LauncherPosition"
4. Compare with physical robot position
5. If offset, recalibrate m_mechanismOffset

### Test 3: Verify Angle Calculation

1. Position robot facing different directions
2. Monitor launcher angle in Shuffleboard
3. Verify it points toward target
4. Test with and without movement

### Test 4: Tune Lookahead Time

1. Shoot at the target from various distances
2. Note whether shots hit short or long
3. Adjust lookahead time:
   - If hitting short: Increase lookahead time
   - If hitting long: Decrease lookahead time
4. Iterate until accurate

### Test 5: Verify Angle Constraints

1. Position robot so target requires angle outside constraints
2. Verify launcher clamps to min/max angle
3. Confirm it still attempts to hit target as closely as possible

## Common Configuration Mistakes

### Mistake 1: Wrong Units

**Problem**: Target coordinates in inches instead of meters
```cpp
// WRONG:
frc::Translation2d m_target{18_in, 13_in};  // Mixes units

// CORRECT:
frc::Translation2d m_target{0.4572_m, 0.3302_m};  // All meters
```

### Mistake 2: Sign Error in Offset

**Problem**: Offset pointing wrong direction
```cpp
// If mechanism is 5.5 inches BACK but you put:
frc::Translation2d m_mechanismOffset{0.1397_m, 0_m};  // Wrong! Points forward

// Should be:
frc::Translation2d m_mechanismOffset{-0.1397_m, 0_m};  // Points back
```

**Fix**: Check against CAD. Negative X = back, Negative Y = right.

### Mistake 3: Inverted Angle Constraints

**Problem**: Min angle > max angle
```cpp
// WRONG:
const units::degree_t m_minLauncherAngle = 270_deg;
const units::degree_t m_maxLauncherAngle = 90_deg;

// CORRECT:
const units::degree_t m_minLauncherAngle = 90_deg;
const units::degree_t m_maxLauncherAngle = 270_deg;
```

### Mistake 4: Zero Lookahead Time

**Problem**: Virtual target calculation is disabled
```cpp
// If you always pass 0_s, no velocity compensation occurs:
calculator->GetLauncherTarget(0_s, currentAngle);  // No compensation

// Provide realistic lookahead time:
calculator->GetLauncherTarget(0.25_s, currentAngle);  // 250 ms flight time
```

## Debugging Checklist

When something isn't working:

- [ ] Check Field2d visualization - is target at correct location?
- [ ] Check mechanism offset visualization - is it offset correctly?
- [ ] Verify target position coordinates with field diagram
- [ ] Confirm launcher angle is within min/max bounds
- [ ] Check that chassis is returning valid poses and velocities
- [ ] Verify lookahead time is reasonable for your projectile
- [ ] Test with robot stationary first (no velocity compensation)
- [ ] Print debug values to understand calculation flow

## Example Debug Output

```cpp
// Add to GetLauncherTarget for debugging:
printf("=== Target Calculator Debug ===\n");
printf("Robot Pose: (%f, %f) @ %f°\n", 
    robotPose.X().value(), robotPose.Y().value(), robotPose.Rotation().Degrees().value());
printf("Target: (%f, %f)\n", GetTargetPosition().X().value(), GetTargetPosition().Y().value());
printf("Mechanism Pos: (%f, %f)\n", 
    GetMechanismWorldPosition().X().value(), GetMechanismWorldPosition().Y().value());
printf("Field Angle: %f°\n", fieldAngleToTarget.value());
printf("Robot-Relative Angle: %f°\n", robotRelativeGoal.value());
printf("Best Angle: %f° (error: %f°)\n", bestAngle.value(), minError.value());
printf("===== End Debug =====\n");
```

## Next Steps

After successful testing:

1. **Integrate with Commands**: Use in shooter/climber command classes
2. **Add Dynamic Target Selection**: Implement game-state-based target selection
3. **Optimize**: Profile and optimize if needed
4. **Document**: Add season-specific notes to your implementation
5. **Iterate**: Tune based on competition feedback

## Quick Reference

| Parameter | Type | Units | Range | Notes |
|-----------|------|-------|-------|-------|
| Target Position | Translation2d | meters | Field-dependent | From field diagram |
| Mechanism Offset X | meters | m | ±2 | Negative = back |
| Mechanism Offset Y | meters | m | ±1 | Negative = right |
| Min Launcher Angle | degree_t | degrees | 0-360 | Must be < Max |
| Max Launcher Angle | degree_t | degrees | 0-360 | Must be > Min |
| Lookahead Time | second_t | seconds | 0.1-0.5 | Projectile flight time |

## Support Resources

- WPILib Documentation: https://docs.wpilib.org/
- Team 302 Resources: [Internal wiki]
- FRC Game Manual: [Year-specific manual]
