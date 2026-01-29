# Target Calculator - Technical Deep Dive

## Mathematical Foundations

### 1. Coordinate Frame Transformations

The target calculator operates across three different coordinate systems:

#### World Frame (Field Coordinates)
- Fixed reference frame relative to the field
- Used for absolute target positions
- Origin is typically at field center

#### Robot Frame (Local Coordinates)
- Moves with the robot
- X-axis points forward (robot heading)
- Y-axis points left
- Origin at robot center

#### Mechanism Frame
- Offset from robot center in robot frame
- Allows for shooting mechanisms not at robot center

### 2. Virtual Target Formula

The virtual target compensates for robot motion during projectile flight:

$$V_{target} = T_{real} - (V_{robot} \times t_{lookahead})$$

Where:
- $V_{target}$ = Virtual target position (world frame)
- $T_{real}$ = Real target position (world frame)
- $V_{robot}$ = Robot velocity (field frame)
- $t_{lookahead}$ = Projectile flight time

**In component form**:
$$V_{target,x} = T_{x} - V_{vx} \times t_{lookahead}$$
$$V_{target,y} = T_{y} - V_{vy} \times t_{lookahead}$$

### 3. Mechanism Position Calculation

The mechanism position in world coordinates is calculated by:

1. Start with offset in robot frame: $M_{robot} = (x_{offset}, y_{offset})$
2. Rotate by robot heading: $M_{rotated} = M_{robot} \cdot R_{robot}$
3. Add to robot position: $M_{world} = P_{robot} + M_{rotated}$

**Matrix form**:
$$\begin{bmatrix} M_x \\ M_y \end{bmatrix} = \begin{bmatrix} R_x \\ R_y \end{bmatrix} + \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} x_{offset} \\ y_{offset} \end{bmatrix}$$

Where $\theta$ is the robot heading.

### 4. Angle Calculation

The angle from mechanism to target:

$$\theta_{target} = \text{atan2}(T_y - M_y, T_x - M_x)$$

Converting to robot-relative (with wraparound handling):

$$\theta_{relative} = \theta_{target} - \theta_{robot}$$

## Code Walkthrough: GetLauncherTarget()

### RebuiltTargetCalculator Implementation

```cpp
units::angle::degree_t RebuiltTargetCalculator::GetLauncherTarget(
    units::time::second_t lookaheadTime, 
    units::angle::degree_t currentLauncherAngle)
{
    // Step 1: Update field visualization with current state
    m_field->UpdateObject("TargetPosition", GetVirtualTargetPose(lookaheadTime));
```

This updates the Field2d widget in Shuffleboard for debugging. The virtual target pose shows where we're actually aiming.

```cpp
    // Step 2: Calculate angle from mechanism to target in field frame
    units::degree_t fieldAngleToTarget = CalculateMechanismAngleToTarget(lookaheadTime);
    auto robotPose = GetChassisPose();
```

At this point:
- `fieldAngleToTarget`: Angle in world frame from mechanism to virtual target
- `robotPose`: Robot position and heading in world frame

```cpp
    // Step 3: Convert to robot-relative angle
    frc::Rotation2d relativeRot = frc::Rotation2d(fieldAngleToTarget) - robotPose.Rotation();
    units::degree_t robotRelativeGoal = relativeRot.Degrees();
```

This subtracts the robot's heading from the absolute angle to get the direction relative to the robot's forward direction.

**Example**:
- Field angle to target: 45°
- Robot heading: 30°
- Robot-relative angle: 45° - 30° = 15° (to the left)

```cpp
    // Step 4: Find best angle within launcher constraints
    units::degree_t bestAngle = 0_deg;
    bool hasFoundValidAngle = false;
    units::degree_t minError = 360_deg; // Large initial value
    
    for (int i = -1; i <= 1; i++)
    {
        // Check angles at -360°, 0°, and +360° offsets
        units::degree_t potentialSetpoint = robotRelativeGoal + (360_deg * i);
```

This loop handles the wraparound issue. Because angles are circular, there can be multiple equivalent angles:
- If goal is 350°, we could command: 350°, -10° (= 350° - 360°), or 710° (= 350° + 360°)

The loop checks all three possibilities (at -360°, 0°, and +360° offsets) to find which one is closest to the current angle.

```cpp
        if (potentialSetpoint >= m_minLauncherAngle && potentialSetpoint <= m_maxLauncherAngle)
        {
            // This angle is within the physical limits of the launcher
            auto error = units::math::abs(potentialSetpoint - currentLauncherAngle);
            if (error < minError)
            {
                bestAngle = potentialSetpoint;
                minError = error;
                hasFoundValidAngle = true;
            }
        }
    }
```

For each valid angle, calculate the distance to the current angle and keep the one that requires the least movement.

```cpp
    // Step 5: Handle case where no angle within constraints is close
    if (!hasFoundValidAngle)
    {
        units::degree_t normalizedGoal = relativeRot.Degrees();
        if (normalizedGoal < 0_deg)
            normalizedGoal += 360_deg;
        bestAngle = std::clamp(normalizedGoal, m_minLauncherAngle, m_maxLauncherAngle);
    }
```

If the goal angle is completely outside the mechanical limits, clamp it to the nearest limit (min or max).

**Example**:
- Goal angle: 80° (above 90° minimum)
- Min angle: 90°
- Clamped to: 90°

```cpp
    // Step 6: Update field visualization with result
    auto robotPose_var = GetChassisPose();
    m_field->UpdateObject("LauncherPosition", 
        frc::Pose2d(GetMechanismWorldPosition(), 
                    robotPose_var.Rotation() + frc::Rotation2d(bestAngle)));
    
    return bestAngle;
}
```

Update the field visualization and return the final angle.

## Worked Example

### Scenario Setup
- **Robot Position**: (2.0 m, 3.0 m) with heading 45°
- **Mechanism Offset**: (-0.1 m, 0 m) in robot frame
- **Target**: (3.0 m, 4.0 m)
- **Robot Velocity**: (1.0 m/s, 0 m/s) in field frame
- **Lookahead Time**: 0.5 s
- **Current Launcher Angle**: 180°
- **Launcher Constraints**: 90° to 270°

### Step-by-Step Calculation

#### Step 1: Calculate Mechanism World Position

Robot rotation: 45° = $\frac{\pi}{4}$ radians

Rotation matrix:
$$\begin{bmatrix} \cos(45°) & -\sin(45°) \\ \sin(45°) & \cos(45°) \end{bmatrix} = \begin{bmatrix} 0.707 & -0.707 \\ 0.707 & 0.707 \end{bmatrix}$$

Offset in world frame:
$$M_{world} = \begin{bmatrix} 0.707 & -0.707 \\ 0.707 & 0.707 \end{bmatrix} \begin{bmatrix} -0.1 \\ 0 \end{bmatrix} = \begin{bmatrix} -0.071 \\ -0.071 \end{bmatrix}$$

Mechanism position:
$$M = \begin{bmatrix} 2.0 \\ 3.0 \end{bmatrix} + \begin{bmatrix} -0.071 \\ -0.071 \end{bmatrix} = \begin{bmatrix} 1.929 \\ 2.929 \end{bmatrix}$$

#### Step 2: Calculate Virtual Target

Velocity offset over lookahead time:
$$\Delta = V \times t = \begin{bmatrix} 1.0 \\ 0 \end{bmatrix} \times 0.5 = \begin{bmatrix} 0.5 \\ 0 \end{bmatrix}$$

Virtual target:
$$V_{target} = \begin{bmatrix} 3.0 \\ 4.0 \end{bmatrix} - \begin{bmatrix} 0.5 \\ 0 \end{bmatrix} = \begin{bmatrix} 2.5 \\ 4.0 \end{bmatrix}$$

#### Step 3: Calculate Angle to Virtual Target

Vector from mechanism to virtual target:
$$\vec{v} = \begin{bmatrix} 2.5 - 1.929 \\ 4.0 - 2.929 \end{bmatrix} = \begin{bmatrix} 0.571 \\ 1.071 \end{bmatrix}$$

Angle:
$$\theta_{field} = \text{atan2}(1.071, 0.571) = 61.87°$$

#### Step 4: Convert to Robot-Relative

Robot-relative angle:
$$\theta_{relative} = 61.87° - 45° = 16.87°$$

#### Step 5: Find Best Valid Angle

Check angles at offsets -360°, 0°, +360°:
- $-360° + 16.87° = -343.13°$ → Outside constraints (< 90°)
- $0° + 16.87° = 16.87°$ → Outside constraints (< 90°)
- $+360° + 16.87° = 376.87°$ → Outside constraints (> 270°)

#### Step 6: Clamp to Constraints

Normalize goal angle:
$$\theta_{normalized} = 16.87°$$

Clamp to [90°, 270°]:
$$\theta_{final} = \text{clamp}(16.87°, 90°, 270°) = 90°$$

### Result

**Launcher Angle: 90°**

The launcher is commanded to 90° (minimum angle) because the target is too far forward for the launcher to reach with its current constraints.

## Implementation Checklist

When implementing a new `TargetCalculator` child class:

- [ ] Create `.h` and `.cpp` files in `src/main/cpp/utils/`
- [ ] Implement singleton pattern with `GetInstance()`
- [ ] Call `SetMechanismOffset()` in constructor
- [ ] Initialize `DragonField` reference for visualization
- [ ] Implement `GetTargetPosition()` override
- [ ] Implement `GetLauncherTarget()` with your launch logic
- [ ] Define `m_minLauncherAngle` and `m_maxLauncherAngle` constants
- [ ] Test with Field2d visualization in Shuffleboard
- [ ] Verify mechanism offset by comparing with robot CAD
- [ ] Calibrate lookahead time based on projectile behavior
- [ ] Document season-specific configuration

## Performance Considerations

- **Calculation Time**: ~1-2 ms per call (dominated by chassis state retrieval)
- **Update Frequency**: Typically 50 Hz (20 ms periodic)
- **Floating Point Precision**: Sufficient for FRC applications (double precision)

## Testing Strategy

### Unit Tests
Test calculations with known inputs:
```cpp
// Example: Verify mechanism position calculation
frc::Pose2d robotPose{2.0_m, 3.0_m, 45_deg};
frc::Translation2d mechanismPos = calculator->GetMechanismWorldPosition();
// Verify against hand-calculated expected value
```

### Field Testing
1. Position robot at known locations
2. Check Field2d visualization
3. Verify target and launcher positions are correct
4. Test with and without velocity compensation

### Simulation
Use WPILib simulation with:
- Fixed robot positions
- Scripted velocities
- Verify angle calculations

## References

### WPILib Classes
- `frc::Translation2d` - 2D position vector
- `frc::Pose2d` - Position + rotation
- `frc::Rotation2d` - Rotation/heading
- `frc::ChassisSpeeds` - Velocity in field frame
- `frc::Field2d` - Visualization tool

### Units Library
- `units::meter_t` - Distance with compile-time checking
- `units::degree_t` - Angle with compile-time checking
- `units::time::second_t` - Time with compile-time checking

### Key Methods
- `Translation2d::Distance()` - Euclidean distance
- `Translation2d::Angle()` - Angle to point
- `Rotation2d::Degrees()` - Convert rotation to degrees
- `ChassisSpeeds::FromRobotRelativeSpeeds()` - Convert velocity frames
