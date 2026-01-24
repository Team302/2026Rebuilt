# TargetCalculator Refactor Summary

## Changes Made

### 1. **Removed GamePieceTargetCalculator**
   - Deleted `GamePieceTargetCalculator.h`
   - Deleted `GamePieceTargetCalculator.cpp`
   - Reason: Will be replaced with integration to FieldElementCalculator and ZoneCalculator when available

### 2. **Updated TargetCalculator Base Class**

#### Made GetTargetPosition() Non-Const
   - **Before**: `virtual frc::Translation2d GetTargetPosition() const;`
   - **After**: `virtual frc::Translation2d GetTargetPosition();`
   - **Reason**: Allows subclasses to dynamically update target based on chassis position, zone, or other factors

#### Updated All Calculation Methods to Non-Const
   - `CalculateDistanceToTarget()`
   - `CalculateLauncherDistanceToTarget()`
   - `CalculateAngleToTarget()`
   - `CalculateLauncherAngleToTarget()`
   - **Reason**: These methods call GetTargetPosition(), which is now non-const

#### Made Launcher Offset Virtual and Moved to Subclass
   - Changed `SetLauncherOffset()` and `GetLauncherOffset()` to virtual
   - Removed `m_launcherOffset` member variable from base class
   - Moved implementation to subclass where it belongs
   - **Reason**: Each mechanism type (shooter, intake, etc.) may have different offsets

#### Updated GetLauncherWorldPosition()
   - Now calls virtual `GetLauncherOffset()` instead of using member variable
   - Maintains const-ness of this method (uses const_cast for temporary non-const access)

### 3. **Created RebuiltTargetCalculator Season-Specific Class**

#### RebuiltTargetCalculator.h
   - Extends `TargetCalculator`
   - Provides hardcoded hub target for testing (4.625m, 4.025m)
   - Overrides `GetTargetPosition()` to return hub target
   - Implements launcher offset storage and configuration
   - Contains TODO comments for future FieldElementCalculator integration

#### RebuiltTargetCalculator.cpp
   - Implements singleton pattern
   - Provides default launcher offset (-0.1397m, 0m = 5.5" back, centered)
   - Implements SetLauncherOffset() and GetLauncherOffset() overrides
   - Ready for future dynamic target selection

### 4. **Updated Documentation**
   - Updated `TargetCalculator_USAGE.txt` with RebuiltTargetCalculator examples
   - Updated `TARGETCALCULATOR_README.md` with new class structure
   - Added TODO comments for future enhancements

## Architecture Now

```
TargetCalculator (Base Class)
├── Pure math and coordinate transformations
├── Virtual methods for subclass customization:
│   ├── GetTargetPosition()
│   ├── SetLauncherOffset()
│   └── GetLauncherOffset()
└── Query methods:
    ├── GetChassisPose()
    └── GetChassisVelocity()

RebuiltTargetCalculator (Season-Specific)
├── Hardcoded hub target for testing
├── Launcher offset configuration (5.5" back)
└── Ready for FieldElementCalculator integration
```

## Key Design Decisions

### 1. Non-Const GetTargetPosition()
- **Pro**: Allows dynamic target calculation based on state
- **Pro**: Can integrate with zone detectors
- **Con**: Not purely const-correct, but necessary for flexibility

### 2. Virtual Launcher Offset Management
- **Pro**: Each mechanism type can have different offsets
- **Pro**: Allows runtime configuration per mechanism
- **Con**: Slight performance cost from virtual function calls (negligible)

### 3. Hardcoded Hub Target
- **Pro**: Enables immediate testing without FieldElementCalculator
- **Pro**: Easy to validate coordinate transformations
- **Con**: Temporary solution; TODO for replacement

## Usage Example

```cpp
#include "utils/RebuiltTargetCalculator.h"

// In robot initialization
auto targetCalc = RebuiltTargetCalculator::GetInstance();
targetCalc->SetLauncherOffset(-0.1397_m, 0_m);  // 5.5" back

// In command loop
auto distance = targetCalc->CalculateLauncherDistanceToTarget();
auto angle = targetCalc->CalculateLauncherAngleToTarget();

// For moving shots with compensation
auto realTarget = targetCalc->GetTargetPosition();
auto virtualTarget = targetCalc->CalculateVirtualTarget(realTarget, 1.0_s);
auto compensatedAngle = targetCalc->CalculateAngleToTarget(&virtualTarget);
```

## Testing Ready

The new system is ready for testing:
- ✅ Distance calculations from chassis center
- ✅ Distance calculations from launcher mechanism
- ✅ Angle calculations in robot frame
- ✅ Virtual target compensation for moving robot
- ✅ Launcher offset configuration
- ⏳ Dynamic zone-based target selection (awaits FieldElementCalculator)
- ⏳ Multiple target types (awaits game analysis)

## Next Steps

1. **Test the basic calculations** with actual robot poses
2. **Verify coordinate transformations** in simulation
3. **Implement FieldElementCalculator** for field element positions
4. **Implement ZoneCalculator** for zone detection
5. **Update GetTargetPosition()** to query FieldElementCalculator based on zone
6. **Add additional mechanism types** (intake, etc.) with their own offsets
