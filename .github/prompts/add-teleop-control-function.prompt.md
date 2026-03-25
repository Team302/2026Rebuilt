# Add Teleop Control Function Mapping

Add a new teleop control function and map it to a gamepad button or axis.

## Required Information (prompt the user for each)

1. **Function identifier** — The `UPPER_SNAKE_CASE` name for the new function (e.g., `SCORE_LOW`, `ARM_EXTEND`).
2. **Controller** — Which controller to map to: `DRIVER`, `CO_PILOT`, `EXTRA1`, `EXTRA2`, `EXTRA3`, or `EXTRA4`.
3. **Input type** — Is this mapped to a **button** or an **axis**?
4. **Which input** — The specific button or axis:
   - **Buttons:** `A_BUTTON`, `B_BUTTON`, `X_BUTTON`, `Y_BUTTON`, `LEFT_BUMPER`, `RIGHT_BUMPER`, `SELECT_BUTTON`, `START_BUTTON`, `LEFT_STICK_PRESSED`, `RIGHT_STICK_PRESSED`, `LEFT_TRIGGER_PRESSED`, `RIGHT_TRIGGER_PRESSED`, `POV_0` (D-pad up), `POV_45`, `POV_90` (D-pad right), `POV_135`, `POV_180` (D-pad down), `POV_225`, `POV_270` (D-pad left), `POV_315`
   - **Axes:** `LEFT_JOYSTICK_X`, `LEFT_JOYSTICK_Y`, `RIGHT_JOYSTICK_X`, `RIGHT_JOYSTICK_Y`, `LEFT_TRIGGER`, `RIGHT_TRIGGER`

## Steps to Perform

### Step 1 — Add the function enum value

Open `src/main/cpp/teleopcontrol/TeleopControlFunctions.h`.

Add the new `UPPER_SNAKE_CASE` function identifier into the `TeleopControlFunctions::FUNCTION` enum. Place it in the appropriate section based on its purpose:
- General drive commands go under `// General Drive Commands`
- Season-specific drive commands go under `// Season Specific Drive Commands`
- Mechanism-specific commands go under the relevant mechanism comment (e.g., `// Intake`, `// Climber`, `// LAUNCHER`)
- If no existing section fits, add a new comment section before the entry

Do NOT add a trailing comma issue — ensure the last entry in the enum does not have a trailing comma (or that the new entry is inserted before the last entry with a comma).

### Step 2 — Add the mapping in TeleopControlMap.h

Open `src/main/cpp/teleopcontrol/TeleopControlMap.h`.

#### If mapping to a BUTTON:

Add an entry to the `teleopControlMapButtonMap` robin_hood map. Use the pre-defined `constexpr` button variables that already exist at the top of the file. The variable naming convention is:
- `driver` prefix → `TeleopControlMappingEnums::DRIVER`
- `copilot` prefix → `TeleopControlMappingEnums::CO_PILOT`
- `extra1` through `extra4` prefix → `TeleopControlMappingEnums::EXTRA1` through `EXTRA4`

Followed by the button name in PascalCase: `AButton`, `BButton`, `XButton`, `YButton`, `LBumper`, `RBumper`, `SelectButton`, `StartButton`, `LStickPressed`, `RStickPressed`, `LTriggerPressed`, `RTriggerPressed`, `DPad0`, `DPad45`, `DPad90`, `DPad135`, `DPad180`, `DPad225`, `DPad270`, `DPad315`.

Example for mapping `MY_FUNCTION` to the co-pilot's A button:
```cpp
{TeleopControlFunctions::MY_FUNCTION, copilotAButton},
```

Place the new entry in the appropriate section of the map (Driver Controls or Season Specific Co-Pilot Controls) with a comment if needed.

#### If mapping to an AXIS:

Add an entry to the `teleopControlMapAxisMap` robin_hood map. Use the pre-defined `constexpr` axis variables. The variable naming convention is the controller prefix (same as buttons) followed by: `LJoystickX`, `LJoystickY`, `RJoystickX`, `RJoystickY`, `LTrigger`, `RTrigger`.

Example for mapping `MY_FUNCTION` to the driver's left joystick Y:
```cpp
{TeleopControlFunctions::MY_FUNCTION, driverLJoystickY},
```

### Step 3 — Build verification

After making the changes, run the build to verify:
```
./build302.bat
```

## Important Notes

- Multiple functions CAN be mapped to the same button/axis (this is used for context-dependent controls, e.g., the same button does different things in climb mode vs. normal mode).
- Function names must be `UPPER_SNAKE_CASE` and unique within the `FUNCTION` enum.
- The `constexpr` button/axis variables at the top of `TeleopControlMap.h` should NOT be modified — use the existing ones.
- If the user needs a button mode other than `STANDARD` (e.g., `TOGGLE`), a new `constexpr TeleopControlButton` variable will need to be created with that mode.
- If the user needs a non-default axis configuration (different deadband, profile, direction, or scale), a new `constexpr TeleopControlAxis` variable will need to be created.
