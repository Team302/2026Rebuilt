```markdown
│   ├── RebuiltTargetCalculator       # Calculates target positions based on field data
│   ├── AngleUtils                   # Utility functions for angle calculations
│   ├── ConversionUtils               # Utility functions for data type conversions
│   ├── DragonField                   # Manages field-related data
│   ├── FMSData                       # Handles FMS data communication
│   ├── HardwareIDValidation          # Validates hardware IDs for components
│   ├── InterpolateUtils              # Provides interpolation functions
│   ├── NetworkTableReader            # Reads data from NetworkTables
│   ├── PoseUtils                     # Utilities for pose calculations
│   ├── RoboRio                       # Interfaces with the RoboRio hardware
│   ├── TargetCalculator              # Calculates target positions based on vision data
│   └── logging/                      # Logging utilities
│       ├── debug/                   # Debug logging utilities
│       ├── signals/                 # Signal logging utilities
│       └── Logger                    # Centralized logging interface
│
└── README.md                        # Project overview and setup instructions
```

## 15. Utilities

The `utils` directory contains various utility classes and functions that support the main robot functionality. Key components include:

- **RebuiltTargetCalculator**: This class is responsible for calculating target positions based on field data. It integrates with the `FieldConstants` to ensure accurate positioning relative to the field elements.

- **AngleUtils**: Provides utility functions for angle calculations, including conversions between degrees and radians, and angle normalization.

- **ConversionUtils**: Contains utility functions for converting between different data types, which is essential for handling inputs from various sensors and systems.

- **DragonField**: Manages field-related data, including the positions of field elements and their interactions with the robot.

- **FMSData**: Handles communication with the Field Management System (FMS), ensuring that the robot receives real-time game data.

- **HardwareIDValidation**: Validates the hardware IDs of components to ensure that the correct devices are being used and configured.

- **InterpolateUtils**: Provides functions for interpolation, which can be useful for estimating values between known data points.

- **NetworkTableReader**: Reads data from NetworkTables, which is used for communication between the robot and the driver station.

- **PoseUtils**: Contains utilities for calculating and manipulating robot poses, which are critical for navigation and control.

- **RoboRio**: Interfaces with the RoboRio hardware, providing functions for accessing and controlling various hardware components.

- **TargetCalculator**: Similar to `RebuiltTargetCalculator`, this class calculates target positions based on vision data, allowing the robot to align itself with game pieces.

- **logging/**: This subdirectory contains various logging utilities, including debug and signal logging, which are essential for monitoring the robot's performance and diagnosing issues.

---

## 18. Season-Dependent Change Checklist

- Review and update the `RebuiltTargetCalculator` to ensure it aligns with the 2026 game rules and field layout.
- Validate all utility functions in `AngleUtils` and `ConversionUtils` for accuracy with the new game elements.
- Ensure `FMSData` is correctly parsing the new game data formats introduced in the 2026 season.
- Confirm that `NetworkTableReader` is properly configured to communicate with the updated driver station software.
- Test all logging functionalities to ensure they capture relevant data for the 2026 season.
```
