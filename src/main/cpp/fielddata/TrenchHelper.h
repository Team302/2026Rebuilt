//====================================================================================================================================================
// Copyright 2026 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================
#pragma once

// C++ includes

#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "fielddata/BumpHelper.h"
#include "fielddata/FieldConstants.h"

//====================================================================================================================================================
/// @enum TRENCH_ID
/// @brief Enumeration identifying the four trenches on the 2026 game field
///
/// Each trench runs along the boundary between an alliance zone and the neutral zone.
/// There are four trenches total — one on each side (red/blue) at both the depot and outpost positions.
/// Robots must navigate through or around a trench when transitioning between zones.
//====================================================================================================================================================
enum class TRENCH_ID
{
    BLUE_DEPOT_TRENCH,   ///< Trench on the blue alliance depot side
    BLUE_OUTPOST_TRENCH, ///< Trench on the blue alliance outpost side
    RED_DEPOT_TRENCH,    ///< Trench on the red alliance depot side
    RED_OUTPOST_TRENCH   ///< Trench on the red alliance outpost side
};

//====================================================================================================================================================
/// @class TrenchHelper
/// @brief Singleton helper class for trench identification
///
/// This utility class provides a convenient interface for determining which field trench is nearest
/// to the robot's current position. It delegates the underlying proximity calculation to BumpHelper
/// and maps the resulting BUMP_ID to the corresponding TRENCH_ID.
///
/// **Key Functionality:**
/// - Identifies the nearest trench among the four field trenches
/// - Wraps BumpHelper::CalcNearestBump() and translates BUMP_ID → TRENCH_ID
///
/// **Usage Pattern:**
/// ```cpp
/// auto trenchHelper = TrenchHelper::GetInstance();
/// TRENCH_ID nearestTrench = trenchHelper->CalcNearestTrench();
/// // Use nearestTrench to determine navigation strategy
/// ```
///
/// **Integration:**
/// The class works in conjunction with:
/// - BumpHelper: Performs the actual nearest-bump distance calculation
/// - FieldConstants: For field element position reference data (via BumpHelper)
/// - ChassisConfigMgr: For access to current robot pose (via BumpHelper)
///
/// @note This is a singleton class - use GetInstance() to access
/// @see TRENCH_ID for the four possible trench identifications
/// @see BumpHelper for the underlying bump identification logic
//====================================================================================================================================================
class TrenchHelper
{
public:
    //------------------------------------------------------------------
    /// @brief      Get the singleton instance of TrenchHelper
    /// @return     TrenchHelper* - Pointer to the singleton instance
    /// @details    Creates the instance on first call using lazy initialization.
    ///             Subsequent calls return the same instance. Thread-safe in
    ///             single-threaded robot code execution context.
    //------------------------------------------------------------------
    static TrenchHelper *GetInstance();

    //------------------------------------------------------------------
    /// @brief      Identifies the trench nearest to the robot's current position
    /// @return     TRENCH_ID - Enumeration indicating which of the four trenches is closest
    /// @details    Delegates to BumpHelper::CalcNearestBump() and maps the result:
    ///
    ///             | BUMP_ID             | TRENCH_ID            |
    ///             |---------------------|----------------------|
    ///             | BLUE_DEPOT_BUMP     | BLUE_DEPOT_TRENCH    |
    ///             | BLUE_OUTPOST_BUMP   | BLUE_OUTPOST_TRENCH  |
    ///             | RED_DEPOT_BUMP      | RED_DEPOT_TRENCH     |
    ///             | RED_OUTPOST_BUMP    | RED_OUTPOST_TRENCH   |
    ///
    /// @note       Method is const - does not modify TrenchHelper state
    /// @see        BumpHelper::CalcNearestBump() for the underlying bump identification logic
    //------------------------------------------------------------------
    TRENCH_ID CalcNearestTrench() const;

private:
    //------------------------------------------------------------------
    /// @brief      Private constructor for singleton pattern
    /// @details    Initializes member pointers by retrieving singleton instances:
    ///             - ChassisConfigMgr: For access to swerve drivetrain and robot pose
    ///             - FieldConstants: For field element position reference data
    ///
    ///             Called only by GetInstance() on first access to create the singleton.
    //------------------------------------------------------------------
    TrenchHelper();

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    /// @details    No special cleanup required as member pointers reference
    ///             other singleton objects managed elsewhere
    //------------------------------------------------------------------
    ~TrenchHelper() = default;

    //------------------------------------------------------------------
    // Member Variables
    //------------------------------------------------------------------

    /// @brief Pointer to the swerve drivetrain subsystem for robot pose queries
    subsystems::CommandSwerveDrivetrain *m_chassis;

    /// @brief Pointer to field constants singleton for field element positions
    FieldConstants *m_fieldConstants;

    /// @brief Singleton instance pointer (lazy initialization)
    static TrenchHelper *m_instance;
};
