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

#include "fielddata/BumpHelper.h"
#include "frc/geometry/Pose2d.h"
#include "units/angle.h"
#include <vector>

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

    //------------------------------------------------------------------
    /// @brief      Returns an ordered list of Pose2d drive targets for navigating through a trench
    /// @param[in]  isRedAlliance - true for red alliance, false for blue alliance
    /// @return     std::vector<frc::Pose2d> - Ordered poses: index 0 is the mid (near-trench)
    ///             pose, index 1 is the end (far-side) pose
    /// @details    Determines the nearest trench via CalcNearestTrench(), selects the correct
    ///             starting and ending trench pair for the given alliance, and builds Pose2d
    ///             waypoints using FieldOffsetValues trench X/Y coordinates.
    /// @see        CalcNearestTrench() for trench identification
    /// @see        FieldOffsetValues for the coordinate source data
    //------------------------------------------------------------------
    std::vector<frc::Pose2d> GetTrenchDrivePositions(bool isRedAlliance) const;

private:
    //------------------------------------------------------------------
    /// @brief      Private constructor for singleton pattern
    /// @details    Initializes member pointers by retrieving singleton instances:
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

    /// @brief Singleton instance pointer (lazy initialization)
    static TrenchHelper *m_instance;

    static constexpr units::angle::degree_t kFaceRedWallRotation{0_deg};    ///< Rotation to face the red alliance wall (used for trench navigation)
    static constexpr units::angle::degree_t kFaceBlueWallRotation{180_deg}; ///< Rotation to face the blue alliance wall (used for trench navigation)
};
