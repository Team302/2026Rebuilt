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
#include <vector>

#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "fielddata/FieldConstants.h"
#include "frc/geometry/Pose2d.h"
#include "units/length.h"

//====================================================================================================================================================
/// @enum BUMP_ID
/// @brief Enumeration identifying the four field bumps that separate alliance and neutral zones
///
/// The 2026 game field has bumps at the boundaries between alliance zones and the neutral zone.
/// There are four bumps total - one on each side (red/blue) at both the depot and outpost positions.
/// These bumps are physical field obstacles that robots must navigate over when transitioning between zones.
//====================================================================================================================================================
enum class BUMP_ID
{
    BLUE_DEPOT_BUMP,   ///< Bump at the blue alliance depot, separating blue alliance zone from neutral zone
    BLUE_OUTPOST_BUMP, ///< Bump at the blue alliance outpost, separating blue alliance zone from neutral zone
    RED_DEPOT_BUMP,    ///< Bump at the red alliance depot, separating red alliance zone from neutral zone
    RED_OUTPOST_BUMP   ///< Bump at the red alliance outpost, separating red alliance zone from neutral zone
};

//====================================================================================================================================================
/// @struct BumpPosition
/// @brief Holds the X and Y coordinates for a bump identified by a BUMP_ID
//====================================================================================================================================================
struct BumpPosition
{
    BUMP_ID bumpId;           ///< Identifier for the bump
    units::length::meter_t x; ///< X-coordinate of the bump (meters)
    units::length::meter_t y; ///< Y-coordinate of the bump (meters)
};

//====================================================================================================================================================
/// @class BumpHelper
/// @brief Singleton helper class for bump-related calculations and identification
///
/// This utility class provides services for determining which field bump is nearest to the robot's
/// current position. It's primarily used by navigation commands that need to cross between the
/// alliance zone and neutral zone, such as DriveOverBump.
///
/// **Key Functionality:**
/// - Identifies the nearest bump among the four field bumps using a two-stage comparison algorithm
/// - First determines which alliance side (red or blue) is closer
/// - Then determines whether depot or outpost is closer on that side
///
/// **Usage Pattern:**
/// ```cpp
/// auto bumpHelper = BumpHelper::GetInstance();
/// BUMP_ID nearestBump = bumpHelper->CalcNearestBump();
/// // Use nearestBump to determine navigation strategy
/// ```
///
/// **Integration:**
/// The class works in conjunction with:
/// - FieldConstants: For bump position reference points
/// - PoseUtils: For distance calculations
/// - DriveOverBump: Primary consumer for bump crossing navigation
/// - FieldOffsetValues: For retrieving bump coordinates
///
/// @note This is a singleton class - use GetInstance() to access
/// @see BUMP_ID for the four possible bump identifications
/// @see DriveOverBump for primary usage example
//====================================================================================================================================================
class BumpHelper
{
public:
    //------------------------------------------------------------------
    /// @brief      Get the singleton instance of BumpHelper
    /// @return     BumpHelper* - Pointer to the singleton instance
    /// @details    Creates the instance on first call using lazy initialization.
    ///             Subsequent calls return the same instance. Thread-safe in
    ///             single-threaded robot code execution context.
    //------------------------------------------------------------------
    static BumpHelper *GetInstance();

    //------------------------------------------------------------------
    /// @brief      Calculate the ID of the nearest bump to the robot
    /// @return     BUMP_ID - Enumeration indicating which of the four bumps is closest
    /// @details    Uses a two-stage hierarchical comparison algorithm to efficiently
    ///             determine the nearest bump:
    ///
    ///             **Stage 1: Alliance Side Determination**
    ///             - Compares robot distance to blue hub center vs red hub center
    ///             - This determines which half of the field the robot is on
    ///             - Hub centers provide the most accurate alliance side determination
    ///
    ///             **Stage 2: Depot vs Outpost Selection**
    ///             - On the identified alliance side, compares distances to depot and outpost
    ///             - Returns the corresponding BUMP_ID for the closest combination
    ///
    ///             **Distance Calculation:**
    ///             Uses Euclidean distance from current robot pose to field element reference points:
    ///             - BLUE_HUB_CENTER (Stage 1)
    ///             - RED_HUB_CENTER (Stage 1)
    ///             - BLUE_DEPOT_NEUTRAL_SIDE (Stage 2)
    ///             - BLUE_OUTPOST_CENTER (Stage 2)
    ///             - RED_DEPOT_NEUTRAL_SIDE (Stage 2)
    ///             - RED_OUTPOST_CENTER (Stage 2)
    ///
    ///             **Return Values:**
    ///             - BLUE_DEPOT_BUMP: Robot closest to blue alliance depot area
    ///             - BLUE_OUTPOST_BUMP: Robot closest to blue alliance outpost area
    ///             - RED_DEPOT_BUMP: Robot closest to red alliance depot area
    ///             - RED_OUTPOST_BUMP: Robot closest to red alliance outpost area
    ///
    /// @note       If chassis is unavailable, uses default origin pose (0,0) for calculations
    /// @note       Method is const - does not modify BumpHelper state
    /// @see        PoseUtils::GetClosestFieldElement() for distance comparison implementation
    //------------------------------------------------------------------
    BUMP_ID CalcNearestBump() const;

    //------------------------------------------------------------------
    /// @brief      Retrieves an ordered pair of BumpPositions for a cross-field sweep
    /// @param[in]  isInNeutralZone - true if the robot is currently in the neutral zone,
    ///             false if it is in the alliance zone
    /// @return     std::vector<BumpPosition> - Two BumpPosition entries (bumpId, x, y) ordered
    ///             nearest-first, where index 0 is the starting bump and index 1 is the
    ///             cross-field destination bump
    /// @details    Identifies the nearest bump, then returns two BumpPosition entries ordered
    ///             nearest-first. The X coordinate reflects the side the robot is currently on
    ///             (alliance or neutral), and the Y coordinate uses the trench-entrance series
    ///             so the sweep endpoint aligns with the trench entrance.
    /// @see        CalcNearestBump() for bump identification
    /// @see        FieldOffsetValues for the coordinate source data
    //------------------------------------------------------------------
    std::vector<BumpPosition> GetNearestAndCrossFieldBumpEdges(bool isInNeutralZone);

private:
    //------------------------------------------------------------------
    /// @brief      Private constructor for singleton pattern
    /// @details    Initializes member pointers by retrieving singleton instances:
    ///             - ChassisConfigMgr: For access to swerve drivetrain and robot pose
    ///             - FieldConstants: For field element position reference data
    ///
    ///             Called only by GetInstance() on first access to create the singleton.
    //------------------------------------------------------------------
    BumpHelper();

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    /// @details    No special cleanup required as member pointers reference
    ///             other singleton objects managed elsewhere
    //------------------------------------------------------------------
    ~BumpHelper() = default;

    // units::in GetOffsetPosition();

    //------------------------------------------------------------------
    // Member Variables
    //------------------------------------------------------------------

    /// @brief Pointer to the swerve drivetrain subsystem for robot pose queries
    subsystems::CommandSwerveDrivetrain *m_chassis;

    /// @brief Pointer to field constants singleton for field element positions
    FieldConstants *m_fieldConstants;

    /// @brief Singleton instance pointer (lazy initialization)
    static BumpHelper *m_instance;

    bool m_driverDPad0ButtonReleased = true;   ///< State tracking for driver D-pad button to prevent multiple toggles on a single press
    bool m_driverDPad180ButtonReleased = true; ///< State tracking for driver D-pad button to prevent multiple toggles on a single press
    bool m_rewindLatch = false;
    int m_lane = 0;
    bool m_incrementLatch = false;
    bool m_decrementLatch = false;
    bool m_isIncrementPressed = false;
    bool m_isDecrementPressed = false;
    void LaneSelector();

    // put in an int called lane selctor that returns 0 1 2
};
