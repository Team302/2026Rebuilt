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

#include "units/length.h"

//====================================================================================================================================================
/// @enum FIELD_OFFSET_ITEMS
/// @brief Enumeration of all field element offset coordinate types available for retrieval
///
/// Defines the different types of field element position data that can be queried from FieldOffsetValues.
/// Each enum value represents a specific X or Y coordinate for alliance-specific field elements like
/// outposts, depots, hubs, and bumps. Used as parameters to GetValue() to specify which coordinate to retrieve.
//====================================================================================================================================================
enum class FIELD_OFFSET_ITEMS
{
    OUTPOST_X,          ///< X-coordinate of the outpost position (meters)
    OUTPOST_APPROACH_X, ///< X-coordinate of the outpost approach position (meters)
    DEPOT_X,            ///< X-coordinate of the depot neutral side position (meters)
    HUB_X,              ///< X-coordinate of the hub center with offset applied (meters)
    ALLIANCE_BUMP_X,    ///< X-coordinate of the bump on the alliance zone side (meters)
    ALLIANCE_BUMP_Y,    ///< Y-coordinate of the bump on the alliance zone side (meters)
    NEUTRAL_BUMP_X,     ///< X-coordinate of the bump on the neutral zone side (meters)
    NEUTRAL_BUMP_Y      ///< Y-coordinate of the bump on the neutral zone side (meters)
    TOWER_OUTPOST_X,    ///< X-coordinate offset of the outpost
    TOWER_DEPOT_X,      ///< X-coordinate offset of the depot neutral side
    TOWER_OUTPOST_Y,    ///< Y-coordinate offset of the outpost
    TOWER_DEPOT_Y       ///< Y-coordinate offset of the depot neutral side
};

//====================================================================================================================================================
/// @class FieldOffsetValues
/// @brief Singleton class providing alliance-aware access to field element positions and offsets
///
/// This singleton manages position data for key 2026 game field elements, providing alliance-specific
/// coordinate retrieval for navigation and positioning. The class stores X and Y coordinates for:
/// - Depots (both red and blue alliance)
/// - Outposts (both red and blue alliance)
/// - Hubs (both red and blue alliance, with navigation offsets)
/// - Bumps (all four bumps, both X and Y coordinates)
///
/// **Key Features:**
/// - Alliance-aware queries: Pass isRedSide boolean to get correct alliance coordinates
/// - Comprehensive field coverage: Supports all major navigation targets
/// - Singleton pattern: Single source of truth for field positions
/// - Initialization from FieldConstants: Automatically loaded at startup
///
/// **Offset Strategy:**
/// The class applies strategic offsets to certain field elements:
/// - Hub positions: Offset by 2.0m toward neutral zone for optimal navigation positioning
/// - Bump positions: Offset by 1.5m on each side for accurate bump crossing waypoints
///
/// **Usage Pattern:**
/// ```cpp
/// auto offsets = FieldOffsetValues::GetInstance();
/// bool isRed = true;
/// auto hubX = offsets->GetValue(isRed, FIELD_OFFSET_ITEMS::HUB_X);
/// auto bumpX = offsets->GetValue(isRed, FIELD_OFFSET_ITEMS::ALLIANCE_BUMP_X);
/// ```
///
/// **Primary Consumers:**
/// - DriveOverBump: Uses bump X/Y coordinates for waypoint navigation
/// - Navigation commands: Use hub, depot, and outpost positions for targeting
/// - Autonomous routines: Alliance-specific positioning for game strategy
///
/// @note Coordinates are in meters using WPILib units system
/// @note All positions are field-relative (blue alliance perspective by default)
/// @see FIELD_OFFSET_ITEMS for available coordinate types
/// @see FieldConstants for source field element data
/// @see BumpHelper for bump identification before retrieving bump coordinates
//====================================================================================================================================================
class FieldOffsetValues
{
public:
    //------------------------------------------------------------------
    /// @brief      Get the singleton instance of FieldOffsetValues
    /// @return     FieldOffsetValues* - Pointer to the singleton instance
    /// @details    Implements lazy initialization. Creates instance on first call,
    ///             returns existing instance on subsequent calls. Ensures single
    ///             source of truth for field offset values throughout the program.
    //------------------------------------------------------------------
    static FieldOffsetValues *GetInstance();

    //------------------------------------------------------------------
    /// @brief      Get the position value for a specific field element
    /// @param[in]  isRedSide - true to retrieve red alliance value, false for blue alliance
    /// @param[in]  item - The field offset item type to retrieve (from FIELD_OFFSET_ITEMS enum)
    /// @return     units::length::meter_t - The coordinate value in meters
    /// @details    Returns alliance-specific position data based on the item type:
    ///
    ///             **X-Coordinate Queries:**
    ///             - OUTPOST_X: Returns red or blue outpost X position
    ///             - DEPOT_X: Returns red or blue depot neutral side X position
    ///             - HUB_X: Returns red or blue hub X position with 2.0m offset toward neutral zone
    ///             - ALLIANCE_BUMP_X: Returns alliance side bump X position (1.5m offset from hub)
    ///             - NEUTRAL_BUMP_X: Returns neutral side bump X position (1.5m offset from hub)
    ///
    ///             **Y-Coordinate Queries:**
    ///             - ALLIANCE_BUMP_Y or NEUTRAL_BUMP_Y:
    ///               * Dynamically calculates Y position based on nearest bump (uses BumpHelper)
    ///               * Returns midpoint between hub center and corresponding trench
    ///               * Same Y value for both alliance and neutral side of the same bump
    ///
    ///             **Fallback:**
    ///             Returns 0.0m for unknown/invalid item types
    ///
    /// @note       For BUMP_Y queries, the method calls BumpHelper to determine which bump,
    ///             then returns the appropriate depot or outpost Y coordinate
    /// @see        FIELD_OFFSET_ITEMS for all available item types
    /// @see        BumpHelper::CalcNearestBump() for bump identification logic
    //------------------------------------------------------------------
    units::length::meter_t GetValue(bool isRedSide, FIELD_OFFSET_ITEMS item) const;

private:
    //------------------------------------------------------------------
    /// @brief      Private constructor for singleton pattern
    /// @details    Initializes all field offset values by querying FieldConstants:
    ///
    ///             **Depot and Outpost Positions:**
    ///             - Retrieves neutral side X positions for both alliances
    ///             - Sets outpost X equal to depot X (aligned on X-axis)
    ///
    ///             **Hub Positions with Offsets:**
    ///             - Red hub: Base position + 2.0m (toward neutral zone)
    ///             - Blue hub: Base position - 2.0m (toward neutral zone)
    ///
    ///             **Bump Positions:**
    ///             - Alliance side: Hub center ± 1.5m
    ///             - Neutral side: Hub center ∓ 1.5m
    ///             - Y coordinates: Midpoint between hub and corresponding trench
    ///
    ///             **Fallback:**
    ///             If FieldConstants unavailable, initializes all values to 0.0m
    ///
    ///             **Debug Logging:**
    ///             Logs all calculated bump positions to NetworkTables for verification
    //------------------------------------------------------------------
    FieldOffsetValues();

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    /// @details    No cleanup required - all members are value types
    //------------------------------------------------------------------
    ~FieldOffsetValues() = default;

    //------------------------------------------------------------------
    // Singleton Instance
    //------------------------------------------------------------------

    /// @brief Singleton instance pointer (lazy initialization)
    static FieldOffsetValues *m_instance;

    //------------------------------------------------------------------
    // Depot X-Coordinates
    //------------------------------------------------------------------

    /// @brief X-coordinate of blue alliance depot neutral side (meters)
    units::length::meter_t m_blueDepotX;

    /// @brief X-coordinate of red alliance depot neutral side (meters)
    units::length::meter_t m_redDepotX;

    //------------------------------------------------------------------
    // Outpost X-Coordinates
    //------------------------------------------------------------------

    /// @brief X-coordinate of blue alliance outpost (meters, equal to depot X)
    units::length::meter_t m_blueOutpostX;

    /// @brief X-coordinate of red alliance outpost (meters, equal to depot X)
    units::length::meter_t m_redOutpostX;

    /// @brief X-coordinate of blue alliance outpost approach position (meters, outpostX + OUTPOST_APPROACH_OFFSET)
    units::length::meter_t m_blueOutpostApproachX;

    /// @brief X-coordinate of red alliance outpost approach position (meters, outpostX - OUTPOST_APPROACH_OFFSET)
    units::length::meter_t m_redOutpostApproachX;

    //------------------------------------------------------------------
    // Tower Coordinates
    //------------------------------------------------------------------
    units::length::meter_t m_redTowerOutpostX;

    units::length::meter_t m_redTowerDepotX;

    units::length::meter_t m_redTowerOutpostY;

    units::length::meter_t m_redTowerDepotY;

    units::length::meter_t m_blueTowerOutpostX;

    units::length::meter_t m_blueTowerDepotX;

    units::length::meter_t m_blueTowerOutpostY;

    units::length::meter_t m_blueTowerDepotY;

    //------------------------------------------------------------------
    // Hub X-Coordinates with Offsets
    //------------------------------------------------------------------

    /// @brief X-coordinate of blue hub center minus 2.0m offset (toward neutral zone)
    units::length::meter_t m_blueHubX;

    /// @brief X-coordinate of red hub center plus 2.0m offset (toward neutral zone)
    units::length::meter_t m_redHubX;

    //------------------------------------------------------------------
    // Bump X-Coordinates
    //------------------------------------------------------------------

    /// @brief X-coordinate of red alliance side bump edge (hub + 1.5m)
    units::length::meter_t m_redAllianceBumpEdgeX;

    /// @brief X-coordinate of red neutral side bump edge (hub - 1.5m)
    units::length::meter_t m_redNeutralBumpEdgeX;

    /// @brief X-coordinate of blue alliance side bump edge (hub - 1.5m)
    units::length::meter_t m_blueAllianceBumpEdgeX;

    /// @brief X-coordinate of blue neutral side bump edge (hub + 1.5m)
    units::length::meter_t m_blueNeutralBumpEdgeX;

    //------------------------------------------------------------------
    // Bump Y-Coordinates
    //------------------------------------------------------------------

    /// @brief Y-coordinate of red depot bump (midpoint between hub and depot trench)
    units::length::meter_t m_redBumpDepotY;

    /// @brief Y-coordinate of red outpost bump (midpoint between hub and outpost trench)
    units::length::meter_t m_redBumpOutpostY;

    /// @brief Y-coordinate of blue depot bump (midpoint between hub and depot trench)
    units::length::meter_t m_blueBumpDepotY;

    /// @brief Y-coordinate of blue outpost bump (midpoint between hub and outpost trench)
    units::length::meter_t m_blueBumpOutpostY;

    //------------------------------------------------------------------
    // Offset Constants
    //------------------------------------------------------------------

    /// @brief Hub offset distance for navigation positioning (2.0 meters toward neutral zone)
    static constexpr units::length::meter_t HUB_OFFSET = 2.0_m;
    static constexpr units::length::inch_t DEPOT_OFFSET = 3.0_in;

    /// @brief Bump offset distance from hub center (1.5 meters on each side)
    static constexpr units::length::meter_t BUMP_OFFSET = 1.5_m;

    static constexpr units::length::meter_t OUTPOST_APPROACH_OFFSET = 0.5_m; // Additional X-offset applied when computing outpost approach positions
};
