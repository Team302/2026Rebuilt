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
#include "units/length.h"
#include <vector>

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
/// @enum FIELD_OFFSET_ITEMS
/// @brief Enumeration of all field element offset coordinate types available for retrieval
///
/// Defines the different types of field element position data that can be queried from FieldOffsetValues.
/// Each enum value represents a specific X or Y coordinate for alliance-specific field elements like
/// outposts, depots, hubs, and bumps. Used as parameters to GetValue() to specify which coordinate to retrieve.
//====================================================================================================================================================
enum class FIELD_OFFSET_ITEMS
{
    DEPOT_X, ///< X-coordinate of the depot neutral side position (meters)

    OUTPOST_X,          ///< X-coordinate of the outpost position (meters)
    OUTPOST_APPROACH_X, ///< X-coordinate of the outpost approach position (meters)

    HUB_X, ///< X-coordinate of the hub center with offset applied (meters)

    BUMP_ALLIANCE_X, ///< X-coordinate of the bump on the alliance zone side (meters)
    BUMP_ALLIANCE_Y, ///< Y-coordinate of the bump on the alliance zone side (meters)
    BUMP_NEUTRAL_X,  ///< X-coordinate of the bump on the neutral zone side (meters)
    BUMP_NEUTRAL_Y,  ///< Y-coordinate of the bump on the neutral zone side (meters)

    TOWER_OUTPOST_X, ///< X-coordinate offset of the outpost
    TOWER_DEPOT_X,   ///< X-coordinate offset of the depot neutral side
    TOWER_OUTPOST_Y, ///< Y-coordinate offset of the outpost
    TOWER_DEPOT_Y    ///< Y-coordinate offset of the depot neutral side
};

//====================================================================================================================================================
/// @class FieldOffsetValues
/// @brief Singleton class providing alliance-aware access to field element positions and offsets
///
/// This singleton manages position data for key 2026 game field elements, providing alliance-specific
/// coordinate retrieval for navigation and positioning. The class stores X and Y coordinates for:
/// - Depots (both red and blue alliance)
/// - Outposts and outpost approach positions (both red and blue alliance)
/// - Hubs (both red and blue alliance, with navigation offsets)
/// - Towers — outpost and depot side (both red and blue alliance)
/// - Bumps (all four bumps, both X and Y coordinates — midpoint and trench-entrance series)
///
/// **Key Features:**
/// - Alliance-aware queries: Pass isRedSide boolean to get correct alliance coordinates
/// - Comprehensive field coverage: Supports all major navigation targets
/// - Singleton pattern: Single source of truth for field positions
/// - Initialization from FieldConstants: Automatically loaded at startup
///
/// **Offset Strategy:**
/// The class applies strategic offsets to certain field elements:
/// - Hub positions: Offset by HUB_OFFSET toward neutral zone for optimal navigation positioning
/// - Bump positions: Offset by BUMP_OFFSET on each side for accurate bump crossing waypoints
/// - Tower positions: Offset by TOWER_X_OFFSET and TOWER_Y_OFFSET from tower center
///
/// **Usage Pattern:**
/// ```cpp
/// auto offsets = FieldOffsetValues::GetInstance();
/// bool isRed = true;
/// auto hubX = offsets->GetValue(isRed, FIELD_OFFSET_ITEMS::HUB_X);
/// auto bumpX = offsets->GetValue(isRed, FIELD_OFFSET_ITEMS::BUMP_ALLIANCE_X);
/// ```
///
/// **Primary Consumers:**
/// - SweepBehindBump: Uses bump X/Y coordinates for cross-field waypoint navigation
/// - Navigation commands: Use hub, depot, outpost, and tower positions for targeting
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
    ///             - OUTPOST_APPROACH_X: Returns red or blue outpost approach X position
    ///               (OUTPOST_APPROACH_OFFSET beyond the outpost X toward the neutral zone)
    ///             - DEPOT_X: Returns red or blue depot neutral side X position
    ///             - HUB_X: Returns red or blue hub X position with HUB_OFFSET toward neutral zone
    ///             - BUMP_ALLIANCE_X: Returns alliance side bump X position (BUMP_OFFSET from hub)
    ///             - BUMP_NEUTRAL_X: Returns neutral side bump X position (BUMP_OFFSET from hub)
    ///             - TOWER_OUTPOST_X: Returns red or blue tower X on the outpost side
    ///             - TOWER_DEPOT_X: Returns red or blue tower X on the depot side
    ///
    ///             **Y-Coordinate Queries:**
    ///             - TOWER_OUTPOST_Y: Returns red or blue tower Y on the outpost side
    ///             - TOWER_DEPOT_Y: Returns red or blue tower Y on the depot side
    ///             - BUMP_ALLIANCE_Y or BUMP_NEUTRAL_Y:
    ///               * Dynamically determines Y position based on nearest bump (uses BumpHelper)
    ///               * Returns the midpoint-series Y for the identified bump
    ///               * Same Y value for both alliance and neutral sides of the same bump
    ///
    ///             **Fallback:**
    ///             Returns 0.0 m for unknown/invalid item types
    ///
    /// @note       For BUMP_Y queries, the method calls BumpHelper to determine which bump,
    ///             then returns the appropriate depot or outpost Y coordinate
    /// @see        FIELD_OFFSET_ITEMS for all available item types
    /// @see        BumpHelper::CalcNearestBump() for bump identification logic
    //------------------------------------------------------------------
    units::length::meter_t GetValue(bool isRedSide, FIELD_OFFSET_ITEMS item) const;

    //------------------------------------------------------------------
    /// @brief      Retrieves an ordered pair of BumpPositions for a cross-field sweep
    /// @param[in]  inNeutralZone - true if the robot is currently in the neutral zone,
    ///             false if it is in the alliance zone
    /// @return     std::vector<BumpPosition> - Two BumpPosition entries (bumpId, x, y) ordered
    ///             nearest-first, where index 0 is the starting bump and index 1 is the
    ///             cross-field destination bump
    /// @details    Uses BumpHelper::CalcNearestBump() to identify the nearest bump, then
    ///             returns two BumpPosition entries ordered nearest-first. The X coordinate
    ///             reflects the side the robot is currently on (alliance or neutral), and the
    ///             Y coordinate uses the trench-entrance series so the sweep endpoint aligns
    ///             with the trench entrance.
    ///
    ///             | Nearest bump      | inNeutralZone | Index 0 (nearest)                                          | Index 1 (cross-field)                                       |
    ///             |-------------------|---------------|------------------------------------------------------------|-------------------------------------------------------------|
    ///             | RED_OUTPOST_BUMP  | true          | {BUMP_ID::RED_OUTPOST_BUMP,  redNeutralX,  redTrenchOutpostY} | {BUMP_ID::RED_DEPOT_BUMP,    redNeutralX,  redTrenchDepotY}   |
    ///             | RED_OUTPOST_BUMP  | false         | {BUMP_ID::RED_OUTPOST_BUMP,  redAllianceX, redTrenchOutpostY} | {BUMP_ID::RED_DEPOT_BUMP,    redAllianceX, redTrenchDepotY}   |
    ///             | RED_DEPOT_BUMP    | true          | {BUMP_ID::RED_DEPOT_BUMP,    redNeutralX,  redTrenchDepotY}   | {BUMP_ID::RED_OUTPOST_BUMP,  redNeutralX,  redTrenchOutpostY} |
    ///             | RED_DEPOT_BUMP    | false         | {BUMP_ID::RED_DEPOT_BUMP,    redAllianceX, redTrenchDepotY}   | {BUMP_ID::RED_OUTPOST_BUMP,  redAllianceX, redTrenchOutpostY} |
    ///             | BLUE_OUTPOST_BUMP | true          | {BUMP_ID::BLUE_OUTPOST_BUMP, blueNeutralX, blueTrenchOutpostY}| {BUMP_ID::BLUE_DEPOT_BUMP,   blueNeutralX, blueTrenchDepotY}  |
    ///             | BLUE_OUTPOST_BUMP | false         | {BUMP_ID::BLUE_OUTPOST_BUMP, blueAllianceX,blueTrenchOutpostY}| {BUMP_ID::BLUE_DEPOT_BUMP,   blueAllianceX,blueTrenchDepotY}  |
    ///             | BLUE_DEPOT_BUMP   | true          | {BUMP_ID::BLUE_DEPOT_BUMP,   blueNeutralX, blueTrenchDepotY}  | {BUMP_ID::BLUE_OUTPOST_BUMP, blueNeutralX, blueTrenchOutpostY}|
    ///             | BLUE_DEPOT_BUMP   | false         | {BUMP_ID::BLUE_DEPOT_BUMP,   blueAllianceX,blueTrenchDepotY}  | {BUMP_ID::BLUE_OUTPOST_BUMP, blueAllianceX,blueTrenchOutpostY}|
    ///
    /// @note       Bump identification queries BumpHelper on every call (not cached)
    /// @note       Method is const - does not modify object state
    /// @see        GetValue() for single scalar coordinate queries
    /// @see        BumpHelper::CalcNearestBump() for bump identification
    /// @see        BumpPosition for the returned struct definition
    //------------------------------------------------------------------
    std::vector<BumpPosition> GetNearestAndCrossFieldBumpEdges(bool inNeutralZone) const;

private:
    //------------------------------------------------------------------
    /// @brief      Private constructor for singleton pattern
    /// @details    Initializes all field offset values by querying FieldConstants:
    ///
    ///             **Depot and Outpost Positions:**
    ///             - Retrieves neutral side X positions for both alliances with DEPOT_OFFSET applied
    ///             - Sets outpost X equal to depot X (aligned on X-axis on the 2026 field)
    ///             - Outpost approach X further offset by OUTPOST_APPROACH_OFFSET
    ///
    ///             **Tower Positions:**
    ///             - Applies TOWER_X_OFFSET and TOWER_Y_OFFSET to the tower center poses
    ///               to derive outpost-side and depot-side approach points for each alliance
    ///
    ///             **Hub Positions with Offsets:**
    ///             - Red hub: Hub center X + HUB_OFFSET (toward neutral zone)
    ///             - Blue hub: Hub center X - HUB_OFFSET (toward neutral zone)
    ///
    ///             **Bump X-Positions:**
    ///             - Alliance side: Hub center X ± BUMP_OFFSET
    ///             - Neutral side:  Hub center X ∓ BUMP_OFFSET
    ///
    ///             **Bump Y-Coordinates (midpoint series):**
    ///             - Midpoint between hub center Y and corresponding trench alliance Y,
    ///               with a ±1 ft fine-tune adjustment per bump
    ///
    ///             **Bump Y-Coordinates (trench entrance series):**
    ///             - Directly uses the trench alliance position Y so the cross-field
    ///               sweep endpoint aligns with the trench entrance
    ///
    ///             **Fallback:**
    ///             If FieldConstants unavailable, initializes all values to 0.0 m
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
    // Positions are computed relative to the tower center pose retrieved from
    // FieldConstants::RED_TOWER_CENTER / BLUE_TOWER_CENTER, with TOWER_X_OFFSET
    // and TOWER_Y_OFFSET applied to derive outpost and depot approach points.
    //------------------------------------------------------------------

    /// @brief X-coordinate of red tower aligned with the outpost side (tower center X - TOWER_X_OFFSET)
    units::length::meter_t m_redTowerOutpostX;

    /// @brief X-coordinate of red tower aligned with the depot side (tower center X - TOWER_X_OFFSET)
    units::length::meter_t m_redTowerDepotX;

    /// @brief Y-coordinate of red tower on the outpost side (tower center Y + TOWER_Y_OFFSET)
    units::length::meter_t m_redTowerOutpostY;

    /// @brief Y-coordinate of red tower on the depot side (tower center Y - TOWER_Y_OFFSET)
    units::length::meter_t m_redTowerDepotY;

    /// @brief X-coordinate of blue tower aligned with the outpost side (tower center X + TOWER_X_OFFSET)
    units::length::meter_t m_blueTowerOutpostX;

    /// @brief X-coordinate of blue tower aligned with the depot side (tower center X + TOWER_X_OFFSET)
    units::length::meter_t m_blueTowerDepotX;

    /// @brief Y-coordinate of blue tower on the outpost side (tower center Y - TOWER_Y_OFFSET)
    units::length::meter_t m_blueTowerOutpostY;

    /// @brief Y-coordinate of blue tower on the depot side (tower center Y + TOWER_Y_OFFSET)
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
    // Bump Midpoint Y-Coordinates
    // Midpoint between hub center and the corresponding trench alliance position,
    // with a ±1 ft fine-tune adjustment. Used by GetValue() for BUMP_ALLIANCE_Y
    // and BUMP_NEUTRAL_Y queries.
    //------------------------------------------------------------------
    units::length::meter_t m_redBumpDepotY;    ///< Y midpoint for the red depot-side bump (hub–trench midpoint + 1 ft)
    units::length::meter_t m_redBumpOutpostY;  ///< Y midpoint for the red outpost-side bump (hub–trench midpoint - 1 ft)
    units::length::meter_t m_blueBumpDepotY;   ///< Y midpoint for the blue depot-side bump (hub–trench midpoint - 1 ft)
    units::length::meter_t m_blueBumpOutpostY; ///< Y midpoint for the blue outpost-side bump (hub–trench midpoint + 1 ft)

    //------------------------------------------------------------------
    // Bump Trench-Entrance Y-Coordinates
    // Directly equal to the trench alliance position Y from FieldConstants.
    // Used by GetNearestAndCrossFieldBumpEdges() so the cross-field sweep
    // endpoint aligns with the trench entrance.
    //------------------------------------------------------------------
    units::length::meter_t m_redBumpTrenchDepotY;    ///< Y of RED_TRENCH_ALLIANCE_DEPOT (trench entrance for red depot bump)
    units::length::meter_t m_redBumpTrenchOutpostY;  ///< Y of RED_TRENCH_ALLIANCE_OUTPOST (trench entrance for red outpost bump)
    units::length::meter_t m_blueBumpTrenchDepotY;   ///< Y of BLUE_TRENCH_ALLIANCE_DEPOT (trench entrance for blue depot bump)
    units::length::meter_t m_blueBumpTrenchOutpostY; ///< Y of BLUE_TRENCH_ALLIANCE_OUTPOST (trench entrance for blue outpost bump)

    //------------------------------------------------------------------
    // Offset Constants
    //------------------------------------------------------------------

    /// @brief Hub offset distance applied toward the neutral zone (meters)
    static constexpr units::length::meter_t HUB_OFFSET = 2.0_m;

    /// @brief Small inward nudge applied to depot neutral-side X positions (inches)
    static constexpr units::length::inch_t DEPOT_OFFSET = 3.0_in;

    /// @brief Bump offset distance from hub center to the bump edge on each side (meters)
    static constexpr units::length::meter_t BUMP_OFFSET = 1.5_m;

    /// @brief Additional X-offset applied when computing outpost approach positions (meters)
    static constexpr units::length::meter_t OUTPOST_APPROACH_OFFSET = 0.5_m;

    /// @brief X-offset from tower center to the outpost/depot side approach position (meters)
    static constexpr units::length::meter_t TOWER_X_OFFSET = 1.0_m;

    /// @brief Y-offset from tower center to the outpost/depot side approach position (meters)
    static constexpr units::length::meter_t TOWER_Y_OFFSET = 0.5_m;
};
