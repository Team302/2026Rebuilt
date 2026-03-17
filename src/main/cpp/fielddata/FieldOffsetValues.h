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

#include "units/angle.h"
#include "units/length.h"
#include <vector>

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
    // Depot X-Coordinate Getters
    //------------------------------------------------------------------

    /// @brief      Get the blue alliance depot neutral-side X-coordinate
    /// @return     units::length::meter_t - Blue depot X position (meters)
    units::length::meter_t GetBlueDepotX() const { return m_blueDepotX; }

    /// @brief      Get the red alliance depot neutral-side X-coordinate
    /// @return     units::length::meter_t - Red depot X position (meters)
    units::length::meter_t GetRedDepotX() const { return m_redDepotX; }

    //------------------------------------------------------------------
    // Outpost X-Coordinate Getters
    //------------------------------------------------------------------

    /// @brief      Get the blue alliance outpost X-coordinate
    /// @return     units::length::meter_t - Blue outpost X position (meters, equal to depot X)
    units::length::meter_t GetBlueOutpostX() const { return m_blueOutpostX; }

    /// @brief      Get the red alliance outpost X-coordinate
    /// @return     units::length::meter_t - Red outpost X position (meters, equal to depot X)
    units::length::meter_t GetRedOutpostX() const { return m_redOutpostX; }

    /// @brief      Get the blue alliance outpost approach X-coordinate
    /// @return     units::length::meter_t - Blue outpost approach X position (meters, outpostX + OUTPOST_APPROACH_OFFSET)
    units::length::meter_t GetBlueOutpostApproachX() const { return m_blueOutpostApproachX; }

    /// @brief      Get the red alliance outpost approach X-coordinate
    /// @return     units::length::meter_t - Red outpost approach X position (meters, outpostX - OUTPOST_APPROACH_OFFSET)
    units::length::meter_t GetRedOutpostApproachX() const { return m_redOutpostApproachX; }

    //------------------------------------------------------------------
    // Tower Coordinate Getters
    //------------------------------------------------------------------

    /// @brief      Get the red tower X-coordinate on the outpost side
    /// @return     units::length::meter_t - Red tower outpost-side X (tower center X - TOWER_X_OFFSET)
    units::length::meter_t GetRedTowerOutpostX() const { return m_redTowerOutpostX; }

    /// @brief      Get the red tower X-coordinate on the depot side
    /// @return     units::length::meter_t - Red tower depot-side X (tower center X - TOWER_X_OFFSET)
    units::length::meter_t GetRedTowerDepotX() const { return m_redTowerDepotX; }

    /// @brief      Get the red tower Y-coordinate on the outpost side
    /// @return     units::length::meter_t - Red tower outpost-side Y (tower center Y + TOWER_Y_OFFSET)
    units::length::meter_t GetRedTowerOutpostY() const { return m_redTowerOutpostY; }

    /// @brief      Get the red tower Y-coordinate on the depot side
    /// @return     units::length::meter_t - Red tower depot-side Y (tower center Y - TOWER_Y_OFFSET)
    units::length::meter_t GetRedTowerDepotY() const { return m_redTowerDepotY; }

    /// @brief      Get the blue tower X-coordinate on the outpost side
    /// @return     units::length::meter_t - Blue tower outpost-side X (tower center X + TOWER_X_OFFSET)
    units::length::meter_t GetBlueTowerOutpostX() const { return m_blueTowerOutpostX; }

    /// @brief      Get the blue tower X-coordinate on the depot side
    /// @return     units::length::meter_t - Blue tower depot-side X (tower center X + TOWER_X_OFFSET)
    units::length::meter_t GetBlueTowerDepotX() const { return m_blueTowerDepotX; }

    /// @brief      Get the blue tower Y-coordinate on the outpost side
    /// @return     units::length::meter_t - Blue tower outpost-side Y (tower center Y - TOWER_Y_OFFSET)
    units::length::meter_t GetBlueTowerOutpostY() const { return m_blueTowerOutpostY; }

    /// @brief      Get the blue tower Y-coordinate on the depot side
    /// @return     units::length::meter_t - Blue tower depot-side Y (tower center Y + TOWER_Y_OFFSET)
    units::length::meter_t GetBlueTowerDepotY() const { return m_blueTowerDepotY; }

    //------------------------------------------------------------------
    // Hub X-Coordinate Getters
    //------------------------------------------------------------------

    /// @brief      Get the blue hub X-coordinate with neutral-zone offset applied
    /// @return     units::length::meter_t - Blue hub X position (hub center X - HUB_OFFSET)
    units::length::meter_t GetBlueHubX() const { return m_blueHubX; }

    /// @brief      Get the red hub X-coordinate with neutral-zone offset applied
    /// @return     units::length::meter_t - Red hub X position (hub center X + HUB_OFFSET)
    units::length::meter_t GetRedHubX() const { return m_redHubX; }

    //------------------------------------------------------------------
    // Bump X-Coordinate Getters
    //------------------------------------------------------------------

    /// @brief      Get the red alliance-side bump edge X-coordinate
    /// @return     units::length::meter_t - Red alliance bump X (hub center X + BUMP_OFFSET)
    units::length::meter_t GetRedAllianceBumpEdgeX() const { return m_redAllianceBumpEdgeX; }

    /// @brief      Get the red neutral-side bump edge X-coordinate
    /// @return     units::length::meter_t - Red neutral bump X (hub center X - BUMP_OFFSET)
    units::length::meter_t GetRedNeutralBumpEdgeX() const { return m_redNeutralBumpEdgeX; }

    /// @brief      Get the blue alliance-side bump edge X-coordinate
    /// @return     units::length::meter_t - Blue alliance bump X (hub center X - BUMP_OFFSET)
    units::length::meter_t GetBlueAllianceBumpEdgeX() const { return m_blueAllianceBumpEdgeX; }

    /// @brief      Get the blue neutral-side bump edge X-coordinate
    /// @return     units::length::meter_t - Blue neutral bump X (hub center X + BUMP_OFFSET)
    units::length::meter_t GetBlueNeutralBumpEdgeX() const { return m_blueNeutralBumpEdgeX; }

    //------------------------------------------------------------------
    // Bump Midpoint Y-Coordinate Getters
    //------------------------------------------------------------------

    /// @brief      Get the red depot-side bump midpoint Y-coordinate
    /// @return     units::length::meter_t - Red depot bump Y (hub–trench midpoint + 1 ft)
    units::length::meter_t GetRedBumpDepotY() const { return m_redBumpDepotY; }

    /// @brief      Get the red outpost-side bump midpoint Y-coordinate
    /// @return     units::length::meter_t - Red outpost bump Y (hub–trench midpoint - 1 ft)
    units::length::meter_t GetRedBumpOutpostY() const { return m_redBumpOutpostY; }

    /// @brief      Get the blue depot-side bump midpoint Y-coordinate
    /// @return     units::length::meter_t - Blue depot bump Y (hub–trench midpoint - 1 ft)
    units::length::meter_t GetBlueBumpDepotY() const { return m_blueBumpDepotY; }

    /// @brief      Get the blue outpost-side bump midpoint Y-coordinate
    /// @return     units::length::meter_t - Blue outpost bump Y (hub–trench midpoint + 1 ft)
    units::length::meter_t GetBlueBumpOutpostY() const { return m_blueBumpOutpostY; }

    //------------------------------------------------------------------
    // Bump Trench-Entrance Y-Coordinate Getters
    //------------------------------------------------------------------

    /// @brief      Get the red depot-side bump trench-entrance Y-coordinate offset
    /// @return     units::length::meter_t - Y of RED_TRENCH_ALLIANCE_DEPOT + 1 ft (trench entrance for red depot bump)
    units::length::meter_t GetRedBumpTrenchDepotYOffset() const { return m_redBumpTrenchDepotYOffset; }

    /// @brief      Get the red outpost-side bump trench-entrance Y-coordinate offset
    /// @return     units::length::meter_t - Y of RED_TRENCH_ALLIANCE_OUTPOST - 1 ft (trench entrance for red outpost bump)
    units::length::meter_t GetRedBumpTrenchOutpostYOffset() const { return m_redBumpTrenchOutpostYOffset; }

    /// @brief      Get the blue depot-side bump trench-entrance Y-coordinate offset
    /// @return     units::length::meter_t - Y of BLUE_TRENCH_ALLIANCE_DEPOT - 1 ft (trench entrance for blue depot bump)
    units::length::meter_t GetBlueBumpTrenchDepotYOffset() const { return m_blueBumpTrenchDepotYOffset; }

    /// @brief      Get the blue outpost-side bump trench-entrance Y-coordinate offset
    /// @return     units::length::meter_t - Y of BLUE_TRENCH_ALLIANCE_OUTPOST + 1 ft (trench entrance for blue outpost bump)
    units::length::meter_t GetBlueBumpTrenchOutpostYOffset() const { return m_blueBumpTrenchOutpostYOffset; }

    //------------------------------------------------------------------
    // Trench X-Coordinate Getters
    //------------------------------------------------------------------

    /// @brief      Get the red alliance trench X-coordinate
    /// @return     units::length::meter_t - X of red alliance trench entrance
    units::length::meter_t GetRedTrenchX() const { return m_redTrenchX; }

    /// @brief      Get the red neutral-side trench X-coordinate
    /// @return     units::length::meter_t - X of red trench on the neutral side
    units::length::meter_t GetNeutralRedTrenchX() const { return m_neutralRedTrenchX; }

    /// @brief      Get the blue alliance trench X-coordinate
    /// @return     units::length::meter_t - X of blue alliance trench entrance
    units::length::meter_t GetBlueTrenchX() const { return m_blueTrenchX; }

    /// @brief      Get the blue neutral-side trench X-coordinate
    /// @return     units::length::meter_t - X of blue trench on the neutral side
    units::length::meter_t GetNeutralBlueTrenchX() const { return m_neutralBlueTrenchX; }

    //------------------------------------------------------------------
    // Trench Y-Coordinate Getters
    //------------------------------------------------------------------

    /// @brief      Get the red depot trench Y-coordinate
    /// @return     units::length::meter_t - Y of RED_TRENCH_ALLIANCE_DEPOT
    units::length::meter_t GetRedDepotTrenchY() const { return m_redDepotTrenchY; }

    /// @brief      Get the red outpost trench Y-coordinate
    /// @return     units::length::meter_t - Y of RED_TRENCH_ALLIANCE_OUTPOST
    units::length::meter_t GetRedOutpostTrenchY() const { return m_redOutpostTrenchY; }

    /// @brief      Get the blue depot trench Y-coordinate
    /// @return     units::length::meter_t - Y of BLUE_TRENCH_ALLIANCE_DEPOT
    units::length::meter_t GetBlueDepotTrenchY() const { return m_blueDepotTrenchY; }

    /// @brief      Get the blue outpost trench Y-coordinate
    /// @return     units::length::meter_t - Y of BLUE_TRENCH_ALLIANCE_OUTPOST
    units::length::meter_t GetBlueOutpostTrenchY() const { return m_blueOutpostTrenchY; }

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
    ///             - Trench alliance position Y with a ±1 ft fine-tune adjustment so the
    ///               cross-field sweep endpoint lands just inside the trench entrance
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
    /// @brief X-coordinate of blue alliance depot neutral side (meters)
    /// @brief X-coordinate of red alliance depot neutral side (meters)
    //------------------------------------------------------------------
    units::length::meter_t m_blueDepotX;
    units::length::meter_t m_redDepotX;

    //------------------------------------------------------------------
    // Outpost X-Coordinates
    /// @brief X-coordinate of blue alliance outpost (meters, equal to depot X)
    /// @brief X-coordinate of red alliance outpost (meters, equal to depot X)
    /// @brief X-coordinate of blue alliance outpost approach position (meters, outpostX + OUTPOST_APPROACH_OFFSET)
    /// @brief X-coordinate of red alliance outpost approach position (meters, outpostX - OUTPOST_APPROACH_OFFSET)
    //------------------------------------------------------------------
    units::length::meter_t m_blueOutpostX;
    units::length::meter_t m_redOutpostX;
    units::length::meter_t m_blueOutpostApproachX;
    units::length::meter_t m_redOutpostApproachX;

    //------------------------------------------------------------------
    // Tower Coordinates
    // Positions are computed relative to the tower center pose retrieved from
    // FieldConstants::RED_TOWER_CENTER / BLUE_TOWER_CENTER, with TOWER_X_OFFSET
    // and TOWER_Y_OFFSET applied to derive outpost and depot approach points.
    /// @brief X-coordinate of red tower aligned with the outpost side (tower center X - TOWER_X_OFFSET)
    /// @brief X-coordinate of red tower aligned with the depot side (tower center X - TOWER_X_OFFSET)
    /// @brief Y-coordinate of red tower on the outpost side (tower center Y + TOWER_Y_OFFSET)
    /// @brief Y-coordinate of red tower on the depot side (tower center Y - TOWER_Y_OFFSET)
    /// @brief X-coordinate of blue tower aligned with the outpost side (tower center X + TOWER_X_OFFSET)
    /// @brief X-coordinate of blue tower aligned with the depot side (tower center X + TOWER_X_OFFSET)
    /// @brief Y-coordinate of blue tower on the outpost side (tower center Y - TOWER_Y_OFFSET)
    /// @brief Y-coordinate of blue tower on the depot side (tower center Y + TOWER_Y_OFFSET)
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
    /// @brief X-coordinate of blue hub center minus 2.0m offset (toward neutral zone)
    /// @brief X-coordinate of red hub center plus 2.0m offset (toward neutral zone)
    //------------------------------------------------------------------
    units::length::meter_t m_blueHubX;
    units::length::meter_t m_redHubX;

    //------------------------------------------------------------------
    // Bump X-Coordinates
    /// @brief X-coordinate of red alliance side bump edge (hub + 1.5m)
    /// @brief X-coordinate of red neutral side bump edge (hub - 1.5m)
    /// @brief X-coordinate of blue alliance side bump edge (hub - 1.5m)
    /// @brief X-coordinate of blue neutral side bump edge (hub + 1.5m)
    //------------------------------------------------------------------
    units::length::meter_t m_redAllianceBumpEdgeX;
    units::length::meter_t m_redNeutralBumpEdgeX;
    units::length::meter_t m_blueAllianceBumpEdgeX;
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
    // Trench alliance position Y from FieldConstants with ±1 ft fine-tune adjustment.
    // Used by BumpHelper so the cross-field sweep endpoint lands just inside the trench entrance.
    //------------------------------------------------------------------
    units::length::meter_t m_redBumpTrenchDepotYOffset;    ///< Y of RED_TRENCH_ALLIANCE_DEPOT + 1 ft (trench entrance for red depot bump)
    units::length::meter_t m_redBumpTrenchOutpostYOffset;  ///< Y of RED_TRENCH_ALLIANCE_OUTPOST - 1 ft (trench entrance for red outpost bump)
    units::length::meter_t m_blueBumpTrenchDepotYOffset;   ///< Y of BLUE_TRENCH_ALLIANCE_DEPOT - 1 ft (trench entrance for blue depot bump)
    units::length::meter_t m_blueBumpTrenchOutpostYOffset; ///< Y of BLUE_TRENCH_ALLIANCE_OUTPOST + 1 ft (trench entrance for blue outpost bump)

    // trench offsets
    units::length::meter_t m_redTrenchX;         ///< X of the red alliance-side trench entrance (alliance zone side)
    units::length::meter_t m_neutralRedTrenchX;  ///< X of the red trench on the neutral-zone side (neutral zone side)
    units::length::meter_t m_blueTrenchX;        ///< X of the blue alliance-side trench entrance (alliance zone side)
    units::length::meter_t m_neutralBlueTrenchX; ///< X of the blue trench on the neutral-zone side (neutral zone side)

    units::length::meter_t m_redDepotTrenchY;    ///< Y of RED_TRENCH_ALLIANCE_DEPOT (trench entrance for red depot)
    units::length::meter_t m_redOutpostTrenchY;  ///< Y of RED_TRENCH_ALLIANCE_OUTPOST (trench entrance for red outpost)
    units::length::meter_t m_blueDepotTrenchY;   ///< Y of BLUE_TRENCH_ALLIANCE_DEPOT (trench entrance for blue depot)
    units::length::meter_t m_blueOutpostTrenchY; ///< Y of BLUE_TRENCH_ALLIANCE_OUTPOST (trench entrance for blue outpost)

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

    /// @brief Robot heading to face the red alliance wall (0°)
    static constexpr units::angle::degree_t FACE_RED_ALLIANCE_WALL = 0_deg;
    /// @brief Robot heading to face the blue alliance wall (180°)
    static constexpr units::angle::degree_t FACE_BLUE_ALLIANCE_WALL = 180_deg;
    /// @brief Robot heading to face the origin-side field wall (270°)
    static constexpr units::angle::degree_t FACE_ORIGIN_SIDE_WALL = 270_deg;
    /// @brief Robot heading to face the non-origin-side field wall (90°)
    static constexpr units::angle::degree_t FACE_NON_ORIGIN_SIDE_WALL = 90_deg;

    /// @brief Half-width offset applied to position the robot at the trench entrance (meters)
    static constexpr units::length::meter_t TRENCH_OFFSET = 0.5_m;
};
