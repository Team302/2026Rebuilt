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

//====================================================================================================================================================
/// @file FieldOffsetValues.cpp
/// @brief Implementation of FieldOffsetValues singleton for field element position management
/// @details This file implements the storage and retrieval of alliance-specific field element positions
///          for the 2026 game field. It calculates strategic offsets for navigation targets including
///          hubs, bumps, depots, outposts, and towers, and provides a unified interface for querying
///          field coordinates throughout the codebase.
///
///          The implementation queries FieldConstants for base positions and applies game-specific offsets
///          to optimize navigation paths and bump crossing trajectories.
///
///          **Public API:**
///          - GetInstance()  – Lazy-initialization singleton accessor
///          - GetValue()     – Returns a single alliance-aware coordinate for a given FIELD_OFFSET_ITEMS type
///          - GetNearestAndCrossFieldBumpEdges()    – Returns an ordered vector of coordinates (nearest-first for bump Y queries)
//====================================================================================================================================================

#include "fielddata/FieldOffsetValues.h"
#include "fielddata/BumpHelper.h"
#include "fielddata/FieldConstants.h"

/// @brief Singleton instance pointer - initialized to nullptr for lazy instantiation
FieldOffsetValues *FieldOffsetValues::m_instance = nullptr;

//------------------------------------------------------------------
/// @brief      Get the singleton instance of FieldOffsetValues
/// @return     FieldOffsetValues* - Pointer to the singleton instance
/// @details    Implements lazy initialization singleton pattern. Creates the
///             instance on first call and returns it. Subsequent calls return
///             the existing instance, ensuring a single source of truth for
///             field position data throughout program execution.
//------------------------------------------------------------------
FieldOffsetValues *FieldOffsetValues::GetInstance()
{
    if (FieldOffsetValues::m_instance == nullptr)
    {
        FieldOffsetValues::m_instance = new FieldOffsetValues();
    }
    return FieldOffsetValues::m_instance;
}

//------------------------------------------------------------------
/// @brief      Constructor for FieldOffsetValues
/// @details    Initializes all field position offsets by querying FieldConstants and applying
///             strategic offsets for navigation optimization:
///
///             **Depot and Outpost Positions:**
///             - Retrieves X coordinates from depot neutral side positions, applying DEPOT_OFFSET
///             - Sets outpost X equal to depot X (aligned on the 2026 field)
///             - Outpost approach X is further offset by OUTPOST_APPROACH_OFFSET
///
///             **Tower Positions:**
///             - Caches red and blue tower center poses from FieldConstants
///             - Applies TOWER_X_OFFSET and TOWER_Y_OFFSET to produce four positions:
///               outpost side (X, Y) and depot side (X, Y) for each alliance
///
///             **Hub Positions with Navigation Offsets:**
///             - Caches red and blue hub center poses from FieldConstants
///             - Applies HUB_OFFSET toward the neutral zone for optimal approach angles:
///               Red hub: Hub center X + HUB_OFFSET, Blue hub: Hub center X - HUB_OFFSET
///
///             **Bump Edge X-Positions:**
///             Calculates bump X locations BUMP_OFFSET from hub centers on both sides:
///             - Red alliance bump: Hub center X + BUMP_OFFSET
///             - Red neutral bump:  Hub center X - BUMP_OFFSET
///             - Blue alliance bump: Hub center X - BUMP_OFFSET
///             - Blue neutral bump:  Hub center X + BUMP_OFFSET
///
///             **Bump Y-Coordinates (midpoint series):**
///             Calculates Y positions as midpoints between hub center and corresponding
///             trench alliance positions, with a 1 ft fine-tune adjustment:
///             - m_redBumpDepotY:   Midpoint(red hub Y, red depot trench Y)   + 1 ft
///             - m_redBumpOutpostY: Midpoint(red hub Y, red outpost trench Y)  - 1 ft
///             - m_blueBumpDepotY:  Midpoint(blue hub Y, blue depot trench Y)  - 1 ft
///             - m_blueBumpOutpostY:Midpoint(blue hub Y, blue outpost trench Y) + 1 ft
///
///             **Bump Y-Coordinates (trench entrance series):**
///             Directly uses the trench alliance position Y values so that the
///             cross-field sweep endpoint aligns with the trench entrance:
///             - m_redBumpTrenchDepotY   = RED_TRENCH_ALLIANCE_DEPOT Y
///             - m_redBumpTrenchOutpostY = RED_TRENCH_ALLIANCE_OUTPOST Y
///             - m_blueBumpTrenchDepotY  = BLUE_TRENCH_ALLIANCE_DEPOT Y
///             - m_blueBumpTrenchOutpostY= BLUE_TRENCH_ALLIANCE_OUTPOST Y
///
///             **Fallback Behavior:**
///             If FieldConstants is unavailable (initialization error), all member
///             variables are set to 0.0 m to prevent undefined behavior.
///
/// @note       This constructor is private and called only by GetInstance()
/// @note       All calculations use WPILib units for type safety
//------------------------------------------------------------------
FieldOffsetValues::FieldOffsetValues()
{
    auto fieldConstants = FieldConstants::GetInstance();

    if (fieldConstants != nullptr)
    {
        m_blueDepotX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE).X() + units::length::meter_t{DEPOT_OFFSET};
        m_redDepotX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_DEPOT_NEUTRAL_SIDE).X() - units::length::meter_t{DEPOT_OFFSET};

        // Cache tower center poses (each looked up once instead of 8 times each)
        auto redTowerCenter = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER);
        auto blueTowerCenter = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER);

        m_redTowerOutpostX = redTowerCenter.X() - TOWER_X_OFFSET;
        m_blueTowerOutpostX = blueTowerCenter.X() + TOWER_X_OFFSET;
        m_redTowerOutpostY = redTowerCenter.Y() + TOWER_Y_OFFSET;
        m_blueTowerOutpostY = blueTowerCenter.Y() - TOWER_Y_OFFSET;
        m_redTowerDepotX = redTowerCenter.X() - TOWER_X_OFFSET;
        m_blueTowerDepotX = blueTowerCenter.X() + TOWER_X_OFFSET;
        m_redTowerDepotY = redTowerCenter.Y() - TOWER_Y_OFFSET;
        m_blueTowerDepotY = blueTowerCenter.Y() + TOWER_Y_OFFSET;

        // Set outpost X coordinates equal to depot X (aligned on 2026 field)
        m_blueOutpostX = m_blueDepotX;
        m_redOutpostX = m_redDepotX;

        m_blueOutpostApproachX = m_blueOutpostX + OUTPOST_APPROACH_OFFSET; // Approach position is offset from outpost X
        m_redOutpostApproachX = m_redOutpostX - OUTPOST_APPROACH_OFFSET;   // Approach position is offset from outpost X

        // Cache hub center poses (each looked up once instead of 3 times each)
        auto redHubCenter = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_HUB_CENTER);
        auto blueHubCenter = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_HUB_CENTER);

        // Calculate hub positions with 2.0m offset toward neutral zone for navigation
        m_blueHubX = blueHubCenter.X() - HUB_OFFSET;
        m_redHubX = redHubCenter.X() + HUB_OFFSET;

        // Calculate bump X positions 1.5m from hub centers
        m_redAllianceBumpEdgeX = redHubCenter.X() + BUMP_OFFSET;   // Alliance side of red bump
        m_redNeutralBumpEdgeX = redHubCenter.X() - BUMP_OFFSET;    // Neutral side of red bump
        m_blueAllianceBumpEdgeX = blueHubCenter.X() - BUMP_OFFSET; // Alliance side of blue bump
        m_blueNeutralBumpEdgeX = blueHubCenter.X() + BUMP_OFFSET;  // Neutral side of blue bump

        // Calculate bump Y positions as midpoints between hub and trenches
        m_redBumpDepotY = (((redHubCenter.Y() +
                             fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TRENCH_ALLIANCE_DEPOT).Y()) /
                            2.0) +
                           1.0_ft);
        m_redBumpOutpostY = ((redHubCenter.Y() +
                              fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TRENCH_ALLIANCE_OUTPOST).Y()) /
                             2.0) -
                            1.0_ft;
        m_blueBumpDepotY = ((blueHubCenter.Y() +
                             fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TRENCH_ALLIANCE_DEPOT).Y()) /
                            2.0) -
                           1.0_ft;
        m_blueBumpOutpostY = ((blueHubCenter.Y() +
                               fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TRENCH_ALLIANCE_OUTPOST).Y()) /
                              2.0) +
                             1.0_ft;

        // Calculate bump Y positions as trench entrance Y values (aligns bumps with trench entrances for optimal crossing)
        m_redBumpTrenchDepotY =
            fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TRENCH_ALLIANCE_DEPOT).Y();
        m_redBumpTrenchOutpostY =
            fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TRENCH_ALLIANCE_OUTPOST).Y();
        m_blueBumpTrenchDepotY =
            fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TRENCH_ALLIANCE_DEPOT).Y();
        m_blueBumpTrenchOutpostY =
            fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TRENCH_ALLIANCE_OUTPOST).Y();
    }
    else
    {
        // Fallback: Initialize all values to zero if FieldConstants unavailable
        m_blueDepotX = units::length::meter_t{0.0};
        m_redDepotX = units::length::meter_t{0.0};

        m_blueOutpostX = m_blueDepotX;
        m_redOutpostX = m_redDepotX;

        m_blueOutpostApproachX = m_blueDepotX;
        m_redOutpostApproachX = m_redDepotX;
        m_redTowerOutpostX = units::length::meter_t{0.0};
        m_blueTowerOutpostX = units::length::meter_t{0.0};
        m_redTowerOutpostY = units::length::meter_t{0.0};
        m_blueTowerOutpostY = units::length::meter_t{0.0};
        m_redTowerDepotX = units::length::meter_t{0.0};
        m_blueTowerDepotX = units::length::meter_t{0.0};
        m_redTowerDepotY = units::length::meter_t{0.0};
        m_blueTowerDepotY = units::length::meter_t{0.0};

        m_blueHubX = units::length::meter_t{0.0};
        m_redHubX = units::length::meter_t{0.0};

        m_redAllianceBumpEdgeX = units::length::meter_t{0.0};
        m_redNeutralBumpEdgeX = units::length::meter_t{0.0};
        m_blueAllianceBumpEdgeX = units::length::meter_t{0.0};
        m_blueNeutralBumpEdgeX = units::length::meter_t{0.0};

        m_redBumpDepotY = units::length::meter_t{0.0};
        m_redBumpOutpostY = units::length::meter_t{0.0};
        m_blueBumpDepotY = units::length::meter_t{0.0};
        m_blueBumpOutpostY = units::length::meter_t{0.0};

        m_redBumpTrenchDepotY = units::length::meter_t{0.0};
        m_redBumpTrenchOutpostY = units::length::meter_t{0.0};
        m_blueBumpTrenchDepotY = units::length::meter_t{0.0};
        m_blueBumpTrenchOutpostY = units::length::meter_t{0.0};
    }
}

//------------------------------------------------------------------
/// @brief      Retrieves alliance-specific position value for a field element
/// @param[in]  isRedSide - true for red alliance, false for blue alliance
/// @param[in]  item - The type of field offset coordinate to retrieve
/// @return     units::length::meter_t - The coordinate value in meters
/// @details    Provides a unified interface for querying field element positions with
///             alliance awareness. Handles both X and Y coordinates for various element types.
///
///             **Item Types:**
///
///             **OUTPOST_X:**
///             Returns X-coordinate of the outpost position for the specified alliance
///
///             **OUTPOST_APPROACH_X:**
///             Returns X-coordinate of the outpost approach position (OUTPOST_APPROACH_OFFSET
///             beyond the outpost X) for the specified alliance
///
///             **DEPOT_X:**
///             Returns X-coordinate of the depot neutral side for the specified alliance
///
///             **TOWER_OUTPOST_X / TOWER_DEPOT_X:**
///             Returns X-coordinate of the tower (outpost or depot side) for the specified alliance
///
///             **TOWER_OUTPOST_Y / TOWER_DEPOT_Y:**
///             Returns Y-coordinate of the tower (outpost or depot side) for the specified alliance
///
///             **HUB_X:**
///             Returns X-coordinate of the hub with HUB_OFFSET applied toward the neutral zone:
///             - Red: Hub center X + HUB_OFFSET
///             - Blue: Hub center X - HUB_OFFSET
///
///             **BUMP_ALLIANCE_X:**
///             Returns X-coordinate of the bump edge on the alliance zone side:
///             - Red: Hub center X + BUMP_OFFSET
///             - Blue: Hub center X - BUMP_OFFSET
///
///             **BUMP_NEUTRAL_X:**
///             Returns X-coordinate of the bump edge on the neutral zone side:
///             - Red: Hub center X - BUMP_OFFSET
///             - Blue: Hub center X + BUMP_OFFSET
///
///             **BUMP_ALLIANCE_Y or BUMP_NEUTRAL_Y:**
///             Dynamically determines Y-coordinate based on nearest bump:
///             1. Calls BumpHelper::CalcNearestBump() to identify which bump
///             2. Returns the corresponding midpoint-series Y position:
///                - RED_OUTPOST_BUMP  → m_redBumpOutpostY
///                - RED_DEPOT_BUMP    → m_redBumpDepotY
///                - BLUE_OUTPOST_BUMP → m_blueBumpOutpostY
///                - BLUE_DEPOT_BUMP   → m_blueBumpDepotY (default)
///             Note: The same Y value is returned for both alliance and neutral sides of the same bump
///
///             **Unknown Item:**
///             Returns 0.0 m as a safe fallback for invalid item types
///
/// @note       For bump Y queries, the nearest bump is determined dynamically on every call (not cached)
/// @note       Method is const - does not modify object state
/// @see        FIELD_OFFSET_ITEMS for available item types
/// @see        BumpHelper::CalcNearestBump() for bump identification
//------------------------------------------------------------------
units::length::meter_t FieldOffsetValues::GetValue(bool isRedSide, FIELD_OFFSET_ITEMS item) const
{
    // Outpost X-coordinate query
    if (item == FIELD_OFFSET_ITEMS::OUTPOST_X)
    {
        return isRedSide ? m_redOutpostX : m_blueOutpostX;
    }
    else if (item == FIELD_OFFSET_ITEMS::TOWER_OUTPOST_X)
    {
        return isRedSide ? m_redTowerOutpostX : m_blueTowerOutpostX;
    }
    else if (item == FIELD_OFFSET_ITEMS::TOWER_DEPOT_X)
    {
        return isRedSide ? m_redTowerDepotX : m_blueTowerDepotX;
    }
    else if (item == FIELD_OFFSET_ITEMS::TOWER_OUTPOST_Y)
    {
        return isRedSide ? m_redTowerOutpostY : m_blueTowerOutpostY;
    }
    else if (item == FIELD_OFFSET_ITEMS::TOWER_DEPOT_Y)
    {
        return isRedSide ? m_redTowerDepotY : m_blueTowerDepotY;
    }

    else if (item == FIELD_OFFSET_ITEMS::OUTPOST_APPROACH_X)
    {
        return isRedSide ? m_redOutpostApproachX : m_blueOutpostApproachX;
    }

    // Depot X-coordinate query
    else if (item == FIELD_OFFSET_ITEMS::DEPOT_X)
    {
        return isRedSide ? m_redDepotX : m_blueDepotX;
    }

    // Hub X-coordinate query (with 2.0m navigation offset)
    else if (item == FIELD_OFFSET_ITEMS::HUB_X)
    {
        return isRedSide ? m_redHubX : m_blueHubX;
    }

    // Alliance-side bump X-coordinate query
    else if (item == FIELD_OFFSET_ITEMS::BUMP_ALLIANCE_X)
    {
        return isRedSide ? m_redAllianceBumpEdgeX : m_blueAllianceBumpEdgeX;
    }

    // Neutral-side bump X-coordinate query
    else if (item == FIELD_OFFSET_ITEMS::BUMP_NEUTRAL_X)
    {
        return isRedSide ? m_redNeutralBumpEdgeX : m_blueNeutralBumpEdgeX;
    }

    // Bump Y-coordinate query (dynamic based on nearest bump)
    else if (item == FIELD_OFFSET_ITEMS::BUMP_ALLIANCE_Y || item == FIELD_OFFSET_ITEMS::BUMP_NEUTRAL_Y)
    {
        // Identify which of the four bumps is nearest to robot
        auto bump = BumpHelper::GetInstance()->CalcNearestBump();

        // Return corresponding Y-coordinate for the identified bump
        if (bump == BUMP_ID::RED_OUTPOST_BUMP)
        {
            return m_redBumpOutpostY;
        }
        else if (bump == BUMP_ID::RED_DEPOT_BUMP)
        {
            return m_redBumpDepotY;
        }
        else if (bump == BUMP_ID::BLUE_OUTPOST_BUMP)
        {
            return m_blueBumpOutpostY;
        }
        // Default to blue depot bump
        return m_blueBumpDepotY;
    }

    // Unknown item type - return safe default
    else
    {
        return units::length::meter_t{0.0}; // Fallback for invalid queries
    }
}

//------------------------------------------------------------------
/// @brief      Retrieves an ordered pair of BumpPositions for a cross-field sweep
/// @param[in]  isInNeutralZone - true if the robot is currently in the neutral zone,
///             false if it is in the alliance zone
/// @return     std::vector<BumpPosition> - Two BumpPosition entries (bumpId, x, y) ordered
///             nearest-first, where index 0 is the starting bump and index 1 is the
///             cross-field destination bump
/// @details    Uses BumpHelper::CalcNearestBump() to identify the nearest bump, then
///             returns two BumpPosition entries ordered nearest-first. The X coordinate
///             reflects the side the robot is currently on (alliance or neutral), and the
///             Y coordinate uses the trench-entrance (BumpTrench) series so the sweep
///             endpoint aligns with the trench entrance.
///
///             | Nearest bump      | isInNeutralZone | Index 0 (nearest)                            | Index 1 (cross-field)                         |
///             |-------------------|-----------------|----------------------------------------------|-----------------------------------------------|
///             | RED_OUTPOST_BUMP  | true            | {RED_OUTPOST,  redNeutralX,  redTrenchOutpostY} | {RED_DEPOT,   redNeutralX,  redTrenchDepotY}  |
///             | RED_OUTPOST_BUMP  | false           | {RED_OUTPOST,  redAllianceX, redTrenchOutpostY} | {RED_DEPOT,   redAllianceX, redTrenchDepotY}  |
///             | RED_DEPOT_BUMP    | true            | {RED_DEPOT,    redNeutralX,  redTrenchDepotY}   | {RED_OUTPOST, redNeutralX,  redTrenchOutpostY}|
///             | RED_DEPOT_BUMP    | false           | {RED_DEPOT,    redAllianceX, redTrenchDepotY}   | {RED_OUTPOST, redAllianceX, redTrenchOutpostY}|
///             | BLUE_OUTPOST_BUMP | true            | {BLUE_OUTPOST, blueNeutralX, blueTrenchOutpostY}| {BLUE_DEPOT,  blueNeutralX, blueTrenchDepotY} |
///             | BLUE_OUTPOST_BUMP | false           | {BLUE_OUTPOST, blueAllianceX,blueTrenchOutpostY}| {BLUE_DEPOT,  blueAllianceX,blueTrenchDepotY} |
///             | BLUE_DEPOT_BUMP   | true            | {BLUE_DEPOT,   blueNeutralX, blueTrenchDepotY}  | {BLUE_OUTPOST,blueNeutralX, blueTrenchOutpostY}|
///             | BLUE_DEPOT_BUMP   | false           | {BLUE_DEPOT,   blueAllianceX,blueTrenchDepotY}  | {BLUE_OUTPOST,blueAllianceX,blueTrenchOutpostY}|
///
/// @note       Bump identification queries BumpHelper on every call (not cached)
/// @note       Method is const - does not modify object state
/// @see        GetValue() for single scalar coordinate queries
/// @see        BumpHelper::CalcNearestBump() for bump identification
/// @see        BumpPosition for the returned struct definition
//------------------------------------------------------------------
std::vector<BumpPosition> FieldOffsetValues::GetNearestAndCrossFieldBumpEdges(bool isInNeutralZone) const
{
    std::vector<BumpPosition> values;

    // Identify which of the four bumps is nearest to robot
    auto bump = BumpHelper::GetInstance()->CalcNearestBump();

    // Return corresponding BumpPosition for the identified bump (nearest first)
    if (bump == BUMP_ID::RED_OUTPOST_BUMP)
    {
        if (isInNeutralZone)
        {
            values.emplace_back(BumpPosition{BUMP_ID::RED_OUTPOST_BUMP, m_redNeutralBumpEdgeX, m_redBumpTrenchOutpostY});
            values.emplace_back(BumpPosition{BUMP_ID::RED_DEPOT_BUMP, m_redNeutralBumpEdgeX, m_redBumpTrenchDepotY});
        }
        else
        {
            values.emplace_back(BumpPosition{BUMP_ID::RED_OUTPOST_BUMP, m_redAllianceBumpEdgeX, m_redBumpTrenchOutpostY});
            values.emplace_back(BumpPosition{BUMP_ID::RED_DEPOT_BUMP, m_redAllianceBumpEdgeX, m_redBumpTrenchDepotY});
        }
    }
    else if (bump == BUMP_ID::RED_DEPOT_BUMP)
    {
        if (isInNeutralZone)
        {
            values.emplace_back(BumpPosition{BUMP_ID::RED_DEPOT_BUMP, m_redNeutralBumpEdgeX, m_redBumpTrenchDepotY});
            values.emplace_back(BumpPosition{BUMP_ID::RED_OUTPOST_BUMP, m_redNeutralBumpEdgeX, m_redBumpTrenchOutpostY});
        }
        else
        {
            values.emplace_back(BumpPosition{BUMP_ID::RED_DEPOT_BUMP, m_redAllianceBumpEdgeX, m_redBumpTrenchDepotY});
            values.emplace_back(BumpPosition{BUMP_ID::RED_OUTPOST_BUMP, m_redAllianceBumpEdgeX, m_redBumpTrenchOutpostY});
        }
    }
    else if (bump == BUMP_ID::BLUE_OUTPOST_BUMP)
    {
        if (isInNeutralZone)
        {
            values.emplace_back(BumpPosition{BUMP_ID::BLUE_OUTPOST_BUMP, m_blueNeutralBumpEdgeX, m_blueBumpTrenchOutpostY});
            values.emplace_back(BumpPosition{BUMP_ID::BLUE_DEPOT_BUMP, m_blueNeutralBumpEdgeX, m_blueBumpTrenchDepotY});
        }
        else
        {
            values.emplace_back(BumpPosition{BUMP_ID::BLUE_OUTPOST_BUMP, m_blueAllianceBumpEdgeX, m_blueBumpTrenchOutpostY});
            values.emplace_back(BumpPosition{BUMP_ID::BLUE_DEPOT_BUMP, m_blueAllianceBumpEdgeX, m_blueBumpTrenchDepotY});
        }
    }
    else if (bump == BUMP_ID::BLUE_DEPOT_BUMP)
    {
        if (isInNeutralZone)
        {
            values.emplace_back(BumpPosition{BUMP_ID::BLUE_DEPOT_BUMP, m_blueNeutralBumpEdgeX, m_blueBumpTrenchDepotY});
            values.emplace_back(BumpPosition{BUMP_ID::BLUE_OUTPOST_BUMP, m_blueNeutralBumpEdgeX, m_blueBumpTrenchOutpostY});
        }
        else
        {
            values.emplace_back(BumpPosition{BUMP_ID::BLUE_DEPOT_BUMP, m_blueAllianceBumpEdgeX, m_blueBumpTrenchDepotY});
            values.emplace_back(BumpPosition{BUMP_ID::BLUE_OUTPOST_BUMP, m_blueAllianceBumpEdgeX, m_blueBumpTrenchOutpostY});
        }
    }
    return values;
}
