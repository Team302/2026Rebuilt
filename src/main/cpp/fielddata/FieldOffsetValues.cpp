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
///          for the 2026 game field. It calculates strategic offsets for navigation targets like hubs
///          and bumps, and provides a unified interface for querying field coordinates throughout the codebase.
///
///          The implementation queries FieldConstants for base positions and applies game-specific offsets
///          to optimize navigation paths and bump crossing trajectories.
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
///             - Retrieves X coordinates from depot neutral side positions
///             - Sets outpost X equal to depot X (aligned on the 2026 field)
///
///             **Hub Positions with Navigation Offsets:**
///             - Applies 2.0m offset toward neutral zone for optimal approach angles
///             - Red hub: Base X + 2.0m (moves toward center)
///             - Blue hub: Base X - 2.0m (moves toward center)
///
///             **Bump Edge Positions:**
///             Calculates bump locations 1.5m from hub centers on both sides:
///             - Red alliance bump: Hub X + 1.5m
///             - Red neutral bump: Hub X - 1.5m
///             - Blue alliance bump: Hub X - 1.5m
///             - Blue neutral bump: Hub X + 1.5m
///
///             **Bump Y-Coordinates:**
///             Calculates Y positions as midpoints between hub and corresponding trenches:
///             - Depot bumps: Midpoint of (hub Y, depot trench Y)
///             - Outpost bumps: Midpoint of (hub Y, outpost trench Y)
///             - Ensures bumps align with trench entrances
///
///             **Debug Logging:**
///             Logs all calculated bump positions to NetworkTables for field verification
///             and tuning during testing.
///
///             **Fallback Behavior:**
///             If FieldConstants is unavailable (initialization error), sets all values
///             to 0.0m to prevent undefined behavior.
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

        m_redTowerOutpostX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER).X();
        m_blueTowerOutpostX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER).X();
        m_redTowerOutpostY = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER).Y();
        m_blueTowerOutpostY = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER).Y();
        m_redTowerDepotX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER).X();
        m_blueTowerDepotX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER).X();
        m_redTowerDepotY = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER).Y();
        m_blueTowerDepotY = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER).Y();

        m_blueHubX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_HUB_CENTER).X() - HUB_OFFSET;
        m_redHubX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_HUB_CENTER).X() + HUB_OFFSET;

        m_redTowerOutpostX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER).X();
        m_blueTowerOutpostX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER).X();
        m_redTowerOutpostY = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER).Y();
        m_blueTowerOutpostY = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER).Y();
        m_redTowerDepotX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER).X();
        m_blueTowerDepotX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER).X();
        m_redTowerDepotY = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TOWER_CENTER).Y();
        m_blueTowerDepotY = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TOWER_CENTER).Y();

        // Calculate hub positions with 2.0m offset toward neutral zone for navigation
        m_blueHubX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_HUB_CENTER).X() - HUB_OFFSET;
        m_redHubX = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_HUB_CENTER).X() + HUB_OFFSET;

        // Get hub center poses for bump calculations
        auto redHubCenter = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_HUB_CENTER);
        auto blueHubCenter = fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_HUB_CENTER);

        // Calculate bump X positions 1.5m from hub centers
        m_redAllianceBumpEdgeX = redHubCenter.X() + BUMP_OFFSET;   // Alliance side of red bump
        m_redNeutralBumpEdgeX = redHubCenter.X() - BUMP_OFFSET;    // Neutral side of red bump
        m_blueAllianceBumpEdgeX = blueHubCenter.X() - BUMP_OFFSET; // Alliance side of blue bump
        m_blueNeutralBumpEdgeX = blueHubCenter.X() + BUMP_OFFSET;  // Neutral side of blue bump

        // Calculate bump Y positions as midpoints between hub and trenches
        m_redBumpDepotY = (redHubCenter.Y() +
                           fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TRENCH_ALLIANCE_DEPOT).Y()) /
                          2.0;
        m_redBumpOutpostY = (redHubCenter.Y() +
                             fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TRENCH_ALLIANCE_OUTPOST).Y()) /
                            2.0;
        m_blueBumpDepotY = (blueHubCenter.Y() +
                            fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TRENCH_ALLIANCE_DEPOT).Y()) /
                           2.0;
        m_blueBumpOutpostY = (blueHubCenter.Y() +
                              fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TRENCH_ALLIANCE_OUTPOST).Y()) /
                             2.0;
    }
    else
    {
        // Fallback: Initialize all values to zero if FieldConstants unavailable
        m_blueDepotX = units::length::meter_t{0.0};
        m_redDepotX = units::length::meter_t{0.0};

        m_redTowerOutpostX = units::length::meter_t{0.0};
        m_blueTowerOutpostX = units::length::meter_t{0.0};
        m_redTowerOutpostY = units::length::meter_t{0.0};
        m_blueTowerOutpostY = units::length::meter_t{0.0};
        m_redTowerDepotX = units::length::meter_t{0.0};
        m_blueTowerDepotX = units::length::meter_t{0.0};
        m_redTowerDepotY = units::length::meter_t{0, 0};
        m_blueTowerDepotY = units::length::meter_t{0, 0};
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
    }

    // Set outpost X coordinates equal to depot X (aligned on 2026 field)
    m_blueOutpostX = m_blueDepotX;
    m_redOutpostX = m_redDepotX;
}

//------------------------------------------------------------------
/// @brief      Retrieves alliance-specific position value for a field element
/// @param[in]  isRedSide - true for red alliance, false for blue alliance
/// @param[in]  item - The type of field offset coordinate to retrieve
/// @return     units::length::meter_t - The coordinate value in meters
/// @details    Provides a unified interface for querying field element positions with
///             alliance awareness. Handles both X and Y coordinates for various element types.
///
///             **Switch Logic by Item Type:**
///
///             **OUTPOST_X:**
///             Returns X-coordinate of outpost position for specified alliance
///
///             **DEPOT_X:**
///             Returns X-coordinate of depot neutral side for specified alliance
///
///             **HUB_X:**
///             Returns X-coordinate of hub with 2.0m navigation offset applied:
///             - Red: Hub center + 2.0m
///             - Blue: Hub center - 2.0m
///
///             **ALLIANCE_BUMP_X:**
///             Returns X-coordinate of bump edge on alliance zone side:
///             - Red: Hub center + 1.5m
///             - Blue: Hub center - 1.5m
///
///             **NEUTRAL_BUMP_X:**
///             Returns X-coordinate of bump edge on neutral zone side:
///             - Red: Hub center - 1.5m
///             - Blue: Hub center + 1.5m
///
///             **ALLIANCE_BUMP_Y or NEUTRAL_BUMP_Y:**
///             Dynamically determines Y-coordinate based on nearest bump:
///             1. Calls BumpHelper::CalcNearestBump() to identify which bump
///             2. Returns corresponding Y position:
///                - RED_OUTPOST_BUMP → m_redBumpOutpostY
///                - RED_DEPOT_BUMP → m_redBumpDepotY
///                - BLUE_OUTPOST_BUMP → m_blueBumpOutpostY
///                - BLUE_DEPOT_BUMP → m_blueBumpDepotY (default)
///             Note: Same Y value for both alliance and neutral sides of the same bump
///
///             **Unknown Item:**
///             Returns 0.0m as safe fallback for invalid item types
///
/// @note       For bump Y queries, the nearest bump is determined dynamically each call
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
    else if (item == FIELD_OFFSET_ITEMS::ALLIANCE_BUMP_X)
    {
        return isRedSide ? m_redAllianceBumpEdgeX : m_blueAllianceBumpEdgeX;
    }

    // Neutral-side bump X-coordinate query
    else if (item == FIELD_OFFSET_ITEMS::NEUTRAL_BUMP_X)
    {
        return isRedSide ? m_redNeutralBumpEdgeX : m_blueNeutralBumpEdgeX;
    }

    // Bump Y-coordinate query (dynamic based on nearest bump)
    else if (item == FIELD_OFFSET_ITEMS::ALLIANCE_BUMP_Y || item == FIELD_OFFSET_ITEMS::NEUTRAL_BUMP_Y)
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
