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
/// @file TrenchHelper.cpp
/// @brief Implementation of TrenchHelper singleton for trench identification
/// @details This file implements the trench identification logic for the 2026 game field. The helper
///          delegates nearest-bump identification to BumpHelper and maps the resulting BUMP_ID to the
///          corresponding TRENCH_ID, providing a unified interface for commands that need to navigate
///          into or through a trench.
//====================================================================================================================================================

#include "fielddata/TrenchHelper.h"
#include "auton/AllianceZoneManager.h"
#include "auton/NeutralZoneManager.h"
#include "fielddata/BumpHelper.h"
#include "fielddata/FieldOffsetValues.h"

/// @brief Singleton instance pointer - initialized to nullptr for lazy instantiation
TrenchHelper *TrenchHelper::m_instance = nullptr;

//------------------------------------------------------------------
/// @brief      Get the singleton instance of TrenchHelper
/// @return     TrenchHelper* - Pointer to the singleton instance
/// @details    Implements lazy initialization singleton pattern. Creates the
///             instance on first call, subsequent calls return existing instance.
///             Ensures only one TrenchHelper object exists throughout program execution.
//------------------------------------------------------------------
TrenchHelper *TrenchHelper::GetInstance()
{
    if (TrenchHelper::m_instance == nullptr)
    {
        TrenchHelper::m_instance = new TrenchHelper();
    }
    return TrenchHelper::m_instance;
}

//------------------------------------------------------------------
/// @brief      Constructor for TrenchHelper
/// @details    Initializes the singleton by retrieving references to required subsystems:
///
///             This constructor is private and called only by GetInstance() during
///             first-time initialization of the singleton.
//------------------------------------------------------------------
TrenchHelper::TrenchHelper()
{
}

//------------------------------------------------------------------
/// @brief      Identifies the trench nearest to the robot's current position
/// @return     TRENCH_ID - Enumeration identifying the closest trench
/// @details    Delegates to BumpHelper::CalcNearestBump() to identify the nearest bump,
///             then maps the result to the corresponding TRENCH_ID:
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
/// @see        TRENCH_ID for all possible return values
//------------------------------------------------------------------
TRENCH_ID TrenchHelper::CalcNearestTrench() const
{
    auto nearestBump = BumpHelper::GetInstance()->CalcNearestBump();
    switch (nearestBump)
    {
    case BUMP_ID::BLUE_DEPOT_BUMP:
        return TRENCH_ID::BLUE_DEPOT_TRENCH;
    case BUMP_ID::BLUE_OUTPOST_BUMP:
        return TRENCH_ID::BLUE_OUTPOST_TRENCH;
    case BUMP_ID::RED_DEPOT_BUMP:
        return TRENCH_ID::RED_DEPOT_TRENCH;
    case BUMP_ID::RED_OUTPOST_BUMP:
        return TRENCH_ID::RED_OUTPOST_TRENCH;
    default:
        return TRENCH_ID::RED_OUTPOST_TRENCH; // Default case to satisfy compiler, should not occur if BumpHelper returns valid BUMP_ID
    }

    // Fallback return to satisfy compiler warnings about non-void function possibly not returning
    return TRENCH_ID::RED_OUTPOST_TRENCH;
}

//------------------------------------------------------------------
/// @brief      Returns an ordered list of Pose2d drive targets for navigating through a trench
/// @param[in]  isRedAlliance - true for red alliance, false for blue alliance
/// @return     std::vector<frc::Pose2d> - Ordered poses: index 0 is the mid (near-trench)
///             pose, index 1 is the end (far-side) pose
/// @details    Determines the nearest trench via CalcNearestTrench(), selects the correct
///             starting and ending trench pair for the given alliance, and builds Pose2d waypoints
///             using FieldOffsetValues trench X/Y coordinates. If the robot is in the other alliance's
///             zone, only the far-side pose is returned since the near-trench pose would be inaccessible.
/// @see        CalcNearestTrench() for trench identification
/// @see        FieldOffsetValues for the coordinate source data
/// @note       The back of the robot faces the alliance wall, so when on red alliance, the robot faces the blue alliance wall.
///------------------------------------------------------------------
std::vector<frc::Pose2d> TrenchHelper::GetTrenchDrivePositions(bool isRedAlliance) const
{
    auto rotation = isRedAlliance ? frc::Rotation2d{kFaceBlueWallRotation} : frc::Rotation2d{kFaceRedWallRotation};
    auto isInNeutralZone = NeutralZoneManager::GetInstance()->IsInNeutralZone();
    auto isInOtherAllianceZone = AllianceZoneManager::GetInstance()->IsInOtherAllianceZone();
    auto nearest = CalcNearestTrench();

    bool isRedDepotPair = (nearest == TRENCH_ID::RED_DEPOT_TRENCH || nearest == TRENCH_ID::BLUE_OUTPOST_TRENCH);
    auto redTrench = isRedDepotPair ? TRENCH_ID::RED_DEPOT_TRENCH : TRENCH_ID::RED_OUTPOST_TRENCH;
    auto blueTrench = isRedDepotPair ? TRENCH_ID::BLUE_OUTPOST_TRENCH : TRENCH_ID::BLUE_DEPOT_TRENCH;

    auto startingTrench = isRedAlliance ? blueTrench : redTrench;
    auto endTrench = isInOtherAllianceZone ? startingTrench : (isRedAlliance ? redTrench : blueTrench);

    auto offsetVals = FieldOffsetValues::GetInstance();
    std::vector<frc::Pose2d> poses;

    auto GetTrenchX = [&](TRENCH_ID id) -> units::length::meter_t
    {
        switch (id)
        {
        case TRENCH_ID::RED_DEPOT_TRENCH:
        case TRENCH_ID::RED_OUTPOST_TRENCH:
            return isInNeutralZone ? offsetVals->GetNeutralRedTrenchX() : offsetVals->GetRedTrenchX();
        case TRENCH_ID::BLUE_DEPOT_TRENCH:
        case TRENCH_ID::BLUE_OUTPOST_TRENCH:
        default:
            return isInNeutralZone ? offsetVals->GetNeutralBlueTrenchX() : offsetVals->GetBlueTrenchX();
        }
    };

    auto GetTrenchY = [&](TRENCH_ID id) -> units::length::meter_t
    {
        switch (id)
        {
        case TRENCH_ID::RED_DEPOT_TRENCH:
            return offsetVals->GetRedDepotTrenchToTrenchY();
        case TRENCH_ID::RED_OUTPOST_TRENCH:
            return offsetVals->GetRedOutpostTrenchToTrenchY();
        case TRENCH_ID::BLUE_DEPOT_TRENCH:
            return offsetVals->GetBlueDepotTrenchToTrenchY();
        case TRENCH_ID::BLUE_OUTPOST_TRENCH:
        default:
            return offsetVals->GetBlueOutpostTrenchToTrenchY();
        }
    };

    if (isInOtherAllianceZone)
    {
        poses.emplace_back(frc::Pose2d{GetTrenchX(endTrench), GetTrenchY(endTrench), rotation});
    }
    else
    {
        poses.emplace_back(frc::Pose2d{GetTrenchX(startingTrench), GetTrenchY(startingTrench), rotation});
        poses.emplace_back(frc::Pose2d{GetTrenchX(endTrench), GetTrenchY(endTrench), rotation});
    }
    return poses;
}
