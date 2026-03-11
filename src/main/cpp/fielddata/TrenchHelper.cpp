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
/// @brief Implementation of TrenchHelper singleton for bump identification and distance calculations
/// @details This file implements the bump identification logic for the 2026 game field. The helper uses a
///          hierarchical distance comparison strategy to efficiently determine which of the four field bumps
///          (red/blue depot/outpost) is nearest to the robot's current position.
///
///          The implementation leverages existing utility functions from PoseUtils and field element data
///          from FieldConstants to provide fast, accurate bump identification for navigation commands.
//====================================================================================================================================================

#include "auton/AllianceZoneManager.h"
#include "auton/NeutralZoneManager.h"
#include "fielddata/TrenchHelper.h"
#include "chassis/ChassisConfigMgr.h"
#include "frc/geometry/Pose2d.h"
#include "utils/FMSData.h"

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
///             - Swerve chassis from ChassisConfigMgr for robot pose queries
///             - FieldConstants singleton for field element positions
///
///             This constructor is private and called only by GetInstance() during
///             first-time initialization of the singleton.
//------------------------------------------------------------------
TrenchHelper::TrenchHelper() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                               m_fieldConstants(FieldConstants::GetInstance())
{
}

//------------------------------------------------------------------
/// @brief      Calculates which bump is nearest to the robot's current position
/// @return     TRENCH_ID - Enumeration identifying the closest bump
/// @details    Implements a two-stage hierarchical comparison algorithm for efficient bump identification:
///
///             **Algorithm Overview:**
///             The method uses a divide-and-conquer approach to avoid comparing all four bumps directly:
///
///             **Stage 1: Determine Alliance Side (Red vs Blue)**
///             - Gets current robot pose from chassis (or origin if chassis unavailable)
///             - Compares distance to BLUE_HUB_CENTER vs RED_HUB_CENTER
///             - Uses hub centers as reference points for more accurate alliance side determination
///             - Result: Identifies which half of the field the robot is on
///
///             **Stage 2: Determine Position on Side (Depot vs Outpost)**
///             Based on the alliance side determined in Stage 1:
///
///             *If Blue Side:*
///             - Compares distance to BLUE_DEPOT_NEUTRAL_SIDE vs BLUE_OUTPOST_CENTER
///             - Returns BLUE_DEPOT_BUMP if depot is closer
///             - Returns BLUE_OUTPOST_BUMP if outpost is closer
///
///             *If Red Side:*
///             - Compares distance to RED_DEPOT_NEUTRAL_SIDE vs RED_OUTPOST_CENTER
///             - Returns RED_DEPOT_BUMP if depot is closer
///             - Returns RED_OUTPOST_BUMP if outpost is closer
///
///             **Efficiency:**
///             This hierarchical approach requires only 2 distance comparisons instead of
///             evaluating all 4 bumps, reducing computation time by 50%.
///
///             **Field Element References:**
///             - BLUE_HUB_CENTER: Center of blue alliance hub (Stage 1)
///             - RED_HUB_CENTER: Center of red alliance hub (Stage 1)
///             - BLUE_DEPOT_NEUTRAL_SIDE: Neutral zone side of blue depot (Stage 2)
///             - RED_DEPOT_NEUTRAL_SIDE: Neutral zone side of red depot (Stage 2)
///             - BLUE_OUTPOST_CENTER: Center of blue outpost area (Stage 2)
///             - RED_OUTPOST_CENTER: Center of red outpost area (Stage 2)
///
/// @note       Returns a valid TRENCH_ID even if chassis is unavailable (uses origin pose)
/// @note       Method is const - safe to call from multiple contexts without side effects
/// @see        PoseUtils::GetClosestFieldElement() for distance comparison implementation
/// @see        TRENCH_ID for possible return values
/// @see        DriveOverBump for primary consumer of this method
//------------------------------------------------------------------
TRENCH_ID TrenchHelper::CalcNearestTrench() const
{
    // Get current robot pose using cached chassis pointer (defaults to origin if chassis unavailable)
    auto currentPose = (m_chassis != nullptr) ? m_chassis->GetPose() : frc::Pose2d();

    if (m_fieldConstants == nullptr)
    {
        return TRENCH_ID::BLUE_DEPOT_TRENCH;
    }

    // Stage 1: Determine which alliance side we are on if we are in the neutral zone by using fms data
    // Compare distance to hub centers as reference points for each side
    // Use cached m_fieldConstants instead of re-fetching singleton through PoseUtils
    if (NeutralZoneManager::GetInstance()->IsInNeutralZone())
    {
        if (FMSData::GetInstance()->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            if (BumpHelper::GetInstance()->CalcNearestBump(currentPose) == TRENCH_ID::BLUE_DEPOT_BUMP)
            {
                return TRENCH_ID::BLUE_DEPOT_TRENCH;
            }
            else
            {
                return TRENCH_ID::BLUE_OUTPOST_TRENCH;
            }
        }
        else
        {
            if (BumpHelper::GetInstance()->CalcNearestBump(currentPose) == TRENCH_ID::RED_DEPOT_BUMP)
            {
                return TRENCH_ID::RED_DEPOT_TRENCH;
            }
            else
            {
                return TRENCH_ID::RED_OUTPOST_TRENCH;
            }
        }
    }
    else if (AllianceZoneManager::GetInstance()->IsInAllianceZone())
    {
        if (FMSData::GetInstance()->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            if (BumpHelper::GetInstance()->CalcNearestBump(currentPose) == TRENCH_ID::BLUE_DEPOT_BUMP)
            {
                return TRENCH_ID::BLUE_DEPOT_TRENCH;
            }
            else
            {
                return TRENCH_ID::BLUE_OUTPOST_TRENCH;
            }
        }
        else
        {
            if (BumpHelper::GetInstance()->CalcNearestBump(currentPose) == TRENCH_ID::RED_DEPOT_BUMP)
            {
                return TRENCH_ID::RED_DEPOT_TRENCH;
            }
            else
            {
                return TRENCH_ID::RED_OUTPOST_TRENCH;
            }
        }
    }
    else
    {
    }

    auto blueHubPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_HUB_CENTER);

    if (closerToBlue) // Robot is closer to blue side
    {
        // Stage 2: On blue side, determine if depot or outpost is closer
        // Use cached m_fieldConstants directly for distance calculations
        auto blueDepotPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TRENCH_ALLIANCE_DEPOT);
        auto blueOutpostPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_TRENCH_ALLIANCE_OUTPOST);
        auto distToDepot = currentPose.Translation().Distance(blueDepotPose.Translation());
        auto distToOutpost = currentPose.Translation().Distance(blueOutpostPose.Translation());
        if (distToDepot < distToOutpost)
        {
            return TRENCH_ID::BLUE_DEPOT_BUMP; // Blue depot bump is nearest
        }
        return TRENCH_ID::BLUE_OUTPOST_BUMP; // Blue outpost bump is nearest
    }

    // Robot is closer to red side
    // Stage 2: On red side, determine if depot or outpost is closer
    auto redDepotPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TRENCH_ALLIANCE_DEPOT);
    auto redOutpostPose = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_TRENCH_ALLIANCE_OUTPOST);
    auto distToDepot = currentPose.Translation().Distance(redDepotPose.Translation());
    auto distToOutpost = currentPose.Translation().Distance(redOutpostPose.Translation());
    if (distToDepot < distToOutpost)
    {
        return TRENCH_ID::RED_DEPOT_BUMP; // Red depot bump is nearest
    }
    return TRENCH_ID::RED_OUTPOST_BUMP; // Red outpost bump is nearest
}
