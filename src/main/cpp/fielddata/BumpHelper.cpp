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
/// @file BumpHelper.cpp
/// @brief Implementation of BumpHelper singleton for bump identification and distance calculations
/// @details This file implements the bump identification logic for the 2026 game field. The helper uses a
///          hierarchical distance comparison strategy to efficiently determine which of the four field bumps
///          (red/blue depot/outpost) is nearest to the robot's current position.
///
///          The implementation leverages existing utility functions from PoseUtils and field element data
///          from FieldConstants to provide fast, accurate bump identification for navigation commands.
//====================================================================================================================================================

#include "fielddata/BumpHelper.h"
#include "auton/NeutralZoneManager.h"
#include "chassis/ChassisConfigMgr.h"
#include "frc/geometry/Pose2d.h"
#include "utils/PoseUtils.h"

/// @brief Singleton instance pointer - initialized to nullptr for lazy instantiation
BumpHelper *BumpHelper::m_instance = nullptr;

//------------------------------------------------------------------
/// @brief      Get the singleton instance of BumpHelper
/// @return     BumpHelper* - Pointer to the singleton instance
/// @details    Implements lazy initialization singleton pattern. Creates the
///             instance on first call, subsequent calls return existing instance.
///             Ensures only one BumpHelper object exists throughout program execution.
//------------------------------------------------------------------
BumpHelper *BumpHelper::GetInstance()
{
    if (BumpHelper::m_instance == nullptr)
    {
        BumpHelper::m_instance = new BumpHelper();
    }
    return BumpHelper::m_instance;
}

//------------------------------------------------------------------
/// @brief      Constructor for BumpHelper
/// @details    Initializes the singleton by retrieving references to required subsystems:
///             - Swerve chassis from ChassisConfigMgr for robot pose queries
///             - FieldConstants singleton for field element positions
///
///             This constructor is private and called only by GetInstance() during
///             first-time initialization of the singleton.
//------------------------------------------------------------------
BumpHelper::BumpHelper() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                           m_fieldConstants(FieldConstants::GetInstance())
{
}

//------------------------------------------------------------------
/// @brief      Calculates which bump is nearest to the robot's current position
/// @return     BUMP_ID - Enumeration identifying the closest bump
/// @details    Implements a two-stage hierarchical comparison algorithm for efficient bump identification:
///
///             **Algorithm Overview:**
///             The method uses a divide-and-conquer approach to avoid comparing all four bumps directly:
///
///             **Stage 1: Determine Alliance Side (Red vs Blue)**
///             - Gets current robot pose from chassis (or origin if chassis unavailable)
///             - Compares distance to BLUE_DEPOT_NEUTRAL_SIDE vs RED_DEPOT_NEUTRAL_SIDE
///             - Uses depot positions as reference points because they're centrally located on each side
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
///             - BLUE_DEPOT_NEUTRAL_SIDE: Neutral zone side of blue depot
///             - RED_DEPOT_NEUTRAL_SIDE: Neutral zone side of red depot
///             - BLUE_OUTPOST_CENTER: Center of blue outpost area
///             - RED_OUTPOST_CENTER: Center of red outpost area
///
/// @note       Returns a valid BUMP_ID even if chassis is unavailable (uses origin pose)
/// @note       Method is const - safe to call from multiple contexts without side effects
/// @see        PoseUtils::GetClosestFieldElement() for distance comparison implementation
/// @see        BUMP_ID for possible return values
/// @see        DriveOverBump for primary consumer of this method
//------------------------------------------------------------------
BUMP_ID BumpHelper::CalcNearestBump() const
{
    // Get current robot pose (defaults to origin if chassis unavailable)
    auto chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();
    auto currentPose = (chassis != nullptr) ? chassis->GetPose() : frc::Pose2d();

    // Stage 1: Determine which alliance side (blue or red) is closer
    // Compare distance to depot neutral sides as reference points for each side
    auto closestDepot = PoseUtils::GetClosestFieldElement(currentPose,
                                                          FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE,
                                                          FieldConstants::FIELD_ELEMENT::RED_DEPOT_NEUTRAL_SIDE);

    if (closestDepot == FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE) // Robot is closer to blue side
    {
        // Stage 2: On blue side, determine if depot or outpost is closer
        auto blueElement = PoseUtils::GetClosestFieldElement(currentPose,
                                                             FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE,
                                                             FieldConstants::FIELD_ELEMENT::BLUE_OUTPOST_CENTER);
        if (blueElement == FieldConstants::FIELD_ELEMENT::BLUE_DEPOT_NEUTRAL_SIDE)
        {
            return BUMP_ID::BLUE_DEPOT_BUMP; // Blue depot bump is nearest
        }
        return BUMP_ID::BLUE_OUTPOST_BUMP; // Blue outpost bump is nearest
    }

    // Robot is closer to red side
    // Stage 2: On red side, determine if depot or outpost is closer
    auto redElement = PoseUtils::GetClosestFieldElement(currentPose,
                                                        FieldConstants::FIELD_ELEMENT::RED_DEPOT_NEUTRAL_SIDE,
                                                        FieldConstants::FIELD_ELEMENT::RED_OUTPOST_CENTER);
    if (redElement == FieldConstants::FIELD_ELEMENT::RED_DEPOT_NEUTRAL_SIDE)
    {
        return BUMP_ID::RED_DEPOT_BUMP; // Red depot bump is nearest
    }
    return BUMP_ID::RED_OUTPOST_BUMP; // Red outpost bump is nearest
}
