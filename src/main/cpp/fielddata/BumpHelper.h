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

#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "fielddata/FieldConstants.h"
#include "frc/geometry/Pose2d.h"

//====================================================================================================================================================
/// @class DepotHelper
/// @brief Helper class for depot-related calculations and navigation
///
/// This singleton class provides utilities for interacting with depots on the field, including:
/// - Determining which depot (red or blue) is closest to the robot
/// - Calculating the center pose of the nearest depot
/// - Computing distances to field elements
///
/// The class uses the robot's current pose and field constants to make alliance-aware decisions
/// about depot locations and navigation targets.
//====================================================================================================================================================
enum class BUMP_ID
{
    BLUE_DEPOT_BUMP,
    BLUE_OUTPOST_BUMP,
    RED_DEPOT_BUMP,
    RED_OUTPOST_BUMP
};

class BumpHelper
{
public:
    //------------------------------------------------------------------
    /// @brief      Get the singleton instance of DepotHelper
    /// @return     DepotHelper* - Pointer to the singleton instance
    //------------------------------------------------------------------
    static BumpHelper *GetInstance();

    //------------------------------------------------------------------
    /// @brief      Calculate the ID of the nearest bump to the robot
    /// @details    Evaluates the robot's current pose relative to all four
    ///             bump locations and returns the ID of the closest one using
    ///             a hierarchical distance comparison algorithm:
    ///             1. First determines which alliance side (red or blue) is 
    ///                closer by comparing depot positions
    ///             2. Then determines whether the outpost or depot bump is 
    ///                closer on that side
    /// @return     BUMP_ID - Enumeration indicating the nearest bump:
    ///             - BLUE_DEPOT_BUMP: Blue alliance depot bump is nearest
    ///             - BLUE_OUTPOST_BUMP: Blue alliance outpost bump is nearest
    ///             - RED_DEPOT_BUMP: Red alliance depot bump is nearest
    ///             - RED_OUTPOST_BUMP: Red alliance outpost bump is nearest
    /// @note       If the chassis is unavailable, defaults to origin pose (0,0)
    ///             for distance calculations
    /// @see        BUMP_ID
    /// @see        PoseUtils::GetClosestFieldElement()
    //------------------------------------------------------------------
    BUMP_ID CalcNearestBump() const;

private:
    //------------------------------------------------------------------
    /// @brief      Private constructor for singleton pattern
    /// @details    Initializes the chassis and field constants references
    //------------------------------------------------------------------
    BumpHelper();

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    //------------------------------------------------------------------
    ~BumpHelper() = default;

    /// @brief Pointer to the swerve drivetrain subsystem
    subsystems::CommandSwerveDrivetrain *m_chassis;

    /// @brief Pointer to the field constants singleton
    FieldConstants *m_fieldConstants;

    /// @brief Singleton instance pointer
    static BumpHelper *m_instance;
};

