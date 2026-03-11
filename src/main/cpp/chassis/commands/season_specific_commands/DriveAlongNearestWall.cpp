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

#include "chassis/commands/season_specific_commands/DriveAlongNearestWall.h"
#include "fielddata/BumpHelper.h"
#include "utils/logging/debug/Logger.h"

//------------------------------------------------------------------
/// @brief      Constructor for DriveAlongNearestWall command
/// @param[in]  chassis - Pointer to the swerve drive subsystem
/// @details    Initializes the base TrajectoryDrive command with the chassis
///             and retrieves the BumpHelper singleton for wall proximity detection.
//------------------------------------------------------------------
DriveAlongNearestWall::DriveAlongNearestWall(subsystems::CommandSwerveDrivetrain *chassis)
    : TrajectoryDrive(chassis),
      m_bumpHelper(BumpHelper::GetInstance())
{
}

//------------------------------------------------------------------
/// @brief      Initialize the command and select appropriate trajectory
/// @details    Determines which wall is nearest to the robot and selects
///             the corresponding trajectory path. The logic works as follows:
///
///             **Step 1: Determine Nearest Bump**
///             Uses BumpHelper::CalcNearestBump() to identify which of the
///             four field bumps (Red/Blue Depot/Outpost) is closest.
///
///             **Step 2: Select Trajectory**
///             - If nearest bump is a DEPOT bump (Red or Blue): Use DepotAllianceSweep
///             - If nearest bump is an OUTPOST bump (Red or Blue): Use OutpostAllianceSweep
///
///             **Step 3: Load Trajectory**
///             Calls SetPath() with the selected path name, then calls the base
///             class Initialize() which will handle:
///             - Loading the trajectory from the path file
///             - Automatically flipping for Red alliance if needed
///             - Preparing controllers and timers
///
/// @note       The trajectory flipping for alliance color is handled automatically
///             by the base TrajectoryDrive class's AutonUtils::GetTrajectoryFromPathFile()
//------------------------------------------------------------------
void DriveAlongNearestWall::Initialize()
{
    // Determine which bump/wall is nearest to the robot's current position
    BUMP_ID nearestBump = m_bumpHelper->CalcNearestBump();

    // Select the appropriate path based on whether the nearest bump is Depot or Outpost
    std::string pathName;
    switch (nearestBump)
    {
    case BUMP_ID::BLUE_DEPOT_BUMP:
    case BUMP_ID::RED_DEPOT_BUMP:
        pathName = kDepotAllianceSweepPath;
        break;

    case BUMP_ID::BLUE_OUTPOST_BUMP:
    case BUMP_ID::RED_OUTPOST_BUMP:
    default:
        pathName = kOutpostAllianceSweepPath;
        break;
    }

    // Set the path for the base TrajectoryDrive class
    SetPath(pathName);

    InitializeForTeleop(nearestBump == BUMP_ID::RED_DEPOT_BUMP || nearestBump == BUMP_ID::RED_OUTPOST_BUMP);
}
