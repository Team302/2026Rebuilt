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
#include "chassis/commands/season_specific_commands/DriveOverBump.h"
#include "fielddata/BumpHelper.h"

#include "auton/AllianceZoneManager.h"
#include "auton/NeutralZoneManager.h"
#include "fielddata/FieldOffsetValues.h"
#include "utils/PoseUtils.h"

//------------------------------------------------------------------
/// @brief      Constructor for DriveToDepot command
/// @param[in]  chassis - Pointer to the swerve drive subsystem
/// @details    Initializes the base DriveToPose command with the chassis.
///             This command autonomously drives the robot to the nearest
///             depot on the field.
//------------------------------------------------------------------
DriveOverBump::DriveOverBump(subsystems::CommandSwerveDrivetrain *chassis) : DriveToPose(chassis), m_midPose(), m_endPose()
{
}

//------------------------------------------------------------------
/// @brief      Calculates the target end pose for the depot
/// @return     frc::Pose2d - The target pose at the center of the nearest depot
/// @details    Uses DepotHelper to determine which depot (red or blue) is
///             closest to the robot and calculates the center pose of that
///             depot. Returns a default pose if DepotHelper is unavailable.
//------------------------------------------------------------------
frc::Pose2d DriveOverBump::GetEndPose()
{
    frc::Pose2d endPose{};
    frc::Pose2d midPose{};
    units::angle::degree_t rotation{45_deg};

    auto bumpHelper = BumpHelper::GetInstance();
    if (bumpHelper != nullptr)
    {
        auto bump = bumpHelper->CalcNearestBump();
        auto isInNeutralZone = NeutralZoneManager::GetInstance()->IsInNeutralZone();

        auto rotation = GetRotation(bump, isInNeutralZone);

        auto isRed = (bump == BUMP_ID::RED_DEPOT_BUMP || bump == BUMP_ID::RED_OUTPOST_BUMP);

        auto offsetVals = FieldOffsetValues::GetInstance();
        auto neutralX = offsetVals->GetValue(isRed, FIELD_OFFSET_ITEMS::NEUTRAL_BUMP_X);
        auto neutralY = offsetVals->GetValue(isRed, FIELD_OFFSET_ITEMS::NEUTRAL_BUMP_Y);

        auto allianceX = offsetVals->GetValue(isRed, FIELD_OFFSET_ITEMS::ALLIANCE_BUMP_X);
        auto allianceY = offsetVals->GetValue(isRed, FIELD_OFFSET_ITEMS::ALLIANCE_BUMP_Y);

        if (isInNeutralZone) // Drive from neutral zone over bump to alliance zone
        {
            m_midPose = frc::Pose2d(neutralX, neutralY, frc::Rotation2d(rotation));
            m_endPose = frc::Pose2d(allianceX, allianceY, frc::Rotation2d(rotation));
        }
        else // Drive from alliance zone over bump to neutral zone
        {
            m_midPose = frc::Pose2d(allianceX, allianceY, frc::Rotation2d(rotation));
            m_endPose = frc::Pose2d(neutralX, neutralY, frc::Rotation2d(rotation));
        }
    }

    return m_midPose;
}

units::angle::degree_t DriveOverBump::GetRotation(BUMP_ID bump, bool isInNeutralZone) const
{

    // Blue alliance sees forward as 0 degrees (toward red alliance wall)
    // Red alliance sees forward as 180 degrees (toward blue alliance wall)
    // 90 degrees is toward red outpost / blue depot
    // 270 degrees is toward blue outpost / red depot

    switch (bump)
    {
    case BUMP_ID::RED_OUTPOST_BUMP:
        return isInNeutralZone ? NeutralZoneTowardHubRedOutpost : RedAllianceOutpostWallTowardHub;
    case BUMP_ID::BLUE_OUTPOST_BUMP:
        return isInNeutralZone ? NeutralZoneTowardHubBlueOutpost : BlueAllianceOutpostWallTowardHub;
    case BUMP_ID::RED_DEPOT_BUMP:
        return isInNeutralZone ? NeutralZoneTowardHubRedDepot : RedAllianceDepotWallTowardHub;
    case BUMP_ID::BLUE_DEPOT_BUMP:
    default:
        return isInNeutralZone ? NeutralZoneTowardHubBlueDepot : BlueAllianceDepotWallTowardHub;
    }
}

//------------------------------------------------------------------
/// @brief Checks if the DriveToDepot command has finished execution.
///
/// @return true if the end pose is at the origin or if the base class's IsFinished
///         condition is met, false otherwise.
///
/// @details This method determines whether the command should terminate by first checking
///          if the end pose is at the origin (which means we had an error calculating the
///          target position) so we stop immediately.  Otherwise, it delegates to the base class's
///          IsFinished() method to determine completion.
//------------------------------------------------------------------
bool DriveOverBump::IsFinished()
{
    auto endPose = GetEndPose();
    if (PoseUtils::IsPoseAtOrigin(endPose, units::length::centimeter_t{1.0}))
    {
        return true;
    }
    if (m_beforeMidPose && DriveToPose::IsFinished())
    {
        SetEndPose(m_endPose);
        m_beforeMidPose = false;
        return false;
    }

    return DriveToPose::IsFinished(); // call base class's IsFinished method
}
