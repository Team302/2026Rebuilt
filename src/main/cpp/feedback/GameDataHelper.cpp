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

#include "feedback/GameDataHelper.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "state/RobotState.h"
#include "utils/FMSData.h"
#include "utils/PeriodicLooper.h"
#include <algorithm>
#include <array>

GameDataHelper::GameDataHelper()
{
    PeriodicLooper::GetInstance()->RegisterAll(this);
    frc::SmartDashboard::PutBoolean(m_hubActiveNT, false);
    frc::SmartDashboard::PutNumber(m_allianceShiftTime, 25.0);
    frc::SmartDashboard::PutString(m_firstActiveHubNT, m_firstActiveHub);
}

void GameDataHelper::PublishHubActive(bool value)
{
    if (m_hubActive != value)
    {
        frc::SmartDashboard::PutBoolean(m_hubActiveNT, value);
        m_hubActive = value;
        RobotState::GetInstance()->PublishStateChange(RobotStateChanges::StateChange::HubActive_Bool, value);
    }
}

void GameDataHelper::PublishStartLaunching(bool value)
{
    if (m_startLaunching != value)
    {
        m_startLaunching = value;
        RobotState::GetInstance()->PublishStateChange(RobotStateChanges::StateChange::StartLaunching_Bool, value);
    }
}

void GameDataHelper::PublishShiftChangeIn5seconds(bool value)
{
    if (m_shiftChangeIn5seconds != value)
    {
        m_shiftChangeIn5seconds = value;
        RobotState::GetInstance()->PublishStateChange(RobotStateChanges::StateChange::ShiftChangeIn5Seconds_Bool, value);
    }
}

void GameDataHelper::RunCurrentState()
{

    if (frc::DriverStation::IsAutonomousEnabled())
    {
        PublishHubActive(true);
        PublishStartLaunching(false);
        PublishShiftChangeIn5seconds(false);
    }
    else
    {
        units::time::second_t matchTime = frc::DriverStation::GetMatchTime();
        std::string gameData = frc::DriverStation::GetGameSpecificMessage();
        bool redInactiveFirst = (gameData == "R");

        // Match Time Decreases - use array with upper_bound for cleaner logic
        static const std::array<units::time::second_t, 5> shiftThresholds = {
            m_shift1Start,
            m_shift2Start,
            m_shift3Start,
            m_shift4Start,
            m_endgameStart};

        // Find which shift we're in by counting how many thresholds we've passed
        int currentShift = std::distance(
            shiftThresholds.begin(),
            std::upper_bound(shiftThresholds.begin(), shiftThresholds.end(), matchTime,
                             std::greater<units::time::second_t>()));
        // currentShift: 0 = transition, 1-4 = shifts, 5 = endgame

        // Logic for Hub Activity (Alternates every shift)
        // Shift 1, 3: Red inactive if redInactiveFirst is true
        // Shift 2, 4: Red active if redInactiveFirst is true
        bool isOddShift = (currentShift % 2 != 0);

        if (currentShift >= 1 && currentShift <= 4)
        {
            // If it's an odd shift (1, 3) and red is supposed to be inactive first...
            bool shouldBeInactive = (isOddShift == redInactiveFirst);

            if (FMSData::GetAllianceColor() == frc::DriverStation::Alliance::kRed)
            {
                PublishHubActive(!shouldBeInactive);
            }
            else // if it's an even shift (2, 4) and red was NOT inactive first...

            {
                PublishHubActive(shouldBeInactive);
            }
        }
        else
        {
            // In Transition or Endgame, both hubs active
            PublishHubActive(true);
        }

        // Logic for Countdown Warnings
        // Check if we are within 5 or 3 seconds of the NEXT shift
        units::time::second_t timeToNextShift = 0_s;
        if (currentShift < static_cast<int>(shiftThresholds.size()))
        {
            timeToNextShift = matchTime - shiftThresholds[currentShift];
        }

        if (gameData == "R" && m_firstActiveHub == "Undecided")
        {
            m_firstActiveHub = "Blue";
            frc::SmartDashboard::PutString(m_firstActiveHubNT, m_firstActiveHub);
        }
        else if (gameData == "B" && m_firstActiveHub == "Undecided")
        {
            m_firstActiveHub = "Red";
            frc::SmartDashboard::PutString(m_firstActiveHubNT, m_firstActiveHub);
        }
        frc::SmartDashboard::PutNumber(m_allianceShiftTime, timeToNextShift.value());
        PublishShiftChangeIn5seconds(timeToNextShift <= 5.0_s && timeToNextShift > 0_s);
        PublishStartLaunching(timeToNextShift <= 1.5_s && timeToNextShift > 0_s);
    }
}
