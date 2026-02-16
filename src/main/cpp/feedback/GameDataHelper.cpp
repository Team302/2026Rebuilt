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
#include "state/RobotState.h"
#include "utils/PeriodicLooper.h"
#include "frc/smartdashboard/SmartDashboard.h"

GameDataHelper::GameDataHelper()
{
    PeriodicLooper::GetInstance()->RegisterAll(this);
    frc::SmartDashboard::PutBoolean(m_hubActiveNT, false);
    frc::SmartDashboard::PutNumber(m_allianceShiftTime, 25.0);
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

void GameDataHelper::PublishShiftChangeIn3seconds(bool value)
{
    if (m_shiftChangeIn3seconds != value)
    {
        m_shiftChangeIn3seconds = value;
        RobotState::GetInstance()->PublishStateChange(RobotStateChanges::StateChange::ShiftChangeIn3Seconds_Bool, value);
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
        PublishShiftChangeIn3seconds(false);
        PublishShiftChangeIn5seconds(false);
    }
    else
    {
        units::time::second_t matchTime = frc::DriverStation::GetMatchTime();
        std::string gameData = frc::DriverStation::GetGameSpecificMessage();
        bool redInactiveFirst = (gameData == "R");

        // Match Time Decreases
        int currentShift = 0;
        if (matchTime > m_shift1Start)
            currentShift = 0; // Transition period
        else if (matchTime > m_shift2Start)
            currentShift = 1;
        else if (matchTime > m_shift3Start)
            currentShift = 2;
        else if (matchTime > m_shift4Start)
            currentShift = 3;
        else if (matchTime > m_endgameStart)
            currentShift = 4;
        else
            currentShift = 5; // Endgame

        // Logic for Hub Activity (Alternates every shift)
        // Shift 1, 3: Red inactive if redInactiveFirst is true
        // Shift 2, 4: Red active if redInactiveFirst is true
        bool isOddShift = (currentShift % 2 != 0);

        if (currentShift >= 1 && currentShift <= 4)
        {
            // If it's an odd shift (1, 3) and red is supposed to be inactive first...
            bool shouldBeInactive = (isOddShift == redInactiveFirst);

            if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
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
        if (matchTime > m_shift1Start)
            timeToNextShift = matchTime - m_shift1Start;
        else if (matchTime > m_shift2Start)
            timeToNextShift = matchTime - m_shift2Start;
        else if (matchTime > m_shift3Start)
            timeToNextShift = matchTime - m_shift3Start;
        else if (matchTime > m_shift4Start)
            timeToNextShift = matchTime - m_shift4Start;
        else if (matchTime > m_endgameStart)
            timeToNextShift = matchTime - m_endgameStart;

        frc::SmartDashboard::PutNumber(m_allianceShiftTime, timeToNextShift.value());
        PublishShiftChangeIn5seconds(timeToNextShift <= 5.0_s && timeToNextShift > 0_s);
        PublishShiftChangeIn3seconds(timeToNextShift <= 3.0_s && timeToNextShift > 0_s);
    }
}
