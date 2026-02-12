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

GameDataHelper::GameDataHelper()
{
    m_timer.Start();
}
GameDataHelper *GameDataHelper::m_instance = nullptr;

GameDataHelper *GameDataHelper::GetInstance()
{
    if (GameDataHelper::m_instance == nullptr)
    {
        GameDataHelper::m_instance = new GameDataHelper();
    }
    return GameDataHelper::m_instance;
}

void GameDataHelper::UpdateGameSpecificMessage()
{
    auto test = frc::DriverStation::GetGameSpecificMessage();
    m_gameSpecificMessage = test[0];
}

void GameDataHelper::PublishShiftChange(bool value)
{
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::StateChange::ShiftChange_Bool, value);
}

void GameDataHelper::PublishShiftChangeIn5seconds(bool value)
{
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::StateChange::ShiftChangeIn5Seconds_Bool, value);
}

void GameDataHelper::PublishShiftChangeIn3seconds(bool value)
{
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::StateChange::ShiftChangeIn3Seconds_Bool, value);
}

void GameDataHelper::PublisherCyclcic()
{
    if (frc::DriverStation::GetGameSpecificMessage().length() == 0)
    {
        return;
    }
    else
    {
        if (m_gameSpecificMessage != frc::DriverStation::GetGameSpecificMessage()[0])
        {
            PublishShiftChange(true);
            m_timer.Reset();
        }
        else
        {
            if (m_timer.Get().value() == 20.0) // 20 seconds
            {
                PublishShiftChangeIn5seconds(true);
            }
            else if (m_timer.Get().value() == 22.0) // 22 seconds
            {
                PublishShiftChangeIn3seconds(true);
            }
            else
            {
                PublishShiftChangeIn5seconds(false);
                PublishShiftChangeIn3seconds(false);
                PublishShiftChange(false);
            }
        }
    }
    UpdateGameSpecificMessage();
}
