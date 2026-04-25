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

#include "teleopcontrol/SweepLaneChanger.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "teleopcontrol/TeleopControl.h"
#include "utils/PeriodicLooper.h"
#include <algorithm>

SweepLaneChanger *SweepLaneChanger::m_instance = nullptr;
SweepLaneChanger *SweepLaneChanger::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new SweepLaneChanger();
    }
    return m_instance;
}

SweepLaneChanger::SweepLaneChanger()
{
    PeriodicLooper::GetInstance()->RegisterAll(this);
    frc::SmartDashboard::PutNumber(m_sweepLaneNT, m_lane);
}

void SweepLaneChanger::RunCurrentState()
{
    m_isIncrementPressed = TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::FUNCTION::SWEEP_BEHIND_HUB_INCREMENT);
    m_isDecrementPressed = TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::FUNCTION::SWEEP_BEHIND_HUB_DECREMENT);

    if (!m_isIncrementPressed)
    {
        m_incrementLatch = false;
    }

    if (!m_isDecrementPressed)
    {
        m_decrementLatch = false;
    }
    if (!m_isIncrementPressed && !m_isDecrementPressed)
    {
        return;
    }

    if (m_isIncrementPressed && !m_incrementLatch)
    {
        m_incrementLatch = true;
        if (m_lane < m_maxLanes)
        {
            SetLane(m_lane + 1);
        }
        return;
    }

    if (m_isDecrementPressed && !m_decrementLatch)
    {
        m_decrementLatch = true;
        if (m_lane > m_minLanes)
        {
            SetLane(m_lane - 1);
        }
    }
}

void SweepLaneChanger::SetLane(int lane)
{
    m_lane = std::clamp(lane, m_minLanes, m_maxLanes);
    frc::SmartDashboard::PutNumber(m_sweepLaneNT, m_lane);
}