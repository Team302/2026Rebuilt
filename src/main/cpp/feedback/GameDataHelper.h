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
#include <frc/DriverStation.h>
#include "frc/Timer.h"
#include "state/StateMgr.h"

class GameDataHelper : StateMgr
{

public:
    static GameDataHelper *GetInstance();

    void PublishHubActive(bool value);
    void PublishShiftChangeIn5seconds(bool value);
    void PublishShiftChangeIn3seconds(bool value);
    void RunCurrentState() override;

private:
    GameDataHelper();
    ~GameDataHelper() = default;
    frc::DriverStation *m_driverStation;
    static GameDataHelper *m_instance;

    bool m_hubActive = false;
    bool m_shiftChangeIn3seconds = true;
    bool m_shiftChangeIn5seconds = true;

    const units::time::second_t m_shift1Start = 130_s; // 2:10
    const units::time::second_t m_shift2Start = 105_s; // 1:45
    const units::time::second_t m_shift3Start = 80_s;  // 1:20
    const units::time::second_t m_shift4Start = 55_s;  // 0:55
    const units::time::second_t m_endgameStart = 30_s; // 0:30
    units::time::second_t m_shiftLength = 25_s;
};