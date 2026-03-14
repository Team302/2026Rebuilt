
//====================================================================================================================================================
// Copyright 2025 Lake Orion Robotics FIRST Team 302
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

// C++ Includes
#include <vector>

// FRC includes
#include "frc/Timer.h"
#include "units/time.h"

// Team 302 includes
#include "auton/ZoneParams.h"
#include "chassis/ChassisOptionEnums.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "state/State.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"

// Third Party Includes

class AutonSelector;
class IPrimitive;
class PrimitiveFactory;
class PrimitiveParams;
class SwerveChassis;
class Launcher;
class Intake;
class Climber;

class CyclePrimitives : public State, DragonDataLogger
{
public:
    CyclePrimitives();
    virtual ~CyclePrimitives() = default;

    void Init() override;
    void Run() override;
    void Exit() override;
    bool AtTarget() override;

    AutonSelector *GetAutonSelector() const { return m_autonSelector; };

    void DataLog(uint64_t timestamp) override;

protected:
    void GetNextPrim();
    void RunDriveStop();

private:
    void SetMechanismStatesFromParam(PrimitiveParams *params);
    bool SetMechanismStatesFromZone(std::pair<ZoneParams *, bool> *params);
    void CacheMechanismPointers();

    std::vector<PrimitiveParams *> m_primParams;
    int m_currentPrimSlot;
    IPrimitive *m_currentPrim;
    PrimitiveFactory *m_primFactory;
    IPrimitive *m_driveStop;
    AutonSelector *m_autonSelector;
    std::unique_ptr<frc::Timer> m_timer;
    units::time::second_t m_maxTime;
    bool m_isDone;
    subsystems::CommandSwerveDrivetrain *m_chassis;
    ChassisOptionEnums::PathUpdateOption m_updatedHeadingOption;
    ZoneParamsVector m_zones;

    // Cached mechanism pointers to avoid repeated lookups
    Launcher *m_cachedLauncher;
    Intake *m_cachedIntake;
    Climber *m_cachedClimber;
};