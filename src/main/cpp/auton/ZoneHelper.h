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
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose2d.h>
#include <utils/FMSData.h>
#include <auton/AutonGrid.h>
#include <auton/ZoneParams.h>
#include <utils/TargetCalculator.h>
#include <chassis/generated/CommandSwerveDrivetrain.h>

// C++ Includes
#include <vector>

// FRC includes

// Team 302 includes

// Third Party Includes

class ZoneHelper
{
public:
    void InitZones();
    virtual std::string GetZoneFile() = 0;
    bool isInZone();
    bool isInZone(std::string zoneFile);

protected:
    frc::Pose2d m_robotPose = m_chassis->GetPose();
    subsystems::CommandSwerveDrivetrain *m_chassis;
    ZoneParams *m_zones;
};
