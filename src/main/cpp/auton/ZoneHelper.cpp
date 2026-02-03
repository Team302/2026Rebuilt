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

#include <auton/ZoneHelper.h>
#include <auton/ZoneParams.h>
#include <auton/ZoneParser.h>

using frc::DriverStation;

void ZoneHelper::InitZones()
{
    m_zones = ZoneParser::ParseXML(GetZoneFile());
}

bool ZoneHelper::isInZone()
{
    return m_zones->IsPoseInZone(m_robotPose);
}

// This function is mainly if you want to check a different zone file than the ones in the managers
bool ZoneHelper::isInZone(std::string zoneFile)
{
    ZoneParams *zoneParams = ZoneParser::ParseXML(zoneFile);
    return zoneParams->IsPoseInZone(m_robotPose);
}