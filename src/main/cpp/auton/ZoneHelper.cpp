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
#include "utils/logging/debug/Logger.h"

using frc::DriverStation;

ZoneHelper::ZoneHelper(std::vector<std::string> zoneFiles) : m_zoneFiles(zoneFiles)
{
    m_chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();
    m_zones = new std::vector<ZoneParams>();
    ParseZoneFiles();
}

void ZoneHelper::ParseZoneFiles()
{
    for (auto file : m_zoneFiles)
    {
        m_zones->emplace_back(*ZoneParser::ParseXML(file));
    }
}

std::vector<ZoneParams *> ZoneHelper::GetAllianceZones(ZoneParams::AllianceColor alliance)
{
    std::vector<ZoneParams *> allianceZones;
    for (auto &zone : *m_zones)
    {
        if (zone.GetAllianceColor() == alliance)
        {
            allianceZones.push_back(&zone);
        }
    }
    return allianceZones;
}

bool ZoneHelper::IsInZones(std::vector<ZoneParams *> zoneFiles)
{
    // std::find_if could be used here
    for (auto zone : zoneFiles)
    {
        if (IsInZone(zone))
        {
            return true;
        }
    }
    return false;
}

bool ZoneHelper::IsInZone(ZoneParams *zoneFile)
{
    return zoneFile->IsPoseInZone(m_chassis->GetPose());
}

ZoneHelper::~ZoneHelper()
{
    delete m_zones;
}