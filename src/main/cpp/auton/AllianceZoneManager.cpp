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
#include <auton/AllianceZoneManager.h>

using frc::DriverStation;

AllianceZoneManager *AllianceZoneManager::m_instance = nullptr;
AllianceZoneManager *AllianceZoneManager::GetInstance()
{
    if (AllianceZoneManager::m_instance == nullptr)
    {
        AllianceZoneManager::m_instance = new AllianceZoneManager();
    }
    return AllianceZoneManager::m_instance;
}



std::string AllianceZoneManager::GetZoneFiles()
{
        for(int i = 0; i <= m_zoneFiles.size(); i++){
        m_parsedZoneFiles.insert(ZoneParser::ParseXML(m_zoneFiles[i]));
    }
    return m_parsedZoneFiles;
}




bool AllianceZoneManager::IsInAllianceZone()
{
    if (DriverStation::GetAlliance() == DriverStation::Alliance::kRed){
        return IsInZone(m_parsedZoneFiles[0]);
    }   
    else{
        return IsInZone(m_parsedZoneFiles[1]);
    }
}
