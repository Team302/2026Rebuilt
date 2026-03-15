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

#include <auton/ZoneHelper.h>

//====================================================================================================================================================
/// @class AllianceZoneManager
/// @brief Singleton manager that determines whether the robot is inside its own or the opposing alliance zone
///
/// This class extends ZoneHelper to provide alliance-zone awareness by loading the Red and Blue alliance
/// zone definitions from XML files and exposing two zone-check predicates:
/// - IsInAllianceZone():      true when the robot is in its own alliance zone
/// - IsInOtherAllianceZone(): true when the robot is in the opposing alliance zone
///
/// Alliance color is queried live from FMSData on every call so the correct zone is automatically
/// selected regardless of which alliance the robot is assigned to.
///
/// @note This is a singleton class — use GetInstance() to access.
/// @see ZoneHelper Base class that parses zone files and performs zone containment checks
/// @see NeutralZoneManager For neutral-zone detection
//====================================================================================================================================================
class AllianceZoneManager : public ZoneHelper
{
public:
    //------------------------------------------------------------------
    /// @brief      Get the singleton instance of AllianceZoneManager
    /// @return     AllianceZoneManager* - Pointer to the singleton instance
    /// @details    Creates the instance on first call (lazy initialization).
    ///             Subsequent calls return the same instance.
    //------------------------------------------------------------------
    static AllianceZoneManager *GetInstance();

    //------------------------------------------------------------------
    /// @brief      Check whether the robot is currently inside its own alliance zone
    /// @return     bool - true if the robot is in the zone corresponding to the FMS-assigned alliance color
    /// @details    Selects the red or blue zone list based on FMSData::GetAllianceColor() and
    ///             delegates containment evaluation to ZoneHelper::IsInZones().
    //------------------------------------------------------------------
    bool IsInAllianceZone();

    //------------------------------------------------------------------
    /// @brief      Check whether the robot is currently inside the opposing alliance's zone
    /// @return     bool - true if the robot is in the zone belonging to the other alliance
    /// @details    Mirrors IsInAllianceZone() but uses the opposing alliance's zone list.
    ///             Useful for detecting when the robot has crossed into opponent territory.
    //------------------------------------------------------------------
    bool IsInOtherAllianceZone();

private:
    //------------------------------------------------------------------
    /// @brief      Private constructor for singleton pattern
    /// @details    Parses the alliance zone XML files and caches the resulting
    ///             ZoneParams vectors for both the blue and red alliance zones.
    ///             Called only by GetInstance() on first access.
    //------------------------------------------------------------------
    AllianceZoneManager();

    //------------------------------------------------------------------
    /// @brief      Destructor (default implementation)
    /// @details    No special cleanup required as zone objects are managed by ZoneHelper
    //------------------------------------------------------------------
    ~AllianceZoneManager() = default;

    //------------------------------------------------------------------
    /// @brief      Returns the list of alliance zone XML filenames to parse
    /// @return     std::vector<std::string> - Filenames for both alliance zone definitions
    /// @details    Overrides ZoneHelper::GetZoneFiles() to supply the alliance-specific files.
    //------------------------------------------------------------------
    std::vector<std::string> GetZoneFiles() override;

    /// @brief Singleton instance pointer (lazy initialization)
    static AllianceZoneManager *m_instance;

    /// @brief XML filenames for the alliance zone definitions (red and blue)
    std::vector<std::string> m_allianceZoneFiles = {
        "RedAllianceZone.xml",
        "BlueAllianceZone.xml"};

    /// @brief Cached list of zone parameters for the blue alliance zone
    std::vector<ZoneParams *> m_blueAllianceZone;

    /// @brief Cached list of zone parameters for the red alliance zone
    std::vector<ZoneParams *> m_redAllianceZone;
};