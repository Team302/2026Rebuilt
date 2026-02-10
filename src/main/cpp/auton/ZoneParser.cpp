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

#include <map>
#include <string>

#include "auton/AutonGrid.h"
#include "auton/PrimitiveEnums.h"
#include "auton/ZoneParams.h"
#include "auton/ZoneParser.h"
#include "frc/Filesystem.h"
#include "pugixml/pugixml.hpp"
#include "RobinHood/robin_hood.h"
#include "utils/logging/debug/Logger.h"

#include "mechanisms/Launcher/Launcher.h"
#include "mechanisms/Intake/Intake.h"
#include "mechanisms/Climber/Climber.h"

using namespace std;
using namespace pugi;

ZoneParams *ZoneParser::ParseXML(string fulldirfile)
{
    auto hasError = false;

    static robin_hood::unordered_map<std::string, ChassisOptionEnums::AutonChassisOptions> xmlStringToChassisOptionEnumMap{
        {"NO_VISION", ChassisOptionEnums::AutonChassisOptions::NO_VISION}};

    static robin_hood::unordered_map<std::string, ChassisOptionEnums::HeadingOption> xmlStringToHeadingOptionEnumMap{

        {"MAINTAIN", ChassisOptionEnums::HeadingOption::MAINTAIN},
        {"SPECIFIED_ANGLE", ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE},
        {"FACE_GAME_PIECE", ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE},
        {"IGNORE", ChassisOptionEnums::HeadingOption::IGNORE}};

    static robin_hood::unordered_map<string, ChassisOptionEnums::DriveStateType> xmlStringToPathUpdateOptionMap{{"NOTHING", ChassisOptionEnums::STOP_DRIVE}};

    static robin_hood::unordered_map<std::string, ZoneAllianceColor> xmlStringToAllianceColorMap{
        {"RED", ZoneAllianceColor::RED},
        {"BLUE", ZoneAllianceColor::BLUE},
        {"BOTH", ZoneAllianceColor::BOTH},

    };
    static robin_hood::unordered_map<std::string, ChassisOptionEnums::AutonAvoidOptions> xmlStringToAvoidOptionEnumMap{
        {"ROBOT_COLLISION", ChassisOptionEnums::AutonAvoidOptions::ROBOT_COLLISION},
        {"NO_AVOID_OPTION", ChassisOptionEnums::AutonAvoidOptions::NO_AVOID_OPTION},

    };

    auto deployDir = frc::filesystem::GetDeployDirectory();
    auto zonedir = deployDir + "/auton/zones/";

    string updfulldirfile = zonedir;
    updfulldirfile += fulldirfile;

    xml_document doc;
    xml_parse_result result = doc.load_file(updfulldirfile.c_str());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PrimitiveParser", "updated File", updfulldirfile.c_str());
    if (result)
    {
        xml_node auton = doc.root();
        for (xml_node zonenode = auton.first_child().first_child(); zonenode; zonenode = zonenode.next_sibling())
        {

            double radius = -1;
            double circleX = -1;
            double circleY = -1;

            units::length::meter_t xgrid1rect = units::length::meter_t(0.0);
            units::length::meter_t ygrid1rect = units::length::meter_t(0.0);
            units::length::meter_t xgrid2rect = units::length::meter_t(0.0);
            units::length::meter_t ygrid2rect = units::length::meter_t(0.0);

            ZoneMode zoneMode = ZoneMode::NOTHING;

            // TODO: add zoneType parsing and check
            bool isLauncherStateChanged = false;
            bool isIntakeStateChanged = false;
            bool isClimberStateChanged = false;
            ChassisOptionEnums::AutonChassisOptions chassisChosenOption = ChassisOptionEnums::AutonChassisOptions::NO_VISION;
            ChassisOptionEnums::HeadingOption chosenHeadingOption = ChassisOptionEnums::HeadingOption::IGNORE;

            ChassisOptionEnums::DriveStateType chosenUpdateOption = ChassisOptionEnums::STOP_DRIVE;
            ZoneAllianceColor chosenAllianceColor = ZoneAllianceColor::BOTH;
            ChassisOptionEnums::AutonAvoidOptions avoidChosenOption = ChassisOptionEnums::AutonAvoidOptions::NO_AVOID_OPTION;

            Launcher::STATE_NAMES launcherState = Launcher::STATE_NAMES::STATE_OFF;
            Intake::STATE_NAMES intakeState = Intake::STATE_NAMES::STATE_OFF;
            Climber::STATE_NAMES climberState = Climber::STATE_NAMES::STATE_OFF;

            auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();

            // looping through the zone xml attributes to define the location of a given zone (based on 2 sets grid coordinates)
            for (xml_attribute attr = zonenode.first_attribute(); attr; attr = attr.next_attribute())
            {

                if (strcmp(attr.name(), "circlex") == 0)
                {
                    zoneMode = ZoneMode::CIRCLE;
                    circleX = attr.as_double();
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ZoneParser", "parsed circlex", circleX);
                }
                else if (strcmp(attr.name(), "circley") == 0)
                {

                    zoneMode = ZoneMode::CIRCLE;
                    circleY = attr.as_double();
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ZoneParser", "parsed circley", circleY);
                }
                else if (strcmp(attr.name(), "radius") == 0)
                {
                    zoneMode = ZoneMode::CIRCLE;
                    radius = attr.as_double();
                }
                else if (strcmp(attr.name(), "x1_rect") == 0)
                {
                    zoneMode = ZoneMode::RECTANGLE;
                    xgrid1rect = units::length::meter_t(attr.as_double());
                }
                else if (strcmp(attr.name(), "y1_rect") == 0)
                {
                    zoneMode = ZoneMode::RECTANGLE;
                    ygrid1rect = units::length::meter_t(attr.as_double());
                }
                else if (strcmp(attr.name(), "x2_rect") == 0)
                {
                    zoneMode = ZoneMode::RECTANGLE;
                    xgrid2rect = units::length::meter_t(attr.as_double());
                }
                else if (strcmp(attr.name(), "y2_rect") == 0)
                {
                    zoneMode = ZoneMode::RECTANGLE;
                    ygrid2rect = units::length::meter_t(attr.as_double());
                }

                else if (strcmp(attr.name(), "chassisOption") == 0)
                {
                    auto itr = xmlStringToChassisOptionEnumMap.find(attr.value());
                    if (itr != xmlStringToChassisOptionEnumMap.end())
                    {
                        chassisChosenOption = itr->second;
                    }
                    else
                    {
                        hasError = true;
                    }
                }

                else if (strcmp(attr.name(), "headingOption") == 0)
                {
                    auto itr = xmlStringToHeadingOptionEnumMap.find(attr.value());
                    if (itr != xmlStringToHeadingOptionEnumMap.end())
                    {
                        chosenHeadingOption = itr->second;
                    }
                    else
                    {
                        hasError = true;
                    }
                }
                else if (strcmp(attr.name(), "pathUpdateOption") == 0)
                {
                    auto itr = xmlStringToPathUpdateOptionMap.find(attr.value());
                    if (itr != xmlStringToPathUpdateOptionMap.end())
                    {
                        chosenUpdateOption = itr->second;
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ZoneParser", "Update Option Parsed", chosenUpdateOption);
                    }
                    else
                    {
                        hasError = true;
                    }
                }
                else if (strcmp(attr.name(), "allianceColor") == 0)
                {
                    auto itr = xmlStringToAllianceColorMap.find(attr.value());
                    if (itr != xmlStringToAllianceColorMap.end())
                    {
                        chosenAllianceColor = itr->second;
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ZoneParser", "Alliance Color Parsed", int(chosenAllianceColor));
                    }
                    else
                    {
                        hasError = true;
                    }
                }
                else if (strcmp(attr.name(), "avoidOption") == 0)
                {
                    auto itr = xmlStringToAvoidOptionEnumMap.find(attr.value());
                    if (itr != xmlStringToAvoidOptionEnumMap.end())
                    {
                        avoidChosenOption = itr->second;
                    }
                    else
                    {
                        hasError = true;
                    }
                }
                else if (strcmp(attr.name(), "launcherState") == 0)
                {
                    if (config != nullptr && config->GetMechanism(MechanismTypes::MECHANISM_TYPE::LAUNCHER) != nullptr)
                    {
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("PrimitiveParser"), string("Found launcher mechanism"), string(attr.value()));

                        auto launcherStateItr = Launcher::stringToSTATE_NAMESEnumMap.find(attr.value());
                        if (launcherStateItr != Launcher::stringToSTATE_NAMESEnumMap.end())
                        {
                            launcherState = launcherStateItr->second;
                            isLauncherStateChanged = true;
                        }
                        else
                        {
                            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML invalid launcher state"), attr.value());
                            hasError = true;
                        }
                    }
                }
                else if (strcmp(attr.name(), "intakeState") == 0)
                {
                    if (config != nullptr && config->GetMechanism(MechanismTypes::MECHANISM_TYPE::INTAKE) != nullptr)
                    {
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("PrimitiveParser"), string("Found intake mechanism"), string(attr.value()));

                        auto intakeStateItr = Intake::stringToSTATE_NAMESEnumMap.find(attr.value());
                        if (intakeStateItr != Intake::stringToSTATE_NAMESEnumMap.end())
                        {
                            intakeState = intakeStateItr->second;
                            isIntakeStateChanged = true;
                        }
                        else
                        {
                            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML invalid intake state"), attr.value());
                            hasError = true;
                        }
                    }
                }
                else if (strcmp(attr.name(), "climberState") == 0)
                {
                    if (config != nullptr && config->GetMechanism(MechanismTypes::MECHANISM_TYPE::CLIMBER) != nullptr)
                    {
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("PrimitiveParser"), string("Found climber mechanism"), string(attr.value()));

                        auto climberStateItr = Climber::stringToSTATE_NAMESEnumMap.find(attr.value());
                        if (climberStateItr != Climber::stringToSTATE_NAMESEnumMap.end())
                        {
                            climberState = climberStateItr->second;
                            isClimberStateChanged = true;
                        }
                        else
                        {
                            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML invalid climber state"), attr.value());
                            hasError = true;
                        }
                    }
                }
            }

            if (!hasError) // if no error returns the zone parameters
            {

                auto circlePose2d = frc::Pose2d(units::length::meter_t(circleX), units::length::meter_t(circleY), units::degree_t(0));
                return (new ZoneParams(circlePose2d,
                                       units::inch_t(radius),
                                       xgrid1rect,
                                       xgrid2rect,
                                       ygrid1rect,
                                       ygrid2rect,
                                       chassisChosenOption,
                                       chosenHeadingOption,
                                       chosenUpdateOption,
                                       avoidChosenOption,
                                       chosenAllianceColor,
                                       zoneMode,
                                       isLauncherStateChanged,
                                       isIntakeStateChanged,
                                       isClimberStateChanged,
                                       launcherState,
                                       intakeState,
                                       climberState));
            }

            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("ZoneParser"), string("ParseXML"), string("Has Error"));
        }
    }
    return nullptr; // if error, return nullptr
}
