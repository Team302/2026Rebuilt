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

// co-Author: notcharlie, creator of dumb code / copy paster of better code

// Includes
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <sys/stat.h>

#ifdef __linux
#include <dirent.h>
#endif

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Filesystem.h>
#include "networktables/NetworkTableInstance.h"

// Team302 includes
#include "auton/AutonSelector.h"
#include "utils/logging/debug/Logger.h"
#include "utils/FMSData.h"
#include "feedback/DriverFeedback.h"

#include <pugixml/pugixml.hpp>

using namespace std;
using frc::DriverStation;

//---------------------------------------------------------------------
// Method: 		<<constructor>>
// Description: This creates this object and reads the auto script (CSV)
//  			files and displays a list on the dashboard.
//---------------------------------------------------------------------
AutonSelector::AutonSelector()
{
	PutChoicesOnDashboard();
}

string AutonSelector::GetSelectedAutoFile()
{
	std::string autonfile(frc::filesystem::GetDeployDirectory());
	autonfile += std::filesystem::path("/auton/").string();
	autonfile += GetAlianceColor();
	autonfile += GetStartPos();
	autonfile += GetDesiredPreload();
	autonfile += GetNeutralZoneAmount();
	autonfile += GetDepotOption();
	autonfile += GetOutpostOption();
	autonfile += GetFuelStrategy();
	autonfile += GetClimbingOption();
	autonfile += std::string(".xml");

	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("auton"), string("file"), autonfile);

	auto table = nt::NetworkTableInstance::GetDefault().GetTable("auton file");

	table.get()->PutString("determined name", autonfile);

	bool fileExists = FileExists(autonfile);
	bool fileValid = FileValid(autonfile);

	DriverFeedback::GetInstance()->SetIsValidAutonFile(fileExists && fileValid);

	table.get()->PutBoolean("File Exists", fileExists);
	table.get()->PutBoolean("File Valid", fileValid);

	if (!fileExists || !fileValid)
	{
		autonfile = frc::filesystem::GetDeployDirectory();
		autonfile += std::filesystem::path("/auton/").string();
		autonfile += GetAlianceColor();
		autonfile += ("DefaultFile.xml");
	}

	table.get()->PutString("actual file", autonfile);

	return autonfile;
}

bool AutonSelector::FileExists(const std::string &name)
{
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

bool AutonSelector::FileValid(const std::string &name)
{

	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file(name.c_str());
	if (result)
	{
		return true;
	}
	Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("AutonSelector"), string("FileInvalid: Description ") + name, string(result.description()));
	Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("AutonSelector"), string("FileInvalid: Offset ") + name, static_cast<int>(result.offset));
	return false;
}

string AutonSelector::GetAlianceColor()
{
	return (FMSData::GetAllianceColor() == DriverStation::Alliance::kRed) ? std::string("Red") : std::string("Blue");
}

string AutonSelector::GetStartPos()
{
	return m_startposchooser.GetSelected();
}

string AutonSelector::GetNeutralZoneAmount()
{
	return m_neutralZoneAmount.GetSelected();
}

string AutonSelector::GetDepotOption()
{
	return m_targetDepot.GetSelected();
}

string AutonSelector::GetOutpostOption()
{
	return m_targetOutpost.GetSelected();
}
string AutonSelector::GetClimbingOption()
{
	return m_climbing.GetSelected();
}
string AutonSelector::GetDesiredPreload()
{
	return m_desiredPreload.GetSelected();
}
string AutonSelector::GetFuelStrategy()
{
	return m_fuelStrategy.GetSelected();
}

//---------------------------------------------------------------------
// Method: 		PutChoicesOnDashboard
// Description: This puts the list of files in the m_csvFiles attribute
//				up on the dashboard for selection.
// Returns:		void
//---------------------------------------------------------------------
void AutonSelector::PutChoicesOnDashboard()
{
	// Starting Position
	m_startposchooser.AddOption("Trench Depot Side", "L");
	m_startposchooser.AddOption("Bump Depot Side", "ML");
	m_startposchooser.AddOption("Hub", "M");
	m_startposchooser.AddOption("Bump Outpost Side", "MR");
	m_startposchooser.SetDefaultOption("Trench Outpost Side", "R");
	frc::SmartDashboard::PutData("StartPos", &m_startposchooser);

	// Amount of times going into NZ
	m_neutralZoneAmount.AddOption("0", "0");
	m_neutralZoneAmount.AddOption("1", "1");
	m_neutralZoneAmount.AddOption("2", "2");
	m_neutralZoneAmount.AddOption("3", "3");
	m_neutralZoneAmount.AddOption("4", "4");
	m_neutralZoneAmount.AddOption("5", "5");
	frc::SmartDashboard::PutData("Times in Neutral Zone", &m_neutralZoneAmount);

	// Depot Option
	m_targetDepot.AddOption("true", "Dep");
	m_targetDepot.AddOption("false", "ND");
	frc::SmartDashboard::PutData("Has Depot?", &m_targetDepot);

	// Outpost Option
	m_targetOutpost.AddOption("true", "Out");
	m_targetOutpost.AddOption("false", "NO");
	frc::SmartDashboard::PutData("Has Outpost?", &m_targetOutpost);

	// Climbing Option
	m_climbing.AddOption("true", "Climb");
	m_climbing.AddOption("false", "NC");
	frc::SmartDashboard::PutData("Climb?", &m_climbing);

	// Preload Option
	m_desiredPreload.AddOption("Launch", "Launch");
	m_desiredPreload.AddOption("Drop", "Drop");
	m_desiredPreload.SetDefaultOption("Launch", "Launch");
	frc::SmartDashboard::PutData("Desired Preload", &m_desiredPreload);

	// Fuel Strategy Option
	m_fuelStrategy.AddOption("Score", "Score");
	m_fuelStrategy.AddOption("Pass", "Pass");
	m_fuelStrategy.SetDefaultOption("Score", "Score");
	frc::SmartDashboard::PutData("Desired Strategy", &m_fuelStrategy);
}
