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

// C++ Includes
#include <memory>
#include <string>
#include <utility>

// FRC includes
#include "frc2/command/button/CommandXboxController.h"
#include <frc/DriverStation.h>

// Team 302 includes
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlAxis.h"
#include "teleopcontrol/TeleopControlButton.h"
#include "utils/logging/debug/Logger.h"
#include <teleopcontrol/TeleopControlFunctions.h>
#include <teleopcontrol/TeleopControlMap.h>

using frc::DriverStation;
using std::make_pair;
using std::pair;
using std::string;
using std::vector;

//----------------------------------------------------------------------------------
// Method:      GetInstance
// Description: If there isn't an instance of this class, it will create one.  The
//              single class instance will be returned.
// Returns:     OperatorInterface*  instance of this class
//----------------------------------------------------------------------------------
TeleopControl *TeleopControl::m_instance = nullptr; // initialize the instance variable to nullptr
TeleopControl *TeleopControl::GetInstance()
{
	if (TeleopControl::m_instance == nullptr)
	{
		TeleopControl::m_instance = new TeleopControl();
	}
	if (TeleopControl::m_instance != nullptr && !TeleopControl::m_instance->IsInitialized())
	{
		TeleopControl::m_instance->Initialize();
	}
	return TeleopControl::m_instance;
}
//----------------------------------------------------------------------------------
// Method:      OperatorInterface <<constructor>>
// Description: This will construct and initialize the object.
//              It maps the functions to the buttons/axis.
//---------------------------------------------------------------------------------
TeleopControl::TeleopControl() : m_numControllers(0)

{
	Initialize();
}

bool TeleopControl::IsInitialized() const
{
	return m_numControllers > 0;
}
void TeleopControl::Initialize()
{
	InitializeControllers();
}

void TeleopControl::InitializeControllers()
{
	for (int inx = 0; inx < DriverStation::kJoystickPorts; ++inx)
	{
		InitializeController(inx);
	}
}

void TeleopControl::InitializeController(int port)
{
	if (DriverStation::GetJoystickIsXbox(port))
	{

		m_controller.emplace_back(frc2::CommandXboxController(port));
	}
}

vector<TeleopControlFunctions::FUNCTION> TeleopControl::GetAxisFunctionsOnController(int controller)
{
	vector<TeleopControlFunctions::FUNCTION> functions;
	functions.reserve(teleopControlMapAxisMap.size());

	for (const auto &[function, axisInfo] : teleopControlMapAxisMap)
	{
		if (axisInfo.controllerNumber == controller)
		{
			functions.emplace_back(function);
		}
	}
	return functions;
}

vector<TeleopControlFunctions::FUNCTION> TeleopControl::GetButtonFunctionsOnController(int controller)
{
	vector<TeleopControlFunctions::FUNCTION> functions;
	functions.reserve(teleopControlMapButtonMap.size());

	for (const auto &[function, buttonInfo] : teleopControlMapButtonMap)
	{
		if (buttonInfo.controllerNumber == controller)
		{
			functions.emplace_back(function);
		}
	}
	return functions;
}

pair<frc2::CommandXboxController *, TeleopControlMappingEnums::AXIS_IDENTIFIER> TeleopControl::GetAxisInfo(
	TeleopControlFunctions::FUNCTION function // <I> - controller with this function
)
{
	frc2::CommandXboxController *controller = nullptr;
	TeleopControlMappingEnums::AXIS_IDENTIFIER axis = TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS;

	if (!IsInitialized())
	{
		Initialize();
	}

	auto itr = teleopControlMapAxisMap.find(function);
	if (itr != teleopControlMapAxisMap.end())
	{
		const auto &axisInfo = itr->second;
		if (m_controller[axisInfo.controllerNumber] != nullptr)
		{
			controller = m_controller[axisInfo.controllerNumber];
			axis = axisInfo.axisId;
		}
	}
	return make_pair(controller, axis);
}

pair<frc2::CommandXboxController *, TeleopControlMappingEnums::BUTTON_IDENTIFIER> TeleopControl::GetButtonInfo(
	TeleopControlFunctions::FUNCTION function // <I> - controller with this function
)
{
	frc2::CommandXboxController *controller = nullptr;
	TeleopControlMappingEnums::BUTTON_IDENTIFIER btn = TeleopControlMappingEnums::UNDEFINED_BUTTON;

	if (!IsInitialized())
	{
		Initialize();
	}

	auto itr = teleopControlMapButtonMap.find(function);
	if (itr != teleopControlMapButtonMap.end())
	{
		const auto &buttonInfo = itr->second;
		if (m_controller[buttonInfo.controllerNumber] != nullptr)
		{
			controller = m_controller[buttonInfo.controllerNumber];
			btn = buttonInfo.buttonId;
		}
	}
	return make_pair(controller, btn);
}
/* System Core To Do: Look into how we want to do these with systemcore rework

//------------------------------------------------------------------
// Method:      SetAxisScaleFactor
// Description: Allow the range of values to be set smaller than
//              -1.0 to 1.0.  By providing a scale factor between 0.0
//              and 1.0, the range can be made smaller.  If a value
//              outside the range is provided, then the value will
//              be set to the closest bounding value (e.g. 1.5 will
//              become 1.0)
// Returns:     void
//------------------------------------------------------------------
void TeleopControl::SetAxisScaleFactor(
	TeleopControlFunctions::FUNCTION function, // <I> - function that will update an axis
	double scaleFactor						   // <I> - scale factor used to limit the range
)
{
	auto info = GetAxisInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS)
	{
		info.first->
	}
}

void TeleopControl::SetDeadBand(
	TeleopControlFunctions::FUNCTION function,
	TeleopControlMappingEnums::AXIS_DEADBAND deadband)
{
	auto info = GetAxisInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS)
	{
		info.first-> (info.second, deadband);
	}
}

//------------------------------------------------------------------
// Method:      SetAxisProfile
// Description: Sets the axis profile for the specifed axis
// Returns:     void
//------------------------------------------------------------------
void TeleopControl::SetAxisProfile(
	TeleopControlFunctions::FUNCTION function,		// <I> - function that will update an axis
	TeleopControlMappingEnums::AXIS_PROFILE profile // <I> - profile to use
)
{
	auto info = GetAxisInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS)
	{
		info.first->SetAxisProfile(info.second, profile);
	}
}
*/

//------------------------------------------------------------------
// Method:      GetAxisValue
// Description: Reads the joystick axis, removes any deadband (small
//              value) and then scales as requested.
// Returns:     double   -  scaled axis value
//------------------------------------------------------------------
double TeleopControl::GetAxisValue(TeleopControlFunctions::FUNCTION function) // <I> - function that whose axis will be read
{
	double value = 0.0;
	auto info = GetAxisInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS)
	{
		if (info.second == TeleopControlMappingEnums::AXIS_IDENTIFIER::LEFT_JOYSTICK_X)
			value = info.first->GetLeftX();
		else if (info.second == TeleopControlMappingEnums::AXIS_IDENTIFIER::LEFT_JOYSTICK_Y)
			value = info.first->GetLeftY();
		else if (info.second == TeleopControlMappingEnums::AXIS_IDENTIFIER::RIGHT_JOYSTICK_X)
			value = info.first->GetRightX();
		else if (info.second == TeleopControlMappingEnums::AXIS_IDENTIFIER::RIGHT_JOYSTICK_Y)
			value = info.first->GetRightY();
		else if (info.second == TeleopControlMappingEnums::AXIS_IDENTIFIER::LEFT_TRIGGER)
			value = info.first->GetLeftTriggerAxis();
		else if (info.second == TeleopControlMappingEnums::AXIS_IDENTIFIER::RIGHT_TRIGGER)
			value = info.first->GetRightTriggerAxis();
	}
	return value;
}

void TeleopControl::SetRumble(
	TeleopControlFunctions::FUNCTION function, // <I> - controller with this function
	bool leftRumble,						   // <I> - rumble left
	bool rightRumble						   // <I> - rumble right
)
{

	auto info = GetButtonInfo(function);
	if (info.first != nullptr)
	{
		if (leftRumble && rightRumble)
		{
			info.first->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1);
		}
		else if (leftRumble)
		{
			info.first->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1);
			info.first->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0);
		}
		else if (rightRumble)
		{
			info.first->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1);
			info.first->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0);
		}
		else
		{
			info.first->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
		}
	}
	else
	{
		auto info2 = GetAxisInfo(function);
		if (info2.first != nullptr)
		{
			if (leftRumble && rightRumble)
			{
				info2.first->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1);
			}
			else if (leftRumble)
			{
				info2.first->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1);
				info2.first->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0);
			}
			else if (rightRumble)
			{
				info2.first->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1);
				info2.first->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0);
			}
			else
			{
				info2.first->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
			}
		}
	}
}

void TeleopControl::SetRumble(
	int controller,	 // <I> - controller to rumble
	bool leftRumble, // <I> - rumble left
	bool rightRumble // <I> - rumble right
)
{
	if (m_controller[controller] != nullptr)
	{
		if (leftRumble && rightRumble)
		{
			m_controller[controller]->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1);
		}
		else if (leftRumble)
		{
			m_controller[controller]->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1);
			m_controller[controller]->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0);
		}
		else if (rightRumble)
		{
			m_controller[controller]->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1);
			m_controller[controller]->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0);
		}
		else
		{
			m_controller[controller]->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
		}
	}
}

void TeleopControl::LogInformation()
{
	for (int inx = 0; inx < DriverStation::kJoystickPorts; ++inx)
	{
		if (m_controller[inx] != nullptr)
		{
			auto functions = GetAxisFunctionsOnController(inx);
			for (auto function : functions)
			{
				Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("TeleopControl-axis"), std::to_string(function), GetAxisValue(function));
			}

			functions.clear();
			functions = GetButtonFunctionsOnController(inx);
			for (auto function : functions)
			{
				Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("TeleopControl-button"), std::to_string(function), GetCommandTrigger(function).Get());
			}
		}
	}
}

frc2::Trigger TeleopControl::GetCommandTrigger(TeleopControlFunctions::FUNCTION function)
{
	// Find the button mapping for the given function
	auto itr = teleopControlMapButtonMap.find(function);
	if (itr == teleopControlMapButtonMap.end())
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("TeleopControl-Command"), std::to_string(function), "Function not found in button map.");
		return frc2::Trigger([]()
							 { return false; });
	}
	bool isSelected = false;
	auto info = GetButtonInfo(function);
	auto controller = info.first;
	auto buttonInfo = itr->second;
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::UNDEFINED_BUTTON)
	{

		// Map the button identifier to the corresponding CommandXboxController method
		switch (buttonInfo.buttonId)
		{
		case TeleopControlMappingEnums::A_BUTTON:
			return controller->A();
		case TeleopControlMappingEnums::B_BUTTON:
			return controller->B();
		case TeleopControlMappingEnums::X_BUTTON:
			return controller->X();
		case TeleopControlMappingEnums::Y_BUTTON:
			return controller->Y();
		case TeleopControlMappingEnums::LEFT_BUMPER:
			return controller->LeftBumper();
		case TeleopControlMappingEnums::RIGHT_BUMPER:
			return controller->RightBumper();
		case TeleopControlMappingEnums::SELECT_BUTTON:
			return controller->Back(); // 'Select' is usually 'Back' in FRC
		case TeleopControlMappingEnums::START_BUTTON:
			return controller->Start();
		case TeleopControlMappingEnums::LEFT_STICK_PRESSED:
			return controller->LeftStick();
		case TeleopControlMappingEnums::RIGHT_STICK_PRESSED:
			return controller->RightStick();
		case TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED:
			return controller->LeftTrigger();
		case TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED:
			return controller->RightTrigger();
		case TeleopControlMappingEnums::POV_0:
			return controller->POVUp();
		case TeleopControlMappingEnums::POV_90:
			return controller->POVRight();
		case TeleopControlMappingEnums::POV_180:
			return controller->POVDown();
		case TeleopControlMappingEnums::POV_270:
			return controller->POVLeft();

		default:
			Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("TeleopControl-Command"), std::to_string(function), "Couldn't map the TeleopControlMapEnum");
		}
	}
	else
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("TeleopControl-Command"), std::to_string(function), "Controller is null.");
	}
	return frc2::Trigger([]()
						 { return false; }); // Return a trigger that is always inactive if the controller is null or the function is not mapped
}

frc2::Trigger TeleopControl::GetAxisAsTrigger(TeleopControlFunctions::FUNCTION function, double threshold)
{
	return frc2::Trigger([this, function, threshold]
						 { return this->GetAxisValue(function) > threshold; });
}
