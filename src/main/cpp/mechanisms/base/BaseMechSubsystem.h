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

// C++ Includes
#include <string>

// FRC Includes
#include <frc2/command/SubsystemBase.h>

// Team 302 Includes
#include "mechanisms/MechanismTypes.h"
#include "mechanisms/base/BaseMech.h"

/// @class BaseMechSubsystem
/// @brief Command-based base class for robot mechanisms.
///
/// This is the command-based replacement for the old @c StateMgr base class. A
/// mechanism derives from @c BaseMechSubsystem to gain:
///   - @c BaseMech behavior (mechanism type, network-table name, control-data reading)
///   - @c frc2::SubsystemBase behavior (automatic registration with the
///     @c CommandScheduler so @c Periodic() runs every loop, plus default-command
///     and requirement support for command bindings)
///
/// Mechanism behavior that used to live in individual @c State classes is now
/// expressed as @c frc2::CommandPtr factories on the concrete mechanism, and the
/// per-loop housekeeping that used to live in @c StateMgr::RunCommonTasks() now
/// lives in the overridden @c Periodic().
class BaseMechSubsystem : public BaseMech, public frc2::SubsystemBase
{
public:
    /// @brief Construct the mechanism subsystem.
    /// @param type the type of mechanism
    /// @param networkTableName the network table name used for logging
    BaseMechSubsystem(MechanismTypes::MECHANISM_TYPE type, std::string networkTableName);

    ~BaseMechSubsystem() override = default;

private:
    BaseMechSubsystem() = delete;
};
