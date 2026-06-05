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

#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"

// Forward Declarations
class Intake;

namespace IntakeCommands
{
    /// @class IntakeOffCommand
    /// @brief Default Intake command - turns the intake/extender off and performs the one-time
    ///        extender reset on first enable. This is the command-based replacement for the old
    ///        @c OffState. The lifecycle methods map onto the old state methods as follows:
    ///        - @c Initialize() == the old @c Init()
    ///        - @c Execute()    == the old @c Run()
    ///        - @c End()        == the old @c Exit()
    ///        - @c IsFinished() == the old @c AtTarget() / @c IsTransitionCondition()
    class IntakeOffCommand : public frc2::CommandHelper<frc2::Command, IntakeOffCommand>
    {
    public:
        IntakeOffCommand() = delete;
        explicit IntakeOffCommand(Intake *intake);

        /// @brief Set the off targets and publish that the intake is no longer intaking (was Init()).
        void Initialize() override;

        /// @brief Zero/reset the extender the first time the robot is enabled (was Run()).
        void Execute() override;

        /// @brief Nothing to clean up (was Exit()).
        void End(bool interrupted) override;

        /// @brief Default command - never finishes on its own (was AtTarget()/IsTransitionCondition()).
        bool IsFinished() override;

    private:
        Intake *m_intake;
    };
}
