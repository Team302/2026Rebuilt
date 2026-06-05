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
    /// @class IntakeExpelCommand
    /// @brief Runs the intake in reverse to expel game pieces. This is the command-based replacement
    ///        for the old @c ExpelState. The lifecycle methods map onto the old state methods:
    ///        - @c Initialize() == the old @c Init()
    ///        - @c Execute()    == the old @c Run()
    ///        - @c End()        == the old @c Exit()
    ///        - @c IsFinished() == the old @c AtTarget() / @c IsTransitionCondition()
    class IntakeExpelCommand : public frc2::CommandHelper<frc2::Command, IntakeExpelCommand>
    {
    public:
        IntakeExpelCommand() = delete;
        explicit IntakeExpelCommand(Intake *intake);

        /// @brief Set the reverse intake/extender targets and publish not-intaking (was Init()).
        void Initialize() override;

        /// @brief Nothing to do per loop - targets hold (was Run()).
        void Execute() override;

        /// @brief Nothing to clean up (was Exit()).
        void End(bool interrupted) override;

        /// @brief Holds until interrupted (was AtTarget()/IsTransitionCondition()).
        bool IsFinished() override;

    private:
        Intake *m_intake;
    };
}
