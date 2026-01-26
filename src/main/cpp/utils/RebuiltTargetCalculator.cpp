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

#include "utils/RebuiltTargetCalculator.h"

RebuiltTargetCalculator::RebuiltTargetCalculator()
{
    // TODO: update launcher offsets and target position(need to use zone logic later and field element calculator to get target position)
    // Mechanism offset initialized in member variable declaration for now

    TargetCalculator();
    SetMechanismOffset(m_mechanismOffset);
}

RebuiltTargetCalculator *RebuiltTargetCalculator::m_instance = nullptr;

RebuiltTargetCalculator *RebuiltTargetCalculator::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new RebuiltTargetCalculator();
    }
    return m_instance;
}

frc::Translation2d RebuiltTargetCalculator::GetTargetPosition()
{
    // TODO: Replace with dynamic target selection based on:
    // - FieldElementCalculator for actual target position
    // - Zone detector for zone-specific targets
    // For now, return hardcoded hub target for testing
    return m_hubTarget;
}
