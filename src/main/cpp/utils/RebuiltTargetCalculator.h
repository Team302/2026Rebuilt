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

#include "utils/TargetCalculator.h"
#include "utils/DragonField.h"

#include <frc/geometry/Translation2d.h>
#include <units/length.h>

/**
 * \class RebuiltTargetCalculator
 * \brief Season-specific target calculator for 2026 Rebuilt
 *
 * This class extends TargetCalculator with 2026-specific configuration:
 * - Hardcoded hub target (for testing, will integrate with field element calculator later)
 * - Mechanism offset configuration for the shooting mechanism
 *
 * TODO: Integrate with FieldElementCalculator and ZoneCalculator when available
 */
class RebuiltTargetCalculator : public TargetCalculator
{
public:
    /**
     * \brief Get singleton instance
     * \return Pointer to the RebuiltTargetCalculator singleton
     */
    static RebuiltTargetCalculator *GetInstance();

    /**
     * \brief Get the current target position
     * \return Translation2d with target position in meters (world frame)
     */
    frc::Translation2d GetTargetPosition() override;
    /**
     * \brief Get the launcher angle target to hit the current target within launcher limits
     * \return Angle in degrees
     */
    units::angle::degree_t GetLauncherTarget(units::time::second_t looheadTime, units::angle::degree_t currentLauncherAngle);

private:
    /**
     * \brief Constructor - initializes with default mechanism offset
     */
    RebuiltTargetCalculator();

    static RebuiltTargetCalculator *m_instance;

    // TODO: Replace with FieldElementCalculator and ZoneCalculator integration
    // Hardcoded hub target for testing (approximate field position in meters)
    frc::Translation2d m_hubTarget{4.625_m, 4.025_m};

    // Mechanism position offset from robot center in robot frame (meters)
    // Default: 5.5 inches (0.1397m) back, centered
    frc::Translation2d m_mechanismOffset{-0.1397_m, 0_m};

    DragonField *m_field; // Want to add targets and launcehr position

    const units::degree_t m_minLauncherAngle = 90_deg;
    const units::degree_t m_maxLauncherAngle = 270_deg;
};
