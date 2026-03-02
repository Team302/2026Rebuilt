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

#include "auton/AllianceZoneManager.h"
#include "fielddata/FieldConstants.h"
#include "utils/DragonField.h"
#include "utils/TargetCalculator.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <string>
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
    units::angle::turn_t GetLauncherTarget(units::time::second_t looheadTime, units::angle::degree_t currentLauncherAngle);

    /**
     * \brief Update the target offset based on driver input
     */
    void UpdateTargetOffset();

private:
    bool ValidateAlliance();

    /**
     * \brief Constructor - initializes with default mechanism offset
     */
    RebuiltTargetCalculator();

    /**
     * \brief Get the passing target offset based on field element type
     * \param fieldElement The field element to check
     * \return X offset value
     */
    units::length::inch_t GetPassingTargetXOffset(FieldConstants::FIELD_ELEMENT fieldElement);

    /**
     * \brief Get the passing target offset based on field element type
     * \param fieldElement The field element to check
     * \return Y offset value
     */
    units::length::inch_t GetPassingTargetYOffset(FieldConstants::FIELD_ELEMENT fieldElement);

    /**
     * \brief Update the passing target positions on the field based on current offsets
     * */
    void UpdatePassingTargetsOnField();

    /**
     * \brief Refresh cached alliance-specific field elements and positions if alliance has changed
     */
    void RefreshAllianceCache();

    static RebuiltTargetCalculator *m_instance;

    // Static string constants to avoid repeated std::string construction
    static const std::string kOutpostPassingTargetName;
    static const std::string kDepotPassingTargetName;
    static const std::string kCurrentTargetName;
    static const std::string kLauncherPositionName;

    // Mechanism position offset from robot center in robot frame (meters)
    // Default: 5.5 inches (0.1397m) back, centered
    frc::Translation2d m_mechanismOffset{-3.333_in, 4.604_in};

    DragonField *m_field; // Want to add targets and launcher position

    const units::degree_t m_minLauncherAngle = 91_deg;
    const units::degree_t m_maxLauncherAngle = 267_deg;

    FieldConstants *m_fieldConstants;
    AllianceZoneManager *m_zoneManager;

    // Cached alliance-specific field elements (initialized in constructor and refreshed on alliance change via RefreshAllianceCache)
    FieldConstants::FIELD_ELEMENT m_hubCenter;
    FieldConstants::FIELD_ELEMENT m_outpostPassingTarget;
    FieldConstants::FIELD_ELEMENT m_depotPassingTarget;

    // Cached field positions (looked up once in constructor instead of every cycle)
    frc::Translation2d m_hubCenterPosition;
    frc::Translation2d m_outpostPassingPosition;
    frc::Translation2d m_depotPassingPosition;

    // Cached alliance for detecting changes
    frc::DriverStation::Alliance m_cachedAlliance;

    units::length::inch_t m_xTargetOffset = 0_in;
    units::length::inch_t m_yTargetOffset = 0_in;

    units::length::inch_t m_passingDepotTargetXOffset = 0_in;
    units::length::inch_t m_passingDepotTargetYOffset = 0_in;
    units::length::inch_t m_passingOutpostTargetXOffset = 0_in;
    units::length::inch_t m_passingOutpostTargetYOffset = 0_in;

    // Button state tracking for single press per button
    bool m_prevUpPressed = false;
    bool m_prevDownPressed = false;
    bool m_prevLeftPressed = false;
    bool m_prevRightPressed = false;

    bool m_validatedWhileEnabled = false;
};
