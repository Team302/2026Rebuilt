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

#include "chassis/ChassisConfigMgr.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/length.h>
#include <units/angle.h>

/**
 * \class TargetCalculator
 * \brief Base class for target calculations relative to chassis pose and velocity.
 *
 * This class provides core functionality for:
 * - Calculating target distances and angles from the chassis center
 * - Computing virtual target positions based on chassis velocity (movement compensation)
 * - Converting between world frame and robot frame coordinates
 *
 * Subclasses should override GetTargetPosition() to define their specific target locations
 * and provide season-specific target selection logic.
 */
class TargetCalculator
{
public:
    /**
     * \brief Constructor - initializes the calculator with chassis reference
     */
    TargetCalculator();

    /**
     * \brief Destructor
     */
    ~TargetCalculator() = default;

    /**
     * \brief Get singleton instance
     * \return Pointer to the TargetCalculator singleton
     */
    static TargetCalculator *GetInstance();

    /**
     * \brief Get the current target position in world coordinates (field frame)
     * \return Translation2d with X, Y position in meters
     *
     * Subclasses should override this to provide their specific target.
     * Non-const to allow dynamic target selection based on chassis position.
     */
    virtual frc::Translation2d GetTargetPosition();

    /**
     * \brief Calculate the virtual target position based on chassis velocity
     *
     * The virtual target compensates for robot movement during projectile flight.
     * It offsets the real target by the distance the robot will travel during the
     * lookahead time (projectile flight time).
     *
     * \param realTarget The actual target position in world coordinates
     * \param lookaheadTime Time in seconds for projectile flight
     * \return Translation2d representing the virtual target in world coordinates
     */
    frc::Translation2d CalculateVirtualTarget(const frc::Translation2d &realTarget, units::second_t lookaheadTime) const;

    /**
     * \brief Get launcher/mechanism position in world coordinates
     *
     * The launcher position is defined relative to the robot's center in robot coordinates,
     * then rotated and translated to world coordinates using the current chassis pose.
     *
     * \return Translation2d with launcher position in meters (world frame)
     */
    frc::Translation2d GetLauncherWorldPosition() const;

    /**
     * \brief Calculate distance from chassis center to target
     * \param target Target position in world coordinates (if nullptr, uses GetTargetPosition())
     * \return Distance in meters
     */
    units::meter_t CalculateDistanceToTarget(const frc::Translation2d *target = nullptr);

    /**
     * \brief Calculate distance from launcher to target
     *
     * More precise than chassis center distance for mechanisms offset from robot center.
     *
     * \param target Target position in world coordinates (if nullptr, uses GetTargetPosition())
     * \return Distance in meters
     */
    units::meter_t CalculateLauncherDistanceToTarget(const frc::Translation2d *target = nullptr);

    /**
     * \brief Calculate angle from chassis center to target in robot frame
     *
     * Robot frame: 0° = forward (robot +X), 90° = left (robot +Y), -90° = right
     *
     * \param target Target position in world coordinates (if nullptr, uses GetTargetPosition())
     * \return Angle in degrees
     */
    units::degree_t CalculateAngleToTarget(const frc::Translation2d *target = nullptr);

    /**
     * \brief Calculate angle from launcher to target in robot frame
     *
     * More precise than chassis center angle for mechanisms offset from robot center.
     * Robot frame: 0° = forward (robot +X), 90° = left (robot +Y), -90° = right
     *
     * \param target Target position in world coordinates (if nullptr, uses GetTargetPosition())
     * \return Angle in degrees
     */
    units::degree_t CalculateLauncherAngleToTarget(const frc::Translation2d *target = nullptr);

    /**
     * \brief Set the launcher offset from robot center in robot coordinates
     *
     * \param xOffset X offset in meters (+ = forward)
     * \param yOffset Y offset in meters (+ = left)
     */
    virtual void SetLauncherOffset(units::meter_t xOffset, units::meter_t yOffset);

    /**
     * \brief Get the current launcher offset in robot coordinates
     * \return Translation2d with X, Y offsets in meters
     */
    virtual frc::Translation2d GetLauncherOffset() const;

protected:
    /**
     * \brief Get the current chassis pose
     * \return Pose2d with X, Y position (meters) and rotation (radians)
     */
    frc::Pose2d GetChassisPose() const;

    /**
     * \brief Get the current chassis velocity
     * \return ChassisSpeeds with vx, vy (m/s) and omega (rad/s) in field frame
     */
    frc::ChassisSpeeds GetChassisVelocity() const;

private:
    static TargetCalculator *m_instance;

    subsystems::CommandSwerveDrivetrain *m_chassis;
};
