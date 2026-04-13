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
#include <units/angle.h>
#include <units/length.h>

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
     * \brief Get the current target position in world coordinates (field frame)
     * \return Translation2d with X, Y position in meters
     *
     * Subclasses should override this to provide their specific target.
     * Non-const to allow dynamic target selection based on chassis position.
     */
    virtual frc::Translation2d GetTargetPosition() = 0;

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
    frc::Translation2d CalculateVirtualTarget(const frc::Translation2d &realTarget, units::time::second_t lookaheadTime) const;

    /**
     * \brief Get mechanism/mechanism position in world coordinates
     *
     * The mechanism position is defined relative to the robot's center in robot coordinates,
     * then rotated and translated to world coordinates using the current chassis pose.
     *
     * \return Translation2d with mechanism position in meters (world frame)
     */
    frc::Translation2d
    GetMechanismWorldPosition() const;

    /**
     * \brief Calculate distance from chassis center to target
     * \param lookaheadTime Time in seconds for virtual target calculation (default 0 = no compensation)
     * \return Distance in meters
     */
    units::meter_t CalculateDistanceToTarget(units::time::second_t lookaheadTime = 0_s);

    /**
     * \brief Calculate distance from mechanism to target
     *
     * More precise than chassis center distance for mechanisms offset from robot center.
     *
     * \param lookaheadTime Time in seconds for virtual target calculation (default 0 = no compensation)
     * \return Distance in meters
     */
    units::meter_t CalculateMechanismDistanceToTarget(units::time::second_t lookaheadTime = 0_s);

    /**
     * \brief Calculate angle from chassis center to target in robot frame
     *
     * Robot frame: 0° = forward (robot +X), 90° = left (robot +Y), -90° = right
     *
     * \param lookaheadTime Time in seconds for virtual target calculation (default 0 = no compensation)
     * \return Angle in degrees
     */
    units::degree_t CalculateAngleToTarget(units::time::second_t lookaheadTime = 0_s);

    /**
     * \brief Calculate angle from mechanism to target in robot frame
     *
     * More precise than chassis center angle for mechanisms offset from robot center.
     * Robot frame: 0° = forward (robot +X), 90° = left (robot +Y), -90° = right
     *
     * \param lookaheadTime Time in seconds for virtual target calculation (default 0 = no compensation)
     * \return Angle in degrees
     */
    units::degree_t CalculateMechanismAngleToTarget(units::time::second_t lookaheadTime = 0_s);

    /**
     * \brief Set the mechanism offset from robot center in robot coordinates
     *
     * \param xOffset X offset in meters (+ = forward)
     * \param yOffset Y offset in meters (+ = left)
     */
    void SetMechanismOffset(frc::Translation2d offset);

    /**
     * \brief Get the virtual target position based on chassis velocity
     *
     * The virtual target compensates for robot movement during projectile flight.
     * It offsets the real target by the distance the robot will travel during the
     * lookahead time (projectile flight time).
     *
     * \param lookaheadTime Time in seconds for projectile flight
     * \return Pose2d with virtual target position and zero rotation
     */
    frc::Pose2d GetVirtualTargetPose(units::time::second_t lookaheadTime);

    void ForceUpdateChassisPose() { UpdateChassisPose(true); }

protected:
    TargetCalculator();
    ~TargetCalculator() = default;

    /**
     * \brief Get the current cached chassis pose
     * \return Pose2d with X, Y position (meters) and rotation (radians)
     */
    frc::Pose2d GetChassisPose() const { return m_chassisPose; };

    /**
     * \brief Update the chassis pose and return it
     */
    void UpdateChassisPose(bool forceUpdate = false);

    /**
     * \brief Get the current chassis velocity
     * \return ChassisSpeeds with vx, vy (m/s) and omega (rad/s) in field frame
     */
    frc::ChassisSpeeds GetChassisVelocity() const { return m_currentChassisSpeeds; };

    /**
     * \brief Updates the current chassis velocity
     */
    void UpdateChassisSpeeds();

    frc::Pose2d m_lastChassisPose{};
    subsystems::CommandSwerveDrivetrain *GetChassis() const { return m_chassis; }
    bool m_isMoving = false;

private:
    static TargetCalculator *m_instance;
    frc::Translation2d m_mechanismOffset;

    frc::ChassisSpeeds m_currentChassisSpeeds{};
    frc::Pose2d m_chassisPose{};

    static constexpr units::velocity::meters_per_second_t m_translationSpeedThreshold{0.05};
    static constexpr units::angular_velocity::degrees_per_second_t m_rotationSpeedThreshold{1};
    static constexpr units::length::inch_t m_isMovingDistanceThreshold{1};
    subsystems::CommandSwerveDrivetrain *m_chassis;

    /// True if the robot was above the speed threshold on the previous cycle.
    /// Used to force one final pose update on the transition from moving → stopped,
    /// ensuring the cache busts once so calculations rerun with zero speeds.
    bool m_wasMoving = false;
    /**
     * \brief Calculate how much the mechanism's world position shifts due to rotation during lookahead.
     *
     * Integrates omega * lookaheadTime to find the future robot heading, then computes the
     * difference between the future and current mechanism-offset world vectors.  This delta
     * can be subtracted from a virtual target so that mechanism-relative callers
     * (CalculateMechanismAngleToTarget, CalculateMechanismDistanceToTarget) aim correctly
     * when the robot is spinning, without affecting chassis-center callers.
     *
     * \param lookaheadTime Time in seconds for projectile flight
     * \return Translation2d representing the change in mechanism world position due to rotation
     */
    frc::Translation2d
    CalculateRotationalMechDelta(units::time::second_t lookaheadTime) const;

    // Cached calculation results to avoid recalculation when pose hasn't changed
    units::meter_t m_cachedDistanceToTarget = 0_m;
    units::meter_t m_cachedMechanismDistanceToTarget = 0_m;
    units::degree_t m_cachedMechanismAngleToTarget = 0_deg;
};
