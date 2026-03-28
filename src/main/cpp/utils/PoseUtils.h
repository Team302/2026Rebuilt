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

// FRC Includes
#include "fielddata/FieldConstants.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "units/angle.h"
#include "units/length.h"

//====================================================================================================================================================
/// @class PoseUtils
/// @brief Utility class providing static methods for pose-based calculations and comparisons
///
/// This class offers a collection of geometric utility functions for working with FRC Pose2d objects.
/// All methods are static, making this a stateless utility class for common pose operations including:
/// - Distance calculations between poses
/// - Pose comparison with tolerance
/// - Field element proximity detection
/// - Angle calculations between three poses
///
/// These utilities are commonly used in autonomous navigation, path planning, and field-relative positioning.
//====================================================================================================================================================
class PoseUtils
{
public:
    PoseUtils() = default;
    ~PoseUtils() = default;

    //------------------------------------------------------------------
    /// @brief      Calculates the Euclidean distance between two poses
    /// @param[in]  translationOne - First pose to compare
    /// @param[in]  translationTwo - Second pose to compare
    /// @return     Distance between the two pose translations in meters
    /// @details    Uses the Pythagorean theorem to compute the straight-line
    ///             distance between pose translations, ignoring rotation.
    ///             Equivalent to sqrt((x2-x1)² + (y2-y1)²).
    //------------------------------------------------------------------
    static units::length::meter_t GetDeltaBetweenPoses(const frc::Pose2d &translationOne,
                                                       const frc::Pose2d &translationTwo);

    //------------------------------------------------------------------
    /// @brief      Checks if two poses are within a specified tolerance
    /// @param[in]  pose1 - First pose to compare
    /// @param[in]  pose2 - Second pose to compare
    /// @param[in]  positionTolerance - Maximum distance in centimeters for poses to be considered the same
    /// @return     true if poses are within positionTolerance, false otherwise
    /// @details    Compares only the translational component (x, y position),
    ///             not the rotational component. Useful for checking if the
    ///             robot has reached a target position.
    //------------------------------------------------------------------
    static bool IsSamePose(const frc::Pose2d &pose1,
                           const frc::Pose2d &pose2,
                           units::length::centimeter_t positionTolerance);

    //------------------------------------------------------------------
    /// @brief      Checks if two poses are within specified translational and rotational tolerances
    /// @param[in]  pose1 - First pose to compare
    /// @param[in]  pose2 - Second pose to compare
    /// @param[in]  positionTolerance - Maximum distance in centimeters for poses to be considered the same
    /// @param[in]  headingTolerance - Maximum heading difference in degrees for poses to be considered the same
    /// @return     true if both position and rotation are within tolerance, false otherwise
    /// @details    First checks translational proximity using IsSamePose(), then additionally
    ///             compares the absolute difference between the two headings against headingTolerance.
    //------------------------------------------------------------------
    static bool IsSamePose(const frc::Pose2d &pose1,
                           const frc::Pose2d &pose2,
                           units::length::centimeter_t positionTolerance,
                           units::angle::degree_t headingTolerance);

    //------------------------------------------------------------------
    /// @brief      Checks if a pose is at the origin (0, 0)
    /// @param[in]  pose - Pose to check
    /// @param[in]  positionTolerance - Maximum distance from origin in centimeters
    /// @return     true if pose is within positionTolerance of origin, false otherwise
    /// @details    Convenience method that compares the given pose against
    ///             the default origin pose Pose2d(0, 0, 0°).
    //------------------------------------------------------------------
    static bool IsPoseAtOrigin(const frc::Pose2d &pose,
                               units::length::centimeter_t positionTolerance);

    //------------------------------------------------------------------
    /// @brief      Determines which of two field elements is closest to a pose
    /// @param[in]  pose - Reference pose to measure from
    /// @param[in]  firstElement - First field element to consider
    /// @param[in]  secondElement - Second field element to consider
    /// @return     The field element that is closest to the given pose
    /// @details    Retrieves field element positions from FieldConstants and
    ///             calculates distances to determine which is nearer. Useful
    ///             for autonomous decision-making when multiple targets exist.
    /// @see        FieldConstants for available field elements
    //------------------------------------------------------------------
    static FieldConstants::FIELD_ELEMENT GetClosestFieldElement(const frc::Pose2d &pose, FieldConstants::FIELD_ELEMENT firstElement, FieldConstants::FIELD_ELEMENT secondElement);

    //------------------------------------------------------------------
    /// @brief      Determines which of two field elements is closest to a pose (using cached FieldConstants)
    /// @param[in]  pose - Reference pose to measure from
    /// @param[in]  firstElement - First field element to consider
    /// @param[in]  secondElement - Second field element to consider
    /// @param[in]  fieldConstants - Pointer to cached FieldConstants (avoids singleton re-lookup)
    /// @return     The field element that is closest to the given pose
    /// @details    Same as the other overload but uses a pre-fetched FieldConstants pointer
    ///             instead of calling FieldConstants::GetInstance() internally. Preferred when
    ///             the caller already has a cached FieldConstants pointer.
    //------------------------------------------------------------------
    static FieldConstants::FIELD_ELEMENT GetClosestFieldElement(const frc::Pose2d &pose, FieldConstants::FIELD_ELEMENT firstElement, FieldConstants::FIELD_ELEMENT secondElement, FieldConstants *fieldConstants);
};