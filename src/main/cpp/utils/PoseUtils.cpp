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

// Team 302 Includes
#include "utils/PoseUtils.h"
#include "auton/AllianceZoneManager.h"
#include "auton/NeutralZoneManager.h"

// Units Includes
#include "units/math.h"

//------------------------------------------------------------------
/// @brief      Calculates the Euclidean distance between two poses
/// @param[in]  pose1 - First pose to compare
/// @param[in]  pose2 - Second pose to compare
/// @return     Distance between the pose translations in meters
/// @details    Computes the straight-line distance between two poses using
///             the built-in Translation2d::Distance() method, which applies
///             the Pythagorean theorem: sqrt((x2-x1)² + (y2-y1)²).
///             Only considers translational components (x, y), not rotation.
//------------------------------------------------------------------
units::length::meter_t PoseUtils::GetDeltaBetweenPoses(const frc::Pose2d &pose1, const frc::Pose2d &pose2)
{
    return pose1.Translation().Distance(pose2.Translation());
}

//------------------------------------------------------------------
/// @brief      Checks if two poses are within tolerance of each other
/// @param[in]  pose1 - First pose to compare
/// @param[in]  pose2 - Second pose to compare
/// @param[in]  positionTolerance - Maximum allowable distance in centimeters
/// @return     true if distance between poses is less than positionTolerance
/// @details    Uses GetDeltaBetweenPoses() to calculate separation and
///             compares against the provided tolerance threshold.
///             Useful for determining if robot has reached a target position.
//------------------------------------------------------------------
bool PoseUtils::IsSamePose(const frc::Pose2d &pose1,
                           const frc::Pose2d &pose2,
                           units::length::centimeter_t positionTolerance)
{
    // Compare squared distances to avoid an unnecessary sqrt; only take the
    // sqrt (via GetDeltaBetweenPoses) when the caller actually needs the value.
    auto dx = pose1.X() - pose2.X();
    auto dy = pose1.Y() - pose2.Y();
    auto toleranceM = units::length::meter_t{positionTolerance};
    return (dx * dx + dy * dy) < (toleranceM * toleranceM);
}

//------------------------------------------------------------------
/// @brief      Checks if two poses are within specified translational and rotational tolerances
/// @param[in]  pose1 - First pose to compare
/// @param[in]  pose2 - Second pose to compare
/// @param[in]  tolerance - Maximum allowable distance in centimeters
/// @param[in]  angleTolerance - Maximum allowable angular difference in degrees
/// @return     true if both position and rotation are within tolerance
/// @details    Calls the translational IsSamePose() first (short-circuits if positions
///             differ), then checks the absolute heading difference against headingTolerance.
//------------------------------------------------------------------
bool PoseUtils::IsSamePose(const frc::Pose2d &pose1,
                           const frc::Pose2d &pose2,
                           units::length::centimeter_t positionTolerance,
                           units::angle::degree_t headingTolerance)
{
    if (!IsSamePose(pose1, pose2, positionTolerance))
    {
        return false;
    }

    auto angleDiff = units::math::abs(pose1.Rotation().Degrees() - pose2.Rotation().Degrees());
    // Normalize to [0°, 180°] to correctly handle wrap-around at the ±180° boundary.
    // e.g. 170° vs -170° should be a 20° difference, not 340°.
    if (angleDiff > units::angle::degree_t{180.0})
    {
        angleDiff = units::angle::degree_t{360.0} - angleDiff;
    }
    return angleDiff < headingTolerance;
}

//------------------------------------------------------------------
/// @brief      Checks if a pose is at the field origin
/// @param[in]  pose - Pose to check
/// @param[in]  tolerance - Maximum distance from origin in centimeters
/// @return     true if pose is within tolerance of (0, 0)
/// @details    Convenience wrapper around IsSamePose() that compares
///             against the default origin pose: Pose2d(0_m, 0_m, 0_deg).
//------------------------------------------------------------------
bool PoseUtils::IsPoseAtOrigin(const frc::Pose2d &pose,
                               units::length::centimeter_t positionTolerance)
{
    static const frc::Pose2d kOrigin{};
    return IsSamePose(pose, kOrigin, positionTolerance);
}

//------------------------------------------------------------------
/// @brief      Checks if a pose is outside the field boundaries
/// @param[in]  pose - Pose to check
/// @return     true if pose is outside the field boundaries, false otherwise
///------------------------------------------------------------------
bool PoseUtils::IsPoseOffField(const frc::Pose2d &pose)
{
    if (pose.X() > m_fieldMinX && pose.X() < m_fieldMaxX && pose.Y() > m_fieldMinY && pose.Y() < m_fieldMaxY)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool PoseUtils::IsPoseJumping(const frc::Pose2d &pose1, const frc::Pose2d &pose2)
{
    if (GetDeltaBetweenPoses(pose1, pose2) > m_jumpThreshold)
    {
        return true;
    }
    return false;
}

//------------------------------------------------------------------
/// @brief      Determines which field element is closest to a reference pose
/// @param[in]  pose - Reference pose to measure from (typically robot position)
/// @param[in]  firstElement - First field element to consider
/// @param[in]  secondElement - Second field element to consider
/// @return     The field element enum that is nearest to the given pose
/// @details    Retrieves field element poses from FieldConstants singleton,
///             calculates distances to each, and returns the closer one.
///             Commonly used in autonomous routines to select between
///             multiple game piece or scoring locations.
///
/// @note       Returns firstElement if distances are equal
/// @see        FieldConstants::GetFieldElementPose2d() for element positions
//------------------------------------------------------------------
FieldConstants::FIELD_ELEMENT PoseUtils::GetClosestFieldElement(const frc::Pose2d &pose, FieldConstants::FIELD_ELEMENT firstElement, FieldConstants::FIELD_ELEMENT secondElement)
{
    return GetClosestFieldElement(pose, firstElement, secondElement, FieldConstants::GetInstance());
}

//------------------------------------------------------------------
/// @brief      Determines which field element is closest using a cached FieldConstants pointer
/// @param[in]  pose - Reference pose to measure from (typically robot position)
/// @param[in]  firstElement - First field element to consider
/// @param[in]  secondElement - Second field element to consider
/// @param[in]  fieldConstants - Pre-fetched FieldConstants pointer (avoids singleton re-lookup)
/// @return     The field element enum that is nearest to the given pose
//------------------------------------------------------------------
FieldConstants::FIELD_ELEMENT PoseUtils::GetClosestFieldElement(const frc::Pose2d &pose, FieldConstants::FIELD_ELEMENT firstElement, FieldConstants::FIELD_ELEMENT secondElement, FieldConstants *fieldConstants)
{
    if (fieldConstants == nullptr)
    {
        fieldConstants = FieldConstants::GetInstance();
        if (fieldConstants == nullptr)
        {
            // Fallback if FieldConstants is unavailable; avoid dereferencing a null pointer.
            return firstElement;
        }
    }
    auto firstElementPose = fieldConstants->GetFieldElementPose2d(firstElement);
    auto secondElementPose = fieldConstants->GetFieldElementPose2d(secondElement);

    // Compare squared distances to avoid two sqrt calls — only the relative
    // order matters here, not the actual distance values.
    auto dxFirst = pose.X() - firstElementPose.X();
    auto dyFirst = pose.Y() - firstElementPose.Y();
    auto dxSecond = pose.X() - secondElementPose.X();
    auto dySecond = pose.Y() - secondElementPose.Y();

    if ((dxFirst * dxFirst + dyFirst * dyFirst) < (dxSecond * dxSecond + dySecond * dySecond))
    {
        return firstElement;
    }

    return secondElement;
}
