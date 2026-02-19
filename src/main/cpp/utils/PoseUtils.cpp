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
/// @param[in]  tolerance - Maximum allowable distance in centimeters
/// @return     true if distance between poses is less than tolerance
/// @details    Uses GetDeltaBetweenPoses() to calculate separation and
///             compares against the provided tolerance threshold.
///             Useful for determining if robot has reached a target position.
//------------------------------------------------------------------
bool PoseUtils::IsSamePose(const frc::Pose2d &pose1,
                           const frc::Pose2d &pose2,
                           units::length::centimeter_t tolerance)
{
    return GetDeltaBetweenPoses(pose1, pose2) < tolerance;
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
                               units::length::centimeter_t tolerance)
{
    return IsSamePose(pose, frc::Pose2d(), tolerance);
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
    auto fieldConstants = FieldConstants::GetInstance();
    auto firstElementPose = fieldConstants->GetFieldElementPose2d(firstElement);
    auto secondElementPose = fieldConstants->GetFieldElementPose2d(secondElement);

    auto distanceToFirst = GetDeltaBetweenPoses(pose, firstElementPose);
    auto distanceToSecond = GetDeltaBetweenPoses(pose, secondElementPose);

    if (distanceToFirst < distanceToSecond)
    {
        return firstElement;
    }

    return secondElement;
}

//------------------------------------------------------------------
/// @brief      Calculates the angle at a vertex formed by three poses
/// @param[in]  startPose - First endpoint of the angle
/// @param[in]  vertexPose - Center point where angle is measured
/// @param[in]  endPose - Second endpoint of the angle
/// @return     Angle in radians (always positive, 0 to π)
/// @details    Computes the interior angle at vertexPose using vector mathematics:
///
///             **Algorithm:**
///             1. Create vector A = startPose - vertexPose
///             2. Create vector B = endPose - vertexPose
///             3. Calculate dot product: A·B = Ax*Bx + Ay*By
///             4. Calculate magnitudes: |A| and |B|
///             5. Compute angle: θ = arccos((A·B) / (|A|*|B|))
///
///             **Edge Cases:**
///             - Returns 0° if either vector has magnitude ≤ 0.01 (near-zero)
///             - Clamps cosine to [-1, 1] to handle floating point precision errors
///
///             Useful for trajectory analysis, path validation, and
///             determining if the robot needs to make sharp turns.
///
/// @note       Result is unsigned angle; does not indicate turn direction
//------------------------------------------------------------------
units::angle::radian_t PoseUtils::GetAngleBetweenPoses(const frc::Pose2d &startPose, const frc::Pose2d &vertexPose, const frc::Pose2d &endPose)
{
    // Create vectors from vertex to start and vertex to end
    auto vertexToStart = startPose.Translation() - vertexPose.Translation();
    auto vertexToEnd = endPose.Translation() - vertexPose.Translation();

    // Get the X and Y components
    auto VS_x = vertexToStart.X().value();
    auto VS_y = vertexToStart.Y().value();
    auto VE_x = vertexToEnd.X().value();
    auto VE_y = vertexToEnd.Y().value();

    // Calculate dot product
    auto dotProduct = VS_x * VE_x + VS_y * VE_y;

    // Calculate magnitudes
    auto magVS = std::sqrt(VS_x * VS_x + VS_y * VS_y);
    auto magVE = std::sqrt(VE_x * VE_x + VE_y * VE_y);

    // Handle edge case of zero-length vectors
    if (magVS <= 0.01 || magVE <= 0.01)
    {
        return 0.0_deg;
    }

    // Calculate angle using arccos
    auto cosTheta = dotProduct / (magVS * magVE);

    // Clamp to [-1, 1] to handle floating point errors
    cosTheta = std::clamp(cosTheta, -1.0, 1.0);

    return units::angle::radian_t(std::acos(cosTheta));
}
