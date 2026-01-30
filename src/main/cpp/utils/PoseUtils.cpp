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

/// @brief Find the distance between two poses by using the Pythagorean Formula
/// @param poseOne first pose to compare
/// @param poseTwo second pose to compare
/// @return frc::Translation2d - the difference in X value and the distance in Y value

units::length::meter_t PoseUtils::GetDeltaBetweenPoses(const frc::Pose2d &pose1, const frc::Pose2d &pose2)
{
    return pose1.Translation().Distance(pose2.Translation());
}

bool PoseUtils::IsSamePose(const frc::Pose2d &pose1,
                           const frc::Pose2d &pose2,
                           units::length::centimeter_t tolerance)
{
    return GetDeltaBetweenPoses(pose1, pose2) < tolerance;
}

bool PoseUtils::IsPoseAtOrigin(const frc::Pose2d &pose,
                               units::length::centimeter_t tolerance)
{
    return IsSamePose(pose, frc::Pose2d(), tolerance);
}