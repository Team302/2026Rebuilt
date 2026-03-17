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

#include "chassis/commands/season_specific_commands/DriveToFuel.h"
#include "vision/DragonVision.h"
#include "vision/PoseOffsetUtils.h"

DriveToFuel::DriveToFuel(subsystems::CommandSwerveDrivetrain *chassis) : frc2::CommandHelper<VisionDrive, DriveToFuel>(chassis)
{
    m_vision = DragonVision::GetDragonVision();
}

swerve::requests::RobotCentric DriveToFuel::GetRobotDriveRequest()
{
    if (m_vision == nullptr)
    {
        return swerve::requests::RobotCentric{};
    }
    m_vision->SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE::OBJECT_DETECTION, DRAGON_LIMELIGHT_PIPELINE::FUEL_PL);

    m_visionCache = m_vision->GetObjectDetectionTargetInfo(VisionTargetOption::CLOSEST_VALID_TARGET, std::vector<int>{});
    if (!m_visionCache.empty() && m_visionCache[0].get() != nullptr)
    {
        units::angular_velocity::degrees_per_second_t rotRate = units::degrees_per_second_t(m_yawController.Calculate(m_visionCache[0]->horizontalOffset.value()));
        rotRate = std::clamp(rotRate, -m_maxRotationalSpeed, m_maxRotationalSpeed);

        units::length::meter_t xDist = PoseOffsetUtils::CalculateXYDistanceFromObject(*m_visionCache[0], 5.5_in).first;
        xDist = std::clamp(xDist, 0_m, m_XdistLimit);
        xDist -= m_IntakeXOffset;
        units::velocity::meters_per_second_t xspeed = units::meters_per_second_t(m_xController.Calculate(xDist.value()));
        xspeed = std::clamp(xspeed, -m_maxXSpeed, m_maxXSpeed);

        auto request = swerve::requests::RobotCentric{}.WithVelocityX(-xspeed).WithVelocityY(0_mps).WithRotationalRate(rotRate);
        request.Deadband = 0.1_mps;
        return request;
    }
    else
    {
        return swerve::requests::RobotCentric{};
    }
}
