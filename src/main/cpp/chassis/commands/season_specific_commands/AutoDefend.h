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

#include <memory>

#include "chassis/commands/VisionDrive.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "frc2/command/CommandHelper.h"
#include "vision/DragonVision.h"

class AutoDefend : public frc2::CommandHelper<VisionDrive, AutoDefend>
{
public:
    AutoDefend(subsystems::CommandSwerveDrivetrain *chassis);

protected:
    swerve::requests::RobotCentric GetRobotDriveRequest() override;

private:
    DragonVision *m_vision = nullptr;

    static constexpr double kPDrive{4.0};
    static constexpr double kIDrive{0.0};
    static constexpr double kDDrive{0.0};

    static constexpr double kPYaw{1.5};
    static constexpr double kIYaw{0.0};
    static constexpr double kDYaw{0.0};

    frc::PIDController m_xController{kPDrive, kIDrive, kDDrive};
    frc::PIDController m_yController{kPDrive, kIDrive, kDDrive};
    frc::PIDController m_yawController{kPYaw, kIYaw, kDYaw};

    units::length::meter_t m_XdistLimit = 5_m;
    units::length::meter_t m_YdistLimit = 5_m;

    units::length::meter_t m_IntakeXOffset = 1_m;

    units::velocity::meters_per_second_t m_maxXSpeed = 3_mps;
    units::velocity::meters_per_second_t m_maxYSpeed = 3_mps;
    units::angular_velocity::degrees_per_second_t m_maxRotationalSpeed = 45_deg_per_s;

    int m_visionCacheI = 0;

    std::vector<std::unique_ptr<DragonVisionStruct>> m_visionCache;
};