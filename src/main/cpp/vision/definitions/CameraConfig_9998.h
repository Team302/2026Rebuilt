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
#include "units/length.h"
#include "vision/DragonLimelight.h"
#include "vision/DragonQuest.h"
#include "vision/definitions/CameraConfig.h"

class CameraConfig_9998 : public CameraConfig
{
public:
    CameraConfig_9998() = default;
    ~CameraConfig_9998() = default;

    void BuildCameraConfig() override;

private:
    static constexpr units::length::inch_t m_limelightFrontMountingXOffset{10.1875};
    static constexpr units::length::inch_t m_limelightFrontMountingYOffset{0};
    static constexpr units::length::inch_t m_limelightFrontMountingZOffset{22.25};
    static constexpr units::angle::degree_t m_limelightFrontPitch{-20};
    static constexpr units::angle::degree_t m_limelightFrontYaw{0};
    static constexpr units::angle::degree_t m_limelightFrontRoll{0};

    static constexpr units::length::inch_t m_limelightFront3MountingXOffset{10.1875};
    static constexpr units::length::inch_t m_limelightFront3MountingYOffset{-0.25};
    static constexpr units::length::inch_t m_limelightFront3MountingZOffset{23.375};
    static constexpr units::angle::degree_t m_limelightFront3Pitch{-20};
    static constexpr units::angle::degree_t m_limelightFront3Yaw{0};
    static constexpr units::angle::degree_t m_limelightFront3Roll{0};

    static constexpr units::length::inch_t m_questMountingXOffset{-18.5};
    static constexpr units::length::inch_t m_questMountingYOffset{0};
    static constexpr units::length::inch_t m_questMountingZOffset{21.75};
    static constexpr units::angle::degree_t m_questPitch{-3};
    static constexpr units::angle::degree_t m_questYaw{-175};
    static constexpr units::angle::degree_t m_questRoll{0};

    std::unique_ptr<DragonLimelight> m_limelightFront;
    std::unique_ptr<DragonLimelight> m_limelightfront3;
    std::unique_ptr<DragonQuest> m_quest;
};