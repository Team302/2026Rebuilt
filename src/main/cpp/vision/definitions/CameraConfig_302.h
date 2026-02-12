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
#include "vision/definitions/CameraConfig.h"

class CameraConfig_302 : public CameraConfig
{
public:
    CameraConfig_302() = default;
    ~CameraConfig_302() = default;

    void BuildCameraConfig() override;

private:
    static constexpr units::length::inch_t m_ll1MountingXOffset{-10.585};
    static constexpr units::length::inch_t m_ll1MountingYOffset{10.257};
    static constexpr units::length::inch_t m_ll1MountingZOffset{9.231};
    static constexpr units::angle::degree_t m_ll1Pitch{5};
    static constexpr units::angle::degree_t m_ll1Yaw{-150};
    static constexpr units::angle::degree_t m_ll1Roll{0};

    static constexpr units::length::inch_t m_ll2MountingXOffset{-10.585};
    static constexpr units::length::inch_t m_ll2MountingYOffset{-10.257};
    static constexpr units::length::inch_t m_ll2MountingZOffset{9.231};
    static constexpr units::angle::degree_t m_ll2Pitch{5};
    static constexpr units::angle::degree_t m_ll2Yaw{150};
    static constexpr units::angle::degree_t m_ll2Roll{0};

    static constexpr units::length::inch_t m_ll3MountingXOffset{7.43};
    static constexpr units::length::inch_t m_ll3MountingYOffset{14.359};
    static constexpr units::length::inch_t m_ll3MountingZOffset{19.589};
    static constexpr units::angle::degree_t m_ll3Pitch{0};
    static constexpr units::angle::degree_t m_ll3Yaw{90};
    static constexpr units::angle::degree_t m_ll3Roll{0};

    static constexpr units::length::inch_t m_questMountingXOffset{-11.171};
    static constexpr units::length::inch_t m_questMountingYOffset{-9.924};
    static constexpr units::length::inch_t m_questMountingZOffset{16.78};
    static constexpr units::angle::degree_t m_questPitch{0.0};
    static constexpr units::angle::degree_t m_questYaw{180.0};
    static constexpr units::angle::degree_t m_questRoll{0.0};
};