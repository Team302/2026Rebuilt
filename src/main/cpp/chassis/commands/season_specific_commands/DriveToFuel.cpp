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
#include "vision/PoseOffsetUtils.h"

DriveToFuel::DriveToFuel(subsystems::CommandSwerveDrivetrain *chassis) : frc2::CommandHelper<VisionDrive, DriveToFuel>(chassis)
{
}

std::vector<std::unique_ptr<DragonVisionStruct>> DriveToFuel::GetObjectSelection()
{
    m_vision = DragonVision::GetDragonVision();
    m_vision->SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE::OBJECT_DETECTION, DRAGON_LIMELIGHT_PIPELINE::MACHINE_LEARNING_PL); // Ensure correct pipeline for fuel targets
                                                                                                                            // Retrieve the closest valid fuel target (class 0)
    auto targets = m_vision->GetObjectDetectionTargetInfo(VisionTargetOption::CLOSEST_VALID_TARGET, std::vector<int>{});

    return targets;
}
