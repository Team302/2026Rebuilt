#include "chassis/generated/TunerConstants9999.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"

std::unique_ptr<subsystems::CommandSwerveDrivetrain> TunerConstants9999::CreateDrivetrain()
{
    return std::make_unique<subsystems::CommandSwerveDrivetrain>(
        TunerConstants9999::DrivetrainConstants,
        TunerConstants9999::FrontLeft,
        TunerConstants9999::FrontRight,
        TunerConstants9999::BackLeft,
        TunerConstants9999::BackRight);
}
