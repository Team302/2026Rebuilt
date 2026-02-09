#include "chassis/generated/TunerConstants9997.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"

std::unique_ptr<subsystems::CommandSwerveDrivetrain> TunerConstants9997::CreateDrivetrain()
{
    return std::make_unique<subsystems::CommandSwerveDrivetrain>(
        TunerConstants9997::DrivetrainConstants,
        TunerConstants9997::FrontLeft,
        TunerConstants9997::FrontRight,
        TunerConstants9997::BackLeft,
        TunerConstants9997::BackRight);
}
