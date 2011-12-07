
#include "PoseQualityMeasurement.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;

namespace VirtualRobot
{


PoseQualityMeasurement::PoseQualityMeasurement(VirtualRobot::RobotNodeSetPtr rns)
:rns(rns)
{
	THROW_VR_EXCEPTION_IF( (!rns || !rns->getTCP()), "NULL data");
	verbose = false;
}


PoseQualityMeasurement::~PoseQualityMeasurement()
{
}

float PoseQualityMeasurement::getPoseQuality()
{
	VR_WARNING << "Please use derived classes..." << endl;
	return 0.0f;
}

void PoseQualityMeasurement::setVerbose( bool v )
{
	verbose = v;
}

}
