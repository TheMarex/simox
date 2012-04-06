
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
	name = "PoseQualityMeasurement";
	considerObstacle = false;
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

std::string PoseQualityMeasurement::getName()
{
	return name;
}

VirtualRobot::RobotNodeSetPtr PoseQualityMeasurement::getRNS()
{
	return rns;
}

bool PoseQualityMeasurement::consideringJointLimits()
{
	return false;
}

void PoseQualityMeasurement::setObstacleDistanceVector( const Eigen::Vector3f &directionSurfaceToObstance )
{
	considerObstacle = true;
	obstacleDir = directionSurfaceToObstance;
}

void PoseQualityMeasurement::disableObstacleDistance()
{
	considerObstacle = false;
}

}
