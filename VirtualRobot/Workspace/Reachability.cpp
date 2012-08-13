#include "Reachability.h"
#include "../VirtualRobotException.h"
#include "../Robot.h"
#include "../RobotNodeSet.h"
#include "../ManipulationObject.h"
#include "../Grasping/Grasp.h"
#include "../Grasping/GraspSet.h"
#include <fstream>
#include <cmath>
#include <float.h>
#include <limits.h>

namespace VirtualRobot
{



Reachability::Reachability(RobotPtr robot) : WorkspaceRepresentation (robot)
{
	type = "Reachbaility";
}

void Reachability::addCurrentTCPPose()
{	
	THROW_VR_EXCEPTION_IF(!data || !nodeSet || !tcpNode, "No reachability data loaded");

	Eigen::Matrix4f p = tcpNode->getGlobalPose();
	toLocal(p);
	//if (baseNode)
		//p = baseNode->toLocalCoordinateSystem(p);

	float x[6];
	MathTools::eigen4f2rpy(p,x);

	// check for achieved values
	for (int i=0;i<6;i++)
	{
		if (x[i] < achievedMinValues[i])
			achievedMinValues[i] = x[i];
		if (x[i] > achievedMaxValues[i])
			achievedMaxValues[i] = x[i];
	}

	// get voxels
	unsigned int v[6];
	if (getVoxelFromPose(x,v))
	{
		data->increaseDatum(v);
	}

	buildUpLoops++;
}

void Reachability::addRandomTCPPoses( unsigned int loops, bool checkForSelfCollisions )
{
	THROW_VR_EXCEPTION_IF(!data || !nodeSet || !tcpNode, "Reachability data not initialized");

	std::vector<float> c;
	nodeSet->getJointValues(c);
	bool visuSate = robot->getUpdateVisualizationStatus();
	robot->setUpdateVisualization(false);

	for (unsigned int i=0;i<loops;i++)
	{
		if (setRobotNodesToRandomConfig(nodeSet, checkForSelfCollisions))
			addCurrentTCPPose();
		else
			VR_WARNING << "Could not find collision-free configuration...";
	}
	robot->setUpdateVisualization(visuSate);
	robot->setJointValues(nodeSet,c);
}
bool Reachability::isReachable( const Eigen::Matrix4f &globalPose )
{
	return isCovered(globalPose);
}

VirtualRobot::GraspSetPtr Reachability::getReachableGrasps( GraspSetPtr grasps, ManipulationObjectPtr object )
{
	THROW_VR_EXCEPTION_IF(!object,"no object");
	THROW_VR_EXCEPTION_IF(!grasps,"no grasps");

	GraspSetPtr result(new GraspSet(grasps->getName(),grasps->getRobotType(),grasps->getEndEffector()));
	for (unsigned int i=0; i<grasps->getSize(); i++)
	{
		Eigen::Matrix4f m = grasps->getGrasp(i)->getTcpPoseGlobal(object->getGlobalPose());
		if (isReachable(m))
			result->addGrasp(grasps->getGrasp(i));
	}
	return result;
}

Eigen::Matrix4f Reachability::sampleReachablePose()
{
    return sampleCoveredPose();
}

} // namespace VirtualRobot
