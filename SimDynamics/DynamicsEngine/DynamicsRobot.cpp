#include "DynamicsRobot.h"
#include "../DynamicsWorld.h"

#include <VirtualRobot/SceneObject.h>

namespace SimDynamics {

DynamicsRobot::DynamicsRobot(VirtualRobot::RobotPtr rob)
{
	THROW_VR_EXCEPTION_IF(!rob,"NULL object");
	robot = rob;
}
	
DynamicsRobot::~DynamicsRobot()
{
}

std::string DynamicsRobot::getName() const
{
	return robot->getName();
}

void DynamicsRobot::ensureKinematicConstraints()
{

}

void DynamicsRobot::createDynamicsNode( VirtualRobot::RobotNodePtr node )
{
	VR_ASSERT(node);

	// check if already created
	if (hasDynamicsRobotNode(node))
		return;

	DynamicsWorldPtr dw = DynamicsWorld::GetWorld();
	DynamicsObjectPtr drn = dw->CreateDynamicsObject(node);
	VR_ASSERT(drn);
	dynamicRobotNodes[node] = drn;
}


std::vector<DynamicsObjectPtr> DynamicsRobot::getDynamicsRobotNodes()
{
	std::map<VirtualRobot::RobotNodePtr, DynamicsObjectPtr>::iterator it = dynamicRobotNodes.begin();
	std::vector<DynamicsObjectPtr> res;
	while (it != dynamicRobotNodes.end())
	{
		res.push_back(it->second);
		it++;
	}
	return res;
}

bool DynamicsRobot::hasDynamicsRobotNode( VirtualRobot::RobotNodePtr node )
{
	if (dynamicRobotNodes.find(node) == dynamicRobotNodes.end())
		return false;
	return true;
}

DynamicsObjectPtr DynamicsRobot::getDynamicsRobotNode( VirtualRobot::RobotNodePtr node )
{
	VR_ASSERT(node);
	if (dynamicRobotNodes.find(node) == dynamicRobotNodes.end())
		return DynamicsObjectPtr();
	return dynamicRobotNodes[node];
}


void DynamicsRobot::actuateNode( VirtualRobot::RobotNodePtr node, float jointValue )
{
	VR_ASSERT(robot);
	VR_ASSERT(node);
	VR_ASSERT(robot->hasRobotNode(node));

	if (!hasDynamicsRobotNode(node))
		createDynamicsNode(node);

	DynamicsObjectPtr dnyRN = getDynamicsRobotNode(node);

	robotNodeActuationTarget target;
	target.enabled = true;
	target.node = node;
	target.jointValueTarget = jointValue;
	target.dynNode = dnyRN;

	actuationTargets[node] = target;
}

void DynamicsRobot::disableNodeActuation( VirtualRobot::RobotNodePtr node )
{
	if (actuationTargets.find(node) != actuationTargets.end())
	{
		actuationTargets.erase(node);
	}
}
void DynamicsRobot::enableActuation()
{
	std::map<VirtualRobot::RobotNodePtr, robotNodeActuationTarget>::iterator it = actuationTargets.begin();
	while (it!=actuationTargets.end())
	{
		it->second.enabled = true;
		it++;
	}
}
void DynamicsRobot::disableActuation()
{
	std::map<VirtualRobot::RobotNodePtr, robotNodeActuationTarget>::iterator it = actuationTargets.begin();
	while (it!=actuationTargets.end())
	{
		it->second.enabled = false;
		it++;
	}
}
void DynamicsRobot::actuateJoints(float dt)
{

}

bool DynamicsRobot::isNodeActuated( VirtualRobot::RobotNodePtr node )
{
	VR_ASSERT(node);
	if (actuationTargets.find(node) == actuationTargets.end())
		return false;
	return actuationTargets[node].enabled;
}

float DynamicsRobot::getNodeTarget( VirtualRobot::RobotNodePtr node )
{
	VR_ASSERT(node);
	if (actuationTargets.find(node) == actuationTargets.end())
		return 0.0f;
	return actuationTargets[node].jointValueTarget;

}

float DynamicsRobot::getJointAngle( VirtualRobot::RobotNodePtr rn )
{
	return 0.0f;
}

float DynamicsRobot::getJointSpeed( VirtualRobot::RobotNodePtr rn )
{
	return 0.0f;
}

Eigen::Matrix4f DynamicsRobot::getComGlobal( VirtualRobot::RobotNodePtr rn )
{
    Eigen::Matrix4f com = Eigen::Matrix4f::Identity();
    com.block(0,3,3,1) = rn->getCoMLocal();
    com = rn->getGlobalPoseVisualization()*com;
    return com;
}

/*
void DynamicsRobot::setPose( const Eigen::Matrix4f &pose )
{
	robot->setGlobalPose(pose);
}

void DynamicsRobot::setPosition( const Eigen::Vector3f &posMM )
{
	Eigen::Matrix4f pose = robot->getGlobalPose();
	pose.block(0,3,3,1) = posMM;
	setPose(pose);
}*/


} // namespace SimDynamics
