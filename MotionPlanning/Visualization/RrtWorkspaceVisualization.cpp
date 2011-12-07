
#include "RrtWorkspaceVisualization.h"
#include "CSpace/CSpace.h"
#include "VirtualRobot/Nodes/RobotNode.h"

namespace Saba {

RrtWorkspaceVisualization::RrtWorkspaceVisualization(VirtualRobot::RobotPtr robot, CSpacePtr cspace, const std::string &TCPName)
{
	this->robot = robot;
	this->cspace = cspace;
	SABA_ASSERT(robot);
	SABA_ASSERT(cspace);
	robotNodeSet = cspace->getRobotNodeSet();
	SABA_ASSERT(robotNodeSet);
	setTCPName(TCPName);
}
	

RrtWorkspaceVisualization::RrtWorkspaceVisualization(VirtualRobot::RobotPtr robot, VirtualRobot::RobotNodeSetPtr robotNodeSet, const std::string &TCPName)
{
	this->robot = robot;
	cspace = CSpacePtr();
	SABA_ASSERT(robot);
	SABA_ASSERT(robotNodeSet);
	this->robotNodeSet = robotNodeSet;
	setTCPName(TCPName);
}
	
RrtWorkspaceVisualization::~RrtWorkspaceVisualization()
{
}
/*
bool RrtWorkspaceVisualization::addCSpacePath(CSpacePathPtr path)
{
	return false;
}

bool RrtWorkspaceVisualization::addTree(CSpaceTreePtr tree)
{
	return false;
}*/

void RrtWorkspaceVisualization::reset()
{
}

void RrtWorkspaceVisualization::setTCPName(const std::string TCPName)
{
	this->TCPName = TCPName;
	TCPNode = robot->getRobotNode(TCPName);
	if (!TCPNode)
	{
		VR_ERROR << "No node with name " << TCPName << " available in robot.." << endl;
	}
}
/*
bool RrtWorkspaceVisualization::addConfig( const Eigen::VectorXf &c )
{
	return false;
}*/

} // namespace Saba
