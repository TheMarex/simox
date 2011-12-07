
#include "RobotFactory.h"
#include "Robot.h"
#include "Nodes/RobotNode.h"
#include "Nodes/RobotNodeRevolute.h"
#include "Nodes/RobotNodePrismatic.h"
#include "Nodes/RobotNodeFixed.h"
#include "Visualization//VisualizationFactory.h"
#include "VirtualRobotException.h"

#include <boost/pointer_cast.hpp>
#include <algorithm>

namespace VirtualRobot {


RobotFactory::RobotFactory()
{
}

RobotFactory::~RobotFactory()
{
}


RobotPtr RobotFactory::createRobot( const std::string &name )
{
	RobotPtr result(new Robot(name));
	return result;

}

bool RobotFactory::initializeRobot( RobotPtr robot, 
									std::vector<RobotNodePtr > &robotNodes, 
									RobotNodePtr rootNode )
{
	bool result = true;

	// check for root
	std::vector<RobotNodePtr >::iterator iter = robotNodes.begin();
	bool foundRoot = false;
	while (iter != robotNodes.end())
	{
		if ((*iter) == rootNode)
			foundRoot=true;
		if ((*iter)->getRobot() != robot)
		{
			THROW_VR_EXCEPTION("Invalid robot node (robot is not set correctly)");
		}
		iter++;
	}
	THROW_VR_EXCEPTION_IF(!foundRoot, "Invalid robot node (root is not available)");

	// initialize robotNodes
	if (!initRobotNode(rootNode,RobotNodePtr(),robotNodes))
	{
		VR_ERROR << "Error while initializing RobotNodes" << endl;
		result = false;
	}
	if (robotNodes.size()!=0)
	{
		VR_ERROR << "There are " << robotNodes.size() << " RobotNodes without parent:" << endl;
		for (unsigned int i=0;i<robotNodes.size();i++)
		{
			cout << "-> " << robotNodes[i]->getName() << endl;
			//really?
			//robot->deregisterRobotNode(robotNodes[i]);
		}
	}

	// register root:
	robot->setRootNode(rootNode);

	return result;
}


bool RobotFactory::initRobotNode(RobotNodePtr n, RobotNodePtr parent, std::vector< RobotNodePtr > &robotNodes)
{
	std::vector< RobotNodePtr >::iterator nodeIter = std::find(robotNodes.begin(), robotNodes.end(), n);

	THROW_VR_EXCEPTION_IF((robotNodes.end() == nodeIter), "Searching child node: no node with name <" << n->getName() << "> found...");
	robotNodes.erase(nodeIter);

	if (!n->initialize(parent))
	{
		THROW_VR_EXCEPTION("Robot Node <" << n->getName() << "> could not be initialized...");
	}

	bool result = true;
	// now the children
	std::vector< RobotNodePtr > children = n->children;
	for (unsigned int i=0; i<children.size(); i++)
	{
		if (!initRobotNode(children[i],n,robotNodes))
		{
			VR_ERROR << "Could not initialize node " << children[i]->getName() << endl;
			result = false;
		}
	}
	return result;
}

} // namespace VirtualRobot
