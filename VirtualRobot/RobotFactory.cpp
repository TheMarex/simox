
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
	RobotPtr result(new LocalRobot(name));
	return result;

}

bool RobotFactory::initializeRobot( RobotPtr robot, 
									std::vector<RobotNodePtr > &robotNodes, 
									std::map< RobotNodePtr, std::vector<std::string> > childrenMap,
									RobotNodePtr rootNode
									)
{
	VR_ASSERT(robot);
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

	// process children
	std::map< RobotNodePtr, std::vector< std::string > >::iterator iterC = childrenMap.begin();
	while (iterC!= childrenMap.end())
	{
		std::vector< std::string > childNames = iterC->second;
		RobotNodePtr node = iterC->first;
		for (size_t i=0;i<childNames.size();i++)
		{
			std::string childName = childNames[i];
			if (!robot->hasRobotNode(childName))
			{
				THROW_VR_EXCEPTION("Robot " << robot->getName() << ": corrupted RobotNode <" << node->getName() << " child :" << childName << " does not exist...");
			}
			RobotNodePtr c = robot->getRobotNode(childName);
			node->attachChild(c);
		}
		iterC++;
	}

	// register root (performs an initialization of all robot nodes)
	robot->setRootNode(rootNode);

	for (size_t i=0;i<robotNodes.size();i++)
	{
		if (!robotNodes[i]->getParent() && robotNodes[i]!=rootNode)
		{
			VR_ERROR << "RobotNode " << robotNodes[i]->getName() << " is not connected to kinematic structure..." << endl;
		}
	}

	return result;
}

/*
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
	std::vector< SceneObjectPtr > children = n->children;
	for (unsigned int i=0; i<children.size(); i++)
	{
		if (!initRobotNode(boost::dynamic_pointer_cast<RobotNode>(children[i]),n,robotNodes))
		{
			VR_ERROR << "Could not initialize node " << children[i]->getName() << endl;
			result = false;
		}
	}
	return result;
}*/

} // namespace VirtualRobot
