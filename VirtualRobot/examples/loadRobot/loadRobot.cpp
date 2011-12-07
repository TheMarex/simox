#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Nodes/RobotNodeRevoluteFactory.h>
#include <VirtualRobot/Transformation/DHParameter.h>
#include <VirtualRobot/XML/RobotIO.h>

#include <boost/shared_ptr.hpp>
#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;


int main(int argc, char *argv[])
{
	cout << "test" << endl;

	boost::shared_ptr<Robot> robot = RobotFactory::createRobot("Robbi");
	std::vector< boost::shared_ptr<RobotNode> > robotNodes;
	const std::vector<std::string> childrenNames;
	VirtualRobot::RobotNodeRevoluteFactory revoluteNodeFactory;
	DHParameter dhParameter(0, 0, 0, 0, true);
	boost::shared_ptr<RobotNode> node1 = revoluteNodeFactory.createRobotNodeDH(robot, "RootNode", childrenNames, VisualizationNodePtr(), CollisionModelPtr(), (float)-M_PI, (float)M_PI, 0.0f, dhParameter);
	robotNodes.push_back(node1);
	bool resInit = RobotFactory::initializeRobot(robot,robotNodes,node1);

	cout << "resInit:" << resInit << endl;
	cout << "First robot:" << endl;
	robot->print();


	std::string filename(VR_BASE_DIR "/examples/loadRobot/RobotExample.xml");
	RobotPtr rob;
	try
	{
		rob = RobotIO::loadRobot(filename,RobotIO::eStructure);
	} catch (VirtualRobotException &e)
	{
		cout << "Error: " << e.what() << endl;
		return -1;
	}
	
	cout << "Second robot (XML):" << endl;
	if (rob)
		rob->print();
	else 
		cout << " ERROR while creating robot" << endl;
}
