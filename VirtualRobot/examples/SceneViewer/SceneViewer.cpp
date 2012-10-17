#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/RuntimeEnvironment.h>



#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/Qt/SoQt.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "showSceneWindow.h"


int main(int argc, char *argv[])
{
	SoDB::init();
	SoQt::init(argc,argv,"showRobot");
	cout << " --- START --- " << endl;
    std::string filename("scenes/examples/SceneViewer/scene1.xml");
	VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);
	VirtualRobot::RuntimeEnvironment::considerKey("scene");
	VirtualRobot::RuntimeEnvironment::processCommandLine(argc,argv);
	VirtualRobot::RuntimeEnvironment::print();

	filename = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("scene",filename);
	showSceneWindow rw(filename);

	rw.main();

	return 0;

}
