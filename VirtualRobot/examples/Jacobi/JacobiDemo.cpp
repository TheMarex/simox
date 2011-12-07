#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>



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

#include "JacobiWindow.h"

bool useColModel = false;


int main(int argc, char *argv[])
{
	SoDB::init();
	SoQt::init(argc,argv,"Ik demo");
	cout << " --- START --- " << endl;
	std::string filename(VR_BASE_DIR "/data/robots/ArmarIII/ArmarIII.xml");

	JacobiWindow rw(filename);

	rw.main();

	return 0;
}
