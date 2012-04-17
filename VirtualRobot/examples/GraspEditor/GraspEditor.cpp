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

#include "GraspEditorWindow.h"


int main(int argc, char *argv[])
{
	SoDB::init();
	SoQt::init(argc,argv,"GraspEditor");
	cout << " --- START --- " << endl;

	std::string filename1(VR_BASE_DIR "/data/objects/plate.xml");
	std::string filename2(VR_BASE_DIR "/data/robots/ArmarIII/ArmarIII.xml");
#if 1
	filename1 = VR_BASE_DIR "/data/objects/iCub/LegoXWing_Righthand_200.xml";
	filename2 = VR_BASE_DIR "/data/robots/iCub/iCub.xml";
#endif

	GraspEditorWindow rw(filename1,filename2);

	rw.main();

	return 0;

}
