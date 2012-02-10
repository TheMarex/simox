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
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "reachabilityWindow.h"

#define ICUB
//#define AXIS_X

using std::cout;
using std::endl;
using namespace VirtualRobot;
int main(int argc, char *argv[])
{
	SoDB::init();
	SoQt::init(argc,argv,"showRobot");
	cout << " --- START --- " << endl;

	std::string filenameReach;
#ifdef ICUB
		std::string filenameRob(VR_BASE_DIR "/data/robots/iCub/iCub.xml");
		Eigen::Vector3f axisTCP(1.0f,0,0);
		filenameReach = "reachability/iCub_HipLeftArm.bin";
#else
	std::string filenameRob(VR_BASE_DIR "/data/robots/ArmarIII/ArmarIII.xml");
	Eigen::Vector3f axisTCP(0,0,1.0f);	
#endif


	VirtualRobot::RuntimeEnvironment::considerKey("robot");
	VirtualRobot::RuntimeEnvironment::considerKey("reachability");
	VirtualRobot::RuntimeEnvironment::considerKey("visualizationTCPAxis");
	VirtualRobot::RuntimeEnvironment::processCommandLine(argc,argv);
	VirtualRobot::RuntimeEnvironment::print();

	cout << " --- START --- " << endl;

	filenameRob = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("robot",filenameRob);

	filenameReach = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("reachability",filenameReach);

	if (VirtualRobot::RuntimeEnvironment::hasValue("visualizationTCPAxis"))
	{
		std::string axisStr = VirtualRobot::RuntimeEnvironment::getValue("visualizationTCPAxis");
		if (!VirtualRobot::RuntimeEnvironment::toVector3f(axisStr,axisTCP))
		{
			cout << "Wrong axis definition:" << axisStr << endl;
		}
	}
	
	cout << "Using robot at " << filenameRob << endl;
	cout << "Using reachability file from " << filenameReach << endl;


	reachabilityWindow rw(filenameRob,filenameReach,axisTCP);

	rw.main();

	return 0;

}
