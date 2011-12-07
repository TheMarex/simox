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

#include "IKRRTWindow.h"


int main(int argc, char *argv[])
{
	SoDB::init();
	SoQt::init(argc,argv,"IKRRT");
	cout << " --- START --- " << endl;

	std::string filenameScene(SIMOX_BASE_DIR "/VirtualRobot/data/scenes/IKRRT_scene_iCub.xml");
	std::string filenameReach(SIMOX_BASE_DIR "/VirtualRobot/data/reachability/iCub_HipLeftArm.bin");
    std::string kinChain("Hip Left Arm");
    std::string eef("Left Hand");
	std::string colModel("Left HandArm ColModel");
	std::string colModelRob("BodyHeadLegsColModel");

	VirtualRobot::RuntimeEnvironment::considerKey("scene");
	VirtualRobot::RuntimeEnvironment::considerKey("reachability");
	VirtualRobot::RuntimeEnvironment::considerKey("kinematicChain");
	VirtualRobot::RuntimeEnvironment::considerKey("endEffector");
	VirtualRobot::RuntimeEnvironment::considerKey("collisionModelKinChain");
	VirtualRobot::RuntimeEnvironment::considerKey("collisionModelRobot");
	VirtualRobot::RuntimeEnvironment::processCommandLine(argc,argv);
	VirtualRobot::RuntimeEnvironment::print();

	std::string scFile = VirtualRobot::RuntimeEnvironment::getValue("scene");
	if (!scFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(scFile))
		filenameScene = scFile;
	std::string reachFile = VirtualRobot::RuntimeEnvironment::getValue("reachability");
	if (!reachFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(reachFile))
		filenameReach = reachFile;
	std::string kc = VirtualRobot::RuntimeEnvironment::getValue("kinematicChain");
	if (!kc.empty())
		kinChain = kc;	
	std::string EndEff = VirtualRobot::RuntimeEnvironment::getValue("endEffector");
	if (!EndEff.empty())
		eef = EndEff;	
	std::string col1 = VirtualRobot::RuntimeEnvironment::getValue("collisionModelKinChain");
	if (!col1.empty())
		colModel = col1;
	std::string col2 = VirtualRobot::RuntimeEnvironment::getValue("collisionModelRobot");
	if (!col2.empty())
		colModelRob = col2;
	

	cout << "Using scene at " << filenameScene << endl;
	cout << "Using reachability at " << filenameReach << endl;
	cout << "Using end effector " << eef << endl;
	cout << "Using col model (kin chain) " << colModel << endl;
	cout << "Using col model (static robot)" << colModelRob << endl;

	IKRRTWindow rw(filenameScene,filenameReach,kinChain,eef,colModel,colModelRob);

	rw.main();

	return 0;

}
