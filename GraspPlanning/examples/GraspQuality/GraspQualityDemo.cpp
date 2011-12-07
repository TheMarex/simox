#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "GraspQualityWindow.h"


int main(int argc, char *argv[])
{
	SoDB::init();
	SoQt::init(argc,argv,"IKRRT");
	cout << " --- START --- " << endl;

	//std::string filename1(VR_BASE_DIR "/data/objects/plate.xml");
	std::string robot(SIMOX_BASE_DIR "/VirtualRobot/data/robots/ArmarIII/ArmarIII.xml");
	//std::string object(SIMOX_BASE_DIR "/VirtualRobot/data/objects/wok.xml");
	std::string object(SIMOX_BASE_DIR "/VirtualRobot/data/objects/riceBox.xml");

	VirtualRobot::RuntimeEnvironment::considerKey("robot");
	VirtualRobot::RuntimeEnvironment::considerKey("object");
	VirtualRobot::RuntimeEnvironment::processCommandLine(argc,argv);
	VirtualRobot::RuntimeEnvironment::print();

	std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");
	if (!robFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
		robot = robFile;

	std::string objFile = VirtualRobot::RuntimeEnvironment::getValue("object");
	if (!objFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(objFile))
		object = objFile;


	cout << "Using robot from " << robot << endl;
	cout << "Using object from " << object << endl;

	GraspQualityWindow rw(robot,object);

	rw.main();

	return 0;
}
