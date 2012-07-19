#include "stabilityWindow.h"
#include <VirtualRobot/RuntimeEnvironment.h>

using namespace VirtualRobot;

int main(int argc, char *argv[])
{
	SoDB::init();
	SoQt::init(argc,argv,"stability demo");
	cout << " --- START --- " << endl;
	//std::string filenameRob("robots/ArmarIII/ArmarIII.xml");
	std::string filenameRob("robots/iCub/iCub.xml");
	VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filenameRob);
	
	VirtualRobot::RuntimeEnvironment::considerKey("robot");
	VirtualRobot::RuntimeEnvironment::processCommandLine(argc,argv);
	VirtualRobot::RuntimeEnvironment::print();

	cout << " --- START --- " << endl;

	if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
	{
		std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");
		if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
		{
			filenameRob = robFile;
		}
	}
	
	cout << "Using robot at " << filenameRob << endl;

	stabilityWindow rw(filenameRob);
	rw.main();

	return 0;

}
