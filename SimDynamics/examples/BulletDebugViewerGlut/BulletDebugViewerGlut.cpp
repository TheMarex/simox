
#include "DynamicsEngine/BulletEngine/BulletOpenGLViewer.h"
#include <SimDynamics/DynamicsWorld.h>
#include <SimDynamics/DynamicsEngine/BulletEngine/BulletEngineFactory.h>

#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/RuntimeEnvironment.h>

using namespace std;
using namespace SimDynamics;

int main(int argc,char* argv[])
{
	SimDynamics::DynamicsWorldPtr world = SimDynamics::DynamicsWorld::Init();
	SIMDYNAMICS_ASSERT(world);

	world->createFloorPlane();

	/*VirtualRobot::ObstaclePtr o = VirtualRobot::Obstacle::createBox(100.0f,100.0f,100.0f);
	o->setMass(1.0f); // 1kg

	SimDynamics::DynamicsObjectPtr dynObj = world->CreateDynamicsObject(o);
	dynObj->setPosition(Eigen::Vector3f(3000,3000,1000.0f));
	world->addObject(dynObj);*/


	//std::string robFile("robots/examples/SimpleRobot/Joint3DH.xml");
	///std::string robFile("robots/iCub/iCub.xml");
	//std::string robFile("robots/iCub/iCub.xml");
	//std::string robFile("robots/ArmarIII/ArmarIII-RightArm.xml");
	//std::string robFile("robots/ArmarIII/ArmarIII-RightHand.xml");
	//std::string robFile("robots/ArmarIII/ArmarIII-RightArmTest2.xml");
	std::string robFile("robots/ArmarIII/ArmarIII.xml");
	//std::string robFile("robots/iCub/iCub_RightHand.xml");
	//std::string robFile("robots/iCub/iCub_testFinger.xml");
	VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile);
	VirtualRobot::RobotPtr robot = VirtualRobot::RobotIO::loadRobot(robFile);
	if (robot)
	{
		Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
		gp(2,3) = 20.0f;
		robot->setGlobalPose(gp);
		DynamicsRobotPtr dynRob = world->CreateDynamicsRobot(robot);
		world->addRobot(dynRob);

	}
	BulletOpenGLViewer demoApp(world);

	return glutmain(argc, argv,640,480,"Show SimDynamics Scene",&demoApp);
}
