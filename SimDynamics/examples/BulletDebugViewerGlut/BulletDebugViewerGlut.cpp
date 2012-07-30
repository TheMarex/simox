
#include "DynamicsEngine/BulletEngine/BulletOpenGLViewer.h"
#include <SimDynamics/DynamicsWorld.h>
#include <SimDynamics/DynamicsEngine/BulletEngine/BulletEngineFactory.h>

#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/RuntimeEnvironment.h>

using namespace std;
using namespace VirtualRobot;
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


	std::string robFile("robots/examples/SimpleRobot/Joint3DH.xml");
	//std::string robFile("robots/iCub/iCub.xml");
	//std::string robFile("robots/iCub/iCub_LeftLegTest.xml");
	//std::string robFile("robots/ArmarIII/ArmarIII-RightArm.xml");
	//std::string robFile("robots/ArmarIII/ArmarIII-RightHandTest.xml");
	//std::string robFile("robots/ArmarIII/ArmarIII-HeadTest.xml");
	//std::string robFile("robots/ArmarIII/ArmarIII-RightArmTest2.xml");
	//std::string robFile("robots/ArmarIII/ArmarIII.xml");
	//std::string robFile("robots/iCub/iCub_RightHand.xml");
	//std::string robFile("robots/iCub/iCub_testFinger.xml");
	VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile);
	VirtualRobot::RobotPtr robot = VirtualRobot::RobotIO::loadRobot(robFile);
	if (robot)
	{
		Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
		//gp(2,3) = 35.0f;
		gp(2,3) = 400.0f;
		robot->setGlobalPose(gp);
		DynamicsRobotPtr dynRob = world->CreateDynamicsRobot(robot);
		dynRob->disableActuation();
		world->addRobot(dynRob);
	}
	BulletOpenGLViewer viewer(world);
	viewer.enableContraintsDebugDrawing();

#if 1
	cout << "TEST7" << endl;
	ObstaclePtr o = Obstacle::createBox(10,10,1500);
	DynamicsObjectPtr do1 = DynamicsWorld::GetWorld()->CreateDynamicsObject(o,DynamicsObject::eStatic);
	ObstaclePtr o2 = Obstacle::createBox(10,10,1000);
	Eigen::Matrix4f gpxy = Eigen::Matrix4f::Identity();
	//gpxy(1,3) -= 213.0f;
	gpxy(0,3) += 3000.0f;
	o2->setGlobalPose(gpxy);
	DynamicsObjectPtr do2 = DynamicsWorld::GetWorld()->CreateDynamicsObject(o2,DynamicsObject::eStatic);
	DynamicsEnginePtr e = DynamicsWorld::GetWorld()->getEngine();
	e->disableCollision(do1.get());	
	e->disableCollision(do2.get());
	/*
	std::vector<DynamicsObjectPtr> dos = e->getObjects();
	for (size_t i=0;i<dos.size();i++)
	{
		e->disableCollision(do1.get(),dos[i].get());
		e->disableCollision(do2.get(),dos[i].get());
		if (e->checkCollisionEnabled(do1.get(),dos[i].get()))
		{
			cout << "OOPS" << endl;
		}
		if (e->checkCollisionEnabled(do2.get(),dos[i].get()))
		{
			cout << "OOPS" << endl;
		}
	}*/
	e->addObject(do1);
	e->addObject(do2);
#endif

	return glutmain(argc, argv,640,480,"Show SimDynamics Scene",&viewer);
}
