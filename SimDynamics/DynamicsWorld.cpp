
#include "DynamicsWorld.h"
#include "DynamicsEngine/DynamicsEngineFactory.h"
#include <boost/thread.hpp> 

namespace SimDynamics
{


namespace 
{ 
	boost::mutex mutex; 
};

DynamicsWorldPtr DynamicsWorld::world;

bool DynamicsWorld::convertMM2M = true;

DynamicsWorld::Cleanup::~Cleanup()
{
	boost::lock_guard<boost::mutex> lock(mutex); 
	DynamicsWorld::world.reset();
}


DynamicsWorldPtr DynamicsWorld::GetWorld()
{
	if (!world)
		Init();
	return world;
}


DynamicsEnginePtr DynamicsWorld::getEngine()
{
	return engine;
}

DynamicsWorldPtr DynamicsWorld::Init()
{
	static Cleanup _Cleanup;

	if (true)
	{
		boost::lock_guard<boost::mutex> lock(mutex); 

		if (!world)
		{
			world.reset(new DynamicsWorld());
		}
	}
	return world;
}



void DynamicsWorld::Close()
{
	world.reset();
}


DynamicsWorld::DynamicsWorld()
{
	DynamicsEngineFactoryPtr factory = DynamicsEngineFactory::first(NULL);
	THROW_VR_EXCEPTION_IF(!factory, "No Physics Engine Found. Re-Compile with engine support...");
	engine = factory->createEngine();
	THROW_VR_EXCEPTION_IF(!engine, "Could not create Physics Engine.");
}        

DynamicsWorld::~DynamicsWorld()
{
}

bool DynamicsWorld::addObject( DynamicsObjectPtr o )
{
	return engine->addObject(o);
}

bool DynamicsWorld::removeObject( DynamicsObjectPtr o )
{
	return engine->removeObject(o);
}

DynamicsObjectPtr DynamicsWorld::CreateDynamicsObject( VirtualRobot::SceneObjectPtr o, DynamicsObject::SimulationType simType )
{
	SIMDYNAMICS_ASSERT(o);

	DynamicsEngineFactoryPtr factory = DynamicsEngineFactory::first(NULL);
	SIMDYNAMICS_ASSERT(factory);

	return factory->createObject(o,simType);
}

void DynamicsWorld::createFloorPlane( const Eigen::Vector3f &pos /*= Eigen::Vector3f(0,0,0)*/, const Eigen::Vector3f &up /*= Eigen::Vector3f(0,0,1.0f)*/ )
{
	engine->createFloorPlane(pos,up);
}

bool DynamicsWorld::addRobot( DynamicsRobotPtr r )
{
	return engine->addRobot(r);
}

bool DynamicsWorld::removeRobot( DynamicsRobotPtr r )
{
	return engine->removeRobot(r);
}

SimDynamics::DynamicsRobotPtr DynamicsWorld::CreateDynamicsRobot( VirtualRobot::RobotPtr rob )
{
	SIMDYNAMICS_ASSERT(rob);

	DynamicsEngineFactoryPtr factory = DynamicsEngineFactory::first(NULL);
	SIMDYNAMICS_ASSERT(factory);

	return factory->createRobot(rob);
}

std::vector<DynamicsRobotPtr> DynamicsWorld::getRobots()
{
	return engine->getRobots();
}

} // namespace
