#include "DynamicsEngine.h"

#include <algorithm>

namespace SimDynamics {

DynamicsEngine::DynamicsEngine()
{
	floorPos.setZero();
	floorUp.setZero();
}
	
DynamicsEngine::~DynamicsEngine()
{
	robots.clear();
	objects.clear();
}

Eigen::Vector3f DynamicsEngine::getGravity()
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	return dynamicsConfig->gravity;
}

bool DynamicsEngine::init( DynamicsEngineConfigPtr config )
{
	if (config)
		dynamicsConfig = config;
	else
		dynamicsConfig.reset(new DynamicsEngineConfig());
	return true;
}

bool DynamicsEngine::addObject( DynamicsObjectPtr o )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	if (find(objects.begin(), objects.end(), o) == objects.end())
	{
		objects.push_back(o);
	}
	return true;
}

bool DynamicsEngine::removeObject( DynamicsObjectPtr o )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	std::vector<DynamicsObjectPtr>::iterator it = find(objects.begin(), objects.end(), o);
	if (it == objects.end())
		return false;

	objects.erase(it);
	return true;
}

void DynamicsEngine::createFloorPlane( const Eigen::Vector3f &pos, const Eigen::Vector3f &up )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	floorPos = pos;
	floorUp = up;
}

bool DynamicsEngine::addRobot( DynamicsRobotPtr r )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	if (find(robots.begin(), robots.end(), r) == robots.end())
	{
        for (size_t i=0;i<robots.size();i++)
        {
            if (robots[i]->getRobot() == r->getRobot())
            {
                VR_ERROR << "Only one DynamicsWrapper per robot allowed. Robot " << r->getName() << endl;
                return false;
            }
        }
		robots.push_back(r);
	}
	return true;
}

bool DynamicsEngine::removeRobot( DynamicsRobotPtr r )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	std::vector<DynamicsRobotPtr>::iterator it = find(robots.begin(), robots.end(), r);
	if (it == robots.end())
		return false;

	robots.erase(it);
	return true;
}

void DynamicsEngine::disableCollision( DynamicsObject* o1, DynamicsObject* o2 )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	std::vector<DynamicsObject*>::iterator i1 = find(collisionDisabled[o1].begin(),collisionDisabled[o1].end(),o2);
	if (i1==collisionDisabled[o1].end())
	{
		collisionDisabled[o1].push_back(o2);
	}
	std::vector<DynamicsObject*>::iterator i2 = find(collisionDisabled[o2].begin(),collisionDisabled[o2].end(),o1);
	if (i2==collisionDisabled[o2].end())
	{
		collisionDisabled[o2].push_back(o1);
	}
}

void DynamicsEngine::disableCollision( DynamicsObject* o1 )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	std::vector<DynamicsObject*>::iterator i1 = find(collisionToAllDisabled.begin(),collisionToAllDisabled.end(),o1);
	if (i1 == collisionToAllDisabled.end())
		collisionToAllDisabled.push_back(o1);
}

void DynamicsEngine::enableCollision( DynamicsObject* o1, DynamicsObject* o2 )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	//if (o1 < o2)
	//{
		std::vector<DynamicsObject*>::iterator i1 = find(collisionDisabled[o1].begin(),collisionDisabled[o1].end(),o2);
		if (i1!=collisionDisabled[o1].end())
		{
			collisionDisabled[o1].erase(i1);
		}
	//} else
	//{
		std::vector<DynamicsObject*>::iterator i2 = find(collisionDisabled[o2].begin(),collisionDisabled[o2].end(),o1);
		if (i2!=collisionDisabled[o2].end())
		{
			collisionDisabled[o2].erase(i2);
		}
	//}
}

void DynamicsEngine::enableCollision( DynamicsObject* o1 )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	std::vector<DynamicsObject*>::iterator i1 = find(collisionToAllDisabled.begin(),collisionToAllDisabled.end(),o1);
	if (i1 != collisionToAllDisabled.end())
		collisionToAllDisabled.erase(i1);
}

bool DynamicsEngine::checkCollisionEnabled( DynamicsObject* o1 )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	std::vector<DynamicsObject*>::iterator i1 = find(collisionToAllDisabled.begin(),collisionToAllDisabled.end(),o1);
	return (i1 == collisionToAllDisabled.end());
}

bool DynamicsEngine::checkCollisionEnabled( DynamicsObject* o1, DynamicsObject* o2 )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	if (!checkCollisionEnabled(o1))
		return false;
	if (!checkCollisionEnabled(o2))
		return false;
	//if (o1 < o2)
	//{
		std::vector<DynamicsObject*>::iterator i1 = find(collisionDisabled[o1].begin(),collisionDisabled[o1].end(),o2);
		if (i1!=collisionDisabled[o1].end())
			return false;

	//} else
	//{
		std::vector<DynamicsObject*>::iterator i2 = find(collisionDisabled[o2].begin(),collisionDisabled[o2].end(),o1);
		if (i2!=collisionDisabled[o2].end())
			return false;
	//}
	return true;
}

void DynamicsEngine::resetCollisions( DynamicsObject* o )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	collisionDisabled.erase(o);
	std::map < DynamicsObject*, std::vector<DynamicsObject*> >::iterator i1 = collisionDisabled.begin();
	while (i1 != collisionDisabled.end())
	{
		std::vector<DynamicsObject*>::iterator i2 = find(i1->second.begin(),i1->second.end(),o);
		if (i2!=i1->second.end())
		{
			i1->second.erase(i2);
		}
		i1++;
	}
	enableCollision(o);
}

std::vector<DynamicsRobotPtr> DynamicsEngine::getRobots()
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	return robots;
}

std::vector<DynamicsObjectPtr> DynamicsEngine::getObjects()
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	return objects;
}

std::vector<DynamicsEngine::DynamicsContactInfo> DynamicsEngine::getContacts()
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	std::vector<DynamicsEngine::DynamicsContactInfo> result;
	return result;
}

SimDynamics::DynamicsRobotPtr DynamicsEngine::getRobot( VirtualRobot::RobotPtr r )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
    for (size_t i=0;i<robots.size();i++)
    {
        if (robots[i]->getRobot() == r)
        {
            return robots[i];
        }
    }
    return DynamicsRobotPtr();
}

} // namespace SimDynamics
