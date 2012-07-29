/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    SimDynamics
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _SimDynamics_DynamicsEngine_h_
#define _SimDynamics_DynamicsEngine_h_

#include "../SimDynamics.h"
#include "DynamicsObject.h"
#include "DynamicsRobot.h"

namespace SimDynamics
{

/*!
	An interface class to encapsulates all calls to the underlying physics engine. 
	Usually there is no need to instantiate this object by your own, it is automatically created when calling DynamicsWorld::Init().
*/
class SIMDYNAMICS_IMPORT_EXPORT DynamicsEngine
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct DynamicsWorldInfo 
	{
		DynamicsWorldInfo()
		{
			gravity << 0, 0, -9.81f;
		}

		Eigen::Vector3f gravity;
	};


	/*!
	Constructor
	*/
	DynamicsEngine();

	/*!
	*/
	virtual ~DynamicsEngine();

	virtual bool init(const DynamicsWorldInfo &info);

	Eigen::Vector3f getGravity();

	virtual bool addObject(DynamicsObjectPtr o);
	virtual bool removeObject(DynamicsObjectPtr o);

	virtual bool addRobot(DynamicsRobotPtr r);
	virtual bool removeRobot(DynamicsRobotPtr r);

	std::vector<DynamicsRobotPtr> getRobots();

	/*!
		Returns all objects that are currently registered.
		Note, that all RobotNdoes are internally handled as DynamicObjects.
	*/
	std::vector<DynamicsObjectPtr> getObjects();

	/*!
		Set floor. A standard floor object is created and added to the dynamics world.
	*/
	virtual void createFloorPlane(const Eigen::Vector3f &pos, const Eigen::Vector3f &up);

	/*!
		By default all collisions are enabled.
		Here, collisions between two objects can be disabled.
	*/
	virtual void disableCollision(DynamicsObject* o1, DynamicsObject* o2);
	virtual void enableCollision(DynamicsObject* o1, DynamicsObject* o2);
	//! removes all <collision disabled> settings for object o
	virtual void resetCollisions(DynamicsObject* o);

	/*!
		Disables collision check to all objects.
	*/
	virtual void disableCollision(DynamicsObject* o1);
	/*!
		Enables collision check to all objects.
	*/
	virtual void enableCollision(DynamicsObject* o1);


	/*!
		Check if two objects should be considered for collision detection.
	*/
	virtual bool checkCollisionEnabled(DynamicsObject* o1, DynamicsObject* o2);
	/*!
		Check if object should be considered for collision detection.
	*/
	virtual bool checkCollisionEnabled(DynamicsObject* o1);

	DynamicsObjectPtr getFloor(){return floor;}

protected:
	DynamicsWorldInfo dynamicsInfo;

	std::vector<DynamicsObjectPtr> objects;
	std::vector<DynamicsRobotPtr> robots;

	std::map < DynamicsObject*, std::vector<DynamicsObject*> > collisionDisabled;
	std::vector<DynamicsObject*> collisionToAllDisabled;
	DynamicsObjectPtr floor;

	Eigen::Vector3f floorPos;
	Eigen::Vector3f floorUp;
};

typedef boost::shared_ptr<DynamicsEngine> DynamicsEnginePtr;


} // namespace SimDynamics

#endif // _SimDynamics_DynamicsEngine_h_
