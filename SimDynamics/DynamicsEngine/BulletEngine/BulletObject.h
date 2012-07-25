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
#ifndef _SimDynamics_BulletObject_h_
#define _SimDynamics_BulletObject_h_

#include "../DynamicsObject.h"
#include "SimoxMotionState.h"

#include <btBulletDynamicsCommon.h>

namespace SimDynamics
{
class SIMDYNAMICS_IMPORT_EXPORT BulletObject : public DynamicsObject
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
		Constructor
	*/
	BulletObject(VirtualRobot::SceneObjectPtr o, SimulationType type = eDynamic);

	/*!
	*/
	virtual ~BulletObject();

	
	boost::shared_ptr<btRigidBody> getRigidBody();

	
	/*!
		Set world position [MM].
	*/
	virtual void setPosition(const Eigen::Vector3f &posMM);

	/*!
		Set world pose [mm].
	*/
	virtual void setPose(const Eigen::Matrix4f &pose);

	Eigen::Vector3f getCom(){return com;}
	
protected:

	btConvexHullShape* createConvexHullShape(VirtualRobot::TriMeshModelPtr trimesh);

	boost::shared_ptr<btRigidBody> rigidBody;
	boost::shared_ptr<btCollisionShape> collisionShape; // bullet collision shape
	
	Eigen::Vector3f com; // com offset of trimesh

	btScalar btMargin;
	SimoxMotionState* motionState;

};

typedef boost::shared_ptr<BulletObject> BulletObjectPtr;

} // namespace SimDynamics

#endif // _SimDynamics_BulletObject_h_
