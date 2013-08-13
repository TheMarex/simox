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
#ifndef _SimDynamics_BulletEngine_h_
#define _SimDynamics_BulletEngine_h_

#include <VirtualRobot/SceneObject.h>
#include "../DynamicsEngine.h"
#include "BulletRobot.h"

#include "btBulletDynamicsCommon.h"
#include <boost/enable_shared_from_this.hpp>

namespace SimDynamics
{

	class BulletEngineConfig : public DynamicsEngineConfig 
	{
	public:
		BulletEngineConfig() : DynamicsEngineConfig()
		{
			bulletObjectRestitution = btScalar(0.2);
			bulletObjectFriction = btScalar(0.5f);
			bulletObjectDampingLinear = btScalar(0.05f);
			//bulletObjectDampingAngular = btScalar(0.85f);
			bulletObjectDampingAngular = btScalar(0.1f);
			bulletObjectDeactivation = btScalar(5.0);//1.0);
			bulletObjectSleepingThresholdLinear = btScalar(0.05f);//1.5);
			bulletObjectSleepingThresholdAngular = btScalar(0.05f);//2.5);

			bulletSolverIterations = 100;
			bulletSolverGlobalContactForceMixing = 0;
			bulletSolverGlobalErrorReductionParameter = btScalar(0.5);//0.1);
		}

		virtual ~BulletEngineConfig(){}

		// global setup values
		btScalar bulletObjectRestitution;
		btScalar bulletObjectFriction;
		btScalar bulletObjectDampingLinear;
		btScalar bulletObjectDampingAngular;
		btScalar bulletObjectSleepingThresholdLinear;
		btScalar bulletObjectSleepingThresholdAngular;
		btScalar bulletObjectDeactivation;
		int bulletSolverIterations;
		btScalar bulletSolverGlobalContactForceMixing; // allow to violate constraints (eg joint limits). A value>0 may increase stablity. (standard:0)
		btScalar bulletSolverGlobalErrorReductionParameter; // How hard should the solver try to correct misaligned joints/constraints/links. (standard 0.2)
	};

	typedef boost::shared_ptr<BulletEngineConfig> BulletEngineConfigPtr;

/*!
	This class encapsulates all calls to the bullet physics engine. 
	Usually there is no need to instantiate this object by your own, it is automatically created when calling DynamicsWorld::Init().
*/
class SIMDYNAMICS_IMPORT_EXPORT BulletEngine : public DynamicsEngine, public boost::enable_shared_from_this<BulletEngine>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	friend class BulletObject;

	/*!
	Constructor
	*/
	BulletEngine();

	/*!
	*/
	virtual ~BulletEngine();

	virtual bool addObject(DynamicsObjectPtr o);
	virtual bool removeObject(DynamicsObjectPtr o);

	virtual bool addRobot(DynamicsRobotPtr r);
	virtual bool removeRobot(DynamicsRobotPtr r);

	/*!
		Initialize the engine with this configuration.
		\param config Either a standard init (could be NULL), or if config is of type BulletEngineConfig, Bullet specific parameters will be considered.
	*/
	virtual bool init(DynamicsEngineConfigPtr config);
	virtual bool init(BulletEngineConfigPtr config);

	virtual bool cleanup();

	/*!
		Set floor
	*/
	virtual void createFloorPlane(const Eigen::Vector3f &pos, const Eigen::Vector3f &up);


    virtual void stepSimulation(float dt, int subSteps);

	btDynamicsWorld* getBulletWorld();

	/*!
		Wake up all objects. 
		Bullet sends objects to sleeping state when no interaction is detected. 
		But this might be unwanted, e.g. robots should be active all the time.
	*/
	void activateAllObjects();


	virtual std::vector<DynamicsEngine::DynamicsContactInfo> getContacts();

	void print();

	/*!
		Transforms pose to bullet.
		Translation is scaled from mm to m.
	*/
	static btTransform getPoseBullet( const Eigen::Matrix4f &pose, bool scaling = true );
	static Eigen::Matrix4f getPoseEigen( const btTransform &pose, bool scaling = true );
	static btVector3 getVecBullet( const Eigen::Vector3f &vec, bool scaling = true );
	static Eigen::Vector3f getVecEigen( const btVector3 &vec, bool scaling = true );
    static btMatrix3x3 getRotMatrix(const Eigen::Matrix4f &pose);
    static Eigen::Matrix4f getRotMatrix(const btMatrix3x3 &pose);
protected:

	class CustomCollisionCallback : public btOverlapFilterCallback
	{
	public:
		CustomCollisionCallback(BulletEngine* e)
		{
			engine = e;
		}
		virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
		{
			VR_ASSERT(engine);
			VR_ASSERT(static_cast<btCollisionObject*>(proxy0->m_clientObject));
			VR_ASSERT(static_cast<btCollisionObject*>(proxy1->m_clientObject));
			btCollisionObject* bt0 = static_cast<btCollisionObject*>(proxy0->m_clientObject);
			btCollisionObject* bt1 = static_cast<btCollisionObject*>(proxy1->m_clientObject);
			SimDynamics::BulletObject* o0 = static_cast<SimDynamics::BulletObject*>(bt0->getUserPointer());
			SimDynamics::BulletObject* o1 = static_cast<SimDynamics::BulletObject*>(bt1->getUserPointer());
			return engine->checkCollisionEnabled(o0,o1);
			//return true;//btOverlapFilterCallback::needBroadphaseCollision(proxy0,proxy1);
		}
	protected:
		BulletEngine* engine;
	};


	virtual bool addLink(BulletRobot::LinkInfo &l);
	virtual bool removeLink(BulletRobot::LinkInfo &l);
    
	btDynamicsWorld *dynamicsWorld;

	btBroadphaseInterface* overlappingPairCache;
	btConstraintSolver* constraintSolver;
	btCollisionDispatcher* dispatcher;
	btDefaultCollisionConfiguration * collision_config;

	btOverlapFilterCallback * collisionFilterCallback;

	VirtualRobot::ObstaclePtr groundObject;

	BulletEngineConfigPtr bulletConfig;
};

typedef boost::shared_ptr<BulletEngine> BulletEnginePtr;

} // namespace SimDynamics

#endif // _SimDynamics_BulletEngine_h_
