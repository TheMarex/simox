#include "BulletEngine.h"
#include "BulletObject.h"
#include "SimoxCollisionDispatcher.h"
#include "../../DynamicsWorld.h"
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/MathTools.h>

//#define DEBUG_FIXED_OBJECTS

namespace SimDynamics {

BulletEngine::BulletEngine()
{
	collision_config = NULL;
	dispatcher = NULL;
	overlappingPairCache = NULL;
	constraintSolver = NULL;
	dynamicsWorld = NULL;
}

BulletEngine::~BulletEngine()
{
	cleanup();
}

bool BulletEngine::init( DynamicsEngineConfigPtr config )
{
	// first check if config is of type BulletEngineConfig
	BulletEngineConfigPtr test = boost::dynamic_pointer_cast<BulletEngineConfig>(config);
	if (!config || !test)
	{
		BulletEngineConfigPtr c(new BulletEngineConfig());
		if (config)
			c->gravity = config->gravity;
		return init(c);
	} else
	{
		return init(test);
	}
}

bool BulletEngine::init( BulletEngineConfigPtr config )
{
	boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	DynamicsEngine::init(config);
	bulletConfig = config;

	// Setup the bullet world
	collision_config = new btDefaultCollisionConfiguration();
	dispatcher = new SimoxCollisionDispatcher(this,collision_config);

	/*
	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	overlappingPairCache = new btAxisSweep3 (worldAabbMin, worldAabbMax);
	*/
	overlappingPairCache = new btDbvtBroadphase();
	//overlappingPairCache = new btSimpleBroadphase();


	constraintSolver = new btSequentialImpulseConstraintSolver;

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,constraintSolver,collision_config);

	dynamicsWorld->setGravity(btVector3(btScalar(config->gravity[0]),btScalar(config->gravity[1]),btScalar(config->gravity[2])));

	collisionFilterCallback = new BulletEngine::CustomCollisionCallback(this);
	dynamicsWorld->getPairCache()->setOverlapFilterCallback(collisionFilterCallback);

	btContactSolverInfo& solverInfo = dynamicsWorld->getSolverInfo();
	solverInfo.m_numIterations = config->bulletSolverIterations;
	solverInfo.m_globalCfm = config->bulletSolverGlobalContactForceMixing;
	solverInfo.m_erp = config->bulletSolverGlobalErrorReductionParameter;
	//solverInfo.m_solverMode |= SOLVER_USE_2_FRICTION_DIRECTIONS;

	/*
	By default, Bullet solves positional constraints and velocity constraints coupled together. 
	This works well in many cases, but the error reduction of position coupled to velocity introduces extra energy (noticeable as 'bounce'). 
	Instead of coupled positional and velocity constraint solving, the two can be solved separately using the 'split impulse' option. 
	This means that recovering from deep penetrations doesn't add any velocity. You can enable the option using:
	*/
	//solverInfo.m_splitImpulse = 1; //enable split impulse feature
	//optionally set the m_splitImpulsePenetrationThreshold (only used when m_splitImpulse  is enabled)
	//only enable split impulse position correction when the penetration is deeper than this m_splitImpulsePenetrationThreshold, otherwise use the regular velocity/position constraint coupling (Baumgarte).
	//solverInfo.m_splitImpulsePenetrationThreshold = btScalar(-0.02);

	return true;
}


bool BulletEngine::cleanup()
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	while (robots.size()>0)
	{
		size_t start = robots.size();
		removeRobot(robots[0]);
		VR_ASSERT(robots.size()<start);
	}

	while (objects.size()>0)
	{
		size_t start = objects.size();
		removeObject(objects[0]);
		VR_ASSERT(objects.size()<start);
	}

	delete dynamicsWorld;
	dynamicsWorld = NULL;
	delete collision_config;
	collision_config = NULL;
	delete dispatcher;
	dispatcher = NULL;
	delete overlappingPairCache;
	overlappingPairCache = NULL;
	delete constraintSolver;
	constraintSolver = NULL;
	delete collisionFilterCallback;
	collisionFilterCallback = NULL;
	return true;
}

bool BulletEngine::addObject( DynamicsObjectPtr o )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	BulletObjectPtr btObject = boost::dynamic_pointer_cast<BulletObject>(o);
	if (!btObject)
	{
		VR_ERROR << "Could only handle BULLET objects?! <" << o->getName() << ">" << endl;
		return false;
	}
	int btColFlag;
    switch (o->getSimType())
	{
    case VirtualRobot::SceneObject::Physics::eStatic:
		btColFlag = btCollisionObject::CF_STATIC_OBJECT;
        break;
    case VirtualRobot::SceneObject::Physics::eKinematic:
		btColFlag = btCollisionObject::CF_KINEMATIC_OBJECT;
		break;
    case VirtualRobot::SceneObject::Physics::eDynamic:
    case VirtualRobot::SceneObject::Physics::eUnknown:
        btColFlag = 0;
		break;
    default:
        // Dynamic Object
        btColFlag = 0;
        break;

    }
	btObject->getRigidBody()->setCollisionFlags(btColFlag);
	btObject->getRigidBody()->setRestitution(bulletConfig->bulletObjectRestitution);
	btObject->getRigidBody()->setFriction(bulletConfig->bulletObjectFriction);
	btObject->getRigidBody()->setDamping(bulletConfig->bulletObjectDampingLinear,bulletConfig->bulletObjectDampingAngular);
	btObject->getRigidBody()->setDeactivationTime(bulletConfig->bulletObjectDeactivation);//5.0f);
	btObject->getRigidBody()->setSleepingThresholds(bulletConfig->bulletObjectSleepingThresholdLinear,bulletConfig->bulletObjectSleepingThresholdAngular);//0.05f, 0.05f);

	//btScalar defaultContactProcessingThreshold = BT_LARGE_FLOAT;
	//btObject->getRigidBody()->setContactProcessingThreshold(defaultContactProcessingThreshold);
	dynamicsWorld->addRigidBody(btObject->getRigidBody().get());
	btObject->getRigidBody()->setAngularVelocity(btVector3(0,0,0));
	btObject->getRigidBody()->setLinearVelocity(btVector3(0,0,0));
	btObject->getRigidBody()->activate(true);

	return DynamicsEngine::addObject(o);
}

bool BulletEngine::removeObject( DynamicsObjectPtr o )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	BulletObjectPtr btObject = boost::dynamic_pointer_cast<BulletObject>(o);
	if (!btObject)
	{
		VR_ERROR << "Could only handle BULLET objects?! <" << o->getName() << ">" << endl;
		return false;
	}
	dynamicsWorld->removeRigidBody(btObject->getRigidBody().get());
	btObject->getRigidBody()->setBroadphaseHandle(NULL);
	return DynamicsEngine::removeObject(o);
}

bool BulletEngine::removeLink( BulletRobot::LinkInfo &l )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	dynamicsWorld->removeConstraint(l.joint.get());
	this->resetCollisions(static_cast<DynamicsObject*>(l.dynNode1.get()));
	this->resetCollisions(static_cast<DynamicsObject*>(l.dynNode2.get()));
	return true;
}

btDynamicsWorld* BulletEngine::getBulletWorld()
{
	return dynamicsWorld;
}

void BulletEngine::createFloorPlane( const Eigen::Vector3f &pos, const Eigen::Vector3f &up )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	DynamicsEngine::createFloorPlane(pos,up);
    float size = floorExtendMM;//50000.0f; // mm
    float sizeSmall = floorDepthMM; 500.0f;
	float w = size;
	float h = size;
	float d = sizeSmall;
	if (up(1) == 0 && up(2) == 0)
	{
		w = sizeSmall;
		h = size;
		d = size;
	} else if (up(0) == 0 && up(2) == 0)
	{
		w = size;
		h = sizeSmall;
		d = size;
	}
	
	groundObject = VirtualRobot::Obstacle::createBox(w,h,d,VirtualRobot::VisualizationFactory::Color::Gray());
	std::string name("Floor");
	groundObject->setName(name);
	Eigen::Matrix4f gp;
	gp.setIdentity();
	gp(2,3) = -sizeSmall*0.5f;
	groundObject->setGlobalPose(gp);

    groundObject->getVisualization();
    groundObject->setSimulationType(VirtualRobot::SceneObject::Physics::eStatic);

    BulletObjectPtr groundObjectBt(new BulletObject(groundObject));
	

	floor = groundObjectBt;

	addObject(groundObjectBt);
}


btMatrix3x3 BulletEngine::getRotMatrix(const Eigen::Matrix4f &pose)
{
    btMatrix3x3 rot(pose(0,0), pose(0,1), pose(0,2),
        pose(1,0), pose(1,1), pose(1,2),
        pose(2,0), pose(2,1), pose(2,2));
    return rot;
}

Eigen::Matrix4f BulletEngine::getRotMatrix(const btMatrix3x3 &pose)
{
    Eigen::Matrix4f rot = Eigen::Matrix4f::Identity();

    for (int a=0;a<3;a++)
        for (int b=0;b<3;b++)
            rot(a,b) = pose[a][b];
    return rot;
}

btTransform BulletEngine::getPoseBullet( const Eigen::Matrix4f &pose, bool scaling )
{
	btTransform res;
	float sc = 1.0f;
	if (scaling && DynamicsWorld::convertMM2M)
		sc = 0.001f; // mm -> m
	btVector3 pos(pose(0,3)*sc,pose(1,3)*sc,pose(2,3)*sc);
    res.setOrigin(pos);
    btMatrix3x3 rot = getRotMatrix(pose);
    //VirtualRobot::MathTools::Quaternion q = VirtualRobot::MathTools::eigen4f2quat(pose);
	//btQuaternion rot(q.x,q.y,q.z,q.w);
	res.setBasis(rot);
	return res;
}

Eigen::Matrix4f BulletEngine::getPoseEigen( const btTransform &pose, bool scaling )
{
	float sc = 1.0f;
	if (scaling && DynamicsWorld::convertMM2M)
		sc = 1000.0f; // m -> mm

	/*btQuaternion q = pose.getRotation();
	VirtualRobot::MathTools::Quaternion qvr;
	qvr.x = q.getX();
	qvr.y = q.getY();
	qvr.z = q.getZ();
	qvr.w = q.getW();
	Eigen::Matrix4f res = VirtualRobot::MathTools::quat2eigen4f(qvr);*/
    Eigen::Matrix4f res = getRotMatrix(pose.getBasis());
	res(0,3) = pose.getOrigin().getX()*sc;
	res(1,3) = pose.getOrigin().getY()*sc;
	res(2,3) = pose.getOrigin().getZ()*sc;
	return res;
}

btVector3 BulletEngine::getVecBullet( const Eigen::Vector3f &vec, bool scaling )
{
	btTransform res;
	float sc = 1.0f;
	if (scaling && DynamicsWorld::convertMM2M)
		sc = 0.001f; // mm -> m
	btVector3 pos(vec(0)*sc,vec(1)*sc,vec(2)*sc);
	return pos;
}

Eigen::Vector3f BulletEngine::getVecEigen( const btVector3 &vec, bool scaling )
{
	float sc = 1.0f;
	if (scaling && DynamicsWorld::convertMM2M)
		sc = 1000.0f; // m -> mm

	Eigen::Vector3f res;
	res(0) =  vec.getX()*sc;
	res(1) =  vec.getY()*sc;
	res(2) =  vec.getZ()*sc;

	return res;
}

bool BulletEngine::addRobot( DynamicsRobotPtr r )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	BulletRobotPtr btRobot = boost::dynamic_pointer_cast<BulletRobot>(r);
	if (!btRobot)
	{
		VR_ERROR << "Could only handle BULLET objects?! <" << r->getName() << ">" << endl;
		return false;
	}

	std::vector<BulletRobot::LinkInfo> links = btRobot->getLinks();
	std::vector<DynamicsObjectPtr> nodes = btRobot->getDynamicsRobotNodes();

	for (size_t i=0;i<nodes.size();i++)
	{
		addObject(nodes[i]);
	}
	for (size_t i=0;i<links.size();i++)
	{
		addLink(links[i]);
	}

	return DynamicsEngine::addRobot(r);
}

bool BulletEngine::removeRobot( DynamicsRobotPtr r )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	BulletRobotPtr btRobot = boost::dynamic_pointer_cast<BulletRobot>(r);
	if (!btRobot)
	{
		VR_ERROR << "Could only handle BULLET objects?! <" << r->getName() << ">" << endl;
		return false;
	}

	std::vector<BulletRobot::LinkInfo> links = btRobot->getLinks();
	std::vector<DynamicsObjectPtr> nodes = btRobot->getDynamicsRobotNodes();


	for (size_t i=0;i<links.size();i++)
	{
		removeLink(links[i]);
	}
	for (size_t i=0;i<nodes.size();i++)
	{
		removeObject(nodes[i]);
	}	
	return DynamicsEngine::removeRobot(r);
}

bool BulletEngine::addLink( BulletRobot::LinkInfo &l )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
#ifdef DEBUG_FIXED_OBJECTS
	cout << "TEST2" << endl;
#else
	dynamicsWorld->addConstraint(l.joint.get(), true);
#endif
	for (size_t i=0; i<l.disabledCollisionPairs.size();i++)
	{
		this->disableCollision(static_cast<DynamicsObject*>(l.disabledCollisionPairs[i].first.get()),static_cast<DynamicsObject*>(l.disabledCollisionPairs[i].second.get()));
	}
	return true;
}

void BulletEngine::print()
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	cout << "------------------ Bullet Engine ------------------" << endl;
	for (size_t i=0;i<objects.size();i++)
	{
		cout << "++ Object " << i << ":" << objects[i]->getName() << endl;
		Eigen::Matrix4f m = objects[i]->getSceneObject()->getGlobalPoseVisualization();
		cout << "   pos (simox)  " << m(0,3) << "," << m(1,3) << "," << m(2,3) << endl;
		BulletObjectPtr bo = boost::dynamic_pointer_cast<BulletObject>(objects[i]);
		boost::shared_ptr<btRigidBody> rb = bo->getRigidBody();
		btVector3 v = rb->getWorldTransform().getOrigin();
		cout << "   pos (bullet) " << v[0] << "," << v[1]  << "," << v[2]  << endl;
		btVector3 va = rb->getAngularVelocity();
		btVector3 vl = rb->getLinearVelocity();
		cout << "   ang vel (bullet) " << va[0] << "," << va[1]  << "," << va[2]  << endl;
		cout << "   lin vel (bullet) " << vl[0] << "," << vl[1]  << "," << vl[2]  << endl;
	}
	for (size_t i=0;i<robots.size();i++)
	{
		cout << "++ Robot " << i << ":" << objects[i]->getName() << endl;
		BulletRobotPtr br = boost::dynamic_pointer_cast<BulletRobot>(robots[i]);
		std::vector<BulletRobot::LinkInfo> links = br->getLinks();
		for (size_t j=0;j<links.size();j++)
		{
            cout << "++++ Link " << j << ":" << links[j].nodeJoint->getName();
            cout << "++++ - ColModelA " << j << ":" << links[j].nodeA->getName();
            cout << "++++ - ColModelB " << j << ":" << links[j].nodeB->getName();
			
			cout << "     enabled:" << links[j].joint->isEnabled() << endl;
            boost::shared_ptr<btHingeConstraint> hinge = boost::dynamic_pointer_cast<btHingeConstraint>(links[j].joint);
            if (hinge)
            {
                cout << "     hinge motor enabled:" << hinge->getEnableAngularMotor() << endl;
                cout << "     hinge angle :" << hinge->getHingeAngle() << endl;
                cout << "     hinge max motor impulse :" << hinge->getMaxMotorImpulse() << endl;
                cout << "     hinge motor target vel :" << hinge->getMotorTargetVelosity() << endl;
            }
            boost::shared_ptr<btGeneric6DofConstraint> dof = boost::dynamic_pointer_cast<btGeneric6DofConstraint>(links[j].joint);
            if (dof)
            {
                btRotationalLimitMotor *m = dof->getRotationalLimitMotor(2);
                VR_ASSERT(m);
                cout << "     generic_6DOF_joint: axis 5 (z)" << endl;
                cout << "     generic_6DOF_joint motor enabled:" << m->m_enableMotor << endl;
                cout << "     generic_6DOF_joint angle :" << m->m_currentPosition << endl;
                cout << "     generic_6DOF_joint max motor force :" << m->m_maxMotorForce << endl;
                cout << "     higeneric_6DOF_jointnge motor target vel :" << m->m_targetVelocity << endl;
            }

		}
	}

	cout << "------------------ Bullet Engine ------------------" << endl;
}

void BulletEngine::activateAllObjects()
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	for (size_t i=0;i<objects.size();i++)
	{
		BulletObjectPtr bo = boost::dynamic_pointer_cast<BulletObject>(objects[i]);
		if (bo)
		{
			bo->getRigidBody()->activate();
		}
	}
}


std::vector<DynamicsEngine::DynamicsContactInfo> BulletEngine::getContacts()
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
	//Assume world->stepSimulation or world->performDiscreteCollisionDetection has been called

	std::vector<DynamicsEngine::DynamicsContactInfo> result;

	int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
		const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());
		SimDynamics::BulletObject* dynObjA = static_cast<SimDynamics::BulletObject*>(obA->getUserPointer());
		SimDynamics::BulletObject* dynObjB = static_cast<SimDynamics::BulletObject*>(obB->getUserPointer());
		int numContacts = contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance()<0.f)
			{
				DynamicsContactInfo i;
				i.objectA = dynObjA;
				i.objectB = dynObjB;
				const btVector3& ptA = pt.getPositionWorldOnA();
				const btVector3& ptB = pt.getPositionWorldOnB();
				const btVector3& normalOnB = pt.m_normalWorldOnB;
				i.posGlobalA = getVecEigen(ptA);
				i.posGlobalB = getVecEigen(ptB);
				i.normalGlobalB(0) = normalOnB.x();
				i.normalGlobalB(1) = normalOnB.y();
				i.normalGlobalB(2) = normalOnB.z();
				result.push_back(i);
			}
		}
	}
	return result;
}

void BulletEngine::stepSimulation( float dt, int subSteps )
{
    boost::recursive_mutex::scoped_lock scoped_lock(engineMutex);
    dynamicsWorld->stepSimulation(dt,subSteps);
}


} // namespace SimDynamics
