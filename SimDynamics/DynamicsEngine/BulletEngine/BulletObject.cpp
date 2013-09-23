#include "BulletObject.h"
#include "BulletEngine.h"
#include "../../DynamicsWorld.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/SceneObject.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>


#include <BulletCollision/CollisionShapes/btShapeHull.h>

//#define DEBUG_FIXED_OBJECTS
//#define USE_BULLET_GENERIC_6DOF_CONSTRAINT

using namespace VirtualRobot;

namespace SimDynamics {

BulletObject::BulletObject(VirtualRobot::SceneObjectPtr o)
    : DynamicsObject(o)
{
	float interatiaFactor = 1.0f;
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
    interatiaFactor = 5.0f;
#endif

	btMargin=(btScalar)(0.000001);
	com.setZero();
	THROW_VR_EXCEPTION_IF(!o,"NULL object");
	CollisionModelPtr colModel = o->getCollisionModel();

	if (!colModel)
	{
		VR_WARNING << "Building empty collision shape for object " << o->getName() << endl;
		collisionShape.reset(new btEmptyShape());

		/*VR_WARNING << "Building Collision model for object " << o->getName() << endl;
		VirtualRobot::ObstaclePtr ob = Obstacle::createBox(10.0f,10.0f,10.0f);
		ob->setGlobalPose(o->getGlobalPose());
		colModel = ob->getCollisionModel();*/
	} else
	{

		TriMeshModelPtr trimesh;
		THROW_VR_EXCEPTION_IF(!colModel,"No CollisionModel, could not create dynamics model...");
		trimesh = colModel->getTriMeshModel();
		
		THROW_VR_EXCEPTION_IF( ( !trimesh || trimesh->faces.size()==0) , "No TriMeshModel, could not create dynamics model...");
		collisionShape.reset(createConvexHullShape(trimesh));
	}

	collisionShape->setMargin(btMargin);

	btScalar mass = o->getMass();
	btVector3 localInertia;
    if (mass<=0 && (o->getSimulationType()==VirtualRobot::SceneObject::Physics::eDynamic || o->getSimulationType()==VirtualRobot::SceneObject::Physics::eUnknown))
	{
		//THROW_VR_EXCEPTION ("mass == 0 -> SimulationType must not be eDynamic! ");
		mass = btScalar(1.0f); // give object a dummy mass
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
       mass = btScalar(1.0f);
#endif

		//type = eKinematic;
		if (colModel)
		{
			VR_WARNING << "Object:" << o->getName() << ": mass == 0 -> SimulationType must not be eDynamic! Setting mass to 1" << endl;
		}
	}
#ifdef DEBUG_FIXED_OBJECTS
	cout << "TEST" << endl;
	mass = 0;
	localInertia.setValue(0.0f, 0.0f, 0.0f);
#else
    if (o->getSimulationType()!=VirtualRobot::SceneObject::Physics::eDynamic && o->getSimulationType()!=VirtualRobot::SceneObject::Physics::eUnknown) {
		mass = 0;
		localInertia.setValue(0.0f, 0.0f, 0.0f);
	} else
	{
		if (colModel)
		{
			collisionShape->calculateLocalInertia(mass,localInertia);
//#ifndef USE_BULLET_GENERIC_6DOF_CONSTRAINT
			// check for small values
			//if (localInertia.length()<1.0f && localInertia.length()>0)
			//	localInertia /= localInertia.length(); // small inertia values result in freaking out joints ?!
//#endif
		} else
#ifndef USE_BULLET_GENERIC_6DOF_CONSTRAINT
			localInertia.setValue(btScalar(1),btScalar(1),btScalar(1)); // give Object a dummy inertia matrix
#else
            localInertia.setValue(btScalar(1),btScalar(1),btScalar(1)); // give Object a dummy inertia matrix
#endif
            //localInertia.setValue(btScalar(40),btScalar(40),btScalar(40)); // give Object a dummy inertia matrix (large values needed, otherwise the objects will not stay connected on strong impulses) 
	}
#endif
	localInertia *= interatiaFactor;
	motionState = new SimoxMotionState(o);
	btRigidBody::btRigidBodyConstructionInfo btRBInfo(mass,motionState,collisionShape.get(),localInertia);
	//btRBInfo.m_additionalDamping = true;

	rigidBody.reset(new btRigidBody(btRBInfo));
	rigidBody->setUserPointer((void*)(this));
#if 0
	rigidBody->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
	cout << "TEST3" << endl;
#endif

    setPoseIntern(o->getGlobalPoseVisualization());
}
	
BulletObject::~BulletObject()
{
	rigidBody.reset();
	delete motionState;
}

btConvexHullShape* BulletObject::createConvexHullShape(VirtualRobot::TriMeshModelPtr trimesh)
{
	VR_ASSERT(trimesh);
	// create triangle shape
	btTriangleMesh* btTrimesh = new btTriangleMesh();
	
	//com = trimesh->getCOM();

	Eigen::Matrix4f comLoc;
	comLoc.setIdentity();
    comLoc.block(0,3,3,1) = sceneObject->getCoMGlobal();
    comLoc = (sceneObject->getGlobalPoseVisualization().inverse()*comLoc);
	com = comLoc.block(0,3,3,1);
	
	float sc = 1.0f;
	if (DynamicsWorld::convertMM2M)
		sc = 0.001f;

	for(size_t i = 0; i < trimesh->faces.size(); i++) {
		btVector3 v1((trimesh->vertices[trimesh->faces[i].id1][0]-com[0])*sc,(trimesh->vertices[trimesh->faces[i].id1][1]-com[1])*sc,(trimesh->vertices[trimesh->faces[i].id1][2]-com[2])*sc);
		btVector3 v2((trimesh->vertices[trimesh->faces[i].id2][0]-com[0])*sc,(trimesh->vertices[trimesh->faces[i].id2][1]-com[1])*sc,(trimesh->vertices[trimesh->faces[i].id2][2]-com[2])*sc);
		btVector3 v3((trimesh->vertices[trimesh->faces[i].id3][0]-com[0])*sc,(trimesh->vertices[trimesh->faces[i].id3][1]-com[1])*sc,(trimesh->vertices[trimesh->faces[i].id3][2]-com[2])*sc);
		btTrimesh->addTriangle(v1,v2,v3);
	}

	// convert COM to visualization frame (->no, the trimesh points are given in local visu frame!)
	/*Eigen::Matrix4f comLoc;
	comLoc.setIdentity();
	comLoc.block(0,3,3,1) = com;
	Eigen::Matrix4f comConv = sceneObject->getGlobalPoseVisualization() * comLoc;
	com = comConv.block(0,3,3,1);*/

	// build convex hull
	boost::shared_ptr<btConvexShape> btConvexShape(new btConvexTriangleMeshShape(btTrimesh));
	btConvexShape->setMargin(btMargin);

	boost::shared_ptr<btShapeHull> btHull(new btShapeHull(btConvexShape.get()));
	btHull->buildHull(btMargin);
	btConvexHullShape* btConvex = new btConvexHullShape();
	btConvex->setLocalScaling(btVector3(1,1,1));
	for (int i=0; i< btHull->numVertices(); i++) {
		btConvex->addPoint(btHull->getVertexPointer()[i]);
	}
	btConvex->setMargin(btMargin);

	// trimesh not needed any more
	delete btTrimesh;
	return btConvex;
}

boost::shared_ptr<btRigidBody> BulletObject::getRigidBody()
{
	return rigidBody;
}

void BulletObject::setPosition( const Eigen::Vector3f &posMM )
{
	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
	pose.block(0,3,3,1) = posMM;
	setPose(pose);
}

void BulletObject::setPoseIntern( const Eigen::Matrix4f &pose )
{
    // notify motionState
    motionState->setGlobalPose(pose);
}

void BulletObject::setPose( const Eigen::Matrix4f &pose )
{
    DynamicsObject::setPose(pose);
    setPoseIntern(pose);
}

Eigen::Vector3f BulletObject::getLinearVelocity()
{
	if (!rigidBody)
		return Eigen::Vector3f::Zero();
	return (BulletEngine::getVecEigen(rigidBody->getLinearVelocity()));
}

Eigen::Vector3f BulletObject::getAngularVelocity()
{
	if (!rigidBody)
		return Eigen::Vector3f::Zero();
    return (BulletEngine::getVecEigen(rigidBody->getAngularVelocity(),false));
}

void BulletObject::setLinearVelocity( const Eigen::Vector3f &vel )
{
	if (!rigidBody)
		return;
	btVector3 btVel = BulletEngine::getVecBullet(vel,false);
	rigidBody->setLinearVelocity(btVel);
}

void BulletObject::setAngularVelocity( const Eigen::Vector3f &vel )
{
	if (!rigidBody)
		return;
	btVector3 btVel = BulletEngine::getVecBullet(vel,false);
	rigidBody->setAngularVelocity(btVel);
}

Eigen::Matrix4f BulletObject::getComGlobal()
{
    btTransform tr;
    motionState->getWorldTransform(tr);
    return BulletEngine::getPoseEigen(tr);
}

void BulletObject::applyForce(const Eigen::Vector3f &force)
{
    if (!rigidBody)
        return;
    btVector3 btVel = BulletEngine::getVecBullet(force,false);
    rigidBody->applyCentralForce(btVel);
}

void BulletObject::applyTorque(const Eigen::Vector3f &torque)
{
    if (!rigidBody)
        return;
    btVector3 btVel = BulletEngine::getVecBullet(torque,false);
    rigidBody->applyTorque(btVel);
}


} // namespace VirtualRobot
