#include "BulletRobot.h"
#include "BulletEngine.h"
#include "BulletEngineFactory.h"
#include "../../DynamicsWorld.h"
#include "../DynamicsObject.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeFixed.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>

// either hinge or generic6DOF constraints can be used
//#define USE_BULLET_GENERIC_6DOF_CONSTRAINT


#include <boost/pointer_cast.hpp>

//#define DEBUG_FIXED_OBJECTS
//#define DEBUG_SHOW_LINKS
using namespace VirtualRobot;

namespace SimDynamics {

BulletRobot::BulletRobot(VirtualRobot::RobotPtr rob, bool enableJointMotors)
	: DynamicsRobot(rob)
{
	bulletMaxMotorImulse = 10.0f; 
	buildBulletModels(enableJointMotors);
}
	
BulletRobot::~BulletRobot()
{
}

/*
void BulletRobot::setPosition( const Eigen::Vector3f &posMM )
{
	Eigen::Matrix4f pose = robot->getGlobalPose();
	pose.block(0,3,3,1) = posMM;
	setPose(pose);
}

void BulletRobot::setPose( const Eigen::Matrix4f &pose )
{
	DynamicsRobot::setPose(pose);

	btTransform btT = BulletEngine::getPoseBullet(pose);
	rigidBody->setWorldTransform(btT);
}*/

void BulletRobot::buildBulletModels(bool enableJointMotors)
{
	if (!robot)
		return;

	robotNodes = robot->getRobotNodes();

	for (size_t i=0;i<robotNodes.size();i++)
	{
		RobotNodePtr rn = robotNodes[i];
		std::vector<SceneObjectPtr> children = rn->getChildren();
		for (size_t c=0;c<children.size();c++)
		{
			// check what to do
			/*
			bool fixed = !(children[c]->isRotationalJoint() || children[c]->isTranslationalJoint());
			if (fixed && !(children[c]->getCollisionModel()))
				continue; // a fixed transformation without model, skip it
				*/
			RobotNodePtr rn2 = boost::dynamic_pointer_cast<RobotNode>(children[c]);
			if (rn2)
				createLink(rn,rn2,enableJointMotors);
		}
	}
}

void BulletRobot::createLink( VirtualRobot::RobotNodePtr node1,VirtualRobot::RobotNodePtr node2, bool enableJointMotors )
{
	THROW_VR_EXCEPTION_IF (!(node1->hasChild(node2)),"node1 must be parent of node2");

	if (dynamicRobotNodes.find(node1) == dynamicRobotNodes.end())
	{
		createDynamicsNode(node1);
	}
	if (dynamicRobotNodes.find(node2) == dynamicRobotNodes.end())
	{
		createDynamicsNode(node2);
	}

	if (hasLink(node1,node2))
	{
		THROW_VR_EXCEPTION("Joints are already connected:" << node1->getName() << "," << node2->getName());
	}

	BulletObjectPtr drn1 = boost::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[node1]);
	BulletObjectPtr drn2 = boost::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[node2]);
	VR_ASSERT(drn1);
	VR_ASSERT(drn2);
	boost::shared_ptr<btRigidBody> btBody1 = drn1->getRigidBody();
	boost::shared_ptr<btRigidBody> btBody2 = drn2->getRigidBody();
	VR_ASSERT(btBody1);
	VR_ASSERT(btBody2);



	Eigen::Matrix4f coordSystemNode1 = node1->getGlobalPoseJoint(); // todo: what if joint is not at 0 ?!
	Eigen::Matrix4f coordSystemNode2 = node2->getGlobalPoseJoint();

	Eigen::Matrix4f anchorPointGlobal = node2->getGlobalPoseJoint();//node1->getGlobalPose() * node2->getPreJointTransformation(); // 

	Eigen::Matrix4f anchor_inNode1 = coordSystemNode1.inverse() * anchorPointGlobal; 
	Eigen::Matrix4f anchor_inNode2 = coordSystemNode2.inverse() * anchorPointGlobal; 


	// The bullet model was adjusted, so that origin is at local com
	// since we computed the anchor in from simox models, we must re-adjust the anchor, in order to consider the com displacement
	Eigen::Matrix4f com1;
	com1.setIdentity();
	com1.block(0,3,3,1) = -drn1->getCom();
	anchor_inNode1 = com1 * anchor_inNode1;

	Eigen::Matrix4f com2;
	com2.setIdentity();
	com2.block(0,3,3,1) = -drn2->getCom();
	anchor_inNode2 = com2 * anchor_inNode2;

#ifdef DEBUG_SHOW_LINKS
	cout << "TEST4" << endl;
	ObstaclePtr o = Obstacle::createSphere(20);
	Eigen::Matrix4f gpxy1 = anchor_inNode1;
	// for visualization, we must again consider the com movement (this time back)
	gpxy1.block(0,3,3,1) += drn1->getCom();
	gpxy1 = coordSystemNode1 * gpxy1;
	o->setGlobalPose(gpxy1);
	DynamicsObjectPtr do1 = DynamicsWorld::GetWorld()->CreateDynamicsObject(o,DynamicsObject::eStatic);
	ObstaclePtr o2 = Obstacle::createBox(30,30,30);
	Eigen::Matrix4f gpxy = anchor_inNode2;
	gpxy.block(0,3,3,1) += drn2->getCom();
	gpxy = coordSystemNode2 * gpxy;
	o2->setGlobalPose(gpxy);
	DynamicsObjectPtr do2 = DynamicsWorld::GetWorld()->CreateDynamicsObject(o2,DynamicsObject::eStatic);
	DynamicsWorld::GetWorld()->getEngine()->disableCollision(do1.get());
	DynamicsWorld::GetWorld()->getEngine()->disableCollision(do2.get());
	DynamicsWorld::GetWorld()->getEngine()->addObject(do1);
	DynamicsWorld::GetWorld()->getEngine()->addObject(do2);
#endif
	// apply com transformation
	//anchor_inNode1.block(0,3,3,1) -= drn1->getCom();
	//anchor_inNode2.block(0,3,3,1) -= drn2->getCom();
		
	 boost::shared_ptr<btTypedConstraint> joint;

	 float vr2bulletOffset = 0.0f;

	 if (node2->isTranslationalJoint())
	 {
		 VR_WARNING << "translational joint nyi, creating a fixed link..." << endl;
	 }
	 bool createJoint = node2->isRotationalJoint();
	 if (createJoint)
	 {

        // create joint
		boost::shared_ptr<RobotNodeRevolute> rnRev2 = boost::dynamic_pointer_cast<RobotNodeRevolute>(node2);

		// transform axis direction (not position!)
		Eigen::Vector4f axisLocal2 = Eigen::Vector4f::Zero();
		axisLocal2.block(0,0,3,1) =  rnRev2->getJointRotationAxisInJointCoordSystem();
		Eigen::Matrix4f tmpGp2 = coordSystemNode2;
		tmpGp2.block(0,3,3,1).setZero();
		Eigen::Matrix4f tmpGp1 = coordSystemNode1;
		tmpGp1.block(0,3,3,1).setZero();
		Eigen::Vector4f axisGlobal = tmpGp2 * axisLocal2;

		Eigen::Vector3f axis_inLocal1 = (tmpGp1.inverse() * axisGlobal).block(0,0,3,1);
		Eigen::Vector3f axis_inLocal2 = rnRev2->getJointRotationAxisInJointCoordSystem();
#ifdef DEBUG_SHOW_LINKS

		cout << "TEST4" << endl;
		ObstaclePtr o3 = Obstacle::createBox(axis_inLocal1(0)*500+20,axis_inLocal1(1)*500+20,axis_inLocal1(2)*500+20);
		o3->setGlobalPose(coordSystemNode1);
		DynamicsObjectPtr do3 = DynamicsWorld::GetWorld()->CreateDynamicsObject(o3,DynamicsObject::eStatic);
		ObstaclePtr o4 = Obstacle::createBox(axis_inLocal2(0)*500+20,axis_inLocal2(1)*500+20,axis_inLocal2(2)*500+20);
		Eigen::Matrix4f gpxy2 = coordSystemNode2;
		o4->setGlobalPose(gpxy2);
		DynamicsObjectPtr do4 = DynamicsWorld::GetWorld()->CreateDynamicsObject(o4,DynamicsObject::eStatic);
		DynamicsWorld::GetWorld()->getEngine()->disableCollision(do3.get());
		DynamicsWorld::GetWorld()->getEngine()->disableCollision(do4.get());
		DynamicsWorld::GetWorld()->getEngine()->addObject(do3);
		DynamicsWorld::GetWorld()->getEngine()->addObject(do4);
#endif

		btVector3 pivot1 = BulletEngine::getVecBullet(anchor_inNode1.block(0,3,3,1));
		btVector3 pivot2 = BulletEngine::getVecBullet(anchor_inNode2.block(0,3,3,1));
        
       
		/*pivTr1.setIdentity();
		pivTr1.setOrigin(pivot1);
		btTransform pivTr2;
		pivTr2.setIdentity();
		pivTr2.setOrigin(pivot2);
		btTransform pivotTest1 = btBody1->getWorldTransform()*pivTr1;
		btTransform pivotTest2 = btBody2->getWorldTransform()*pivTr2;*/
	
        float limMin,limMax;
		limMin = node2->getJointLimitLo();
		limMax = node2->getJointLimitHi();

#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT

        MathTools::Quaternion axisRotQuat1 = MathTools::getRotation(axis_inLocal1,Eigen::Vector3f::UnitX());
        MathTools::Quaternion axisRotQuat2 = MathTools::getRotation(axis_inLocal2,Eigen::Vector3f::UnitX());
        Eigen::Matrix4f axisRot1 = MathTools::quat2eigen4f(axisRotQuat1);
        Eigen::Matrix4f axisRot2 = MathTools::quat2eigen4f(axisRotQuat2);
        float ang1,ang2;
        Eigen::Vector3f ax1,ax2;
        // test
        MathTools::eigen4f2axisangle(axisRot1,ax1,ang1);
        MathTools::eigen4f2axisangle(axisRot2,ax2,ang2);

        Eigen::Matrix4f anchorAxisXAligned1;// = anchor_inNode1 * axisRot1;
        Eigen::Matrix4f anchorAxisXAligned2 = anchor_inNode2 * axisRot2;

        // we have to express anchorAxisXAligned2 in node1 coord system (only rotation!)
        Eigen::Matrix4f anchorAxisXAligned2_global = tmpGp2 * anchorAxisXAligned2;
        Eigen::Matrix4f anchorAxisXAligned2_local1 = tmpGp1.inverse() * anchorAxisXAligned2_global;
        anchorAxisXAligned1 = anchorAxisXAligned2_local1;
        // re-set position
        anchorAxisXAligned1.block(0,3,3,1) = anchor_inNode1.block(0,3,3,1);

        btTransform pivTr1 = BulletEngine::getPoseBullet(anchorAxisXAligned1);
        btTransform pivTr2 = BulletEngine::getPoseBullet(anchorAxisXAligned2);
        boost::shared_ptr<btGeneric6DofConstraint> dof(new btGeneric6DofConstraint(*btBody1, *btBody2, pivTr1, pivTr2, true));
        btRotationalLimitMotor *m = dof->getRotationalLimitMotor(0);
        VR_ASSERT(m);
        m->m_targetVelocity = 0;
        m->m_currentPosition = node2->getJointValue();
        /*btScalar startAngle = node2->getJointValue();
        btScalar startAngleBT = dof->getRotationalLimitMotor(2)->m_currentPosition;
        btScalar limMinBT, limMaxBT;
        btScalar diff = (startAngleBT + startAngle);
        limMinBT = diff - limMax;// limMin + diff;// 
        limMaxBT = diff - limMin;// limMax + diff;// 
        if (fabs(startAngleBT - startAngle)>1e-6)
        {
            cout << "joint " << node2->getName() << ": jv diff:" << diff << endl;
            cout << "Simox limits: " << limMin << "/" << limMax << ", bullet limits:" << limMinBT << "/" << limMaxBT << endl;
        }*/
        dof->setLimit(0,0,0);
        dof->setLimit(1,0,0);
        dof->setLimit(2,0,0);
        dof->setLimit(3,btScalar(limMin),btScalar(limMax));
        //dof->setLimit(3,-btScalar(limMax),-btScalar(limMin)); // inverted joint direction
        dof->setLimit(4,0,0);
        dof->setLimit(5,0,0);

        //dof->setLimit(5,btScalar(limMin),btScalar(limMax));
        vr2bulletOffset = 0;//diff;
        //hinge->setLimit(btScalar(limMin),btScalar(limMax));
        //hinge->setAngularOnly(true);
        joint = dof;
#else
        btVector3 axis1 = BulletEngine::getVecBullet(axis_inLocal1,false);
        btVector3 axis2 = BulletEngine::getVecBullet(axis_inLocal2,false);
		boost::shared_ptr<btHingeConstraint> hinge(new btHingeConstraint(*btBody1, *btBody2, pivot1, pivot2, axis1, axis2,false));

		//hinge->setParam(BT_CONSTRAINT_STOP_ERP,0.9f);
		/*
		hinge->setParam(BT_CONSTRAINT_CFM,0.9f);
		hinge->setParam(BT_CONSTRAINT_STOP_CFM,0.01f);*/
		//hinge->setLimit(limMin,limMax);//,btScalar(1.0f));
		//hinge->setParam(BT_CONSTRAINT_CFM,1.0f);

		btScalar startAngle = node2->getJointValue();
		btScalar startAngleBT = hinge->getHingeAngle();
		btScalar limMinBT, limMaxBT;
		btScalar diff = 0;//(startAngleBT + startAngle);
		limMinBT = limMin + diff;//diff - limMax;// 
		limMaxBT = limMax + diff;//diff - limMin;// 
		if (fabs(startAngleBT - startAngle)>1e-6)
		{
			cout << "joint " << node2->getName() << ": jv diff:" << diff << endl;
			cout << "Simox limits: " << limMin << "/" << limMax << ", bullet limits:" << limMinBT << "/" << limMaxBT << endl;
		}
		hinge->setLimit(btScalar(limMinBT),btScalar(limMaxBT));
		vr2bulletOffset = diff;
		//hinge->setLimit(btScalar(limMin),btScalar(limMax));
		//hinge->setAngularOnly(true);
		joint = hinge;

#endif
	} else /*if (node2->isTranslationalJoint())
	{
		
		// todo: btSliderConstraints always assume that you move along the x axis, but which transform should be created then?!
		boost::shared_ptr<RobotNodePrismatic> rnPris2 = boost:dynamic_pointer_cast<RobotNodePrismatic>(node2);
		Eigen::Vector3f axis_inLocal1 = gp1.inverse() * rnPris2->getJointTranslationDirection();
		Eigen::Vector3f axis_inLocal2 = rnPris2->getJointTranslationDirectionJointCoordSystem();

		btVector3 axis1 = BulletEngine::getVecBullet(axis_inLocal1,false);
		btVector3 axis2 = BulletEngine::getVecBullet(axis_inLocal2,false);
		joint.reset(new btSliderConstraint(*btBody1, *btBody2, axis1, axis2, true));
	} else*/
	{
		// fixed joint

#if 0
		// transform axis direction (not position!)
		Eigen::Vector4f axisLocal2 = Eigen::Vector4f::Zero();
		axisLocal2(1,0) = 1.0f;
		Eigen::Matrix4f tmpGp2 = coordSystemNode2;
		tmpGp2.block(0,3,3,1).setZero();
		Eigen::Matrix4f tmpGp1 = coordSystemNode1;
		tmpGp1.block(0,3,3,1).setZero();
		Eigen::Vector4f axisGlobal = tmpGp2 * axisLocal2;

		Eigen::Vector3f axis_inLocal1 = (tmpGp1.inverse() * axisGlobal).block(0,0,3,1);
		Eigen::Vector3f axis_inLocal2 = Eigen::Vector3f::Zero();
		axis_inLocal2(1,0) = 1.0f;


		btVector3 pivot1 = BulletEngine::getVecBullet(anchor_inNode1.block(0,3,3,1));
		btVector3 pivot2 = BulletEngine::getVecBullet(anchor_inNode2.block(0,3,3,1));
		btTransform pivTr1;
		pivTr1.setIdentity();
		pivTr1.setOrigin(pivot1);
		btTransform pivTr2;
		pivTr2.setIdentity();
		pivTr2.setOrigin(pivot2);
		btTransform pivotTest1 = btBody1->getWorldTransform()*pivTr1;
		btTransform pivotTest2 = btBody2->getWorldTransform()*pivTr2;

		btVector3 axis1 = BulletEngine::getVecBullet(axis_inLocal1,false);
		btVector3 axis2 = BulletEngine::getVecBullet(axis_inLocal2,false);

		boost::shared_ptr<btHingeConstraint> hinge(new btHingeConstraint(*btBody1, *btBody2, pivot1, pivot2, axis1, axis2));
	
		hinge->setLimit(hinge->getHingeAngle(),hinge->getHingeAngle());
		//hinge->setLimit(btScalar(limMin),btScalar(limMax));
		vr2bulletOffset = hinge->getHingeAngle();
		joint = hinge;
#else
		btTransform localA,localB;
		localA = BulletEngine::getPoseBullet(anchor_inNode1);
		localB = BulletEngine::getPoseBullet(anchor_inNode2);
		//localA.setIdentity(); localB.setIdentity();
		//btVector3 pivot1 = BulletEngine::getVecBullet(anchor_inNode1.block(0,3,3,1));
		//btVector3 pivot2 = BulletEngine::getVecBullet(anchor_inNode2.block(0,3,3,1));
		//localA.setOrigin(pivot1);
		//localB.setOrigin(pivot2);
		boost::shared_ptr<btGeneric6DofConstraint> generic6Dof(new btGeneric6DofConstraint(*btBody1, *btBody2, localA, localB, true));

        generic6Dof->setOverrideNumSolverIterations(100);

        //float totalMass = 1.f/btBody1->getInvMass() + 1.f/btBody2->getInvMass();

        //generic6Dof->setBreakingImpulseThreshold(2*totalMass);//needed? copied from voronoi demo

        for (int i=0;i<6;i++)
            generic6Dof->setLimit(i,0,0);
		/*btVector3 limitZero(0,0,0);
		generic6Dof->setAngularLowerLimit(limitZero);
		generic6Dof->setAngularUpperLimit(limitZero);*/

		/*generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,0);
		generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,1);
		generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,2);
		generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,3);
		generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,4);
		generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,5);*/
		/*generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,0);
		generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,1);
		generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,2);
		generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,3);
		generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,4);
		generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,5);*/
		/*btRotationalLimitMotor *r = generic6Dof->getRotationalLimitMotor(0);
		r->m_maxLimitForce = 1000.0f;
		r = generic6Dof->getRotationalLimitMotor(1);
		r->m_maxLimitForce = 1000.0f;
		r = generic6Dof->getRotationalLimitMotor(2);
		r->m_maxLimitForce = 1000.0f;*/
		joint = generic6Dof;
#endif
	}
	LinkInfo i;
	i.node1 = node1;
	i.node2 = node2;
	i.dynNode1 = drn1;
	i.dynNode2 = drn2;
	i.joint = joint;
	i.jointValueOffset = vr2bulletOffset;
	
	// check if node1 owns a 3d model or a parent
	// -> disable according model, so it won't be considered for collision detection during simulation
	//if (!node1->getCollisionModel())
	{	
    RobotNodePtr parent = boost::dynamic_pointer_cast<RobotNode>(node1);
    while (parent)
		{
			if (parent->getCollisionModel())
			{
				if (!hasDynamicsRobotNode(parent))
					createDynamicsNode(parent);
				DynamicsObjectPtr dynParent = dynamicRobotNodes[parent];
				VR_ASSERT(dynParent);
				i.disabledCollisionPairs.push_back(
					std::pair<DynamicsObjectPtr,DynamicsObjectPtr>(
					boost::dynamic_pointer_cast<DynamicsObject>(dynParent),
					boost::dynamic_pointer_cast<DynamicsObject>(drn2)));
				// stop search
				break;
			}
			parent = boost::dynamic_pointer_cast<RobotNode>(parent->getParent());
		}
	}
	links.push_back(i);
#ifndef DEBUG_FIXED_OBJECTS
	if (enableJointMotors && node2->isRotationalJoint())
	{
		// start standard actuator
		//cout << "TEST6" << endl;
#if 1
		actuateNode(node2,node2->getJointValue());
#endif
	}
#endif
}


bool BulletRobot::hasLink( VirtualRobot::RobotNodePtr node1, VirtualRobot::RobotNodePtr node2 )
{
	for (size_t i=0; i<links.size();i++)
	{
		if (links[i].node1 == node1 && links[i].node2 == node2)
			return true;
	}
	return false;
}

std::vector<BulletRobot::LinkInfo> BulletRobot::getLinks()
{
	return links;
}


void BulletRobot::ensureKinematicConstraints()
{
	// results in strange behavior?!
#if 0
	// update globalpose of robot
	Eigen::Matrix4f gpRoot = robot->getRootNode()->getGlobalPoseVisualization();
	Eigen::Matrix4f rootPreJoint = robot->getRootNode()->getPreJointTransformation();
	robot->setGlobalPose (gpRoot*rootPreJoint.inverse());
	//robot->applyJointValues();
	std::map<VirtualRobot::RobotNodePtr, DynamicsObjectPtr>::iterator i = dynamicRobotNodes.begin();
	while (i!=dynamicRobotNodes.end())
	{
		i->second->setPose(i->first->getGlobalPoseVisualization());
		i++;
	}
#endif
}

void BulletRobot::actuateJoints(float dt)
{
	std::map<VirtualRobot::RobotNodePtr, robotNodeActuationTarget>::iterator it = actuationTargets.begin();

	while (it!=actuationTargets.end())
	{
		BulletObjectPtr drn = boost::dynamic_pointer_cast<BulletObject>(it->second.dynNode);
		VR_ASSERT(drn);
		if (it->second.node->isRotationalJoint())
		{
			LinkInfo link = getLink(it->second.node);
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
            boost::shared_ptr<btGeneric6DofConstraint> dof = boost::dynamic_pointer_cast<btGeneric6DofConstraint>(link.joint);
            VR_ASSERT(dof);
            btRotationalLimitMotor *m = dof->getRotationalLimitMotor(0);
            VR_ASSERT(m);
            if (it->second.enabled)
            {
                btScalar targ = btScalar(it->second.jointValueTarget+link.jointValueOffset);
                //btScalar act = btScalar(it->first->getJointValue());
                btScalar act = btScalar(getJointAngle(it->first));
                m->m_enableMotor = true;
                m->m_targetVelocity = (targ-act); // inverted joint dir?!
            } else
                m->m_enableMotor = false;
#else
			boost::shared_ptr<btHingeConstraint> hinge = boost::dynamic_pointer_cast<btHingeConstraint>(link.joint);
			VR_ASSERT(hinge);
			if (it->second.enabled)
			{
                btScalar targ = btScalar(it->second.jointValueTarget+link.jointValueOffset);
                //btScalar act = btScalar(it->first->getJointValue());
                btScalar act = btScalar(getJointAngle(it->first));
                hinge->enableAngularMotor(true,(targ-act),10.0f);
                //hinge->enableMotor(true);
				//hinge->setMotorTarget(it->second.jointValueTarget+link.jointValueOffset,dt);
			} else
				hinge->enableMotor(false);
#endif
		}

		it++;
	}
}

BulletRobot::LinkInfo BulletRobot::getLink( VirtualRobot::RobotNodePtr node )
{
	for (size_t i=0;i<links.size();i++)
	{
		if (links[i].node2 == node)
			return links[i];
	}
	THROW_VR_EXCEPTION("No link with node " << node->getName());
	return LinkInfo();
}

bool BulletRobot::hasLink( VirtualRobot::RobotNodePtr node )
{
	for (size_t i=0;i<links.size();i++)
	{
		if (links[i].node2 == node)
			return true;
	}
	return false;
}

void BulletRobot::actuateNode( VirtualRobot::RobotNodePtr node, float jointValue )
{
	VR_ASSERT(node);
	if (node->isRotationalJoint())
	{
		LinkInfo link = getLink(node);
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
        boost::shared_ptr<btGeneric6DofConstraint> dof = boost::dynamic_pointer_cast<btGeneric6DofConstraint>(link.joint);
        VR_ASSERT(dof);
        btRotationalLimitMotor *m = dof->getRotationalLimitMotor(0);
        VR_ASSERT(m);
        m->m_enableMotor = true;
        m->m_maxMotorForce = 5;//bulletMaxMotorImulse; //?!
        m->m_maxLimitForce = 300;
        DynamicsRobot::actuateNode(node,jointValue);
#else
		boost::shared_ptr<btHingeConstraint> hinge = boost::dynamic_pointer_cast<btHingeConstraint>(link.joint);
		VR_ASSERT(hinge);
		hinge->enableAngularMotor(true,0.0f,bulletMaxMotorImulse);// is max impulse ok?! (10 seems to be ok, 1 oscillates)
        DynamicsRobot::actuateNode(node,jointValue); // inverted joint direction in bullet
#endif
    } else
	{
		VR_ERROR << "Only Revolute joints implemented so far..." << endl;
	}
}

float BulletRobot::getJointAngle( VirtualRobot::RobotNodePtr rn )
{
	VR_ASSERT(rn);
	if (!hasLink(rn))
	{
		VR_ERROR << "No link with node " << rn->getName() << endl;
		return 0.0f;
	}
	LinkInfo link = getLink(rn);
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
    boost::shared_ptr<btGeneric6DofConstraint> dof = boost::dynamic_pointer_cast<btGeneric6DofConstraint>(link.joint);
    VR_ASSERT(dof);
    btRotationalLimitMotor *m = dof->getRotationalLimitMotor(0);
    VR_ASSERT(m);
    dof->calculateTransforms();
    float a1 = dof->getAngle(0);
    float a2 = m->m_currentPosition;
    if (fabs(a1-a2)>0.05f)
    {
        VR_INFO << "Angle diff " << a1 << ", " << a2 << endl;
    }
    return (a2-link.jointValueOffset);// inverted joint direction in bullet
#else
	boost::shared_ptr<btHingeConstraint> hinge = boost::dynamic_pointer_cast<btHingeConstraint>(link.joint);
	if (!hinge)
	{
		//VR_WARNING << "RobotNode " << rn->getName() << " is not associated with a hinge joint?!" << endl;
		return 0.0f;
	}
    return (hinge->getHingeAngle()-link.jointValueOffset);// inverted joint direction in bullet
#endif
}

float BulletRobot::getJointSpeed( VirtualRobot::RobotNodePtr rn )
{
	VR_ASSERT(rn);
	if (!hasLink(rn))
	{
		VR_ERROR << "No link with node " << rn->getName() << endl;
		return 0.0f;
	}
	LinkInfo link = getLink(rn);
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
    boost::shared_ptr<btGeneric6DofConstraint> dof = boost::dynamic_pointer_cast<btGeneric6DofConstraint>(link.joint);
    VR_ASSERT(dof);
    btRotationalLimitMotor *m = dof->getRotationalLimitMotor(0);
    VR_ASSERT(m);
    return m->m_targetVelocity;
#else
	boost::shared_ptr<btHingeConstraint> hinge = boost::dynamic_pointer_cast<btHingeConstraint>(link.joint);
	if (!hinge)
	{
		VR_WARNING << "RobotNode " << rn->getName() << " is not associated with a hinge joint?!" << endl;
		return 0.0f;
	}
    return hinge->getMotorTargetVelosity();
#endif
}

float BulletRobot::getNodeTarget( VirtualRobot::RobotNodePtr node )
{
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
    return DynamicsRobot::getNodeTarget(node);
#else
	return DynamicsRobot::getNodeTarget(node);
#endif

}

Eigen::Matrix4f BulletRobot::getComGlobal( VirtualRobot::RobotNodePtr rn )
{
    BulletObjectPtr bo = boost::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(rn));
    if (!bo)
    {
        VR_ERROR << "Could not cast object..." << endl;
        return Eigen::Matrix4f::Identity();
    }
    return bo->getComGlobal();
}

} // namespace VirtualRobot
