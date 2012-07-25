#include "BulletRobot.h"
#include "BulletEngine.h"
#include "BulletEngineFactory.h"
#include "../../DynamicsWorld.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeFixed.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>

using namespace VirtualRobot;

namespace SimDynamics {

BulletRobot::BulletRobot(VirtualRobot::RobotPtr rob, bool enableJointMotors)
	: DynamicsRobot(rob)
{
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
		std::vector<RobotNodePtr> children = rn->getChildren();
		for (size_t c=0;c<children.size();c++)
		{
			// check what to to
			/*
			bool fixed = !(children[c]->isRotationalJoint() || children[c]->isTranslationalJoint());
			if (fixed && !(children[c]->getCollisionModel()))
				continue; // a fixed transformation without model, skip it
				*/
			createLink(rn,children[c],enableJointMotors);
		}
	}
}

void BulletRobot::createLink( VirtualRobot::RobotNodePtr node1,VirtualRobot::RobotNodePtr node2, bool enableJointMotors )
{
	THROW_VR_EXCEPTION_IF (!(node1->hasChildNode(node2)),"node1 must be parent of node2");

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

	Eigen::Matrix4f anchorPointGlobal = node1->getGlobalPose() * node2->getPreJointTransformation(); // node2->getGlobalPoseJoint();//

	Eigen::Matrix4f anchor_inNode1 = coordSystemNode1.inverse() * anchorPointGlobal; 
	Eigen::Matrix4f anchor_inNode2 = coordSystemNode2.inverse() * anchorPointGlobal; 

	Eigen::Matrix4f com1;
	com1.setIdentity();
	com1.block(0,3,3,1) = -drn1->getCom();
	anchor_inNode1 = com1 * anchor_inNode1;

	Eigen::Matrix4f com2;
	com2.setIdentity();
	com2.block(0,3,3,1) = -drn2->getCom();
	anchor_inNode2 = com2 * anchor_inNode2;

	// apply com transformation
	//anchor_inNode1.block(0,3,3,1) -= drn1->getCom();
	//anchor_inNode2.block(0,3,3,1) -= drn2->getCom();
		
	 boost::shared_ptr<btTypedConstraint> joint;

	 float vr2bulletOffset = 0.0f;

	 if (node2->isTranslationalJoint())
	 {
		 VR_WARNING << "translational joint nyi, creating a fixed link..." << endl;
	 }

	if (node2->isRotationalJoint())
	{
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

		btVector3 pivot1 = BulletEngine::getVecBullet(anchor_inNode1.block(0,3,3,1));
		btVector3 pivot2 = BulletEngine::getVecBullet(anchor_inNode2.block(0,3,3,1));
		btVector3 axis1 = BulletEngine::getVecBullet(axis_inLocal1,false);
		btVector3 axis2 = BulletEngine::getVecBullet(axis_inLocal2,false);

		boost::shared_ptr<btHingeConstraint> hinge(new btHingeConstraint(*btBody1, *btBody2, pivot1, pivot2, axis1, axis2));
		float limMin,limMax;
		limMin = node2->getJointLimitLo();
		limMax = node2->getJointLimitHi();
		//hinge->setParam(BT_CONSTRAINT_STOP_ERP,0.9f);
		/*
		hinge->setParam(BT_CONSTRAINT_CFM,0.9f);
		hinge->setParam(BT_CONSTRAINT_STOP_CFM,0.01f);*/
		//hinge->setLimit(limMin,limMax);//,btScalar(1.0f));
		//hinge->setParam(BT_CONSTRAINT_CFM,1.0f);

		btScalar startAngle = node2->getJointValue();
		btScalar startAngleBT = hinge->getHingeAngle();
		btScalar limMinBT, limMaxBT;
		btScalar diff = (startAngleBT + startAngle);
		if (fabs(diff)>1e-6)
		{
			cout << "joint " << node2->getName() << ": jv diff:" << diff << endl;
		}
		limMinBT = diff - limMax;
		limMaxBT = diff - limMin;
		hinge->setLimit(btScalar(limMinBT),btScalar(limMaxBT));
		vr2bulletOffset = diff;
		//hinge->setLimit(btScalar(limMin),btScalar(limMax));
		//hinge->setAngularOnly(true);
		joint = hinge;
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
		btTransform localA,localB;
		localA = BulletEngine::getPoseBullet(anchor_inNode1);
		localB = BulletEngine::getPoseBullet(anchor_inNode2);
		//localA.setIdentity(); localB.setIdentity();
		//btVector3 pivot1 = BulletEngine::getVecBullet(anchor_inNode1.block(0,3,3,1));
		//btVector3 pivot2 = BulletEngine::getVecBullet(anchor_inNode2.block(0,3,3,1));
		//localA.setOrigin(pivot1);
		//localB.setOrigin(pivot2);
		boost::shared_ptr<btGeneric6DofConstraint> generic6Dof(new btGeneric6DofConstraint(*btBody1, *btBody2, localA, localB, true));
		btVector3 limitZero(0,0,0);
		generic6Dof->setAngularLowerLimit(limitZero);
		generic6Dof->setAngularUpperLimit(limitZero);
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
		joint = generic6Dof;
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
	if (!node1->getCollisionModel())
	{
		RobotNodePtr parent = node1->getParent();
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
			parent = parent->getParent();
		}
	}
	links.push_back(i);

	if (enableJointMotors && node2->isRotationalJoint())
	{
		// start standard actuator
		actuateNode(node2,node2->getJointValue());
	}
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
		if (it->second.enabled)
		{
			BulletObjectPtr drn = boost::dynamic_pointer_cast<BulletObject>(it->second.dynNode);
			VR_ASSERT(drn);
			if (it->second.node->isRotationalJoint())
			{
				LinkInfo link = getLink(it->second.node);
				boost::shared_ptr<btHingeConstraint> hinge = boost::dynamic_pointer_cast<btHingeConstraint>(link.joint);
				VR_ASSERT(hinge);
				hinge->enableMotor(true);
				hinge->setMotorTarget(it->second.jointValueTarget+link.jointValueOffset,dt);
			}
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

void BulletRobot::actuateNode( VirtualRobot::RobotNodePtr node, float jointValue )
{
	VR_ASSERT(node);
	if (node->isRotationalJoint())
	{
		LinkInfo link = getLink(node);
		boost::shared_ptr<btHingeConstraint> hinge = boost::dynamic_pointer_cast<btHingeConstraint>(link.joint);
		VR_ASSERT(hinge);
		hinge->enableAngularMotor(true,0.0f,10.0f);// is max impulse ok?! (10 seems to be ok, 1 oscillates)
		DynamicsRobot::actuateNode(node,jointValue);
	} else
	{
		VR_ERROR << "Only Revolute joints implemented so far..." << endl;
	}
}


} // namespace VirtualRobot
