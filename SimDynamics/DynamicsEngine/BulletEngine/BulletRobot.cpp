#include "BulletRobot.h"
#include "BulletEngine.h"
#include "BulletEngineFactory.h"
#include "../../DynamicsWorld.h"
#include "../DynamicsObject.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeFixed.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/Nodes/ForceTorqueSensor.h>

// either hinge or generic6DOF constraints can be used
//#define USE_BULLET_GENERIC_6DOF_CONSTRAINT


#include <boost/pointer_cast.hpp>

//#define DEBUG_FIXED_OBJECTS
//#define DEBUG_SHOW_LINKS
using namespace VirtualRobot;
using namespace std;

namespace SimDynamics {

BulletRobot::BulletRobot(VirtualRobot::RobotPtr rob, bool enableJointMotors)
	: DynamicsRobot(rob)
{
    bulletMaxMotorImulse = 1500.0f;

    bulletMotorVelFactor = 10.0f;
	buildBulletModels(enableJointMotors);

    // activate force torque sensors
    std::vector<SensorPtr>::iterator it = sensors.begin();
    for(; it != sensors.end(); it++)
    {
//        SensorPtr sensor = *it;
        ForceTorqueSensorPtr ftSensor = boost::dynamic_pointer_cast<ForceTorqueSensor>(*it);
        if(ftSensor)
        {
            VirtualRobot::RobotNodePtr node = ftSensor->getRobotNode();//boost::dynamic_pointer_cast<VirtualRobot::RobotNode>(ftSensor->getParent());
            THROW_VR_EXCEPTION_IF(!node, "parent of sensor could not be casted to RobotNode")

            const LinkInfo& link = getLink(node);
            enableForceTorqueFeedback(link);
            std::cout << "Found force torque sensor: " << node->getName() << std::endl;
        }
    }
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
        CollisionModelPtr colModel = rn->getCollisionModel();

        if (colModel)
        {
            addIgnoredCollisionModels(rn);
            // search joint and connected model
            RobotNodePtr bodyA;
            RobotNodePtr bodyB = rn;
            RobotNodePtr joint;
            RobotNodePtr joint2;

            if (rn->isTranslationalJoint() || rn->isRotationalJoint())
                joint = rn;
            RobotNodePtr parent = boost::dynamic_pointer_cast<RobotNode>(rn->getParent());
            while (parent && !bodyA)
            {
                if (!parent->getCollisionModel() && (parent->isTranslationalJoint() || parent->isRotationalJoint()))
                {
                    if (!joint)
                    {
                        joint = parent;
                    } else
                    {
                        // check for hinge2 joint
                        THROW_VR_EXCEPTION_IF(joint2,"three joints in a row not supported:" << joint->getName() << ", " << joint2->getName() << "," << parent->getName());
                        joint2 = parent;
                        Eigen::Matrix4f p1 = joint->getGlobalPoseJoint();
                        Eigen::Matrix4f p2 = joint2->getGlobalPoseJoint();

                        double d = (p1.block(0,3,3,1) - p2.block(0,3,3,1)).norm();
                        THROW_VR_EXCEPTION_IF( (d>1e-6), "Could not create hinge2 joint: Joint coord systems must be located at the same position:" << joint->getName() << ", " << joint2->getName());
                        RobotNodeRevolutePtr rev1 = boost::dynamic_pointer_cast<RobotNodeRevolute>(joint);
                        RobotNodeRevolutePtr rev2 = boost::dynamic_pointer_cast<RobotNodeRevolute>(joint2);
                        THROW_VR_EXCEPTION_IF( !rev1 || !rev2 , "Could not create hinge2 joint: Joints must be revolute nodes:" << joint->getName() << ", " << joint2->getName());
                        Eigen::Vector3f ax1 = rev1->getJointRotationAxis();
                        Eigen::Vector3f ax2 = rev2->getJointRotationAxis();
                        double ang = MathTools::getAngle(ax1,ax2);
                        THROW_VR_EXCEPTION_IF( fabs(fabs(ang)-M_PI_2) > 1e-6, "Could not create hinge2 joint: Joint axes must be orthogonal to each other:" << joint->getName() << ", " << joint2->getName());
                    }
                }
                if (parent->getCollisionModel())
                {
                    //if (!joint)
                    //    THROW_VR_EXCEPTION("Could not build dynamic model: Robotnode " << parent->getName() << " and " << bodyB->getName() << " got a collision model but they are not linked by a joint.");
                    bodyA = parent;
                    break;
                }

                parent = boost::dynamic_pointer_cast<RobotNode>(parent->getParent());
            }
            if (!bodyA)
            {
                // root node
                /*if (joint)
                {
                    THROW_VR_EXCEPTION ("Not able to build valid dynamic model: First node of robot with collision model (" << rn->getName() << ") is connected with a joint (" << joint->getName() <<") but no parent collision model found.");
                }*/
                bodyA = robot->getRootNode();
            }
            //} else
            {
                /* Eigen::Matrix4f trafoA2J = Eigen::Matrix4f::Identity(); // bodyA->joint
                Eigen::Matrix4f trafoJ2B = Eigen::Matrix4f::Identity(); // joint->bodyB
                if (joint)
                {
                    // now we have bodyA->joint->bodyB
                    // compute trafoA2J
                    std::vector<Eigen::Matrix4f> trafosA;
                    // visualization is affected by preJointTransformation
                    trafosA.push_back(joint->getPreJointTransformation());

                    // go parents up until bodyA is reached
                    parent =  boost::dynamic_pointer_cast<RobotNode>(joint->getParent());
                    while (parent && parent!=bodyA)
                    {
                        trafosA.push_back(parent->getPostJointTransformation());
                        trafosA.push_back(parent->getPreJointTransformation());
                        parent = boost::dynamic_pointer_cast<RobotNode>(parent->getParent());
                    }
                    THROW_VR_EXCEPTION_IF(!parent,"internal error, no parent 2");
                    trafosA.push_back(bodyA->getPostJointTransformation());
                    std::vector<Eigen::Matrix4f>::reverse_iterator rit;
                    // accumulate transformations from bodyA_post to joint_pre
                    for (rit=trafosA.rbegin(); rit != trafosA.rend(); rit++)
                    {
                        trafoA2J *= *rit;
                    }

                    // compute trafoJ2B
                    std::vector<Eigen::Matrix4f> trafosB;
               
                    // visualization is affected by preJointTransformation
                    trafosB.push_back(bodyB->getPreJointTransformation());
                    if (joint!=bodyB)
                    {
                        // go parents up until joint is reached
                        parent = boost::dynamic_pointer_cast<RobotNode>(bodyB->getParent());
                        while (parent && parent!=joint)
                        {
                            trafosB.push_back(parent->getPostJointTransformation());
                            trafosB.push_back(parent->getPreJointTransformation());
                            parent = boost::dynamic_pointer_cast<RobotNode>(parent->getParent());
                        }
                        THROW_VR_EXCEPTION_IF(!parent,"internal error, no parent");
                        trafosB.push_back(joint->getPostJointTransformation());
                    }
                    // accumulate transformations from joint_post to bodyB_pre
                    for (rit=trafosB.rbegin(); rit != trafosB.rend(); rit++)
                    {
                        trafoJ2B *= *rit;
                    }
                } else
                {
                    // fixed joint: bodyA -> bodyB
                    joint = bodyB;

                    // compute trafoA2J (which is trafo from A to B)
                    std::vector<Eigen::Matrix4f> trafosA;
                    // visualization is affected by preJointTransformation
                    trafosA.push_back(bodyB->getPreJointTransformation());

                    // go parents up until bodyA is reached
                    parent =  boost::dynamic_pointer_cast<RobotNode>(bodyB->getParent());
                    while (parent && parent!=bodyA)
                    {
                        trafosA.push_back(parent->getPostJointTransformation());
                        trafosA.push_back(parent->getPreJointTransformation());
                        parent = boost::dynamic_pointer_cast<RobotNode>(parent->getParent());
                    }
                    THROW_VR_EXCEPTION_IF(!parent,"internal error, no parent 3");
                    trafosA.push_back(bodyA->getPostJointTransformation());
                    std::vector<Eigen::Matrix4f>::reverse_iterator rit;
                    // accumulate transformations from bodyA_post to joint_pre
                    for (rit=trafosA.rbegin(); rit != trafosA.rend(); rit++)
                    {
                        trafoA2J *= *rit;
                    }

                }*/

                // check for fixed joint
                if (!joint)
                    joint = bodyB;

                createLink(bodyA,joint,joint2,bodyB);//,trafoA2J,trafoJ2B);
            }

        }

    /*
    RobotNodePtr bodyA;
    RobotNodePtr bodyB;
    RobotNodePtr joint;
    Eigen::Matrix4f trafoA = Eigen::Matrix4f;
    Eigen::Matrix4f trafoB = Eigen::Matrix4f;
	for (size_t i=0;i<robotNodes.size();i++)
	{

		RobotNodePtr rn = robotNodes[i];

        switch(rn->getType())
        {
        case RobotNode::Generic:
            THROW_VR_EXCEPTION ("Could not build dynamic model. Only <JointNode> <TransformationNode> and <BodyNode> tags allowed!");
            break;
        case RobotNode::Body:
            break;
        default: 
            continue;
        }
        // a body
        bodyB = rn;
        bodyA.reset();
        joint.reset();
        trafoA = Eigen::Matrix4f;
        trafoB = Eigen::Matrix4f;

        // search parent body
        RobotNodePtr parent = rn->getParent();
        bool built = false;
        while (parent && !built)
        {
            switch (parent->getType())
            {
            case RobotNode::Joint:
                if (joint)
                {
                    THROW_VR_EXCEPTION("Need bodies between joints. Two succeeding joints found in kinematic structure");
                }
                joint = parent;
                break;
            case RobotNode::Body:
                if (!joint)
                {
                    THROW_VR_EXCEPTION("Need joint between body. Two succeeding bodies found in kinematic structure");
                }
                createLink(bodyA,joint,bodyB,trafoA,trafoB);
                built = true;
                break;
            case RobotNode::Transform:
                if (!joint)
                    trafoA *= parent->getPostJointTransformation
                built = true;
                break;
            }
        }

        */

        /*
		std::vector<SceneObjectPtr> children = rn->getChildren();
		for (size_t c=0;c<children.size();c++)
		{
			RobotNodePtr rn2 = boost::dynamic_pointer_cast<RobotNode>(children[c]);
			if (rn2)
				createLink(rn,rn2,enableJointMotors);
		}*/
	}
}

void BulletRobot::addIgnoredCollisionModels(RobotNodePtr rn)
{
    VR_ASSERT (rn);
    if (!rn->getCollisionModel())
        return; // nothing to do: no col model -> no bullet model -> no collisions
    createDynamicsNode(rn);
    std::vector<std::string> ic = rn->getIgnoredCollisionModels();
    RobotPtr robot = rn->getRobot();
    BulletObjectPtr drn1 = boost::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[rn]);
    VR_ASSERT(drn1);

    for (size_t i=0;i<ic.size();i++)
    {
        RobotNodePtr rn2 = robot->getRobotNode(ic[i]);
        if (!rn2)
        {
            VR_ERROR << "Error while processing robot node <" << rn->getName() << ">: Ignored collision model <" << ic[i] << "> is not part of robot..." << endl;
        } else
        {
            createDynamicsNode(rn2);
            BulletObjectPtr drn2 = boost::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[rn2]);
            VR_ASSERT(drn2);
            DynamicsWorld::GetWorld()->getEngine()->disableCollision(drn1.get(),drn2.get());
        }
    }
}

void BulletRobot::createLink( VirtualRobot::RobotNodePtr bodyA, VirtualRobot::RobotNodePtr joint, VirtualRobot::RobotNodePtr joint2, VirtualRobot::RobotNodePtr bodyB, bool enableJointMotors )
{
    // ensure dynamics nodes are created
    createDynamicsNode(bodyA);
    createDynamicsNode(bodyB);

    if (hasLink(bodyA,bodyB))
    {
        THROW_VR_EXCEPTION("Joints are already connected:" << bodyA->getName() << "," << bodyB->getName());
    }
    BulletObjectPtr drn1 = boost::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[bodyA]);
    BulletObjectPtr drn2 = boost::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[bodyB]);
    VR_ASSERT(drn1);
    VR_ASSERT(drn2);
    boost::shared_ptr<btRigidBody> btBody1 = drn1->getRigidBody();
    boost::shared_ptr<btRigidBody> btBody2 = drn2->getRigidBody();
    VR_ASSERT(btBody1);
    VR_ASSERT(btBody2);


    Eigen::Matrix4f coordSystemNode1 = bodyA->getGlobalPoseJoint(); // todo: what if joint is not at 0 ?!
    Eigen::Matrix4f coordSystemNode2 = bodyB->getGlobalPoseJoint();
    Eigen::Matrix4f coordSystemJoint = joint->getGlobalPoseJoint();

    Eigen::Matrix4f anchorPointGlobal = joint->getGlobalPoseJoint();//node1->getGlobalPose() * node2->getPreJointTransformation(); // 

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

    boost::shared_ptr<btTypedConstraint> jointbt;

    double vr2bulletOffset = 0.0f;

    THROW_VR_EXCEPTION_IF (joint->isTranslationalJoint(), "Translational joints nyi...");
    if (joint->isRotationalJoint())
    {
        // create joint
        boost::shared_ptr<RobotNodeRevolute> rnRevJoint = boost::dynamic_pointer_cast<RobotNodeRevolute>(joint);

        // transform axis direction (not position!)
        Eigen::Vector4f axisLocalJoint = Eigen::Vector4f::Zero();
        axisLocalJoint.block(0,0,3,1) =  rnRevJoint->getJointRotationAxisInJointCoordSystem();
        Eigen::Matrix4f tmpGpJoint = coordSystemJoint;
        tmpGpJoint.block(0,3,3,1).setZero(); // coordSystemJoint
        Eigen::Vector4f axisGlobal = tmpGpJoint * axisLocalJoint;

        double limMin,limMax;
        limMin = joint->getJointLimitLo();
        limMax = joint->getJointLimitHi();

        if (joint2)
        {

            VR_WARNING << "HINGE2 Joints are experimental (1:" << joint->getName() << ", 2:" << joint2->getName() << "): Assuming hing2/universal joint is defined as needed by bullet (see universal constraint header documentation)" << endl;
            // UNIVERSAL/HINGE2 joint
            boost::shared_ptr<RobotNodeRevolute> rnRevJoint2 = boost::dynamic_pointer_cast<RobotNodeRevolute>(joint2);
            THROW_VR_EXCEPTION_IF (!rnRevJoint2, "Second joint must be a revolute joint...");
            Eigen::Matrix4f coordSystemJoint2 = joint2->getGlobalPoseJoint();

            Eigen::Vector4f axisLocalJoint2= Eigen::Vector4f::Zero();
            axisLocalJoint2.block(0,0,3,1) =  rnRevJoint2->getJointRotationAxisInJointCoordSystem();
            Eigen::Matrix4f tmpGpJoint2 = coordSystemJoint2;
            tmpGpJoint2.block(0,3,3,1).setZero();

            Eigen::Vector4f axisGlobal2 = tmpGpJoint2 * axisLocalJoint2;


            btVector3 axis1 = BulletEngine::getVecBullet(axisGlobal.head(3),false);
            btVector3 axis2 = BulletEngine::getVecBullet(axisGlobal2.head(3),false);
            btVector3 pivot = BulletEngine::getVecBullet(anchorPointGlobal.block(0,3,3,1));
            boost::shared_ptr<btUniversalConstraint> hinge2(new btUniversalConstraint(*btBody1, *btBody2, pivot, axis1, axis2));
            double limMin2,limMax2;
            limMin2 = joint2->getJointLimitLo();
            limMax2 = joint2->getJointLimitHi();
            hinge2->setLowerLimit(btScalar(limMin),btScalar(limMin2));
            hinge2->setUpperLimit(btScalar(limMax),btScalar(limMax2));
            jointbt = hinge2;
        } else
        {
            // HINGE joint
            Eigen::Matrix4f tmpGp1 = coordSystemNode1;
            tmpGp1.block(0,3,3,1).setZero();
            Eigen::Matrix4f tmpGp2 = coordSystemNode2;
            tmpGp2.block(0,3,3,1).setZero();
            Eigen::Vector3f axis_inLocal1 = (tmpGp1.inverse() * axisGlobal).block(0,0,3,1);
            Eigen::Vector3f axis_inLocal2 = (tmpGp2.inverse() * axisGlobal).block(0,0,3,1);
            //Eigen::Vector3f axis_inLocal2 = rnRev2->getJointRotationAxisInJointCoordSystem();

            btVector3 pivot1 = BulletEngine::getVecBullet(anchor_inNode1.block(0,3,3,1));
            btVector3 pivot2 = BulletEngine::getVecBullet(anchor_inNode2.block(0,3,3,1));
        
        

        #ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
            THROW_VR_EXCEPTION ("USE_BULLET_GENERIC_6DOF_CONSTRAINT nyi in this method...");
        #endif
            //btVector3 axis1 = BulletEngine::getVecBullet(axis_inLocal1,false);
            //btVector3 axis2 = BulletEngine::getVecBullet(axis_inLocal2,false);
            //boost::shared_ptr<btHingeConstraint> hinge(new btHingeConstraint(*btBody1, *btBody2, pivot1, pivot2, axis1, axis2,false));

            // we need to align coord system joint, so that z-axis is rotation axis
            Eigen::Vector3f axisGlobal = rnRevJoint->getJointRotationAxis();
            Eigen::Vector3f axisLocal = rnRevJoint->getJointRotationAxisInJointCoordSystem();
            MathTools::Quaternion q1 = MathTools::getRotation(Eigen::Vector3f::UnitZ(),axisLocal);
            Eigen::Matrix4f rotationzAlignment = MathTools::quat2eigen4f(q1);
            Eigen::Matrix4f coordSystemJoint_zAligned =  coordSystemJoint * rotationzAlignment ;

            // get transformation coord1 -> joint coord
            Eigen::Matrix4f trafoNode1 = coordSystemNode1.inverse() * coordSystemJoint_zAligned; 
            // get transformation coord2 -> joint coord
            Eigen::Matrix4f trafoNode2 = coordSystemNode2.inverse() * coordSystemJoint_zAligned; 

            // now we need to pivot points in local coord systems
            btTransform tr1 = BulletEngine::getPoseBullet(trafoNode1);
            btTransform tr2 = BulletEngine::getPoseBullet(trafoNode2);
            tr1.getOrigin() = pivot1;
            tr2.getOrigin() = pivot2;

            boost::shared_ptr<btHingeConstraint> hinge(new btHingeConstraint(*btBody1, *btBody2, tr1, tr2, true));
            

            // todo: check effects of parameters...
            hinge->setParam(BT_CONSTRAINT_STOP_ERP,0.9f);
            /*
            hinge->setParam(BT_CONSTRAINT_CFM,0.9f);
            hinge->setParam(BT_CONSTRAINT_STOP_CFM,0.01f);*/
            //hinge->setLimit(limMin,limMax);//,btScalar(1.0f));
            //hinge->setParam(BT_CONSTRAINT_CFM,1.0f);

            btScalar startAngle = joint->getJointValue();
            btScalar startAngleBT = hinge->getHingeAngle();
            btScalar limMinBT, limMaxBT;
            btScalar diff = joint->getJointValueOffset();//startAngleBT + startAngle);
            limMinBT = limMin + diff;//diff - limMax;// 
            limMaxBT = limMax + diff;//diff - limMin;// 
            // what does it mean if there are different startAngles?!
            /*if (fabs(startAngleBT - startAngle)>1e-6)
            {
	            cout << "joint " << joint->getName() << ": jv diff:" << diff << endl;
	            cout << "Simox limits: " << limMin << "/" << limMax << ", bullet limits:" << limMinBT << "/" << limMaxBT << endl;
            }*/
            hinge->setLimit(btScalar(limMinBT),btScalar(limMaxBT));
             vr2bulletOffset = diff;
            //hinge->setLimit(btScalar(limMin),btScalar(limMax));
            //hinge->setAngularOnly(true);

            jointbt = hinge;
        }
    } else
    {
        // create fixed joint
        btTransform localA,localB;
		localA = BulletEngine::getPoseBullet(anchor_inNode1);
		localB = BulletEngine::getPoseBullet(anchor_inNode2);
		boost::shared_ptr<btGeneric6DofConstraint> generic6Dof(new btGeneric6DofConstraint(*btBody1, *btBody2, localA, localB, true));
        generic6Dof->setOverrideNumSolverIterations(100);

        for (int i=0;i<6;i++)
            generic6Dof->setLimit(i,0,0);

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
		jointbt = generic6Dof;
    }
	LinkInfo i;
	i.nodeA = bodyA;
	i.nodeB = bodyB;
	i.dynNode1 = drn1;
	i.dynNode2 = drn2;
    i.nodeJoint = joint;
    i.nodeJoint2 = joint2;
    i.joint = jointbt;
	i.jointValueOffset = vr2bulletOffset;

    // disable col model 
    i.disabledCollisionPairs.push_back(
        std::pair<DynamicsObjectPtr,DynamicsObjectPtr>(
        boost::dynamic_pointer_cast<DynamicsObject>(drn1),
        boost::dynamic_pointer_cast<DynamicsObject>(drn2)));

	links.push_back(i);
#ifndef DEBUG_FIXED_OBJECTS
	if (enableJointMotors && joint->isRotationalJoint())
	{
		// start standard actuator
		actuateNode(joint,joint->getJointValue());
	}
#endif
}
/*
void BulletRobot::createLink( VirtualRobot::RobotNodePtr node1,VirtualRobot::RobotNodePtr node2, bool enableJointMotors )
{
	THROW_VR_EXCEPTION_IF (!(node1->hasChild(node2)),"node1 must be parent of node2");

    // ensure dynamics nodes are created
    createDynamicsNode(node1);
    createDynamicsNode(node2);

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

	 double vr2bulletOffset = 0.0f;

	 if (node2->isTranslationalJoint())
	 {
		 VR_WARNING << "translational joint nyi, creating a fixed link..." << endl;
     }rnRevJoint->getJointRotationAxis();
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

	
        double limMin,limMax;
		limMin = node2->getJointLimitLo();
		limMax = node2->getJointLimitHi();

#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT

        MathTools::Quaternion axisRotQuat1 = MathTools::getRotation(axis_inLocal1,Eigen::Vector3f::UnitX());
        MathTools::Quaternion axisRotQuat2 = MathTools::getRotation(axis_inLocal2,Eigen::Vector3f::UnitX());
        Eigen::Matrix4f axisRot1 = MathTools::quat2eigen4f(axisRotQuat1);
        Eigen::Matrix4f axisRot2 = MathTools::quat2eigen4f(axisRotQuat2);
        double ang1,ang2;
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
        }* /
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
    } else* /
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

        //double totalMass = 1.f/btBody1->getInvMass() + 1.f/btBody2->getInvMass();

        //generic6Dof->setBreakingImpulseThreshold(2*totalMass);//needed? copied from voronoi demo

        for (int i=0;i<6;i++)
            generic6Dof->setLimit(i,0,0);

		joint = generic6Dof;
#endif
	}
	LinkInfo i;
	i.nodeA = node1;
    i.nodeB = node2;
    i.nodeJoint = node2;
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
*/

bool BulletRobot::hasLink( VirtualRobot::RobotNodePtr node1, VirtualRobot::RobotNodePtr node2 )
{
	for (size_t i=0; i<links.size();i++)
	{
		if (links[i].nodeA == node1 && links[i].nodeB == node2)
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

void BulletRobot::actuateJoints(btScalar dt)
{
    //cout << "=== === BulletRobot: actuateJoints() 1 === " << endl;

    std::map<VirtualRobot::RobotNodePtr, robotNodeActuationTarget>::iterator it = actuationTargets.begin();

    int jointCounter = 0;

    for (; it != actuationTargets.end(); it++)
    {
        //BulletObjectPtr drn;
        //if (it->second.dynNode)
        //	drn = boost::dynamic_pointer_cast<BulletObject>(it->second.dynNode);
        //VR_ASSERT(drn);

        VelocityMotorController& controller = actuationControllers[it->first];

        if (it->second.node->isRotationalJoint())
        {
            LinkInfo link = getLink(it->second.node);

            const ActuationMode& actuation = it->second.actuation;

            btScalar posTarget = btScalar(it->second.jointValueTarget + link.jointValueOffset);
            btScalar posActual = btScalar(getJointAngle(it->first));
            btScalar velocityTarget = it->second.jointVelocityTarget;

#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
            boost::shared_ptr<btGeneric6DofConstraint> dof = boost::dynamic_pointer_cast<btGeneric6DofConstraint>(link.joint);
            VR_ASSERT(dof);
            btRotationalLimitMotor *m = dof->getRotationalLimitMotor(0);
            VR_ASSERT(m);

            if (actuation.mode == 0) {
                m->m_enableMotor = false;
                continue;
            }

            m->m_enableMotor = true;

            if (actuation.modes.position && actuation.modes.velocity)
            {
                m->m_targetVelocity = controller.update(posTarget - posActual, velocityTarget, actuation, dt);
            }
            else if (actuation.modes.position)
            {
                m->m_targetVelocity = controller.update(posTarget - posActual, 0, actuation, dt);
            }
            else if (actuation.modes.velocity)
            {
                m->m_targetVelocity = controller.update(0, velocityTarget, actuation, dt);
            }

            // FIXME torque based control is ignored
#else
            boost::shared_ptr<btHingeConstraint> hinge = boost::dynamic_pointer_cast<btHingeConstraint>(link.joint);

            if (actuation.mode == 0) {
                hinge->enableMotor(false);
                continue;
            }

            if (actuation.modes.position && actuation.modes.velocity)
            {
                hinge->enableAngularMotor(true,
                        controller.update(posTarget - posActual, velocityTarget, actuation, dt),
                        bulletMaxMotorImulse);

            }
            else if (actuation.modes.position)
            {
                hinge->enableAngularMotor(true,
                        controller.update(posTarget - posActual, 0.0, actuation, dt),
                        bulletMaxMotorImulse);
            }
            else if (actuation.modes.velocity)
            {
                hinge->enableAngularMotor(true,
                        controller.update(0.0, velocityTarget, actuation, dt),
                        bulletMaxMotorImulse);
            }
            // FIXME this bypasses the controller (and doesn't work..)
            else if (actuation.modes.torque)
            {
                //Only first try (using torques as velocity targets...)
                hinge->enableAngularMotor(true,it->second.jointTorqueTarget,bulletMaxMotorImulse);
                //cout << "jointTorqueTarget for joint " << it->second.node->getName() << " :" << it->second.jointTorqueTarget << endl;

                /*

                //=======
                //Here is some code that sets torques directly to the finger joints (bypassing the Bullet motors).
                //Unfortunately, this does not seem to work, so far.
                //1. With hinge->enableAngularMotor(true,...), the fingers do not move at all.
                //2. Wtih hinge->enableAngularMotor(false,...), the fingers are simply actuated by gravity...
                //=======

                //cout << " === == === === === > BulletRobot (hinge !): eTorque NEW!!! ====" << endl;

                cout << "Disabling Angular Motor... " << endl;
                hinge->enableAngularMotor(false,0,bulletMaxMotorImulse);


                cout << "=== === === jointCounter: " << jointCounter << " === === ===" << endl;

                //get the links that are connected by the hinge.
                btRigidBody rbA = hinge->getRigidBodyA();
                btRigidBody rbB = hinge->getRigidBodyB();

                //get joint axis from the hinge ...
                btMatrix3x3 rbAFrameBasis = hinge->getAFrame().getBasis();
                //z-Achse ist Gelenkachse? (das steht in btHingeConstraint.h; setAxis() bzw. struct btHingeConstraintDoubleData)
                btVector3 hingeAxis = rbAFrameBasis.getColumn(2);

                //calc 3dim torque by multiplication with joint axis!
                btVector3 resTorqueA = hingeAxis * it->second.jointTorqueTarget;


                //TODO (maybe): calc "realistic" torque to be applied (using dt)

                //apply torques to the bodies connected by the joint
                rbA.applyTorqueImpulse(resTorqueA);
                rbB.applyTorqueImpulse(-resTorqueA);

                //DEBUG OUT:
                //--> TODO!
                //cout << "==== ==== ==== DEBUG OUT: ==== ==== ==== " << endl;
                cout << "rbAFrameBasis:" << endl;
                btVector3 row0 = rbAFrameBasis.getRow(0);
                btVector3 row1 = rbAFrameBasis.getRow(1);
                btVector3 row2 = rbAFrameBasis.getRow(2);
                cout << row0.getX() << " " << row0.getY() << " " << row0.getZ() << endl;
                cout << row1.getX() << " " << row1.getY() << " " << row1.getZ() << endl;
                cout << row2.getX() << " " << row2.getY() << " " << row2.getZ() << endl;

                cout << "hingeAxis: " << hingeAxis.getX() << " " << hingeAxis.getY() << " " << hingeAxis.getZ() << endl;
                cout << "resTorqueA: " << resTorqueA.getX() << " " << resTorqueA.getY() << " " << resTorqueA.getZ() << endl;



                jointCounter++;

*/
            }

            // Universal constraint instead of hinge constraint
            /*
               boost::shared_ptr<btUniversalConstraint> hinge = boost::dynamic_pointer_cast<btUniversalConstraint>(link.joint);
               VR_ASSERT(hinge2);
               btRotationalLimitMotor *m;
               if (it->second.node==link.nodeJoint)
               {
               m = hinge2->getRotationalLimitMotor(1); // second motor
               } else
               {
               VR_ASSERT(it->second.node==link.nodeJoint2);
               m = hinge2->getRotationalLimitMotor(2); // third motor
               }
               VR_ASSERT(m);
               switch (it->second.actuation)
               {
               case ePosition:
               {
               btScalar targ = btScalar(it->second.jointValueTarget+link.jointValueOffset);
            //btScalar act = btScalar(it->first->getJointValue());
            btScalar act = btScalar(getJointAngle(it->first));
            m->m_enableMotor = true;
            m->m_targetVelocity = (targ-act)*bulletMotorVelFactor;
            }
            break;

            case eVelocity:
            {
            m->m_enableMotor = true;
            m->m_targetVelocity = it->second.jointVelocityTarget;
            break;
            }
            case eTorque:
            {
            m->m_enableMotor = true;
            m->m_targetVelocity = it->second.jointVelocityTarget;
            break;
            }
            case ePositionVelocity:
            {
            btScalar pos = btScalar(getJointAngle(it->first));
            double gain = 0.5;
            m->m_targetVelocity = it->second.jointVelocityTarget + gain*(it->second.jointValueTarget - pos) / dt;
            break;
            }

            default:
            m->m_enableMotor = false;
            }
            */
#endif
        }
    }

    setPoseNonActuatedRobotNodes();
}

void BulletRobot::updateSensors()
{
    std::vector<SensorPtr>::iterator it = sensors.begin();
    for(; it != sensors.end(); it++)
    {
//        SensorPtr sensor = *it;
        ForceTorqueSensorPtr ftSensor = boost::dynamic_pointer_cast<ForceTorqueSensor>(*it);
        if(ftSensor)
        {
            VirtualRobot::RobotNodePtr node = ftSensor->getRobotNode();
                    //boost::dynamic_pointer_cast<VirtualRobot::RobotNode>(ftSensor->getParent());
            THROW_VR_EXCEPTION_IF(!node, "parent of sensor could not be casted to RobotNode")

            const LinkInfo& link = getLink(node);
            Eigen::VectorXf forceTorques = getJointForceTorqueGlobal(link);
            ftSensor->updateSensors(forceTorques);
            std::cout << "Updating force torque sensor: " << node->getName() << ": " << forceTorques << std::endl;
        }
    }
}

BulletRobot::LinkInfo BulletRobot::getLink( VirtualRobot::RobotNodePtr node )
{
	for (size_t i=0;i<links.size();i++)
	{
		if (links[i].nodeJoint == node || links[i].nodeJoint2 == node)
			return links[i];
	}
	THROW_VR_EXCEPTION("No link with node " << node->getName());
	return LinkInfo();
}

std::vector<BulletRobot::LinkInfo> BulletRobot::getLinks( VirtualRobot::RobotNodePtr node )
{
    std::vector<BulletRobot::LinkInfo> result;
	for (size_t i=0;i<links.size();i++)
	{
		if (links[i].nodeJoint == node || links[i].nodeJoint2 == node || links[i].nodeA == node || links[i].nodeB == node)
			result.push_back(links[i]);
	}
	return result;
}

bool BulletRobot::hasLink( VirtualRobot::RobotNodePtr node )
{
	for (size_t i=0;i<links.size();i++)
	{
		if (links[i].nodeJoint == node || links[i].nodeJoint2 == node)
			return true;
	}
	return false;
}

void BulletRobot::actuateNode( VirtualRobot::RobotNodePtr node, double jointValue )
{
	VR_ASSERT(node);

	if (node->isRotationalJoint())
	{
        if (!hasLink(node))
        {
            VR_ERROR << "No link for node " << node->getName() << endl;
            return;
        }
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
        if (!hinge)
        {
            // hinge2 / universal joint
            boost::shared_ptr<btUniversalConstraint> hinge2 = boost::dynamic_pointer_cast<btUniversalConstraint>(link.joint);
		    VR_ASSERT(hinge2);
            btRotationalLimitMotor *m;
            if (node==link.nodeJoint)
            {
                 m = hinge2->getRotationalLimitMotor(1); // second motor
            } else
            {
                VR_ASSERT(node==link.nodeJoint2);
                m = hinge2->getRotationalLimitMotor(2); // third motor
            }
            VR_ASSERT(m);
            m->m_enableMotor = true;
            m->m_maxMotorForce = 5;//bulletMaxMotorImulse; //?!
            m->m_maxLimitForce = 300;
        } else
        {
            //hinge->enableAngularMotor(true,0.0f,bulletMaxMotorImulse);// is max impulse ok?! (10 seems to be ok, 1 oscillates)
        }
        DynamicsRobot::actuateNode(node,jointValue);
#endif
    } else
	{
		VR_ERROR << "Only Revolute joints implemented so far..." << endl;
    }
}

void BulletRobot::actuateNodeVel(RobotNodePtr node, double jointVelocity)
{
    VR_ASSERT(node);

    if (node->isRotationalJoint())
    {
        if (!hasLink(node))
        {
            VR_ERROR << "No link for node " << node->getName() << endl;
            return;
        }
        LinkInfo link = getLink(node);
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
        boost::shared_ptr<btGeneric6DofConstraint> dof = boost::dynamic_pointer_cast<btGeneric6DofConstraint>(link.joint);
        VR_ASSERT(dof);
        btRotationalLimitMotor *m = dof->getRotationalLimitMotor(0);
        VR_ASSERT(m);
        m->m_enableMotor = true;
        m->m_maxMotorForce = 5;//bulletMaxMotorImulse; //?!
        m->m_maxLimitForce = 300;
        DynamicsRobot::actuateNodeVel(node,jointVelocity);
#else
        boost::shared_ptr<btHingeConstraint> hinge = boost::dynamic_pointer_cast<btHingeConstraint>(link.joint);
        if (!hinge)
        {
            // hinge2 / universal joint
            boost::shared_ptr<btUniversalConstraint> hinge2 = boost::dynamic_pointer_cast<btUniversalConstraint>(link.joint);
            VR_ASSERT(hinge2);
            btRotationalLimitMotor *m;
            if (node==link.nodeJoint)
            {
                 m = hinge2->getRotationalLimitMotor(1); // second motor
            } else
            {
                VR_ASSERT(node==link.nodeJoint2);
                m = hinge2->getRotationalLimitMotor(2); // third motor
            }
            VR_ASSERT(m);
            m->m_enableMotor = true;
            m->m_maxMotorForce = 5;//bulletMaxMotorImulse; //?!
            m->m_maxLimitForce = 300;
        } else
        {
            //hinge->enableAngularMotor(true,jointVelocity,bulletMaxMotorImulse);// is max impulse ok?! (10 seems to be ok, 1 oscillates)
        }
        DynamicsRobot::actuateNodeVel(node,jointVelocity); // inverted joint direction in bullet
#endif
    } else
    {
        VR_ERROR << "Only Revolute joints implemented so far..." << endl;
    }
}

double BulletRobot::getJointAngle( VirtualRobot::RobotNodePtr rn )
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
    double a1 = dof->getAngle(0);
    double a2 = m->m_currentPosition;
    if (fabs(a1-a2)>0.05f)
    {
        VR_INFO << "Angle diff " << a1 << ", " << a2 << endl;
    }
    return (a2-link.jointValueOffset);// inverted joint direction in bullet
#else
	boost::shared_ptr<btHingeConstraint> hinge = boost::dynamic_pointer_cast<btHingeConstraint>(link.joint);
	if (!hinge)
	{
        // hinge2 / universal joint
        boost::shared_ptr<btUniversalConstraint> hinge2 = boost::dynamic_pointer_cast<btUniversalConstraint>(link.joint);
        if (!hinge2)
            return 0.0f;
        btRotationalLimitMotor *m;
        if (rn==link.nodeJoint)
        {
            m = hinge2->getRotationalLimitMotor(1); // second motor
        } else if (rn==link.nodeJoint2)
        {
            m = hinge2->getRotationalLimitMotor(2); // third motor
        } else
            return 0.0f;
        VR_ASSERT(m);
        hinge2->calculateTransforms();
        double a2 = m->m_currentPosition;
        return (a2-link.jointValueOffset);// inverted joint direction in bullet
	}
    return (hinge->getHingeAngle()-link.jointValueOffset);// inverted joint direction in bullet
#endif
}

double BulletRobot::getJointTargetSpeed( VirtualRobot::RobotNodePtr rn )
{
    VR_ASSERT(rn);
    if (!hasLink(rn))
    {
        VR_ERROR << "No link with node " << rn->getName() << endl;
        return 0.0f;
    }
    LinkInfo link = getLink(rn);
    boost::shared_ptr<btHingeConstraint> hinge = boost::dynamic_pointer_cast<btHingeConstraint>(link.joint);
    if (!hinge)
    {
        // hinge2 / universal joint
        boost::shared_ptr<btUniversalConstraint> hinge2 = boost::dynamic_pointer_cast<btUniversalConstraint>(link.joint);
        if (!hinge2)
            return 0.0f;
        btRotationalLimitMotor *m;
        if (rn==link.nodeJoint)
        {
            m = hinge2->getRotationalLimitMotor(1); // second motor
        } else if (rn==link.nodeJoint2)
        {
            m = hinge2->getRotationalLimitMotor(2); // third motor
        } else
            return 0.0f;
        VR_ASSERT(m);
        return m->m_targetVelocity;
    }
    return hinge->getMotorTargetVelosity();
}

double BulletRobot::getJointSpeed( VirtualRobot::RobotNodePtr rn )
{
    VR_ASSERT(rn);
	if (!hasLink(rn))
	{
		VR_ERROR << "No link with node " << rn->getName() << endl;
		return 0.0f;
	}
	LinkInfo link = getLink(rn);
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
    VR_WARNING << "NYI" << endl;
    return 0.0;
#else
	boost::shared_ptr<btHingeConstraint> hinge = boost::dynamic_pointer_cast<btHingeConstraint>(link.joint);
	if (!hinge)
    { 
        VR_WARNING << "NYI" << endl;
       return 0.0;
	}

    Eigen::Vector3f deltaVel = link.dynNode1->getAngularVelocity() - link.dynNode2->getAngularVelocity();
    boost::shared_ptr<RobotNodeRevolute> rnRevJoint = boost::dynamic_pointer_cast<RobotNodeRevolute>(link.nodeJoint);
    double speed = deltaVel.dot(rnRevJoint->getJointRotationAxis());
    return speed;//hinge->getMotorTargetVelosity();
#endif
}

double BulletRobot::getNodeTarget( VirtualRobot::RobotNodePtr node )
{
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
    return DynamicsRobot::getNodeTarget(node);
#else
	return DynamicsRobot::getNodeTarget(node);
#endif

}

Eigen::Vector3f BulletRobot::getJointTorques(RobotNodePtr rn)
{
    VR_ASSERT(rn);
    Eigen::Vector3f result;
    result.setZero();
    if (!hasLink(rn))
    {
        VR_ERROR << "No link with node " << rn->getName() << endl;
        return result;
    }
    LinkInfo link = getLink(rn);




    if(rn->isRotationalJoint())
    {
        result = getJointForceTorqueGlobal(link).tail(3);
    }

    return result;
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

Eigen::Vector3f BulletRobot::getComGlobal( VirtualRobot::RobotNodeSetPtr set)
{
	Eigen::Vector3f com = Eigen::Vector3f::Zero();
	double totalMass = 0.0;
	for (int i = 0; i < set->getSize(); i++)
	{
		VirtualRobot::RobotNodePtr node = (*set)[i];
		BulletObjectPtr bo = boost::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
		Eigen::Matrix4f pose = bo->getComGlobal();
		com += node->getMass() * pose.block(0, 3, 3, 1);
		totalMass += node->getMass();
	}

	com *= 1.0f/totalMass;
	return com;
}

Eigen::Vector3f BulletRobot::getComGlobalVelocity( VirtualRobot::RobotNodeSetPtr set)
{
	Eigen::Vector3f com = Eigen::Vector3f::Zero();
	double totalMass = 0.0;
	for (int i = 0; i < set->getSize(); i++)
	{
		VirtualRobot::RobotNodePtr node = (*set)[i];
		BulletObjectPtr bo = boost::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
		Eigen::Vector3f vel = bo->getLinearVelocity();
		com += node->getMass() * vel;
		totalMass += node->getMass();
	}

	com *= 1.0f/totalMass;
	return com;
}

void BulletRobot::setPoseNonActuatedRobotNodes()
{
    VR_ASSERT(robot);
    std::vector<RobotNodePtr> rns = robot->getRobotNodes();
    std::vector<RobotNodePtr> actuatedNodes;
    std::vector<RobotNodePtr> notActuatedNodes;
    size_t i;
    // only objects with collisionmodel are processed by bullet
    for (i=0;i<rns.size();i++)
    {
        if (rns[i]->getCollisionModel())
            actuatedNodes.push_back(rns[i]);
        else
            notActuatedNodes.push_back(rns[i]);
    }
    size_t lastSize = notActuatedNodes.size();
    while (notActuatedNodes.size()>0)
    {
        vector<RobotNodePtr>::iterator it = notActuatedNodes.begin();
        while (it!=notActuatedNodes.end())
        {
            SceneObjectPtr parent = (*it)->getParent();
            if (!parent || find(actuatedNodes.begin(), actuatedNodes.end(), parent)!=actuatedNodes.end())
            {
                // parent is at correct pose, we can update *it
                if (parent)
                    (*it)->updatePose(false);

                // if root, we also have to delete node from list
                actuatedNodes.push_back(*it);
                it = notActuatedNodes.erase(it);
            } else
                it++;
        }


        // just a sanity check
        if (lastSize ==  notActuatedNodes.size())
        {
            VR_ERROR << "Internal error?!" << endl;
            return;
        } else
            lastSize =  notActuatedNodes.size();
    }
}

void BulletRobot::enableForceTorqueFeedback(const LinkInfo& link , bool enable)
{
    if (!link.joint->needsFeedback() && enable)
	{
        link.joint->enableFeedback(true);
		btJointFeedback* feedback = new btJointFeedback;
		feedback->m_appliedForceBodyA = btVector3(0, 0, 0);
		feedback->m_appliedForceBodyB = btVector3(0, 0, 0);
		feedback->m_appliedTorqueBodyA = btVector3(0, 0, 0);
		feedback->m_appliedTorqueBodyB = btVector3(0, 0, 0);
		link.joint->setJointFeedback(feedback);
	}
    else if(link.joint->needsFeedback() && !enable)
    {
        link.joint->enableFeedback(false);
    }
}

Eigen::VectorXf BulletRobot::getForceTorqueFeedbackA( const LinkInfo& link )
{
	Eigen::VectorXf r(6);
	r.setZero();
	if (!link.joint || !link.joint->needsFeedback())
	{
		return r;
	}

	btJointFeedback* feedback = link.joint->getJointFeedback();
	if (!feedback)
		return r;
	r << feedback->m_appliedForceBodyA[0],feedback->m_appliedForceBodyA[1],feedback->m_appliedForceBodyA[2],feedback->m_appliedTorqueBodyA[0],feedback->m_appliedTorqueBodyA[1],feedback->m_appliedTorqueBodyA[2];
	return r;
}
Eigen::VectorXf BulletRobot::getForceTorqueFeedbackB( const LinkInfo& link )
{
	Eigen::VectorXf r(6);
	r.setZero();
	if (!link.joint || !link.joint->needsFeedback())
	{
		return r;
	}

	btJointFeedback* feedback = link.joint->getJointFeedback();
	if (!feedback)
		return r;
	r << feedback->m_appliedForceBodyB[0],feedback->m_appliedForceBodyB[1],feedback->m_appliedForceBodyB[2],feedback->m_appliedTorqueBodyB[0],feedback->m_appliedTorqueBodyB[1],feedback->m_appliedTorqueBodyB[2];
    return r;
}

Eigen::VectorXf BulletRobot::getJointForceTorqueGlobal(const BulletRobot::LinkInfo &link)
{
    Eigen::VectorXf ftA = getForceTorqueFeedbackA(link);
    Eigen::VectorXf ftB = getForceTorqueFeedbackB(link);

    Eigen::Vector3f jointGlobal = link.nodeJoint->getGlobalPose().block(0,3,3,1);
    Eigen::Vector3f comBGlobal = link.nodeB->getCoMGlobal();

    // force that is applied on objectA by objectB -> so the force that object B applies on the joint
    Eigen::Vector3f forceOnBGlobal =  ftB.head(3);

    Eigen::Vector3f torqueBGlobal =  ftB.tail(3);

    // the lever from Object B CoM to Joint
    Eigen::Vector3f leverOnJoint = (comBGlobal-jointGlobal) * 0.001f;
    // Calculate the torque in Joint by taking the torque that presses on the CoM of BodyB and the Torque of BodyB on the joint
    // forceOnBGlobal is inverted in next line because it is the force of A on B to hold it in position
    // torqueBGlobal is inverted in next line because it is the torque on B from A to compensate torque of other objects (which is the torque we would like) to hold it in place and therefore needs to be inverted as well
    Eigen::Vector3f torqueJointGlobal =  (leverOnJoint).cross(-forceOnBGlobal)  + (-1)*torqueBGlobal;
    Eigen::VectorXf result(6);
    result.head(3) = ftA.head(3); // force in joint is same as force on CoM of A
    result.tail(3) = torqueJointGlobal;
    return result;
}

} // namespace VirtualRobot
