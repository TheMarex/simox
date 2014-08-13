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
#include <VirtualRobot/Nodes/ContactSensor.h>

// either hinge or generic6DOF constraints can be used
//#define USE_BULLET_GENERIC_6DOF_CONSTRAINT

#include <boost/pointer_cast.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

#include <boost/math/special_functions/fpclassify.hpp>

//#define DEBUG_FIXED_OBJECTS
//#define DEBUG_SHOW_LINKS
using namespace VirtualRobot;
using namespace std;

namespace SimDynamics {

BulletRobot::BulletRobot(VirtualRobot::RobotPtr rob, bool enableJointMotors)
	: DynamicsRobot(rob)
{
    bulletMaxMotorImulse = 800.0f;

	buildBulletModels(enableJointMotors);

    // activate force torque sensors
    std::vector<SensorPtr>::iterator it = sensors.begin();
    for(; it != sensors.end(); it++)
    {
        ForceTorqueSensorPtr ftSensor = boost::dynamic_pointer_cast<ForceTorqueSensor>(*it);
        if(ftSensor)
        {
            VirtualRobot::RobotNodePtr node = ftSensor->getRobotNode();
            THROW_VR_EXCEPTION_IF(!node, "parent of sensor could not be casted to RobotNode")

            const LinkInfo& link = getLink(node);
            enableForceTorqueFeedback(link);
            std::cout << "Found force torque sensor: " << node->getName() << std::endl;
        }
    }
#ifdef 0
    std::string nameBodies = "BulletRobot_RNS_Bodies_All";
    std::vector<RobotNodePtr> rnAll = robot->getRobotNodes();
    std::vector<RobotNodePtr> rnsBodies;
    for (size_t i=0;i<rnAll.size();i++)
    {
        RobotNodePtr r = rnAll[i];
        //if (r->isRotationalJoint() || r->isTranslationalJoint())
        //    rnsJoints.push_back(r);
        if (r->getMass()>0)
            rnsBodies.push_back(r);
    }
    RobotNodeSetPtr rnsB = RobotNodeSet::createRobotNodeSet(getRobot(),nameBodies,rnsBodies,RobotNodePtr(),RobotNodePtr(),true);
#endif
}
	
BulletRobot::~BulletRobot()
{
}

void BulletRobot::buildBulletModels(bool enableJointMotors)
{
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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
                    bodyA = parent;
                    break;
                }

                parent = boost::dynamic_pointer_cast<RobotNode>(parent->getParent());
            }
            if (!bodyA)
            {
                bodyA = robot->getRootNode();
            }

            // check for fixed joint
            if (!joint)
                joint = bodyB;

            createLink(bodyA,joint,joint2,bodyB);
        }

	}
}

void BulletRobot::addIgnoredCollisionModels(RobotNodePtr rn)
{
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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
            limMinBT = btScalar(limMin) + diff;//diff - limMax;// 
            limMaxBT = btScalar(limMax) + diff;//diff - limMin;// 
            hinge->setLimit(btScalar(limMinBT),btScalar(limMaxBT));
            vr2bulletOffset = diff;
            jointbt = hinge;
        }
    } else
    {
        VR_WARNING << "Creating fixed joint between " << bodyA->getName() << " and " << bodyB->getName() << ". This might result in some artefacts (e.g. no strict ridgid connection)" << endl;
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

bool BulletRobot::hasLink( VirtualRobot::RobotNodePtr node1, VirtualRobot::RobotNodePtr node2 )
{
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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

void BulletRobot::actuateJoints(double dt)
{
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
    //cout << "=== === BulletRobot: actuateJoints() 1 === " << this << endl;

    std::map<VirtualRobot::RobotNodePtr, robotNodeActuationTarget>::iterator it = actuationTargets.begin();

    //int jointCounter = 0;
    //cout << "**** Control Values: ";

    for (; it != actuationTargets.end(); it++)
    {
        //cout << "it:" << it->first << ", name: " << it->first->getName() << endl;
        VelocityMotorController& controller = actuationControllers[it->first];

        if (it->second.node->isRotationalJoint())
        {
            LinkInfo link = getLink(it->second.node);

            const ActuationMode& actuation = it->second.actuation;

            btScalar posTarget = btScalar(it->second.jointValueTarget + link.jointValueOffset);
            btScalar posActual = btScalar(getJointAngle(it->first));
            btScalar velActual = btScalar(getJointSpeed(it->first));
            btScalar velocityTarget = btScalar(it->second.jointVelocityTarget);

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
            double targetVelocity;
            if (actuation.modes.position && actuation.modes.velocity)
            {
                //cout << "################### posActual:" << posActual << ", posTarget" << posTarget << ", velTarget:" << velocityTarget << endl;
                targetVelocity = controller.update(posTarget - posActual, velocityTarget, actuation, btScalar(dt));
            }
            else if (actuation.modes.position)
            {
               // cout << "################### posActual:" << posActual << ", posTarget" << posTarget << endl;
                targetVelocity = controller.update(posTarget - posActual, 0.0, actuation, btScalar(dt));
            }
            else if (actuation.modes.velocity)
            {
                //cout << "################### velActual:" << velActual << ", velTarget" << velocityTarget << endl;
                controller.setCurrentVelocity(velActual);
                targetVelocity = controller.update(0.0, velocityTarget, actuation, btScalar(dt));
            }
            // FIXME this bypasses the controller (and doesn't work..)
            else if (actuation.modes.torque)
            {
                //cout << "################### torque:" << it->second.jointTorqueTarget << endl;
                targetVelocity = it->second.jointTorqueTarget;
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

            btScalar maxImpulse = bulletMaxMotorImulse;
            /*if (it->second.node->getMaxTorque()>0)
            {
                maxImpulse = it->second.node->getMaxTorque() * btScalar(dt);
            }*/
#ifdef 0
            if (it->first->getName() == "Elbow R")
                cout << "################### " << it->first->getName() <<": posActual:" << posActual << ", posTarget:" << posTarget << ", actvel:"  << velActual << ", target vel:" << targetVelocity << ", maxImpulse" << maxImpulse << endl;
#endif
#ifdef 0
            std::string nameBodies = "BulletRobot_RNS_Bodies_All";
            VirtualRobot::RobotNodeSetPtr rnsBodies = robot->getRobotNodeSet(nameBodies);
            Eigen::Vector3f v = getComVelocityGlobal(rnsBodies);
            if (boost::math::isnan(v(0)) || boost::math::isnan(v(1)) || boost::math::isnan(v(2)))
            {
                VR_ERROR << "NAN before bullet call!!!" << endl;
            }
#endif
            hinge->enableAngularMotor(true, btScalar(targetVelocity), maxImpulse);
#ifdef 0
            //std::string nameBodies = "BulletRobot_RNS_Bodies_All";
            //VirtualRobot::RobotNodeSetPtr rnsBodies = robot->getRobotNodeSet(nameBodies);
            v = getComVelocityGlobal(rnsBodies);
            if (boost::math::isnan(v(0)) || boost::math::isnan(v(1)) || boost::math::isnan(v(2)))
            {
                VR_ERROR << "NAN after bullet call!!!" << endl;
            }
#endif



#endif
        }
    }
    //cout << endl;
    setPoseNonActuatedRobotNodes();
}

void BulletRobot::updateSensors(double dt)
{
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
    boost::unordered_set<std::string> contactObjectNames;
    // this seems stupid and it is, but that is abstract interfaces for you.
    for(std::vector<SensorPtr>::iterator it = sensors.begin(); it != sensors.end(); it++)
    {
        ContactSensorPtr contactSensor = boost::dynamic_pointer_cast<ContactSensor>(*it);
        if (contactSensor)
        {
            contactObjectNames.insert(contactSensor->getRobotNode()->getName());
        }
    }

    DynamicsWorldPtr world = DynamicsWorld::GetWorld();
    std::vector<SimDynamics::DynamicsEngine::DynamicsContactInfo> contacts = world->getEngine()->getContacts();
    boost::unordered_map<std::string, VirtualRobot::ContactSensor::ContactFrame> frameMap;
    for (std::vector<SimDynamics::DynamicsEngine::DynamicsContactInfo>::iterator it = contacts.begin();
         it != contacts.end(); it++)
    {
        const SimDynamics::DynamicsEngine::DynamicsContactInfo& contact = *it;

        float sign;
        std::string key;
        if (contactObjectNames.find(contact.objectA->getName()) != contactObjectNames.end())
        {
            sign = 1.0f;
            key = contact.objectA->getName();
        }
        else if (contactObjectNames.find(contact.objectB->getName()) != contactObjectNames.end())
        {
            sign = -1.0f;
            key = contact.objectB->getName();
        }
        else
        {
            continue;
        }

        VirtualRobot::ContactSensor::ContactFrame& frame = frameMap[key];
        double zForce = sign * contact.normalGlobalB.z() * contact.appliedImpulse;
        VirtualRobot::ContactSensor::ContactForce cf;
        cf.contactPoint = contact.posGlobalB;
        cf.zForce = zForce;
        frame.forces.push_back(cf);
    }

    // Update forces and torques
    for(std::vector<SensorPtr>::iterator it = sensors.begin(); it != sensors.end(); it++)
    {
        ForceTorqueSensorPtr ftSensor = boost::dynamic_pointer_cast<ForceTorqueSensor>(*it);
        if(ftSensor)
        {
            VirtualRobot::RobotNodePtr node = ftSensor->getRobotNode();
            THROW_VR_EXCEPTION_IF(!node, "parent of sensor could not be casted to RobotNode")

            const LinkInfo& link = getLink(node);
            Eigen::VectorXf forceTorques = getJointForceTorqueGlobal(link);
            ftSensor->updateSensors(forceTorques);
        }
        else
        {
            ContactSensorPtr contactSensor = boost::dynamic_pointer_cast<ContactSensor>(*it);
            if (contactSensor)
            {
                VirtualRobot::RobotNodePtr node = contactSensor->getRobotNode();
                THROW_VR_EXCEPTION_IF(!node, "parent of sensor could not be casted to RobotNode")
                const VirtualRobot::ContactSensor::ContactFrame& frame = frameMap[node->getName()];
                contactSensor->updateSensors(frame, dt);
            }
        }
    }
}

BulletRobot::LinkInfo BulletRobot::getLink( VirtualRobot::RobotNodePtr node )
{
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
	for (size_t i=0;i<links.size();i++)
	{
		if (links[i].nodeJoint == node || links[i].nodeJoint2 == node)
			return true;
	}
	return false;
}

void BulletRobot::actuateNode( VirtualRobot::RobotNodePtr node, double jointValue )
{
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
	VR_ASSERT(rn);
	if (!hasLink(rn))
	{
        //VR_ERROR << "No link with node " << rn->getName() << endl;
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
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
    VR_ASSERT(rn);
    if (!hasLink(rn))
    {
        //VR_ERROR << "No link with node " << rn->getName() << endl;
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
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
    VR_ASSERT(rn);
	if (!hasLink(rn))
	{
        //VR_ERROR << "No link with node " << rn->getName() << endl;
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
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
    return DynamicsRobot::getNodeTarget(node);
#else
	return DynamicsRobot::getNodeTarget(node);
#endif

}

Eigen::Vector3f BulletRobot::getJointTorques(RobotNodePtr rn)
{
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
    VR_ASSERT(rn);
    Eigen::Vector3f result;
    result.setZero();
    if (!hasLink(rn))
    {
        //VR_ERROR << "No link with node " << rn->getName() << endl;
        return result;
    }
    LinkInfo link = getLink(rn);

    if(rn->isRotationalJoint())
    {
		enableForceTorqueFeedback(link, true);
        result = getJointForceTorqueGlobal(link).tail(3);
    }

    return result;
}

double BulletRobot::getJointTorque(RobotNodePtr rn)
{
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
    VR_ASSERT(rn);
    if (!hasLink(rn))
    {
        //VR_ERROR << "No link with node " << rn->getName() << endl;
        return 0.0;
    }
    LinkInfo link = getLink(rn);

    if(!rn->isRotationalJoint())
		return 0.0;

	enableForceTorqueFeedback(link, true);
	Eigen::Vector3f torqueVector = getJointForceTorqueGlobal(link).tail(3);

	// project onto joint axis
	double troque = (torqueVector.adjoint() * link.nodeJoint->getGlobalPose().block(0, 2, 3, 1))(0, 0);
	return troque;
}

Eigen::Vector3f BulletRobot::getJointForces(RobotNodePtr rn)
{
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
    VR_ASSERT(rn);
    Eigen::Vector3f result;
    result.setZero();
    if (!hasLink(rn))
    {
        //VR_ERROR << "No link with node " << rn->getName() << endl;
        return result;
    }
    LinkInfo link = getLink(rn);

    if(rn->isRotationalJoint())
    {
		enableForceTorqueFeedback(link, true);
        result = getJointForceTorqueGlobal(link).head(3);
    }

    return result;
}

Eigen::Matrix4f BulletRobot::getComGlobal( VirtualRobot::RobotNodePtr rn )
{
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
	Eigen::Vector3f com = Eigen::Vector3f::Zero();
	double totalMass = 0.0;
	for (unsigned int i = 0; i < set->getSize(); i++)
	{
		VirtualRobot::RobotNodePtr node = (*set)[i];
		BulletObjectPtr bo = boost::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
		Eigen::Matrix4f pose = bo->getComGlobal();
		com += node->getMass() * pose.block(0, 3, 3, 1);
		totalMass += node->getMass();
	}

	com *= float(1.0f/totalMass);
	return com;
}

Eigen::Vector3f BulletRobot::getComVelocityGlobal( VirtualRobot::RobotNodeSetPtr set)
{
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
	Eigen::Vector3f com = Eigen::Vector3f::Zero();
	double totalMass = 0.0;
	for (unsigned int i = 0; i < set->getSize(); i++)
	{
		VirtualRobot::RobotNodePtr node = (*set)[i];
		BulletObjectPtr bo = boost::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
		Eigen::Vector3f vel = bo->getLinearVelocity();
        if (boost::math::isnan(vel(0)) || boost::math::isnan(vel(1)) || boost::math::isnan(vel(2)))
        {
            VR_ERROR << "NAN result: getLinearVelocity:" << bo->getName() << ", i:" << i << endl;
            node->print();
            VR_ERROR << "BULLETOBJECT com:" << bo->getCom() << endl;
            VR_ERROR << "BULLETOBJECT: ang vel:" << bo->getAngularVelocity() << endl;
            VR_ERROR << "BULLETOBJECT:" << bo->getRigidBody()->getWorldTransform().getOrigin() << endl;


        }

		com += node->getMass() * vel;
		totalMass += node->getMass();
	}
    if (fabs(totalMass)<1e-5)
    {
        VR_ERROR << "Little mass: " << totalMass << ". Could not compute com velocity..." << endl;
    }
    else
        com *= float(1.0f/totalMass);
	return com;
}

void BulletRobot::setPoseNonActuatedRobotNodes()
{
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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
    boost::recursive_mutex::scoped_lock scoped_lock(*engineMutexPtr);
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
