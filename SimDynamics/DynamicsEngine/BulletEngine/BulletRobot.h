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
#ifndef _SimDynamics_BulletRobot_h_
#define _SimDynamics_BulletRobot_h_

#include "../DynamicsRobot.h"
#include "BulletObject.h"

#include <btBulletDynamicsCommon.h>

namespace SimDynamics
{
class SIMDYNAMICS_IMPORT_EXPORT BulletRobot : public DynamicsRobot
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
		Constructor.
		Create a dynamic representation by building all related bullet objects.
	*/
	BulletRobot(VirtualRobot::RobotPtr rob, bool enableJointMotors = true);

	/*!
	*/
	virtual ~BulletRobot();

	struct LinkInfo
	{
		VirtualRobot::RobotNodePtr nodeA; // parent
        VirtualRobot::RobotNodePtr nodeB; // child
        VirtualRobot::RobotNodePtr nodeJoint; // joint
        VirtualRobot::RobotNodePtr nodeJoint2; // joint2 (only used for hinge2/universal joints)
		BulletObjectPtr dynNode1; // parent
		BulletObjectPtr dynNode2; // child
		std::vector< std::pair<DynamicsObjectPtr,DynamicsObjectPtr> > disabledCollisionPairs;
		boost::shared_ptr<btTypedConstraint> joint;
		float jointValueOffset; // offset simox -> bullet joint values
	};


	void enableForceTorqueFeedback(const LinkInfo& link);
	Eigen::VectorXf getForceTorqueFeedbackA(const LinkInfo& link);
	Eigen::VectorXf getForceTorqueFeedbackB(const LinkInfo& link);
	
	// We do not allow to re-adjust the robot. 
	// The position of the robot is queried once on construction. 
	// Then the physics simulation takes over.
	//virtual void setPosition(const Eigen::Vector3f &posMM);
	//virtual void setPose(const Eigen::Matrix4f &pose);

	bool hasLink(VirtualRobot::RobotNodePtr node1, VirtualRobot::RobotNodePtr node2);

	//! Returns true if the joint of node is covered by a link
	bool hasLink( VirtualRobot::RobotNodePtr node );
	
	std::vector<LinkInfo> getLinks();

    virtual void actuateNode(VirtualRobot::RobotNodePtr node, float jointValue);
    virtual void actuateNodeVel(VirtualRobot::RobotNodePtr node, float jointVelocity);

	/*!
		Usually this method is called by the framework in every tick to perform joint actuation.
		\param dt Timestep
	*/
	virtual void actuateJoints(float dt);

	virtual float getJointAngle(VirtualRobot::RobotNodePtr rn);
    virtual float getJointSpeed(VirtualRobot::RobotNodePtr rn);
    virtual float getJointTargetSpeed(VirtualRobot::RobotNodePtr rn);
    virtual float getNodeTarget(VirtualRobot::RobotNodePtr node);

    /*!
        Returns the CoM pose, which is reported by bullet
    */
    virtual Eigen::Matrix4f getComGlobal(VirtualRobot::RobotNodePtr rn);

	// experimental...
	virtual void ensureKinematicConstraints();

	/*!
		Returns link where the given node is the second connected node. 
		There could only one link with this setup, since on VirtualRobot side node2 is the actuated one.
	*/
	LinkInfo getLink(VirtualRobot::RobotNodePtr node);


protected:
	void buildBulletModels(bool enableJointMotors);

    //void createLink( VirtualRobot::RobotNodePtr bodyA, VirtualRobot::RobotNodePtr joint, VirtualRobot::RobotNodePtr bodyB, Eigen::Matrix4f &trafoA2J, Eigen::Matrix4f &trafoJ2B, bool enableJointMotors = true );
    // Possible joint types:
    // fixed                (joint=fixed        !joint2)
    // hinge                (joint=revolute     !joint2)
    // universal (hinge2)   (joint=revolute     joint2=revolute) // experimental
    void createLink( VirtualRobot::RobotNodePtr bodyA, VirtualRobot::RobotNodePtr joint, VirtualRobot::RobotNodePtr joint2, VirtualRobot::RobotNodePtr bodyB, bool enableJointMotors = true );

	void createLink(VirtualRobot::RobotNodePtr node1,VirtualRobot::RobotNodePtr node2, bool enableJointMotors);

    // ensure that all robot nodes, which are not actuated directly, are at the correct pose
    void setPoseNonActuatedRobotNodes();

    // process all ignoreCollision tags of physics section of RobotNode. Adds according collision disabled information to physics engine.
    void addIgnoredCollisionModels(VirtualRobot::RobotNodePtr rn);

    std::vector<LinkInfo> links;

	btScalar bulletMaxMotorImulse;
    btScalar bulletMotorVelFactor;

};

typedef boost::shared_ptr<BulletRobot> BulletRobotPtr;

} // namespace SimDynamics

#endif // _SimDynamics_BulletRobot_h_
