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
#ifndef _SimDynamics_DynamicsRobot_h_
#define _SimDynamics_DynamicsRobot_h_

#include "../SimDynamics.h"
#include "DynamicsObject.h"
#include <VirtualRobot/Robot.h>

namespace SimDynamics
{
class SIMDYNAMICS_IMPORT_EXPORT DynamicsRobot
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
		Constructor
	*/
	DynamicsRobot(VirtualRobot::RobotPtr rob);

	/*!
	*/
	virtual ~DynamicsRobot();

	std::string getName() const;

	VirtualRobot::RobotPtr getRobot(){return robot;}

	bool hasDynamicsRobotNode(VirtualRobot::RobotNodePtr node);
	std::vector<DynamicsObjectPtr> getDynamicsRobotNodes();

	/*!
		Returns dynamic model of node.
		An empty DynamicsObjectPtr is returned in case no dynamic version has been created so far.
	*/
	DynamicsObjectPtr getDynamicsRobotNode(VirtualRobot::RobotNodePtr node);


	/*!
		Enable joint actuation for given node.
	*/
	virtual void actuateNode(VirtualRobot::RobotNodePtr node, float jointValue);
	virtual void disableNodeActuation(VirtualRobot::RobotNodePtr node);
	virtual bool isNodeActuated(VirtualRobot::RobotNodePtr node);
	virtual float getNodeTarget(VirtualRobot::RobotNodePtr node);
	virtual void enableActuation();
	virtual void disableActuation();

	/*!
		Usually this method is called by the framework in every tick to perform joint actuation.
		\param dt Timestep
	*/
	virtual void actuateJoints(float dt);

	// experimental...
	virtual void ensureKinematicConstraints();

	// We do not allow to re-adjust the robot. 
	// The position of the robot is queried once on construction. 
	// Then the physics simulation takes over.
	//virtual void setPosition(const Eigen::Vector3f &posMM);
	//virtual void setPose(const Eigen::Matrix4f &pose);


	virtual float getJointAngle(VirtualRobot::RobotNodePtr rn);
	virtual float getJointSpeed(VirtualRobot::RobotNodePtr rn);

    virtual Eigen::Matrix4f getComGlobal(VirtualRobot::RobotNodePtr rn);

protected:

	virtual void createDynamicsNode(VirtualRobot::RobotNodePtr node);

	struct robotNodeActuationTarget
	{
		float jointValueTarget;
		VirtualRobot::RobotNodePtr node;
		//DynamicsObjectPtr dynNode; // if node is a joint without model, there is no dyn node!
		bool enabled;
	};

	std::map<VirtualRobot::RobotNodePtr, robotNodeActuationTarget> actuationTargets;

	VirtualRobot::RobotPtr robot;

	std::vector<VirtualRobot::RobotNodePtr> robotNodes;
	std::map<VirtualRobot::RobotNodePtr, DynamicsObjectPtr> dynamicRobotNodes;
};

typedef boost::shared_ptr<DynamicsRobot> DynamicsRobotPtr;

} // namespace SimDynamics

#endif // _SimDynamics_DynamicsRobot_h_
