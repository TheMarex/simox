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
* @package    VirtualRobot
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010,2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_RobotNodeRevolute_h_
#define _VirtualRobot_RobotNodeRevolute_h_

#include "../VirtualRobotImportExport.h"

#include "RobotNode.h" 
#include "../RobotFactory.h"

#include <Eigen/Core>

#include <string>
#include <vector>


namespace VirtualRobot
{
class Robot;

class VIRTUAL_ROBOT_IMPORT_EXPORT RobotNodeRevolute : public RobotNode
{
public:
	friend class RobotFactory;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
	*/

	RobotNodeRevolute(	RobotWeakPtr rob, 									//!< The robot
						const std::string &name,							//!< The name
						const std::vector<std::string> &childrenNames,		//!< The list of all children
						float jointLimitLo,									//!< lower joint limit
						float jointLimitHi,									//!< upper joint limit
						const Eigen::Matrix4f &preJointTransform,			//!< This transformation is applied before the translation of the joint is done
						const Eigen::Vector3f &axis,						//!< The rotation axis (in local joint coord system)
						const Eigen::Matrix4f &postJointTransform,			//!< This is an additional transformation, that is applied after the rotation is done
						VisualizationNodePtr visualization = VisualizationNodePtr(),	//!< A visualization model
						CollisionModelPtr collisionModel = CollisionModelPtr(),			//!< A collision model
						float jointValueOffset = 0.0f,									//!< An offset that is internally added to the joint value
						const SceneObject::Physics &p = SceneObject::Physics(),			//!< physics information
						CollisionCheckerPtr colChecker = CollisionCheckerPtr()			//!< A collision checker instance (if not set, the global col checker is used)
						);
	RobotNodeRevolute(	RobotWeakPtr rob, 									//!< The robot
						const std::string &name,							//!< The name
						const std::vector<std::string> &childrenNames,		//!< The list of all children
						float jointLimitLo,									//!< lower joint limit
						float jointLimitHi,									//!< upper joint limit
						float a, 											//!< dh paramters
						float d, 											//!< dh paramters
						float alpha, 										//!< dh paramters
						float theta,										//!< dh paramters
						VisualizationNodePtr visualization = VisualizationNodePtr(),//!< A visualization model
						CollisionModelPtr collisionModel = CollisionModelPtr(),		//!< A collision model
						float jointValueOffset = 0.0f,								//!< An offset that is internally added to the joint value
						const SceneObject::Physics &p = SceneObject::Physics(),		//!< physics information
						CollisionCheckerPtr colChecker = CollisionCheckerPtr()		//!< A collision checker instance (if not set, the global col checker is used)
						);
	/*!
	*/
	virtual ~RobotNodeRevolute();

	virtual bool initialize(RobotNodePtr parent, bool initializeChildren = false);

	/*!
	Print status information.
	*/
	virtual void print(bool printChildren = false, bool printDecoration = true) const;

	virtual bool isRotationalJoint() const;
	/*!
		Standard: In global coordinate system.
		\param coordSystem When not set the axis is transformed to global coordinate system. Otherwise any scene object can be used as coord system.
	*/
	Eigen::Vector3f getJointRotationAxis(const SceneObjectPtr coordSystem = SceneObjectPtr()) const;

	/*!
		This is the original joint axis, without any transformations applied.
	*/
	Eigen::Vector3f getJointRotationAxisInJointCoordSystem() const;
protected:
	/*!
		Can be called by a RobotNodeActuator in order to set the pose of this node.
		This is useful, if the node is actuated externally, i.e. via a physics engine. 
		\param globalPose The new global pose. The joint value is determined from this pose (implemented in derived RobtoNodes).
		\param updateChildren Usually it is assumed that all RobotNodes are updated this way (updateChildren=false). If not, the children poses can be updated according to this node (updateCHildren=true).
	*/
	virtual void updateVisualizationPose(const Eigen::Matrix4f &globalPose, bool updateChildren = false);


	RobotNodeRevolute(){};
	
	virtual void updateTransformationMatrices();
	virtual void updateTransformationMatrices(const Eigen::Matrix4f &globalPose);

	Eigen::Vector3f jointRotationAxis;			// eRevoluteJoint  (given in local joint coord system)

	virtual RobotNodePtr _clone(const RobotPtr newRobot, const std::vector<std::string> newChildren, const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel, CollisionCheckerPtr colChecker);
};

typedef boost::shared_ptr<RobotNodeRevolute> RobotNodeRevolutePtr;

} // namespace VirtualRobot

#endif // _VirtualRobot_RobotNodeRevolute_h_
