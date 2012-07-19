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
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_RobotNodeFixed_h_
#define _VirtualRobot_RobotNodeFixed_h_

#include "../VirtualRobotImportExport.h"

#include "RobotNode.h" 
#include "../RobotFactory.h"

#include <Eigen/Core>

#include <string>
#include <vector>


namespace VirtualRobot
{
class Robot;

class VIRTUAL_ROBOT_IMPORT_EXPORT RobotNodeFixed : public RobotNode
{
public:
	friend class RobotFactory;

	/*!
	Constructor
	*/
	RobotNodeFixed(RobotWeakPtr rob,				//!< The robot
		const std::string &name,					//!< The name
		const std::vector<std::string> &childrenNames, //!< The list of all children
		const Eigen::Matrix4f &preJointTransform,   //!<  This is the fixed transformation of this RobotNode (used to compute globalPose)
		const Eigen::Matrix4f &postJointTransform,	//!<  This is the fixed transformation of this RobotNode (used to compute transformation to children)
		VisualizationNodePtr visualization = VisualizationNodePtr(),//!< A visualization model
		CollisionModelPtr collisionModel = CollisionModelPtr(),		//!< A collision model
		const SceneObject::Physics &p = SceneObject::Physics(),		//!< physics information
		CollisionCheckerPtr colChecker = CollisionCheckerPtr()		//!< A collision checker instance (if not set, the global col checker is used)
		);
	/*!
		Initialize with DH parameters.

		Since the DH convention expects the visualization to be linked at the end of the joint transformation and VirtualRobot is linking the visualization at the local (=preJoint*jointTrafo) coordinate system,
		there might be an unexpected behavior when using fixed joints with visualizations:
		The visualization is linked after the theta transformation is applied and d, alpha and a transformations are considered as postJoint transformations.
		When converting a prismatic joint with visualization to a fixed join, the (fixed) d transformation is not applied to the visualization of this joint. 
		Since the prismatic joint considers theta and d for computing the visualization pose, there might be a different visualization.
	*/
	RobotNodeFixed(RobotWeakPtr rob,						//!< The robot
		const std::string &name,							//!< The name
		const std::vector<std::string> &childrenNames,		//!< The list of all children
		float a, 											//!< Use fixed DH parameters to specify the transformation of this RobotNode
		float d, 											//!< Use fixed DH parameters to specify the transformation of this RobotNode
		float alpha, 										//!< Use fixed DH parameters to specify the transformation of this RobotNode
		float theta,										//!< Use fixed DH parameters to specify the transformation of this RobotNode
		VisualizationNodePtr visualization = VisualizationNodePtr(),  //!< A visualization model
		CollisionModelPtr collisionModel = CollisionModelPtr(),	//!< A collision model
		const SceneObject::Physics &p = SceneObject::Physics(),	//!< physics information
		CollisionCheckerPtr colChecker = CollisionCheckerPtr()	//!< A collision checker instance (if not set, the global col checker is used)
		);

	/*!
	*/
	virtual ~RobotNodeFixed();

	virtual bool initialize(RobotNodePtr parent, bool initializeChildren = false);

	/*!
	Print status information.
	*/
	virtual void print(bool printChildren = false, bool printDecoration = true) const;

protected:

	RobotNodeFixed(){};
	virtual void updateTransformationMatrices();
	virtual void updateTransformationMatrices(const Eigen::Matrix4f &globalPose);
	virtual RobotNodePtr _clone(const RobotPtr newRobot, const std::vector<std::string> newChildren, const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel, CollisionCheckerPtr colChecker);
};

} // namespace VirtualRobot

#endif // _VirtualRobot_RobotNodeFixed_h_
