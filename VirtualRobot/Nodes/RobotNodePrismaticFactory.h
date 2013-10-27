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
#ifndef _VirtualRobot_RobotNodePrismaticFactory_h_
#define _VirtualRobot_RobotNodePrismaticFactory_h_

#include "../VirtualRobotImportExport.h"
#include "../SceneObject.h"
#include "RobotNodeFactory.h"

#include <boost/shared_ptr.hpp>


namespace VirtualRobot
{
class RobotNode;

class VIRTUAL_ROBOT_IMPORT_EXPORT RobotNodePrismaticFactory  : public RobotNodeFactory
{
public:
	RobotNodePrismaticFactory();
	virtual ~RobotNodePrismaticFactory();

	virtual RobotNodePtr createRobotNode(RobotPtr robot, const std::string& nodeName, VisualizationNodePtr visualizationModel, CollisionModelPtr collisionModel, float limitLow, float limitHigh, float jointValueOffset, const Eigen::Matrix4f& preJointTransform, const Eigen::Vector3f& axis, const Eigen::Vector3f& translationDirection, const SceneObject::Physics &p = SceneObject::Physics(), RobotNode::RobotNodeType rntype = RobotNode::Generic) const;
	virtual RobotNodePtr createRobotNodeDH(RobotPtr robot, const std::string& nodeName, VisualizationNodePtr visualizationModel, CollisionModelPtr collisionModel, float limitLow, float limitHigh, float jointValueOffset, const DHParameter& dhParameters, const SceneObject::Physics &p = SceneObject::Physics(), RobotNode::RobotNodeType rntype = RobotNode::Generic) const;
// AbstractFactoryMethod
public:
	static std::string getName();
	static boost::shared_ptr<RobotNodeFactory> createInstance(void*);
private:
	static SubClassRegistry registry;
};

} // namespace VirtualRobot

#endif // _VirtualRobot_RobotNodePrismaticFactory_h_
