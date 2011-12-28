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
#ifndef _VirtualRobot_OSGVisualizationFactory_h_
#define _VirtualRobot_OSGVisualizationFactory_h_


#include "../../VirtualRobotImportExport.h"
#include "../VisualizationFactory.h"
#include "../../BoundingBox.h"
#include "../../SceneObject.h"
#include "../../EndEffector/EndEffector.h"
#include "../ColorMap.h"

#include <boost/shared_ptr.hpp>

#include <string>

#include <osg/Node>
#include <osg/Group>

namespace VirtualRobot
{
class VisualizationNode;

class VIRTUAL_ROBOT_IMPORT_EXPORT OSGVisualizationFactory  : public VisualizationFactory
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	OSGVisualizationFactory();
	virtual ~OSGVisualizationFactory();

	virtual VisualizationNodePtr getVisualizationFromFile(const std::string& filename, bool boundingBox = false);
	virtual VisualizationNodePtr createBox(float width, float height, float depth, float colorR, float colorG, float colorB, CollisionCheckerPtr colChecker = CollisionCheckerPtr());
	virtual VisualizationNodePtr createLine(const Eigen::Matrix4f &from, const Eigen::Matrix4f &to, float width = 1.0f, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
	virtual VisualizationNodePtr createSphere(float radius, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f, CollisionCheckerPtr colChecker = CollisionCheckerPtr());
	virtual VisualizationNodePtr createCoordSystem(float scaling = 1.0f, std::string *text = NULL, float axisLength = 100.0f, float axisSize = 3.0f, int nrOfBlocks = 10);
	virtual VisualizationNodePtr createBoundingBox(const BoundingBox &bbox);
	virtual VisualizationNodePtr createVertexVisualization(const Eigen::Vector3f &position, float radius, float transparency,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
	virtual VisualizationNodePtr createTriMeshModelVisualization(TriMeshModelPtr model, bool showNormals, Eigen::Matrix4f &pose);
	virtual VisualizationNodePtr createPlane(const Eigen::Vector3f &position, const Eigen::Vector3f &normal, float extend, float transparency,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
	virtual VisualizationNodePtr createArrow(const Eigen::Vector3f &n, float length = 50.0f, float width = 2.0f, const Color &color = Color::Gray());

	/*!
		Create an empty VisualizationNode.
	*/
	virtual VisualizationNodePtr createVisualization(CollisionCheckerPtr colChecker = CollisionCheckerPtr());
// AbstractFactoryMethod
public:
	static std::string getName();
	static boost::shared_ptr<VisualizationFactory> createInstance(void*);
private:
	static SubClassRegistry registry;
};

} // namespace VirtualRobot

#endif // _VirtualRobot_OSGVisualizationFactory_h_
