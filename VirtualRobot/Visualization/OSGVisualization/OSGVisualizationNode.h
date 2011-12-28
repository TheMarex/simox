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
#ifndef _VirtualRobot_OSGVisualizationNode_h_
#define _VirtualRobot_OSGVisualizationNode_h_

#include "../../VirtualRobotImportExport.h"
#include "../VisualizationNode.h"
#include "OSGVisualizationFactory.h"

#include "../TriMeshModel.h"

#include <boost/shared_ptr.hpp>

#include <osg/Node>
#include <osg/Group>
#include <osg/MatrixTransform>

namespace VirtualRobot
{
class TriMeshModel;

class VIRTUAL_ROBOT_IMPORT_EXPORT OSGVisualizationNode : virtual public VisualizationNode
{
public:
	OSGVisualizationNode();
	OSGVisualizationNode(osg::Node* visualizationNode);
	~OSGVisualizationNode();
	virtual TriMeshModelPtr getTriMeshModel();

	osg::Node* getOSGVisualization();

	virtual void setGlobalPose (const Eigen::Matrix4f &m);

	virtual void print();

	/*!
		Attach an optional visualization to this VisualizationNode. The attached visualizations will not show up in the TriMeshModel.
		If there is already a visualization attached with the given name, it is quietly replaced.
	*/
	virtual void attachVisualization(const std::string &name, VisualizationNodePtr v);

	/*!
		Remove an attached visualization.
	*/
	virtual void detachVisualization(const std::string &name);

	/*!
		Setup the visualization of this object.
		\param showVisualization If false, the visualization is disabled.
		\param showAttachedVisualizations If false, the visualization of any attached optional visualizations is disabled.
	*/
	virtual void setupVisualization(bool showVisualization, bool showAttachedVisualizations);


/*!
		Clone this visualization.
		\param deepCopy When true, the underlying visualization is copied, otherwise a reference to the existing visualization is passed. 
		Since the underlying implementation may be able to re-use the visualization data, a deep copy may not be necessary in some cases.
	*/
	virtual VisualizationNodePtr clone(bool deepCopy = true);

	virtual std::string getType(){return OSGVisualizationFactory::getName();}

protected:
	void createTriMeshModel();
	void addGeodeTriData(osg::Geode* geode, TriMeshModelPtr mesh);
	void addGroupTriData(osg::Group* visuGroup, TriMeshModelPtr mesh);
	osg::Node* visualization;
	osg::Group* visualizationAtGlobalPose;
	osg::Group* attachedVisualizationsSeparator;
	std::map< std::string, osg::Node* > attachedOSGVisualizations;	//< These optional visualizations will not show up in the TriMeshModel

	//SoNode* visualization;
	//SoSeparator* visualizationAtGlobalPose;
	//SoSeparator* attachedVisualizationsSeparator;
	//std::map< std::string, SoNode* > attachedOSGVisualizations;	//< These optional visualizations will not show up in the TriMeshModel

	osg::MatrixTransform *globalPoseTransform;
	TriMeshModelPtr triMeshModel;

	struct osgTriangleConverter {
		inline void operator () ( const osg::Vec3& _v1, const osg::Vec3& _v2, const osg::Vec3& _v3, bool treatVertexDataAsTemporary ) {
			osg::Vec3 v1 = _v1;// * m_mat;
			osg::Vec3 v2 = _v2;// * m_mat;
			osg::Vec3 v3 = _v3;// * m_mat;
			osg::Vec3 vV1V2 = v2-v1;
			osg::Vec3 vV1V3 = v3-v1;
			osg::Vec3 vNormal = vV1V2.operator ^(vV1V3);
			/**m_stream << "facet normal " << vNormal[0] << " " << vNormal[1] << " " << vNormal[2] << std::endl;
			*m_stream << "outer loop" << std::endl;
			*m_stream << "vertex " << v1[0] << " " << v1[1] << " " << v1[2] << std::endl;
			*m_stream << "vertex " << v2[0] << " " << v2[1] << " " << v2[2] << std::endl;
			*m_stream << "vertex " << v3[0] << " " << v3[1] << " " << v3[2] << std::endl;
			*m_stream << "endloop" << std::endl;
			*m_stream << "endfacet " << std::endl;*/

			// read out vertices
			Eigen::Vector3f a, b, c, n;
			a << _v1.x(), _v1.y(), _v1.z();
			b << _v2.x(), _v2.y(), _v2.z();
			c << _v3.x(), _v3.y(), _v3.z();
			n << vNormal.x(), vNormal.y(), vNormal.z();
			// add new triangle to the model
			triMeshModel->addTriangleWithFace(a, b, c, n);
		}

		TriMeshModelPtr triMeshModel;

	};

};

} // namespace VirtualRobot

#endif // _VirtualRobot_OSGVisualizationNode_h_
