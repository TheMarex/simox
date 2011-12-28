/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*/

#include "OSGVisualizationFactory.h"
#include "../VisualizationNode.h"
#include "OSGVisualizationNode.h"
#include "../../VirtualRobotException.h"
#include "OSGVisualization.h"
#include "../../Robot.h"
#include "../../Grasp.h"
#include "../../GraspSet.h"
#include "../../SceneObject.h"
#include "../TriMeshModel.h"
#include "../../ReachabilitySpace.h"
#include <iostream>
#include <algorithm>
#include <boost/pointer_cast.hpp>

#include <osgDB/ReadFile> 
#include <osg/ShapeDrawable>

namespace VirtualRobot {

OSGVisualizationFactory::OSGVisualizationFactory()
{
}


OSGVisualizationFactory::~OSGVisualizationFactory()
{
}


/**
 * This method creates a VirtualRobot::OSGVisualizationNode from a given \p filename.
 * An instance of VirtualRobot::VisualizationNode is returned in case of an occured error.
 *
 * \param filename file to load the OSG3D visualization from.
 * \param boundingBox Use bounding box instead of full model.
 * \return instance of VirtualRobot::OSGVisualizationNode upon success and VirtualRobot::VisualizationNode on error.
 */
VisualizationNodePtr OSGVisualizationFactory::getVisualizationFromFile(const std::string& filename, bool boundingBox)
{
	VisualizationNodePtr visualizationNode(new VisualizationNode);

	osg::Node* n = osgDB::readNodeFile(filename.c_str());


	if (n)
	{
		n->ref();
		if (boundingBox)
		{
			VR_INFO << "BoundingBox nyi..." << endl;
		}
		visualizationNode.reset(new OSGVisualizationNode(n));
		visualizationNode->setFilename(filename, boundingBox);
		n->unref();
	} else
		VR_WARNING << "Could not read file:" << filename << endl;

	
	return visualizationNode;
}


/**
 * register this class in the super class factory
 */
VisualizationFactory::SubClassRegistry OSGVisualizationFactory::registry(OSGVisualizationFactory::getName(), &OSGVisualizationFactory::createInstance);


/**
 * \return "osg"
 */
std::string OSGVisualizationFactory::getName() {return "osg";}


/**
 * \return new instance of OSGVisualizationFactory
 */
boost::shared_ptr<VisualizationFactory> OSGVisualizationFactory::createInstance(void*)
{    
    boost::shared_ptr<OSGVisualizationFactory> OSGFactory(new OSGVisualizationFactory());
    return OSGFactory;
}

VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createBox( float width, float height, float depth, float colorR, float colorG, float colorB, CollisionCheckerPtr colChecker )
{
	osg::Box* b = new osg::Box(osg::Vec3(0,0,0),width,height,depth);
	osg::ShapeDrawable* bd = new osg::ShapeDrawable(b);
	osg::Geode* bg = new osg::Geode();
	bg->addDrawable(bd);
	osg::Group* s = new osg::Group;
	s->addChild(bg);

    VisualizationNodePtr visualizationNode(new OSGVisualizationNode(s));
	return visualizationNode;
}


VisualizationNodePtr OSGVisualizationFactory::createLine(const Eigen::Matrix4f &from, const Eigen::Matrix4f &to, float width, float colorR, float colorG, float colorB)
{
	osg::Vec3 sp(from(0,3),from(1,3),from(2,3)); 
	osg::Vec3 ep(to(0,3),to(1,3),to(2,3)); 
	osg::ref_ptr<osg::Geometry> beam( new osg::Geometry); 
	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array; 
	points->push_back(sp); 
	points->push_back(ep); 
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array; 
	color->push_back(osg::Vec4(colorR,colorG,colorB,1.0)); 
	beam->setVertexArray(points.get()); 
	beam->setColorArray(color.get()); 
	beam->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE); 
	beam->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));


	osg::Geode* bg = new osg::Geode();
	bg->addDrawable(beam);
	osg::Group* s = new osg::Group;
	s->addChild(bg);

	VisualizationNodePtr visualizationNode(new OSGVisualizationNode(s));
	return visualizationNode;
}

VisualizationNodePtr OSGVisualizationFactory::createSphere(float radius, float colorR, float colorG, float colorB, CollisionCheckerPtr colChecker)
{
	osg::Sphere* b = new osg::Sphere(osg::Vec3(0,0,0),radius);
	osg::ShapeDrawable* bd = new osg::ShapeDrawable(b);
	osg::Geode* bg = new osg::Geode();
	bg->addDrawable(bd);
	osg::Group* s = new osg::Group;
	s->addChild(bg);

	VisualizationNodePtr visualizationNode(new OSGVisualizationNode(s));
	return visualizationNode;
}

VisualizationNodePtr OSGVisualizationFactory::createCoordSystem(float scaling, std::string *text, float axisLength, float axisSize, int nrOfBlocks)
{
    VR_INFO << "init nyi..." << endl;
	return VisualizationNodePtr();
}

/*VisualizationNodePtr OSGVisualizationFactory::createVisualization(CollisionCheckerPtr colChecker)
{
    VR_INFO << "init nyi..." << endl;
	return VisualizationNodePtr();
}*/



VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createVertexVisualization( const Eigen::Vector3f &position, float radius, float transparency, float colorR /*= 0.5f*/, float colorG /*= 0.5f*/, float colorB /*= 0.5f*/ )
{
    VR_INFO << "init nyi..." << endl;
	return VisualizationNodePtr();
}

VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createPlane( const Eigen::Vector3f &position, const Eigen::Vector3f &normal, float extend, float transparency, float colorR /*= 0.5f*/, float colorG /*= 0.5f*/, float colorB /*= 0.5f*/ )
{
    VR_INFO << "init nyi..." << endl;
	return VisualizationNodePtr();
}

VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createBoundingBox( const BoundingBox &bbox )
{
    VR_INFO << "init nyi..." << endl;
	return VisualizationNodePtr();
}


VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createTriMeshModelVisualization( TriMeshModelPtr model, bool showNormals, Eigen::Matrix4f &pose )
{
    VR_INFO << "init nyi..." << endl;
	return VisualizationNodePtr();
}

VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createArrow( const Eigen::Vector3f &n, float length /*= 50.0f*/, float width /*= 2.0f*/, const Color &color /*= Color::Gray()*/ )
{
    VR_INFO << "init nyi..." << endl;
	return VisualizationNodePtr();
}

VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createVisualization( CollisionCheckerPtr colChecker /*= CollisionCheckerPtr()*/ )
{
	osg::Group* s = new osg::Group;
	VisualizationNodePtr visualizationNode(new OSGVisualizationNode(s));
	return visualizationNode;
}

} // namespace VirtualRobot
