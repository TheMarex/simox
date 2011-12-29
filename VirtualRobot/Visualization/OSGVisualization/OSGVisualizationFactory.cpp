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
#include <osg/BoundingBox>
#include <osg/PolygonMode>
#include <osg/ComputeBoundsVisitor>
#include <osg/LineWidth>
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
			osg::Node* bboxVisu = CreateBoundingBox(n);
			bboxVisu->ref();
			n->unref();
			n = bboxVisu;
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
	bd->setColor(osg::Vec4(colorR,colorG,colorB,1.0));
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
	bg->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
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
	bd->setColor(osg::Vec4(colorR,colorG,colorB,1.0));
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

void OSGVisualizationFactory::switchToWireframe( osg::Node *srcNode )
{
		if( !srcNode )
			return;

		osg::StateSet *state = srcNode->getOrCreateStateSet();
		osg::PolygonMode *polyModeObj;

		polyModeObj = dynamic_cast< osg::PolygonMode* >
			( state->getAttribute( osg::StateAttribute::POLYGONMODE ));

		if ( !polyModeObj ) 
		{
			polyModeObj = new osg::PolygonMode;
			state->setAttribute( polyModeObj );    
		}

		polyModeObj->setMode(  osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
} 

osg::Node* OSGVisualizationFactory::CreateBoundingBox(osg::Node *model, bool wireFrame)
{
	if (!model)
		return NULL;

	const osg::MatrixList& m = model->getWorldMatrices(); 
	osg::ComputeBoundsVisitor cbv; 
	model->accept( cbv ); 
	osg::BoundingBox bboxOSG = cbv.getBoundingBox(); 
	osg::Vec3 minV = bboxOSG._min * m.front(); 
	osg::Vec3 maxV = bboxOSG._max * m.front(); 

	BoundingBox bbox;
	for (int i=0;i<3;i++)
	{
		bbox.min(i) = minV[i];
		bbox.max(i) = maxV[i];
	}
	return CreateBoundingBoxVisualization(bbox,wireFrame);
}

osg::Node* OSGVisualizationFactory::CreateBoundingBoxVisualization( const BoundingBox &bbox, bool wireFrame )
{
	osg::BoundingBox bboxOSG;

	bboxOSG.expandBy( bbox.min(0),bbox.min(1),bbox.min(2) );
	bboxOSG.expandBy( bbox.max(0),bbox.max(1),bbox.max(2) );

	osg::Vec3 ext( bboxOSG._max - bboxOSG._min ); 
	osg::Box* box = new osg::Box(bboxOSG.center(), ext[0], ext[1], ext[2]);
	osg::ShapeDrawable* shapeDraw = new osg::ShapeDrawable(box);
	osg::Geode* boundingBoxGeode = new osg::Geode();
	boundingBoxGeode->addDrawable(shapeDraw);

	if (wireFrame)
		switchToWireframe(boundingBoxGeode);

	return boundingBoxGeode;
}

VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createBoundingBox( const BoundingBox &bbox, bool wireFrame )
{
	osg::Node* res = CreateBoundingBoxVisualization(bbox,wireFrame);

	VisualizationNodePtr node(new OSGVisualizationNode(res));
	return node;
}

osg::MatrixTransform* OSGVisualizationFactory::getMatrixTransform(const Eigen::Matrix4f &pose)
{
	osg::MatrixTransform* mt = new osg::MatrixTransform;
	osg::Matrix mat(pose.data());
	mt->setMatrix(mat);
	return mt;
}

VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createTriMeshModelVisualization( TriMeshModelPtr model, bool showNormals, Eigen::Matrix4f &pose )
{
	osg::Group *res = new osg::Group;

	osg::MatrixTransform* globalPoseTransform = getMatrixTransform(pose);
	res->addChild(globalPoseTransform);

	osg::Node *res1 = OSGVisualizationFactory::getOSGVisualization(model,showNormals);
	globalPoseTransform->addChild(res1);

	VisualizationNodePtr node(new OSGVisualizationNode(res));
	return node;
}

osg::Node* OSGVisualizationFactory::getOSGVisualization( TriMeshModelPtr model, bool showNormals, VisualizationFactory::Color color )
{
	osg::Group *res = new osg::Group;
	res->ref();
	Eigen::Vector3f v1,v2,v3;
	for (size_t i=0;i<model->faces.size();i++)
	{
		v1 = model->vertices[model->faces[i].id1];
		v2 = model->vertices[model->faces[i].id2];
		v3 = model->vertices[model->faces[i].id3];
		//v2.setValue(model->vertices[model->faces[i].id2](0),model->vertices[model->faces[i].id2](1),model->vertices[model->faces[i].id2](2));
		//v3.setValue(model->vertices[model->faces[i].id3](0),model->vertices[model->faces[i].id3](1),model->vertices[model->faces[i].id3](2));
		std::vector<Eigen::Vector3f> v;
		v.push_back(v1);
		v.push_back(v2);
		v.push_back(v3);
		osg::Node* s = CreatePolygonVisualization(v,color);
		res->addChild(s);
		if (showNormals)
		{
			v1 = (v1 + v2 + v3) / 3.0f;
			osg::Group* ar = new osg::Group;
			Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
			mat.block(0,3,3,1) = v1;
			osg::MatrixTransform *mt = getMatrixTransform(mat);
			ar->addChild(mt);

			//osg::Node *n = CreateArrow(model->faces[i].normal,30.0f,1.5f);
			//mt->addChild(n);

			res->addChild(ar);
		}
	}
	res->unref_nodelete();
	return res;
}

osg::Node* OSGVisualizationFactory::CreatePolygonVisualization( const std::vector<Eigen::Vector3f> &points, VisualizationFactory::Color colorInner /*= VisualizationFactory::Color::Blue()*/, VisualizationFactory::Color colorLine /*= VisualizationFactory::Color::Black()*/, float lineSize /*= 5.0f*/ )
{
	osg::Geode* geode = new osg::Geode();
	// create Geometry object to store all the vertices and lines primitive.
	osg::Geometry* polyGeom = new osg::Geometry();
	osg::Vec3Array* vertices = new osg::Vec3Array;
	Eigen::Vector3f normal = MathTools::findNormal(points);
	for (unsigned int i=0;i<points.size();i++)
	{
		vertices->push_back(osg::Vec3(points[i](0),points[i](1),points[i](2)));

	}
	int numCoords = int(points.size());

	// pass the created vertex array to the points geometry object.
	polyGeom->setVertexArray(vertices);
			
	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(colorInner.r,colorInner.g,colorInner.b,1.0f-colorInner.transparency));
	polyGeom->setColorArray(colors);
	polyGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
			
	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(-normal(0),-normal(1),-normal(2)));
	polyGeom->setNormalArray(normals);
	polyGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

	polyGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON,0,numCoords));
	//printTriangles("Polygon",*polyGeom);
	geode->addDrawable(polyGeom);


	osg::Geometry* linesGeom = new osg::Geometry();
	linesGeom->setVertexArray(vertices);
	osg::Vec4Array* colorsL = new osg::Vec4Array;
	colorsL->push_back(osg::Vec4(colorLine.r,colorLine.g,colorLine.b,1.0f-colorLine.transparency));
	linesGeom->setColorArray(colorsL);
	linesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
	linesGeom->setNormalArray(normals);
	linesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
	linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,numCoords));
	osg::LineWidth* linewidth = new osg::LineWidth(); 
	linewidth->setWidth(lineSize); 
	geode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON); 
	geode->addDrawable(linesGeom);

	return geode;
}


VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createArrow( const Eigen::Vector3f &n, float length /*= 50.0f*/, float width /*= 2.0f*/, const Color &color /*= Color::Gray()*/ )
{
	/*float coneHeight = width*6.0f;
	float coneBotomRadius = width*2.5f;
	osg::Group *res = new osg::Group;

	osg::Vec3 objNormal(n(0),n(1),n(2));
	osg::Matrix objNormalTrafo;
	objNormalTrafo.makeIdentity();

	osg::MatrixTransform* arrow = new osg::MatrixTransform;
	arrow->addChild(lineGeode);
	arrow->addChild(geode);

	// Rotate X-axis arrow appropriately.
	osg::Quat rotation; rotation.makeRotate(osg::Vec3(0,1.0f,0),objNormal);
	arrow->setMatrix(osg::Matrix(rotation));

	res->addChild(arrow);


	SoTranslation *tr = new SoTranslation;
	tr->translation.setValue(0,length*0.5f,0);
	res->addChild(tr);


	SoCylinder *c = new SoCylinder();
	c->radius = width;
	c->height = length;
	res->addChild(c);

	SoTranslation *transl = new SoTranslation;
	transl->translation.setValue(0,length*0.5f+coneHeight*0.5f,0);
	res->addChild(transl);

	SoCone *cone = new SoCone();
	cone->bottomRadius.setValue(coneBotomRadius);
	cone->height.setValue(coneHeight);
	res->addChild(cone);

	return res;



	// Create a line.
	osg::Geode* lineGeode = new osg::Geode;
	{
		osg::Geometry* geometry = new osg::Geometry();

		osg::Vec3Array* vertices = new osg::Vec3Array(2);
		(*vertices)[0] = osg::Vec3(0.0f,0.0f,-0.5f);
		(*vertices)[1] = osg::Vec3(0.0f,0.0f,0.5f);

		geometry->setVertexArray(vertices);
		geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,2));

		lineGeode->addDrawable(geometry);
	}

	// Turn of lighting for line and set line width.
	osg::LineWidth* linewidth = new osg::LineWidth();
	linewidth->setWidth(2.0f);
	lineGeode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);
	lineGeode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

	*/
	VR_INFO << "nyi.." << endl;
	return VisualizationNodePtr();
}

VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createVisualization( CollisionCheckerPtr colChecker /*= CollisionCheckerPtr()*/ )
{
	osg::Group* s = new osg::Group;
	VisualizationNodePtr visualizationNode(new OSGVisualizationNode(s));
	return visualizationNode;
}

osg::Matrix* OSGVisualizationFactory::getGlobalPose( osg::Node* n )
{
	globalPoseNodeVisitor* ncv = new globalPoseNodeVisitor();
	if (n && ncv)
	{
		n->accept(*ncv);
		return ncv->getGlobalPose();
	}
	osg::Matrix* resId = new osg::Matrix;
	resId->makeIdentity();
	return resId;
}

} // namespace VirtualRobot
