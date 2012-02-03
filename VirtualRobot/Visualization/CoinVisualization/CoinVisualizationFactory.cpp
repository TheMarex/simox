/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @author     Manfred Kroehnert <mkroehnert _at_ users dot sourceforge dot net>
* @copyright  2010,2011 Nikolaus Vahrenkamp, Manfred Kroehnert
*/

#include "CoinVisualizationFactory.h"
#include "../VisualizationNode.h"
#include "CoinVisualizationNode.h"
#include "../../VirtualRobotException.h"
#include "CoinVisualization.h"
#include "../../Robot.h"
#include "../../Grasp.h"
#include "../../GraspSet.h"
#include "../../SceneObject.h"
#include "../TriMeshModel.h"
#include "../../ReachabilitySpace.h"
#include <Inventor/SoDB.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodes/SoUnits.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoAsciiText.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/SbViewportRegion.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/nodes/SoComplexity.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoTextureCoordinateBinding.h>
#include <Inventor/nodes/SoTexture2.h>
#include <Inventor/nodes/SoTextureCoordinate2.h>
#include <Inventor/nodes/SoNormal.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoLightModel.h>
#include <iostream>
#include <algorithm>
#include <boost/pointer_cast.hpp>

namespace VirtualRobot {

CoinVisualizationFactory::CoinVisualizationFactory()
{
}


CoinVisualizationFactory::~CoinVisualizationFactory()
{
}


/**
 * This method creates a VirtualRobot::CoinVisualizationNode from a given \p filename.
 * An instance of VirtualRobot::VisualizationNode is returned in case of an occured error.
 *
 * \param filename file to load the Coin3D visualization from.
 * \param boundingBox Use bounding box instead of full model.
 * \return instance of VirtualRobot::CoinVisualizationNode upon success and VirtualRobot::VisualizationNode on error.
 */
VisualizationNodePtr CoinVisualizationFactory::getVisualizationFromFile(const std::string& filename, bool boundingBox)
{
	VisualizationNodePtr visualizationNode(new VisualizationNode);

	// passing an empty string to SoInput and trying to open it aborts the program
	if (filename.empty())
	{
		std::cerr <<  "No filename given" << std::endl;
		return visualizationNode;
	}
	
	// try to open the given file
	SoInput fileInput;
	if (!fileInput.openFile(filename.c_str()))
	{
		std::cerr <<  "Cannot open file " << filename << std::endl;
		return visualizationNode;
	}
	
	CoinVisualizationFactory::GetVisualizationFromSoInput(fileInput, visualizationNode, boundingBox);
	
	fileInput.closeFile();
	visualizationNode->setFilename(filename, boundingBox);
	
	return visualizationNode;
}

/**
 * This method creates a VirtualRobot::CoinVisualizationNode from a given \p modelString.
 * An instance of VirtualRobot::VisualizationNode is returned in case of an occured error.
 *
 * \param modelString string to load the Coin3D visualization from.
 * \param boundingBox Use bounding box instead of full model.
 * \return instance of VirtualRobot::CoinVisualizationNode upon succes and VirtualRobot::VisualizationNode on error.
 */
VisualizationNodePtr CoinVisualizationFactory::getVisualizationFromString(const std::string& modelString, bool boundingBox)
{
    VisualizationNodePtr visualizationNode(new VisualizationNode);

    if (modelString.empty())
    {
        std::cerr << "No modelString given" << std::endl;
        return visualizationNode;
    }

    SoInput stringInput;
    stringInput.setBuffer(const_cast<char*>(modelString.c_str()), modelString.size());
    CoinVisualizationFactory::GetVisualizationFromSoInput(stringInput, visualizationNode, boundingBox);
    return visualizationNode;
}


/**
 * This method reads the data from the given \p soInput and creates a new CoinVisualizationNode
 * with the read Coin model if no error occured during reading the model.
 * The newly created CoinVisualizationNode is then stored in \p visualisationNode.
 *
 * \param soInput SoInput instance from which the model is read.
 * \param visualizationNode VisualizationNodePtr instance in which the created CoinVisualizationNode is stored.
 * \param boundingBox Use bounding box instead of full model.
 */
void CoinVisualizationFactory::GetVisualizationFromSoInput(SoInput& soInput, VisualizationNodePtr& visualizationNode, bool boundingBox)
{
	// read the contents of the file
	SoNode* coinVisualization = SoDB::readAll(&soInput);
	coinVisualization->ref();
		
	// check if the visualization was read
	if (NULL == coinVisualization)
		std::cerr <<  "Problem reading model from SoInput" << std::endl;

	if (boundingBox)
	{
		SoSeparator* bboxVisu = CreateBoundingBox(coinVisualization, false);
		bboxVisu->ref();
		coinVisualization->unref();
		coinVisualization = bboxVisu;
	}
	
	// create new CoinVisualizationNode if no error occured 
	visualizationNode.reset(new CoinVisualizationNode(coinVisualization));
	
	coinVisualization->unref();
}



SoSeparator* CoinVisualizationFactory::CreateBoundingBox(SoNode *ivModel, bool wireFrame)
{
	THROW_VR_EXCEPTION_IF(!ivModel,"NULL ivModel!");

	float minX;
	float minY;
	float minZ;
	float maxX;
	float maxY;
	float maxZ;

	// get dimensions of oivMod
	SbViewportRegion vpr;
	SoGetBoundingBoxAction boxAction(vpr);
	boxAction.apply(ivModel);

	//boxAction.getXfBoundingBox().getBounds(minX, minY, minZ, maxX, maxY, maxZ);
	boxAction.getBoundingBox().getBounds(minX, minY, minZ, maxX, maxY, maxZ);
	cout << "x: " << minX << "," << maxX << " ; Y: "<< minY << "," << maxY << " ; Z: "<< minZ << "," << maxZ << endl;


	SoCube *cu = new SoCube();

	cu->width = (maxX-minX);
	cu->height = (maxY-minY);
	cu->depth = (maxZ-minZ);


	SoDrawStyle *s = new SoDrawStyle();
	if (wireFrame)
	    s->style =  SoDrawStyle::LINES;
	else
	    s->style =  SoDrawStyle::FILLED;
	SoSeparator *n = new SoSeparator();
	SoTranslation *t = new SoTranslation();
	t->translation.setValue((maxX-minX)*0.5f+minX,(maxY-minY)*0.5f+minY,(maxZ-minZ)*0.5f+minZ);
	n->addChild(t);
	n->addChild(s);
	n->addChild(cu);
	return n;
}


/**
 * register this class in the super class factory
 */
VisualizationFactory::SubClassRegistry CoinVisualizationFactory::registry(CoinVisualizationFactory::getName(), &CoinVisualizationFactory::createInstance);


/**
 * \return "inventor"
 */
std::string CoinVisualizationFactory::getName() {return "inventor";}


/**
 * \return new instance of CoinVisualizationFactory and call SoDB::init()
 * if it has not already been called.
 */
boost::shared_ptr<VisualizationFactory> CoinVisualizationFactory::createInstance(void*)
{
    if (!SoDB::isInitialized())
        SoDB::init();
    boost::shared_ptr<CoinVisualizationFactory> coinFactory(new CoinVisualizationFactory());
    return coinFactory;
}

VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createBox( float width, float height, float depth, float colorR, float colorG, float colorB, CollisionCheckerPtr colChecker )
{
	SoSeparator *s = new SoSeparator();
	s->ref();

	SoMaterial *m = new SoMaterial();
	s->addChild(m);
	m->ambientColor.setValue(colorR,colorG,colorB);
	m->diffuseColor.setValue(colorR,colorG,colorB);

	SoCube *c = new SoCube();
	s->addChild(c);
	c->width = width;
	c->height = height;
	c->depth = depth;
	
	VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
	s->unref();
	return visualizationNode;
}

SoNode* CoinVisualizationFactory::createCoinLine(const Eigen::Matrix4f &from, const Eigen::Matrix4f &to, float width, float colorR, float colorG, float colorB)
{
	SoSeparator *s = new SoSeparator();
	s->ref();

	SoMaterial *m = new SoMaterial();
	s->addChild(m);
	m->ambientColor.setValue(colorR,colorG,colorB);
	m->diffuseColor.setValue(colorR,colorG,colorB);

	// create line 
	float x = from(0,3);
	float y = from(1,3);
	float z = from(2,3);
	float x2 = to(0,3);
	float y2 = to(1,3);
	float z2 = to(2,3);

	SbVec3f points[2];
	points[0].setValue(x2,y2,z2);
	points[1].setValue(x,y,z);

	SoDrawStyle *lineSolutionStyle = new SoDrawStyle();
	lineSolutionStyle->lineWidth.setValue(width);
	s->addChild(lineSolutionStyle);

	SoCoordinate3* coordinate3 = new SoCoordinate3;
	coordinate3->point.set1Value(0,points[0]);
	coordinate3->point.set1Value(1,points[1]);
	s->addChild(coordinate3);

	SoLineSet* lineSet = new SoLineSet;
	lineSet->numVertices.setValue(2);
	lineSet->startIndex.setValue(0);
	s->addChild(lineSet);
	s->unrefNoDelete();
	return s;
}

VisualizationNodePtr CoinVisualizationFactory::createLine(const Eigen::Matrix4f &from, const Eigen::Matrix4f &to, float width, float colorR, float colorG, float colorB)
{
	SoNode* s = createCoinLine(from,to,width,colorR,colorG,colorB);
	VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
	return visualizationNode;
}

VisualizationNodePtr CoinVisualizationFactory::createSphere(float radius, float colorR, float colorG, float colorB, CollisionCheckerPtr colChecker)
{
	SoSeparator *s = new SoSeparator();
	s->ref();

	SoMaterial *m = new SoMaterial();
	s->addChild(m);
	m->ambientColor.setValue(colorR,colorG,colorB);
	m->diffuseColor.setValue(colorR,colorG,colorB);

	SoSphere *c = new SoSphere();
	s->addChild(c);
	c->radius = radius;

	VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
	s->unref();
	return visualizationNode;
}

VisualizationNodePtr CoinVisualizationFactory::createCoordSystem(float scaling, std::string *text, float axisLength, float axisSize, int nrOfBlocks)
{
	SoSeparator *s = CoinVisualizationFactory::CreateCoordSystemVisualization(scaling,text,axisLength,axisSize,nrOfBlocks);
	s->ref();

	VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
	s->unref();
	return visualizationNode;
}

VisualizationNodePtr CoinVisualizationFactory::createVisualization(CollisionCheckerPtr colChecker)
{
	SoSeparator *s = new SoSeparator();
	VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
	return visualizationNode;
}


SoSeparator* CoinVisualizationFactory::CreateCoordSystemVisualization(float scaling, std::string *text, float axisLength, float axisSize, int nrOfBlocks)
{
	float blockSize = axisSize+0.5f;
	float blockWidth = 0.1f;
	if (axisSize>10.0f)
	{
		blockSize += axisSize / 10.0f;
		blockWidth += axisSize / 10.0f;
	}

	float axisBlockTranslation;
	if (nrOfBlocks!=0)
	{
		axisBlockTranslation = axisLength / nrOfBlocks;
	} else
		axisBlockTranslation = axisLength / 10.0f;

	SoSeparator* result = new SoSeparator();

	SbMatrix m;
	m.makeIdentity();
	SoMatrixTransform *mtr = new SoMatrixTransform();
	mtr->matrix.setValue(m);
	result->addChild(mtr);

	//SoScale *sc = new SoScale();
	//sc->scaleFactor.setValue(scaling,scaling,scaling);
	//result->addChild(sc);

	for (int i=0;i<3;i++)
	{
		SoSeparator *tmp1 = new SoSeparator();
		SoTransform *t = new SoTransform();
		SoMaterial *m = new SoMaterial();
		if (i==0)
		{
			m->diffuseColor.setValue(1.0f,0,0);
			t->translation.setValue((axisLength/2.0f + axisSize/2.0f)*scaling,0,0);
		} else if (i==1)
		{
			m->diffuseColor.setValue(0,1.0f,0);
			t->translation.setValue(0,(axisLength/2.0f + axisSize/2.0f)*scaling,0);
		} else
		{
			m->diffuseColor.setValue(0,0,1.0f);
			t->translation.setValue(0,0,(axisLength/2.0f + axisSize/2.0f)*scaling);
		}

		tmp1->addChild(m);
		tmp1->addChild(t);
		SoCube *c = new SoCube();
		SoCube *c2 = new SoCube();
		SoTransform *t2 = new SoTransform();
		if (i==0)
		{
			c->width = axisLength*scaling;
			c->height = axisSize*scaling;
			c->depth = axisSize*scaling;
			c2->width = blockWidth*scaling;
			c2->height = blockSize*scaling;
			c2->depth = blockSize*scaling;
			t2->translation.setValue(axisBlockTranslation*scaling,0,0);
		} else if (i==1)
		{
			c->height = axisLength*scaling;
			c->width = axisSize*scaling;
			c->depth = axisSize*scaling;
			c2->width = blockSize*scaling;
			c2->height = blockWidth*scaling;
			c2->depth = blockSize*scaling;
			t2->translation.setValue(0,axisBlockTranslation*scaling,0);
		} else
		{
			c->depth = axisLength*scaling;
			c->height = axisSize*scaling;
			c->width = axisSize*scaling;
			c2->width = blockSize*scaling;
			c2->height = blockSize*scaling;
			c2->depth = blockWidth*scaling;
			t2->translation.setValue(0,0,axisBlockTranslation*scaling);
		}
		tmp1->addChild(c);
		result->addChild(tmp1);

		SoSeparator *tmp2 = new SoSeparator();
		SoMaterial *m2 = new SoMaterial();
		m2->diffuseColor.setValue(1.0f,1.0f,1.0f);
		tmp2->addChild(m2);

		for (int j=0;j<nrOfBlocks;j++)
		{
			tmp2->addChild(t2);
			tmp2->addChild(c2);
		}

		result->addChild(tmp2);
	}

	if (text!=NULL)
	{
		SoSeparator *textSep = new SoSeparator();
		SoTranslation *moveT = new SoTranslation();
		moveT->translation.setValue(2.0f,2.0f,0.0f);
		textSep->addChild(moveT);
		SoAsciiText *textNode = new SoAsciiText();
		/*std::string text2(*text);
		text2.replace( ' ', "_" );*/
		SbString text2(text->c_str());
		text2.apply( &IVToolsHelper_ReplaceSpaceWithUnderscore );
		textNode->string.set(text2.getString());
		textSep->addChild(textNode);
		result->addChild(textSep);
	}
	return result;
}

SoSeparator* CoinVisualizationFactory::CreateVertexVisualization( const Eigen::Vector3f &position, float radius, float transparency, float colorR /*= 0.5f*/, float colorG /*= 0.5f*/, float colorB /*= 0.5f*/ )
{
	SoSeparator *res = new SoSeparator;

	// Control complexity  of the scene's primitives
	SoComplexity *comp = new SoComplexity;
	comp->value.setValue(0.1f);
	res->addChild(comp);

	// Set the vertex-position
	SoTranslation *t = new SoTranslation;
	t->translation.setValue(position(0), position(1), position(2));

	// Set material
	SoMaterial *m = new SoMaterial;
	m->transparency.setValue(transparency);


	m->diffuseColor.setValue(colorR, colorG, colorB);
	m->ambientColor.setValue(colorR, colorG, colorB);

	// Set shape
	SoSphere *s = new SoSphere;
	s->radius = radius;

	res->addChild(t);
	res->addChild(m);
	res->addChild(s);
	return res;
}

VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createVertexVisualization( const Eigen::Vector3f &position, float radius, float transparency, float colorR /*= 0.5f*/, float colorG /*= 0.5f*/, float colorB /*= 0.5f*/ )
{
	VisualizationNodePtr node(new CoinVisualizationNode(CreateVertexVisualization(position,radius,transparency,colorR, colorG, colorB)));
	return node;
}

VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createPlane( const Eigen::Vector3f &position, const Eigen::Vector3f &normal, float extend, float transparency, float colorR /*= 0.5f*/, float colorG /*= 0.5f*/, float colorB /*= 0.5f*/ )
{
	SoSeparator *res = CreatePlaneVisualization(position,normal,extend,transparency,false,colorR,colorG,colorB);

	VisualizationNodePtr node(new CoinVisualizationNode(res));
	return node;
}

SoSeparator* CoinVisualizationFactory::CreatePlaneVisualization( const Eigen::Vector3f &position, const Eigen::Vector3f &normal, float extend, float transparency, bool grid, float colorR /*= 0.5f*/, float colorG /*= 0.5f*/, float colorB /*= 0.5f*/ )
{
	SoSeparator *res = new SoSeparator();

	SoMatrixTransform *matrix = new SoMatrixTransform;
	SbMatrix mat;
	SbVec3f t(position(0),position(1),position(2));
	SbRotation r(SbVec3f(0,0,1.0f), SbVec3f(normal(0),normal(1),normal(2))); // rotateFrom, rotateTo
	mat.setTransform(t,r,SbVec3f(1.0f,1.0f,1.0f));
	matrix->matrix.setValue(mat);
	res->addChild(matrix);

	// Set material
	SoMaterial *m = new SoMaterial;
	m->transparency.setValue(transparency);
	m->diffuseColor.setValue(colorR, colorG, colorB);
	m->ambientColor.setValue(colorR, colorG, colorB);
	res->addChild(m);

	if (grid)
	{
		SoSeparator *res2;
		if (transparency==0)
			res2 = CreateGrid(extend,extend,extend/500.0f,extend/500.0f,true,VR_BASE_DIR"/data/images/FloorWhite.png",transparency);
		else
			res2 = CreateGrid(extend,extend,extend/500.0f,extend/500.0f,true,VR_BASE_DIR"/data/images/Floor.png",transparency);

		res->addChild(res2);
	} else
	{
		// Set shape
		SoCube *c = new SoCube;
		c->width = extend;
		c->depth = 1.0f; // Z
		c->height = extend;
		res->addChild(c);
	}

	return res;
}


SoSeparator* CoinVisualizationFactory::CreateGrid(float width,float depth,float widthMosaic,float depthMosaic,bool InvertNormal,const char* pFileName,float Transparency)
{
	SoSeparator* pGrid = new SoSeparator;
	pGrid->ref();
	SoMaterial* pSoMaterial = new SoMaterial;
	pSoMaterial->transparency = Transparency;
	float X = width/2.0f;
	float Z = depth/2.0f;
	SoCoordinate3 *pImagePlaneSoCoordinate3 = new SoCoordinate3;
	pImagePlaneSoCoordinate3->point.set1Value(0,SbVec3f(X,Z,0.0f));
	pImagePlaneSoCoordinate3->point.set1Value(1,SbVec3f(-X,Z,0.0f));
	pImagePlaneSoCoordinate3->point.set1Value(2,SbVec3f(-X,-Z,0.0f));
	pImagePlaneSoCoordinate3->point.set1Value(3,SbVec3f(X,-Z,0.0f));
	/*SoNormal* pSoNormal = new SoNormal;
	pSoNormal->vector.set1Value(0, SbVec3f(0.0f, 0.0f, InvertNormal?-1.0f:1.0f));
	SoNormalBinding* pSoNormalBinding = new SoNormalBinding;
	pSoNormalBinding->value.setValue(SoNormalBinding::OVERALL);*/
	SoFaceSet * pSoFaceSet = new SoFaceSet;
	pSoFaceSet->numVertices.set1Value(0, 4);
	SoTextureCoordinate2* pSoTextureCoordinate2 = new SoTextureCoordinate2;
	pSoTextureCoordinate2->point.set1Value(0, SbVec2f(widthMosaic, 0));
	pSoTextureCoordinate2->point.set1Value(1, SbVec2f(0, 0));
	pSoTextureCoordinate2->point.set1Value(2, SbVec2f(0, depthMosaic));
	pSoTextureCoordinate2->point.set1Value(3, SbVec2f(widthMosaic, depthMosaic));
	SoTextureCoordinateBinding* pSoTextureCoordinateBinding =  new SoTextureCoordinateBinding;
	pSoTextureCoordinateBinding->value.setValue (SoTextureCoordinateBinding::PER_VERTEX);
	SoTexture2* pSoTexture2 = new SoTexture2;
	if(pFileName)
		pSoTexture2->filename.setValue(pFileName);
	pSoTexture2->wrapS = pSoTexture2->wrapT = SoTexture2::REPEAT;
	pGrid->addChild(pSoMaterial);
	pGrid->addChild(pSoTextureCoordinate2);
	pGrid->addChild(pSoTextureCoordinateBinding);
	pGrid->addChild(pSoTexture2);
	pGrid->addChild(pImagePlaneSoCoordinate3);
	//pGrid->addChild(pSoNormal);
	//pGrid->addChild(pSoNormalBinding);
	pGrid->addChild(pSoFaceSet);
	pGrid->unrefNoDelete();
	return pGrid;
}

SoSeparator* CoinVisualizationFactory::CreatePolygonVisualization( const std::vector<Eigen::Vector3f> &points, VisualizationFactory::Color colorInner /*= VisualizationFactory::Color::Blue()*/, VisualizationFactory::Color colorLine /*= VisualizationFactory::Color::Black()*/, float lineSize /*= 5.0f*/ )
{
	SoSeparator* visu = new SoSeparator;
	if (points.size()==0)
		return visu;

	SoMaterial *m = new SoMaterial;
	m->diffuseColor.setValue(colorInner.r, colorInner.g, colorInner.b);
	m->ambientColor.setValue(colorInner.r, colorInner.g, colorInner.b);
	m->transparency.setValue(colorInner.transparency);
	visu->addChild(m);


	SoCoordinate3* coordinate3 = new SoCoordinate3;
	SoCoordinate3* coordinate3b = new SoCoordinate3;

	std::vector<SbVec3f> pt;
	for (size_t i=0;i<points.size();i++)
	{
		SbVec3f pt(points[i](0),points[i](1),points[i](2));
		coordinate3->point.set1Value(i,pt);
		coordinate3b->point.set1Value(i,pt);
	}
	SbVec3f pt0(points[0](0),points[0](1),points[0](2));
	coordinate3b->point.set1Value(points.size(),pt0);
	visu->addChild(coordinate3);
	SoFaceSet* faceSet = new SoFaceSet;
	faceSet->numVertices.set1Value(0,points.size());
	visu->addChild(faceSet);

	// create line around polygon
	SoSeparator *lineSep = new SoSeparator;
	visu->addChild(lineSep);
	SoMaterial *m2 = new SoMaterial;
	m2->diffuseColor.setValue(colorLine.r, colorLine.g, colorLine.b);
	m2->ambientColor.setValue(colorLine.r, colorLine.g, colorLine.b);
	m2->transparency.setValue(colorLine.transparency);
	lineSep->addChild(m2);
	lineSep->addChild(coordinate3b);

	SoDrawStyle *lineSolutionStyle = new SoDrawStyle();
	lineSolutionStyle->lineWidth.setValue(4.0f);
	lineSep->addChild(lineSolutionStyle);

	SoLineSet* lineSet = new SoLineSet;
	lineSet->numVertices.set1Value(0,points.size()+1);
	//faceSet->startIndex.setValue(0);
	lineSep->addChild(lineSet);

	return visu;
}


SoSeparator* CoinVisualizationFactory::CreateConvexHull2DVisualization( const MathTools::ConvexHull2DPtr ch, MathTools::Plane &p, VisualizationFactory::Color colorInner /*= VisualizationFactory::Color::Blue()*/, VisualizationFactory::Color colorLine /*= VisualizationFactory::Color::Black()*/, float lineSize /*= 5.0f*/, const Eigen::Vector3f &offset /*=Eigen::Vector3f::Zero() */ )
{
	std::vector<Eigen::Vector3f> cvHull3d;
	for (size_t u=0;u<ch->vertices.size();u++)
	{
		Eigen::Vector3f pt3d = MathTools::planePoint3D(ch->vertices[u],p);
		pt3d += offset;
		cvHull3d.push_back(pt3d);
	}

	return CoinVisualizationFactory::CreatePolygonVisualization(cvHull3d);
}

SoNode * CoinVisualizationFactory::getCoinVisualization( RobotPtr robot, SceneObject::VisualizationType visuType )
{
	if (!robot)
		return new SoSeparator;

	boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationRobot = robot->getVisualization<CoinVisualization>(visuType);
	if (visualizationRobot)
	{
		SoSeparator *result = new SoSeparator();
		result->ref();
		result->addChild(visualizationRobot->getCoinVisualization());
		result->unrefNoDelete();
		return result;
	}
	return new SoSeparator;
}

SoNode * CoinVisualizationFactory::getCoinVisualization( SceneObjectPtr object, SceneObject::VisualizationType visuType )
{
	if (!object)
		return new SoSeparator;

	boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject = object->getVisualization<CoinVisualization>(visuType);
	if (visualizationObject)
	{
		SoSeparator *result = new SoSeparator();
		result->ref();
		result->addChild(visualizationObject->getCoinVisualization());
		result->unrefNoDelete();
		return result;
	}

	return new SoSeparator;

}

SoNode * CoinVisualizationFactory::getCoinVisualization( VisualizationNodePtr visu )
{
	boost::shared_ptr< CoinVisualizationNode > coinVisu(boost::dynamic_pointer_cast< CoinVisualizationNode >(visu));
	if (!coinVisu)
		return new SoSeparator;
	return coinVisu->getCoinVisualization();	 
}

SoNode * CoinVisualizationFactory::getCoinVisualization( EndEffector::ContactInfo &contact )
{
	SoSeparator *result = new SoSeparator();
	result->ref();

	// add gfx for contact point on object
	SoSeparator *sep = new SoSeparator();
	SoTranslation *tr = new SoTranslation();
	SoMaterial *mat = new SoMaterial();
	mat->diffuseColor.setValue(1.0f,0,0);
	tr->translation.setValue(contact.contactPointObstacleGlobal(0),contact.contactPointObstacleGlobal(1),contact.contactPointObstacleGlobal(2));
	SoSphere *sph = new SoSphere();
	sph->radius.setValue(1);
	sep->addChild(mat);
	sep->addChild(tr);
	sep->addChild(sph);
	result->addChild(sep);

	// add gfx for contact point on finger
	SoSeparator *sep2 = new SoSeparator();
	SoTranslation *tr2 = new SoTranslation();
	tr2->translation.setValue(contact.contactPointFingerGlobal(0),contact.contactPointFingerGlobal(1),contact.contactPointFingerGlobal(2));
	SoSphere *sph2 = new SoSphere();
	sph2->radius.setValue(1);
	sep2->addChild(tr2);
	sep2->addChild(sph2);
	result->addChild(sep2);

	// add gfx for approach direction
	SoSeparator *sep3 = new SoSeparator();
	SoMatrixTransform *tr3 = new SoMatrixTransform();
	SbVec3f transl3(contact.contactPointObstacleGlobal(0),contact.contactPointObstacleGlobal(1),contact.contactPointObstacleGlobal(2));
	// compute rotation
	SbVec3f rotFrom(1.0f,0.0f,0.0f);
	SbVec3f rotTo;
	rotTo[0] = contact.contactPointObstacleGlobal[0] - contact.contactPointFingerGlobal[0];
	rotTo[1] = contact.contactPointObstacleGlobal[1] - contact.contactPointFingerGlobal[1];
	rotTo[2] = contact.contactPointObstacleGlobal[2] - contact.contactPointFingerGlobal[2];
	SbRotation rot3(rotFrom,rotTo);

	SbVec3f sc3;
	sc3[0] = sc3[1] = sc3[2] = 1.0f;
	SbMatrix m3;
	m3.setTransform(transl3,rot3,sc3);
	tr3->matrix.setValue(m3);

	// create cone
	float fConeHeight = 30.0f;
	float fConeRadius = 15.0f;
	SoCone *cone3 = new SoCone();
	cone3->bottomRadius = fConeRadius;
	cone3->height = fConeHeight;
	SoSeparator *ConeSep = new SoSeparator;
	SbMatrix orientCone;
	SbMatrix orientCone2;
	orientCone.makeIdentity();
	SbVec3f orientConeA(0.0f,0.0f,1.0f);
	SbRotation orientConeR(orientConeA,(float)(-M_PI/2.0f));
	orientCone.setRotate(orientConeR);
	SbVec3f coneTr(0,-fConeHeight/2.0f,0);
	orientCone2.setTranslate(coneTr);
	SoMatrixTransform * coneOri = new SoMatrixTransform();
	coneOri->matrix.setValue(orientCone2.multRight(orientCone));
	ConeSep->addChild(coneOri);
	ConeSep->addChild(cone3);
	// material
	SoMaterial *mat3 = new SoMaterial();
	mat3->diffuseColor.setValue(0.2f,0.7f,0.2f);
	mat3->ambientColor.setValue(0.2f,0.7f,0.2f);
	mat3->transparency.setValue(0.5f);

	sep3->addChild(mat3);
	sep3->addChild(tr3);
	sep3->addChild(ConeSep);
	result->addChild(sep3);
	// finished GFX for approach direction

	/*if (bHighlightFinger && pRobot)
	{
		CFinger *pFinger = contactInfo.pFinger;
		if (pFinger)
		{
			CRobotCollisionModelCollection* pCol = pFinger->GetCollisionModel();
			if (pCol)
			{
				std::set<CRobotNode*> vNodes;
				pCol->GetNodes(vNodes);
				std::set<CRobotNode*>::iterator iter = vNodes.begin();
				while (iter!=vNodes.end())
				{
					pRobot->SetHighlightNode(true,(*iter)->GetName());
					iter++;
				}
			}
		}
	}*/
	return result;
}

SoNode * CoinVisualizationFactory::getCoinVisualization( std::vector <EndEffector::ContactInfo> &contacts )
{
	SoSeparator *res = new SoSeparator;
	res->ref();
	for (size_t i=0;i<contacts.size();i++)
		res->addChild(getCoinVisualization(contacts[i]));
	res->unrefNoDelete();
	return res;
}

SoNode * CoinVisualizationFactory::getCoinVisualization( TriMeshModelPtr model, bool showNormals, VisualizationFactory::Color color )
{
	SoSeparator *res = new SoSeparator;
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
		SoSeparator* s = CreatePolygonVisualization(v,color);
		res->addChild(s);
		if (showNormals)
		{
			v1 = (v1 + v2 + v3) / 3.0f;

			SoSeparator *n = CreateArrow(model->faces[i].normal,30.0f,1.5f);
			SoMatrixTransform *mt = new SoMatrixTransform;
			Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
			mat.block(0,3,3,1) = v1;
			SbMatrix m(reinterpret_cast<SbMat*>(mat.data()));
			mt->matrix.setValue(m);
			SoSeparator *sn = new SoSeparator();
			sn->addChild(mt);
			sn->addChild(n);
			res->addChild(sn);
		}

	}
	res->unrefNoDelete();
	return res;
}




SoSeparator* CoinVisualizationFactory::CreateBBoxVisualization( const BoundingBox &bbox, bool wireFrame )
{
	SoSeparator *res = new SoSeparator;

	SoDrawStyle *ds = new SoDrawStyle;
	if (wireFrame)
	    ds->style = SoDrawStyle::LINES;
	else
	    ds->style = SoDrawStyle::FILLED;
	res->addChild(ds);

	SoTranslation *tr = new SoTranslation();
	float x1 = std::min(bbox.min(0), bbox.max(0));
	float x2 = std::max(bbox.min(0), bbox.max(0));
	float y1 = std::min(bbox.min(1), bbox.max(1));
	float y2 = std::max(bbox.min(1), bbox.max(1));
	float z1 = std::min(bbox.min(2), bbox.max(2));
	float z2 = std::max(bbox.min(2), bbox.max(2));
	float x = x1 + (x2 - x1) *0.5f;
	float y = y1 + (y2 - y1) *0.5f;
	float z = z1 + (z2 - z1) *0.5f;
	tr->translation.setValue(x,y,z);
	res->addChild(tr);

	SoCube *c = new SoCube;
	c->width = x2-x1;
	c->height = y2-y1;
	c->depth = z2-z1;

	res->addChild(c);
	return res;
}

VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createBoundingBox( const BoundingBox &bbox, bool wireFrame )
{
	SoSeparator *res = CreateBBoxVisualization(bbox, wireFrame);

	VisualizationNodePtr node(new CoinVisualizationNode(res));
	return node;

}

SoSeparator* CoinVisualizationFactory::CreateGraspVisualization( GraspPtr grasp, SoSeparator *eefVisu, const Eigen::Matrix4f &pose /*= Eigen::Matrix4f::Identity()*/ )
{
	if (!grasp || !eefVisu)
		return new SoSeparator;
	SoSeparator *res = new SoSeparator;
	res->ref();

	// grasp and transform
	Eigen::Matrix4f mat = pose * grasp->getTransformation().inverse();

	// grasp visu
	SoSeparator *sepGrasp = new SoSeparator;
	res->addChild(sepGrasp);

	// transform
	SoMatrixTransform *mT = new SoMatrixTransform();
	SbMatrix m(reinterpret_cast<SbMat*>(mat.data()));
	mT->matrix.setValue(m);
	sepGrasp->addChild(mT);

	// eef Visu
	sepGrasp->addChild(eefVisu);

	res->unrefNoDelete();
	return res;
}

SoSeparator* CoinVisualizationFactory::CreateGraspVisualization( GraspPtr grasp, EndEffectorPtr eef, const Eigen::Matrix4f &pose /*= Eigen::Matrix4f::Identity()*/, SceneObject::VisualizationType visu )
{
	THROW_VR_EXCEPTION_IF (!grasp,"NULL data");
	SoSeparator *eefV = CreateEndEffectorVisualization(eef, visu);
	if (!eefV)
		return new SoSeparator();
	eefV->ref();
	SoSeparator *res = CreateGraspVisualization(grasp, eefV, pose);
	eefV->unrefNoDelete();
	return res;
}

SoSeparator* CoinVisualizationFactory::CreateGraspSetVisualization( GraspSetPtr graspSet, EndEffectorPtr eef, const Eigen::Matrix4f &pose /*= Eigen::Matrix4f::Identity()*/, SceneObject::VisualizationType visu )
{
	THROW_VR_EXCEPTION_IF (!graspSet,"NULL data");
	SoSeparator *visual = new SoSeparator;
	visual->ref();
	SoSeparator *eefV = CreateEndEffectorVisualization(eef, visu);
	visual->addChild(eefV);

	SoSeparator *res = new SoSeparator;
	res->ref();
	for (unsigned int i=0;i<graspSet->getSize();i++)
	{
		// grasp and transform
		GraspPtr g = graspSet->getGrasp(i);
		SoSeparator *sepGrasp = CreateGraspVisualization(g, visual, pose);
		res->addChild(sepGrasp);
		
		/*Eigen::Matrix4f mat = pose * g->getTransformation().inverse();
		
		// grasp visu
		SoSeparator *sepGrasp = new SoSeparator;
		res->addChild(sepGrasp);

		// transform
		SoMatrixTransform *mT = new SoMatrixTransform();
		SbMatrix m(reinterpret_cast<SbMat*>(mat.data()));
		mT->matrix.setValue(m);
		sepGrasp->addChild(mT);

		// eef Visu
		sepGrasp->addChild(visual);*/
	}


	visual->unrefNoDelete();
	res->unrefNoDelete();	
	return res;
}

SoSeparator* CoinVisualizationFactory::CreateEndEffectorVisualization( EndEffectorPtr eef, SceneObject::VisualizationType visu)
{
	//THROW_VR_EXCEPTION_IF (!eef,"NULL data");
	SoSeparator * res = new SoSeparator;
	res->ref();
	RobotNodePtr tcp;
	bool ok = true;
	if (!eef)
		ok = false;
	else {
		tcp = eef->getTcp();
		if (!tcp)
		{
			VR_ERROR << " No tcp in eef " << eef->getName() << endl;
			ok = false;
		}
	}
	
	if (!ok)
	{
		SoSphere *s = new SoSphere();
		s->radius.setValue(50.0f);
		res->addChild(s);
	} else
	{
		RobotPtr r = eef->createEefRobot(eef->getName(),eef->getName());
		RobotNodePtr tcpN = r->getEndEffector(eef->getName())->getTcp();
		r->setGlobalPoseForRobotNode(tcpN,Eigen::Matrix4f::Identity());
		res->addChild(CoinVisualizationFactory::getCoinVisualization(r,visu));
	}
	res->unrefNoDelete();
	return res;
}

SoSeparator* CoinVisualizationFactory::CreatePointVisualization( const MathTools::ContactPoint &point, bool showNormals /*= false*/ )
{
	SoSeparator *res = new SoSeparator;
	
	SoMatrixTransform *mt = new SoMatrixTransform;
	mt->matrix.setValue(getSbMatrix(point.p));
	res->addChild(mt);

	SoSphere *s = new SoSphere;
	s->radius = 10.0f;
	res->addChild(s);
	
	if (showNormals)
	{
		res->addChild(CreateArrow(point.n));
	}

	return res;
}

SoSeparator* CoinVisualizationFactory::CreatePointsVisualization( const std::vector<MathTools::ContactPoint> &points, bool showNormals /*= false*/ )
{
	SoSeparator *res = new SoSeparator;
	std::vector<MathTools::ContactPoint>::const_iterator i = points.begin();
	while (i != points.end())
	{
		res->addChild(CreatePointVisualization(*i,showNormals));
		i++;
	}
	return res;
}

SbMatrix CoinVisualizationFactory::getSbMatrix( Eigen::Matrix4f &m )
{
	SbMatrix res(reinterpret_cast<SbMat*>(m.data()));
	return res;
}

SbMatrix CoinVisualizationFactory::getSbMatrix( const Eigen::Vector3f &p )
{
	SbMatrix res;
	res.makeIdentity();
	res.setTranslate(SbVec3f(p(0),p(1),p(2)));
	return res;
}

SoSeparator* CoinVisualizationFactory::CreateArrow( const Eigen::Vector3f &n, float length, float width, const Color &color )
{
	float coneHeight = width*6.0f;
	float coneBotomRadius = width*2.5f;
	SoSeparator *res = new SoSeparator;

	SbVec3f objNormal(n(0),n(1),n(2));
	SbMatrix objNormalTrafo;
	objNormalTrafo.makeIdentity();
	SbRotation objNormalRot(SbVec3f(0,1.0f,0),objNormal);

	// get rif of warnings when angle==0
	SbVec3f axis;
	float angle;
	objNormalRot.getValue(axis, angle); 
	if (angle!=0)
		objNormalTrafo.setRotate(objNormalRot);
	SoMatrixTransform *mt = new SoMatrixTransform;
	mt->matrix.setValue(objNormalTrafo);
	res->addChild(mt);

	if (!color.isNone())
	{
		SoMaterial *col = new SoMaterial();
		col->ambientColor.setValue(color.r,color.g,color.b);
		col->diffuseColor.setValue(color.r,color.g,color.b);
		col->transparency.setValue(color.transparency);
		res->addChild(col);
	}


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
}

VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createTriMeshModelVisualization( TriMeshModelPtr model, bool showNormals, Eigen::Matrix4f &pose )
{
	SoSeparator *res = new SoSeparator;
	SoNode *res1 = CoinVisualizationFactory::getCoinVisualization(model,showNormals);
	SoMatrixTransform *mt = getMatrixTransform(pose);
	res->addChild(mt);
	res->addChild(res1);

	VisualizationNodePtr node(new CoinVisualizationNode(res));
	return node;
}

SoMatrixTransform* CoinVisualizationFactory::getMatrixTransform( Eigen::Matrix4f &m )
{
	SoMatrixTransform *mt = new SoMatrixTransform;
	SbMatrix m_(reinterpret_cast<SbMat*>(m.data()));
	mt->matrix.setValue(m_);
	return mt;
}

VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createArrow( const Eigen::Vector3f &n, float length /*= 50.0f*/, float width /*= 2.0f*/, const Color &color /*= Color::Gray()*/ )
{
	SoSeparator *res = CreateArrow(n,length,width,color);

	VisualizationNodePtr node(new CoinVisualizationNode(res));
	return node;
}


SoNode* CoinVisualizationFactory::getCoinVisualization(ReachabilitySpacePtr reachSpace, int a, int b, int c, /*const Eigen::Vector3f &positionGlobal,*/ int nrBestEntries, SoSeparator* arrow, const VirtualRobot::ColorMap &cm, bool transformToGlobalPose, unsigned char minValue)
{
	SoSeparator *res = new SoSeparator;
	res->ref();
	if(!reachSpace || reachSpace->numVoxels[0] <= 0 || reachSpace->numVoxels[1] <= 0 || reachSpace->numVoxels[2] <= 0)
	{
		res->unrefNoDelete();
		return NULL;
	}

	//float x[6];
	unsigned int v[6];
	Eigen::Matrix4f m;
/*	MathTools::posrpy2eigen4f(positionGlobal,Eigen::Vector3f::Zero(),m);
	Eigen::Vector3f posLocal = positionGlobal;
	if (reachSpace->baseNode)
	{
		m = reachSpace->baseNode->toLocalCoordinateSystem(m);
		posLocal = reachSpace->baseNode->toLocalCoordinateSystem(posLocal);
	}
	MathTools::eigen4f2rpy(m,x);
	// get voxels
	if (!reachSpace->getVoxelFromPose(x,v))
	{
		VR_ERROR << "could not get voxel from pose " << m << endl;
		res->unrefNoDelete();
		return NULL;
	}
*/
	v[0] = a;
	v[1] = b;
	v[2] = c;
	v[3] = 0;
	v[4] = 0;
	v[5] = 0;
	Eigen::Vector3f sizePos;//voxelOrientationLocal, size;
	sizePos(0) = reachSpace->spaceSize[0] / reachSpace->numVoxels[0];
	sizePos(1) = reachSpace->spaceSize[1] / reachSpace->numVoxels[1];
	sizePos(2) = reachSpace->spaceSize[2] / reachSpace->numVoxels[2];
	Eigen::Vector3f posLocal(reachSpace->minBounds[0] + ((float)a + 0.5f)*sizePos(0),reachSpace->minBounds[1] + ((float)b + 0.5f)*sizePos(1),reachSpace->minBounds[2] + ((float)c + 0.5f)*sizePos(2));


	Eigen::Vector3f size;//voxelOrientationLocal, size;
	size(0) = reachSpace->spaceSize[3] / reachSpace->numVoxels[3];
	size(1) = reachSpace->spaceSize[4] / reachSpace->numVoxels[4];
	size(2) = reachSpace->spaceSize[5] / reachSpace->numVoxels[5];
	std::map< unsigned char, std::vector<Eigen::Vector3f> > entryRPY;

	for(unsigned int d = 0; d < (unsigned int)reachSpace->numVoxels[3]; d++)
	{
		//voxelOrientationLocal(0) = reachSpace->minBounds[3] + (d + 0.5f)*size(0);
		v[3] = d;// reachSpace->minBounds[3] + (d + 0.5f)*size(0);
		for(unsigned int e = 0; e < (unsigned int)reachSpace->numVoxels[4]; e++)
		{
			//voxelOrientationLocal(1) = reachSpace->minBounds[4] + (e + 0.5f)*size(1);
			v[4] = e;//reachSpace->minBounds[4] + (e + 0.5f)*size(1);

			for(unsigned int f = 0; f < (unsigned int)reachSpace->numVoxels[5]; f++)
			{
				//voxelOrientationLocal(2) = reachSpace->minBounds[5] + (f + 0.5f)*size(2);
				v[5] = f;//reachSpace->minBounds[5] + (f + 0.5f)*size(2);
				/*
				Eigen::Matrix4f m;
				Eigen::Vector3f oGlobal = voxelOrientationLocal;
				MathTools::rpy2eigen4f(voxelOrientationLocal(0),voxelOrientationLocal(1),voxelOrientationLocal(2),m);
				if (reachSpace->baseNode)
				{
					m = reachSpace->baseNode->toGlobalCoordinateSystem(m);
					MathTools::eigen4f2rpy(m,oGlobal);
				}
				MathTools::posrpy2eigen4f(positionGlobal,oGlobal,m);
				unsigned int entry = reachSpace->getEntry(m);
				*/
				unsigned int entry = reachSpace->data->get(v);
				if (entry>0)
					entryRPY[entry].push_back(Eigen::Vector3f(reachSpace->minBounds[3] + ((float)d + 0.5f)*size(0),reachSpace->minBounds[4] + ((float)e + 0.5f)*size(1),reachSpace->minBounds[5] + ((float)f + 0.5f)*size(2)));
			}
		}
	}
	if (entryRPY.size()==0)
	{
		res->unrefNoDelete();
		return NULL;
	}
	VirtualRobot::VisualizationFactory::Color color = VirtualRobot::VisualizationFactory::Color::None();
	std::map< unsigned char, std::vector<Eigen::Vector3f> >::iterator i = entryRPY.end();
	int nr = 0;
	while (i!= entryRPY.begin() && nr<nrBestEntries)
	{
		i--;
		for (size_t j=0; j<i->second.size();j++)
		{
			// create visu
			SoSeparator *sep = new SoSeparator;
			float intensity = (float)i->first;
			if (reachSpace->getMaxEntry()>0)
				intensity /= (float)reachSpace->getMaxEntry();
			if (intensity>1.0f)
				intensity = 1.0f;
			color = cm.getColor(intensity);

			SoMaterial *col = new SoMaterial();
			col->ambientColor.setValue(color.r,color.g,color.b);
			col->diffuseColor.setValue(color.r,color.g,color.b);
			col->transparency.setValue(color.transparency);
			sep->addChild(col);
			Eigen::Matrix4f pose;
			MathTools::posrpy2eigen4f(posLocal,i->second[j],pose);

			if (transformToGlobalPose && reachSpace->baseNode)
				pose = reachSpace->baseNode->toGlobalCoordinateSystem(pose);
			SoMatrixTransform *mt = getMatrixTransform(pose);
			sep->addChild(mt);
			sep->addChild(arrow);
			res->addChild(sep);
			nr++;
			if (nr>=nrBestEntries)
				break;
		}
		i--;
	}
	res->unrefNoDelete();
	return res;
}

SoNode* CoinVisualizationFactory::getCoinVisualization(ReachabilitySpacePtr reachSpace, const Eigen::Vector3f &fixedEEFOrientationGlobalRPY, VirtualRobot::ColorMap::type cmType, bool transformToGlobalPose, const Eigen::Vector3f &axis, unsigned char minValue, float arrowSize)
{
	SoSeparator *res = new SoSeparator;
	res->ref();
	if(!reachSpace || reachSpace->numVoxels[0] <= 0 || reachSpace->numVoxels[1] <= 0 || reachSpace->numVoxels[2] <= 0)
	{
		res->unrefNoDelete();
		return res;
	}

	Eigen::Matrix4f m;
	Eigen::Vector3f oLocal = fixedEEFOrientationGlobalRPY;
	MathTools::rpy2eigen4f(fixedEEFOrientationGlobalRPY(0),fixedEEFOrientationGlobalRPY(1),fixedEEFOrientationGlobalRPY(2),m);
	if (reachSpace->baseNode)
	{
		m = reachSpace->baseNode->toLocalCoordinateSystem(m);
		MathTools::eigen4f2rpy(m,oLocal);
	}

	ColorMap cm(cmType);

	Eigen::Vector3f voxelPosition, size;
	int d,e,f;

	size(0) = reachSpace->spaceSize[0] / reachSpace->numVoxels[0];
	size(1) = reachSpace->spaceSize[1] / reachSpace->numVoxels[1];
	size(2) = reachSpace->spaceSize[2] / reachSpace->numVoxels[2];
	float minS = size(0);
	if (size(1)<minS) minS = size(1);
	if (size(2)<minS) minS = size(2);
	if (arrowSize!=0)
		minS = arrowSize;
	//Eigen::Vector3f zAxis(0,0,1.0f);
	int value;
	VirtualRobot::VisualizationFactory::Color color = VirtualRobot::VisualizationFactory::Color::None();
	SoSeparator* arrow = CreateArrow(axis,minS*0.75f,minS/20.0f,color);

	if (minValue<=0) 
		minValue = 1;

	for(int a = 0; a < reachSpace->numVoxels[0]; a++)
	{
		voxelPosition(0) = reachSpace->minBounds[0] + (a + 0.5f)*size(0);
		for(int b = 0; b < reachSpace->numVoxels[1]; b++)
		{
			voxelPosition(1) = reachSpace->minBounds[1] + (b + 0.5f)*size(1);

			//int cSize = testMode? numVoxels[2]/2 : numVoxels[2];
			int cSize = reachSpace->numVoxels[2];
			for(int c = 0; c < cSize; c++)
			{
				voxelPosition(2) = reachSpace->minBounds[2] + (c + 0.5f)*size(2);

				// get voxel from orientation
				d = (int)(((oLocal(0)-reachSpace->minBounds[3]) / reachSpace->spaceSize[3]) * (float)reachSpace->numVoxels[3]);
				e = (int)(((oLocal(1)-reachSpace->minBounds[4]) / reachSpace->spaceSize[4]) * (float)reachSpace->numVoxels[4]);
				f = (int)(((oLocal(2)-reachSpace->minBounds[5]) / reachSpace->spaceSize[5]) * (float)reachSpace->numVoxels[5]);
				if (d>=0 && d<=reachSpace->numVoxels[3] && e>=0 && e<=reachSpace->numVoxels[4] && f>=0 && f<=reachSpace->numVoxels[5])
					value = reachSpace->data->get(a, b, c, d, e, f);
				else 
					value = 0;
				if(value >= minValue)
				{
					MathTools::posrpy2eigen4f(voxelPosition,oLocal,m);
					if(transformToGlobalPose && reachSpace->baseNode)
					{
						m = reachSpace->baseNode->toGlobalCoordinateSystem(m);
					}
					SoSeparator *sep = new SoSeparator;
					float intensity = (float)value;
					if (reachSpace->getMaxEntry()>0)
						intensity /= (float)reachSpace->getMaxEntry();
					if (intensity>1.0f)
						intensity = 1.0f;
					color = cm.getColor(intensity);

					SoMaterial *col = new SoMaterial();
					col->ambientColor.setValue(color.r,color.g,color.b);
					col->diffuseColor.setValue(color.r,color.g,color.b);
					col->transparency.setValue(color.transparency);
					sep->addChild(col);
					SoMatrixTransform *mt = getMatrixTransform(m);
					sep->addChild(mt);
					sep->addChild(arrow);
					res->addChild(sep);
				}
			}
		}
	}
	res->unrefNoDelete();
	return res;
}
/*
SoNode* CoinVisualizationFactory::getCoinVisualization(ReachabilitySpacePtr reachSpace,  VirtualRobot::ColorMap::type cmType, const Eigen::Vector3f &axis, bool transformToGlobalPose, unsigned char minValue)
{
	SoSeparator *res = new SoSeparator;
	res->ref();
	if(!reachSpace || reachSpace->numVoxels[3] <= 0 || reachSpace->numVoxels[4] <= 0 || reachSpace->numVoxels[5] <= 0)
	{
		res->unrefNoDelete();
		return res;
	}
	int steps = 2;
	Eigen::Vector3f voxelOrientationLocal, size;
	size(0) = reachSpace->spaceSize[3] / reachSpace->numVoxels[3];
	size(1) = reachSpace->spaceSize[4] / reachSpace->numVoxels[4];
	size(2) = reachSpace->spaceSize[5] / reachSpace->numVoxels[5];
	for(int d = 0; d < reachSpace->numVoxels[3]; d+=steps)
	{
		voxelOrientationLocal(0) = reachSpace->minBounds[3] + (d + 0.5f)*size(0);
		for(int e = 0; e < reachSpace->numVoxels[4]; e+=steps)
		{
			voxelOrientationLocal(1) = reachSpace->minBounds[4] + (e + 0.5f)*size(1);

			for(int f = 0; f < reachSpace->numVoxels[5]; f+=steps)
			{
				voxelOrientationLocal(2) = reachSpace->minBounds[5] + (f + 0.5f)*size(2);

				Eigen::Matrix4f m;
				Eigen::Vector3f oGlobal = voxelOrientationLocal;
				MathTools::rpy2eigen4f(voxelOrientationLocal(0),voxelOrientationLocal(1),voxelOrientationLocal(2),m);
				if (reachSpace->baseNode)
				{
					m = reachSpace->baseNode->toGlobalCoordinateSystem(m);
					MathTools::eigen4f2rpy(m,oGlobal);
				}
				SoNode* resOri = getCoinVisualization(reachSpace,oGlobal,cmType,transformToGlobalPose,axis,minValue);
				res->addChild(resOri);
			}
		}
	}
	res->unrefNoDelete();
	if (res->getNumChildren()==0)
		return NULL;
	return res;
}
*/

SoNode* CoinVisualizationFactory::getCoinVisualization(ReachabilitySpacePtr reachSpace, VirtualRobot::ColorMap::type cmType, const Eigen::Vector3f &axis, bool transformToGlobalPose, unsigned char minValue, float arrowSize)
{
	SoSeparator *res = new SoSeparator;
	res->ref();
	if(!reachSpace || reachSpace->numVoxels[0] <= 0 || reachSpace->numVoxels[1] <= 0 || reachSpace->numVoxels[2] <= 0)
	{
		res->unrefNoDelete();
		return res;
	}
	Eigen::Vector3f size;
	size(0) = reachSpace->spaceSize[0] / reachSpace->numVoxels[0];
	size(1) = reachSpace->spaceSize[1] / reachSpace->numVoxels[1];
	size(2) = reachSpace->spaceSize[2] / reachSpace->numVoxels[2];
	float minS = size(0);
	if (size(1)<minS) minS = size(1);
	if (size(2)<minS) minS = size(2);
	if (arrowSize!=0)
		minS = arrowSize;
	VirtualRobot::VisualizationFactory::Color color = VirtualRobot::VisualizationFactory::Color::None();
	SoSeparator* arrow = CreateArrow(axis,minS*0.7f,minS/25.0f,color);

	VirtualRobot::ColorMap cm(cmType);
	Eigen::Vector3f voxelPosition;
	int step = 1;
	for(int a = 0; a < reachSpace->numVoxels[0]; a+=step)
	{
		voxelPosition(0) = reachSpace->minBounds[0] + (a + 0.5f)*size(0);
		for(int b = 0; b < reachSpace->numVoxels[1]; b+=step)
		{
			voxelPosition(1) = reachSpace->minBounds[1] + (b + 0.5f)*size(1);

			//int cSize = testMode? numVoxels[2]/2 : numVoxels[2];
			int cSize = reachSpace->numVoxels[2];
			for(int c = 0; c < cSize; c+=step)
			{
				voxelPosition(2) = reachSpace->minBounds[2] + (c + 0.5f)*size(2);
				SoNode *n = getCoinVisualization(reachSpace, a,b,c, 1, arrow, cm, transformToGlobalPose, minValue);
				if (n)
					res->addChild(n);
			}
		}
	}
	return res;
}


SoNode* CoinVisualizationFactory::getCoinVisualization(ReachabilitySpacePtr reachSpace, const VirtualRobot::ColorMap::type cmType, bool transformToGlobalPose)
{
	SoSeparator *res = new SoSeparator;
	res->ref();
	if(!reachSpace || reachSpace->numVoxels[0] <= 0 || reachSpace->numVoxels[1] <= 0 || reachSpace->numVoxels[2] <= 0)
	{
		res->unrefNoDelete();
		return res;
	}
	Eigen::Vector3f size;
	size(0) = reachSpace->spaceSize[0] / reachSpace->numVoxels[0];
	size(1) = reachSpace->spaceSize[1] / reachSpace->numVoxels[1];
	size(2) = reachSpace->spaceSize[2] / reachSpace->numVoxels[2];
	float minS = size(0);
	if (size(1)<minS) minS = size(1);
	if (size(2)<minS) minS = size(2);

	ColorMap cm(cmType);

	VirtualRobot::VisualizationFactory::Color color = VirtualRobot::VisualizationFactory::Color::None();
	float radius = minS*0.5f*0.75f;
	Eigen::Vector3f voxelPosition;
	int step = 1;
	int maxValue = 0;
	for(int a = 0; a < reachSpace->numVoxels[0]; a+=step)
	{
		for(int b = 0; b < reachSpace->numVoxels[1]; b+=step)
		{
			for(int c = 0; c < reachSpace->numVoxels[2]; c+=step)
			{
				int value = reachSpace->sumAngleReachabilities(a, b, c);
				if (value>=maxValue)
					maxValue = value;
			}
		}
	}

	for(int a = 0; a < reachSpace->numVoxels[0]; a+=step)
	{
		voxelPosition(0) = reachSpace->minBounds[0] + (a + 0.5f)*size(0);
		for(int b = 0; b < reachSpace->numVoxels[1]; b+=step)
		{
			voxelPosition(1) = reachSpace->minBounds[1] + (b + 0.5f)*size(1);

			//int cSize = testMode? numVoxels[2]/2 : numVoxels[2];
			int cSize = reachSpace->numVoxels[2];
			for(int c = 0; c < cSize; c+=step)
			{
				voxelPosition(2) = reachSpace->minBounds[2] + (c + 0.5f)*size(2);

				int value = reachSpace->sumAngleReachabilities(a, b, c);
				if (value>0)
				{
					if(transformToGlobalPose && reachSpace->baseNode)
					{
						voxelPosition = reachSpace->baseNode->toGlobalCoordinateSystem(voxelPosition);
					}
					float intensity = (float)value;
					if (maxValue>0)
						intensity /= maxValue;
					if (intensity>1.0f)
						intensity = 1.0f;
					color = cm.getColor(intensity);

					SoNode *n = CreateVertexVisualization(voxelPosition,radius, color.transparency,color.r,color.g,color.b);
					if (n)
						res->addChild(n);
				}
			}
		}
	}
	return res;
}

SoSeparator* CoinVisualizationFactory::Colorize( SoNode *model, VisualizationFactory::Color c )
{
	SoSeparator *result = new SoSeparator;
	SoBaseColor *bc = new SoBaseColor();
	bc->rgb.setValue(c.r,c.g,c.b);
	bc->rgb.setIgnored(FALSE);
	bc->setOverride(TRUE);
	result->addChild(bc);
	if (model)
		result->addChild(model);
	return result;
}

SoOffscreenRenderer* CoinVisualizationFactory::createOffscreenRenderer( int width, int height )
{
	// Set up the offscreen renderer
	SbViewportRegion vpRegion(width, height);
	SoOffscreenRenderer *offscreenRenderer = new SoOffscreenRenderer(vpRegion);
	offscreenRenderer->setComponents(SoOffscreenRenderer::RGB);
	offscreenRenderer->setBackgroundColor(SbColor(1.0f,1.0f,1.0f));
	return offscreenRenderer;
}

bool CoinVisualizationFactory::renderOffscreen( SoOffscreenRenderer* renderer, RobotNodePtr camNode, SoNode* scene, unsigned char **buffer )
{
	if (!camNode)
	{
		VR_ERROR << "No cam node to render..." << endl;
		return false;
	}

	SoPerspectiveCamera *cam = new SoPerspectiveCamera();
	cam->ref();
	// set camera position and orientation
	Eigen::Matrix4f camPose = camNode->getGlobalPose();
	Eigen::Vector3f camPos = MathTools::getTranslation(camPose);
	cam->position.setValue(camPos[0],camPos[1],camPos[2]);
	SbRotation align(SbVec3f(1,0,0),(float)(M_PI)); // first align from  default direction -z to +z by rotating with 180 degree around x axis
	SbRotation align2(SbVec3f(0,0,1),(float)(M_PI/2.0)); // align up vector by rotating with 90 degree around z axis
	SbRotation trans(CoinVisualizationFactory::getSbMatrix(camPose)); // get rotation from global pose
	cam->orientation.setValue( align2*align*trans ); // perform total transformation

	// todo: check these values....
	cam->nearDistance.setValue(1.0f);
	cam->farDistance.setValue(5000.0f);

	bool res = renderOffscreen(renderer,cam,scene,buffer);
	cam->unref();
	return res;
}



bool CoinVisualizationFactory::renderOffscreen( SoOffscreenRenderer* renderer, SoCamera* cam, SoNode* scene, unsigned char **buffer )
{
	if (!renderer || !cam || !scene || buffer==NULL)
		return false;

	// we use MM in VirtualRobot
	SoUnits *unit = new SoUnits();
	unit->units = SoUnits::MILLIMETERS;

	// add all to a inventor scene graph
	SoSeparator *root = new SoSeparator();
	root->ref();
	SoDirectionalLight *light = new SoDirectionalLight;
	root->addChild(light);

	// easy light model, no shadows or something
	//SoLightModel *lightModel = new SoLightModel();
	//lightModel->model = SoLightModel::BASE_COLOR;
	//root->addChild(lightModel);

	root->addChild(unit);
	root->addChild(cam);
	root->addChild(scene);


	bool ok = renderer->render(root)==TRUE?true:false;
	root->unref();
	if (!ok)
	{
		VR_ERROR << "Rendering not successful!" << endl;
		return false;
	}
	*buffer = renderer->getBuffer();
	return true;
}



} // namespace VirtualRobot
