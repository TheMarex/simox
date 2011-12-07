
#include "CoinRrtWorkspaceVisualization.h"
#include "CSpace/CSpacePath.h"
#include "CSpace/CSpaceTree.h"
#include "CSpace/CSpaceNode.h"

#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoShape.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMatrixTransform.h>


#include <Inventor/nodes/SoComplexity.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoLineSet.h>

#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoUnits.h>


namespace Saba {

CoinRrtWorkspaceVisualization::CoinRrtWorkspaceVisualization(VirtualRobot::RobotPtr robot, CSpacePtr cspace, const std::string &TCPName) :
RrtWorkspaceVisualization(robot,cspace,TCPName)
{
	init();
}

CoinRrtWorkspaceVisualization::CoinRrtWorkspaceVisualization(VirtualRobot::RobotPtr robot, VirtualRobot::RobotNodeSetPtr robotNodeSet, const std::string &TCPName) :
RrtWorkspaceVisualization(robot,robotNodeSet,TCPName)
{
	init();
}

void CoinRrtWorkspaceVisualization::init()
{
	visualization = new SoSeparator();
	visualization->ref();
	setPathStyle();
	setTreeStyle();

	RenderColors red;
	red.nodeR = 1.0f;
	red.nodeG = 0;
	red.nodeB = 0;
	red.lineR = 0.5f;
	red.lineG = 0.5f;
	red.lineB = 0.5f;
	colors[CoinRrtWorkspaceVisualization::eRed] = red;

	RenderColors blue;
	blue.nodeR = 0;
	blue.nodeG = 0;
	blue.nodeB = 1.0f;
	blue.lineR = 0.5f;
	blue.lineG = 0.5f;
	blue.lineB = 0.5f;
	colors[CoinRrtWorkspaceVisualization::eBlue] = blue;

	RenderColors green;
	green.nodeR = 0;
	green.nodeG = 1.0f;
	green.nodeB = 0;
	green.lineR = 0.5f;
	green.lineG = 0.5f;
	green.lineB = 0.5f;
	colors[CoinRrtWorkspaceVisualization::eGreen] = green;

	RenderColors custom;
	custom.nodeR = 1.0f;
	custom.nodeG = 1.0f;
	custom.nodeB = 0;
	custom.lineR = 0.5f;
	custom.lineG = 0.5f;
	custom.lineB = 0.5f;
	colors[CoinRrtWorkspaceVisualization::eCustom] = custom;

}

/**
 * If CoinRrtWorkspaceVisualization::visualization is a valid object call SoNode::unref()
 * on it.
 */
CoinRrtWorkspaceVisualization::~CoinRrtWorkspaceVisualization()
{
	if (visualization)
		visualization->unref();
}



/**
 * This mehtod returns the internal CoinRrtWorkspaceVisualization::visualization.
 */
SoSeparator* CoinRrtWorkspaceVisualization::getCoinVisualization()
{
	return visualization;
}

void CoinRrtWorkspaceVisualization::setPathStyle(float lineSize, float nodeSize, float renderComplexity)
{
	pathLineSize = lineSize;
	pathNodeSize = nodeSize;
	pathRenderComplexity = renderComplexity;
}

void CoinRrtWorkspaceVisualization::setTreeStyle(float lineSize, float nodeSize, float renderComplexity)
{
	treeLineSize = lineSize;
	treeNodeSize = nodeSize;
	treeRenderComplexity = renderComplexity;
}

void CoinRrtWorkspaceVisualization::setCustomColor(float nodeR, float nodeG, float nodeB, float lineR, float lineG, float lineB )
{
	RenderColors custom;
	custom.nodeR = nodeR;
	custom.nodeG = nodeG;
	custom.nodeB = nodeB;
	custom.lineR = lineR;
	custom.lineG = lineG;
	custom.lineB = lineB;
	colors[CoinRrtWorkspaceVisualization::eCustom] = custom;
}


bool CoinRrtWorkspaceVisualization::addCSpacePath(CSpacePathPtr path, CoinRrtWorkspaceVisualization::ColorSet colorSet)
{
	if (!path)
		return false;
	if (path->getDimension() != robotNodeSet->getSize())
	{
		VR_ERROR << " Dimensions do not match: " << path->getDimension() <<"!="<< robotNodeSet->getSize()<<endl;
		return false;
	}
	if (!TCPNode)
		return false;

	float nodeSolutionSize = pathNodeSize;//15.0;//1.0f
	float lineSolutionSize = pathLineSize;//4.0;
	SoMaterial *materialNodeSolution = new SoMaterial();
	SoMaterial *materialLineSolution = new SoMaterial();
	materialNodeSolution->ambientColor.setValue(colors[colorSet].nodeR,colors[colorSet].nodeG,colors[colorSet].nodeB);
	materialNodeSolution->diffuseColor.setValue(colors[colorSet].nodeR,colors[colorSet].nodeG,colors[colorSet].nodeB);
	materialLineSolution->ambientColor.setValue(colors[colorSet].lineR,colors[colorSet].lineG,colors[colorSet].lineB);
	materialLineSolution->diffuseColor.setValue(colors[colorSet].lineR,colors[colorSet].lineG,colors[colorSet].lineB);
	SoSphere *sphereNodeSolution = new SoSphere();
	sphereNodeSolution->radius.setValue(nodeSolutionSize);
	SoDrawStyle *lineSolutionStyle = new SoDrawStyle();
	lineSolutionStyle->lineWidth.setValue(lineSolutionSize);

	Eigen::VectorXf actConfig;
	Eigen::VectorXf parentConfig;
	float x,y,z;
	float x2 = 0.0f,y2 = 0.0f,z2 = 0.0f;

	SoSeparator *sep = new SoSeparator();

	SoComplexity *comple;
	comple = new SoComplexity();
	comple->value = pathRenderComplexity;
	sep->addChild(comple);

	for (unsigned int i = 0; i < path->getNrOfPathPoints(); i++)
	{
		actConfig = path->getPathEntry(i);

		// create 3D model for nodes
		SoSeparator *s = new SoSeparator();

		s->addChild(materialNodeSolution);
		SoTranslation *t = new SoTranslation();

		if (cspace->hasExclusiveRobotAccess())
			CSpace::lock();

		// get tcp coords:
		robotNodeSet->setJointValues(actConfig);
		Eigen::Matrix4f m;
		m = TCPNode->getGlobalPose();
		x = m(0,3);
		y = m(1,3);
		z = m(2,3);

		if (cspace->hasExclusiveRobotAccess())
			CSpace::unlock();

		t->translation.setValue(x,y,z);
		s->addChild(t);
		// display a solution node different
		s->addChild(sphereNodeSolution);
		sep->addChild(s);

		if (i>0) // lines for all configurations
		{
			// create line to parent
			SoSeparator *s2 = new SoSeparator();

			SbVec3f points[2];
			points[0].setValue(x2,y2,z2);
			points[1].setValue(x,y,z);

			s2->addChild(lineSolutionStyle);
			s2->addChild(materialLineSolution);

			SoCoordinate3* coordinate3 = new SoCoordinate3;
			coordinate3->point.set1Value(0,points[0]);
			coordinate3->point.set1Value(1,points[1]);
			s2->addChild(coordinate3);

			SoLineSet* lineSet = new SoLineSet;
			lineSet->numVertices.setValue(2);
			lineSet->startIndex.setValue(0);
			s2->addChild(lineSet);

			sep->addChild(s2);
		}
		x2 = x;
		y2 = y;
		z2 = z;
		parentConfig = actConfig;
	} // for

	visualization->addChild(sep);
	return true;
}

bool CoinRrtWorkspaceVisualization::addTree(CSpaceTreePtr tree, CoinRrtWorkspaceVisualization::ColorSet colorSet)
{
	if (!tree)
		return false;
	if (tree->getDimension() != robotNodeSet->getSize())
	{
		VR_ERROR << " Dimensions do not match: " << tree->getDimension() <<"!="<< robotNodeSet->getSize()<<endl;
		return false;
	}
	if (!TCPNode)
		return false;

	Eigen::VectorXf actConfig;
	Eigen::VectorXf parentConfig;

	CSpaceNodePtr actualNode;
	CSpaceNodePtr parentNode;
	int parentid;

	SoMaterial *materialNode = new SoMaterial();
	SoMaterial *materialLine = new SoMaterial();
	materialLine->ambientColor.setValue(colors[colorSet].lineR,colors[colorSet].lineG,colors[colorSet].lineB);
	materialLine->diffuseColor.setValue(colors[colorSet].lineR,colors[colorSet].lineG,colors[colorSet].lineB);
	materialNode->ambientColor.setValue(colors[colorSet].nodeR,colors[colorSet].nodeG,colors[colorSet].nodeB);
	materialNode->diffuseColor.setValue(colors[colorSet].nodeR,colors[colorSet].nodeG,colors[colorSet].nodeB);

	SoSphere *sphereNode = new SoSphere();
	sphereNode->radius.setValue(treeNodeSize);

	SoDrawStyle *lineStyle = new SoDrawStyle();
	lineStyle->lineWidth.setValue(treeLineSize);

	SoSeparator *sep = new SoSeparator();

	SoComplexity *comple;
	comple = new SoComplexity();
	comple->value = treeRenderComplexity;
	sep->addChild(comple);
	
	std::vector<CSpaceNodePtr> nodes = tree->getNodes();

	// pre-compute tcp positions
	bool updateVisMode = robot->getUpdateVisualizationStatus();
	robot->setUpdateVisualization(false);

	std::map<CSpaceNodePtr,Eigen::Vector3f> tcpCoords;
	Eigen::Vector3f p;
	Eigen::Vector3f p2;
	for (unsigned int i = 0; i < nodes.size(); i++)
	{
		actualNode = nodes[i];

		// get tcp coords:
		robotNodeSet->setJointValues(actualNode->configuration);
		Eigen::Matrix4f m;
		m = TCPNode->getGlobalPose();
		p(0) = m(0,3);
		p(1) = m(1,3);
		p(2) = m(2,3);

		tcpCoords[actualNode] = p;
	}


	for (unsigned int i = 0; i < nodes.size(); i++)
	{

		actualNode = nodes[i];
		parentid = actualNode->parentID;

		// create 3D model for nodes
		SoSeparator *s = new SoSeparator();

	
		s->addChild(materialNode);

		// get tcp coords
		p = tcpCoords[actualNode];

		SoTranslation *t = new SoTranslation();
		t->translation.setValue(p(0),p(1),p(2));
		s->addChild(t);
		s->addChild(sphereNode);
		sep->addChild(s);

		// not for the start node! startNode->parentID < 0
		if (parentid >= 0) // lines for all configurations
		{
			// create line to parent
			SoSeparator *s2 = new SoSeparator();
			parentNode = tree->getNode(parentid);
			if (parentNode)
			{

				// get tcp coords
				p2 = tcpCoords[parentNode];

				s2->addChild(lineStyle);
				s2->addChild(materialLine);

				SbVec3f points[2];
				points[0].setValue(p(0),p(1),p(2));
				points[1].setValue(p2(0),p2(1),p2(2));

				SoCoordinate3* coordinate3 = new SoCoordinate3;
				coordinate3->point.set1Value(0,points[0]);
				coordinate3->point.set1Value(1,points[1]);
				s2->addChild(coordinate3);

				SoLineSet* lineSet = new SoLineSet;
				lineSet->numVertices.setValue(2);
				lineSet->startIndex.setValue(0);
				s2->addChild(lineSet);

				sep->addChild(s2);
			}
		} 
	}
	visualization->addChild(sep);

	robot->setUpdateVisualization(updateVisMode);

	return true;
}

void CoinRrtWorkspaceVisualization::reset()
{
	if (visualization)
		visualization->removeAllChildren();
}

bool CoinRrtWorkspaceVisualization::addConfiguration( const Eigen::VectorXf &c, CoinRrtWorkspaceVisualization::ColorSet colorSet, float nodeSizeFactor )
{
	if (c.rows() != robotNodeSet->getSize())
	{
		VR_ERROR << " Dimensions do not match: " << c.rows() <<"!="<< robotNodeSet->getSize() << endl;
		return false;
	}
	if (!TCPNode)
		return false;

	float nodeSolutionSize = pathNodeSize*nodeSizeFactor;//15.0;//1.0f
	SoMaterial *materialNodeSolution = new SoMaterial();
	materialNodeSolution->ambientColor.setValue(colors[colorSet].nodeR,colors[colorSet].nodeG,colors[colorSet].nodeB);
	materialNodeSolution->diffuseColor.setValue(colors[colorSet].nodeR,colors[colorSet].nodeG,colors[colorSet].nodeB);
	SoSphere *sphereNodeSolution = new SoSphere();
	sphereNodeSolution->radius.setValue(nodeSolutionSize);

	Eigen::VectorXf actConfig;
	float x,y,z;
	float x2 = 0.0f,y2 = 0.0f,z2 = 0.0f;

	SoSeparator *sep = new SoSeparator();

	// create 3D model for nodes
	SoSeparator *s = new SoSeparator();

	s->addChild(materialNodeSolution);
	SoTranslation *t = new SoTranslation();

	// get tcp coords:
	robotNodeSet->setJointValues(c);
	Eigen::Matrix4f m;
	m = TCPNode->getGlobalPose();
	x = m(0,3);//[0][3];
	y = m(1,3);//m[1][3];
	z = m(2,3);//m[2][3];

	t->translation.setValue(x,y,z);
	s->addChild(t);
	// display a solution node different
	s->addChild(sphereNodeSolution);
	sep->addChild(s);

	visualization->addChild(sep);
	return true;
}

} // namespace Saba
