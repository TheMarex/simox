
#include "ManipulationObject.h"
#include "VirtualRobotException.h"
#include "Visualization/VisualizationNode.h"
#include "Grasping/GraspSet.h"
#include "XML/BaseIO.h"

namespace VirtualRobot
{


ManipulationObject::ManipulationObject(const std::string &name, VisualizationNodePtr visualization, CollisionModelPtr collisionModel, const SceneObject::Physics &p, CollisionCheckerPtr colChecker)
:Obstacle(name,visualization,collisionModel,p,colChecker)
{
}

ManipulationObject::~ManipulationObject()
{
}

void ManipulationObject::print( bool printDecoration )
{
	if (printDecoration)
		cout << "**** Manipulation Object ****" << endl;

	Obstacle::print(false);

	for (size_t i=0;i<graspSets.size();i++)
	{
		cout << "* Grasp set " << i << ":" << endl;
		graspSets[i]->print();
	}
	if (printDecoration)
		cout << endl;
}

void ManipulationObject::addGraspSet( GraspSetPtr graspSet )
{
	THROW_VR_EXCEPTION_IF(!graspSet,"NULL data");
	THROW_VR_EXCEPTION_IF(hasGraspSet(graspSet),"Grasp set already added");
	// don't be too strict
	//THROW_VR_EXCEPTION_IF(hasGraspSet(graspSet->getRobotType(), graspSet->getEndEffector()), "Only one GraspSet per EEF allowed.");
	this->graspSets.push_back(graspSet);
}

bool ManipulationObject::hasGraspSet( GraspSetPtr graspSet )
{
	VR_ASSERT_MESSAGE(graspSet,"NULL data");
	for (size_t i=0;i<graspSets.size();i++)
		if (graspSets[i] == graspSet)
			return true;
	return false;
}

bool ManipulationObject::hasGraspSet( const std::string &robotType, const std::string &eef )
{
	for (size_t i=0;i<graspSets.size();i++)
		if (graspSets[i]->getRobotType() == robotType && graspSets[i]->getEndEffector() == eef)
			return true;
	return false;
}

VirtualRobot::GraspSetPtr ManipulationObject::getGraspSet( EndEffectorPtr eef )
{
	THROW_VR_EXCEPTION_IF(!eef,"NULL data");

	return getGraspSet(eef->getRobotType(),eef->getName());


}

VirtualRobot::GraspSetPtr ManipulationObject::getGraspSet( const std::string &robotType,const std::string &eefName )
{
	for (size_t i=0;i<graspSets.size();i++)
		if (graspSets[i]->getRobotType() == robotType && graspSets[i]->getEndEffector() == eefName)
			return graspSets[i];
	return GraspSetPtr();
}

VirtualRobot::GraspSetPtr ManipulationObject::getGraspSet( const std::string &name )
{
	for (size_t i=0;i<graspSets.size();i++)
		if (graspSets[i]->getName() == name)
			return graspSets[i];
	return GraspSetPtr();
}

std::string ManipulationObject::toXML(const std::string &basePath, int tabs, bool storeLinkToFile)
{
	std::stringstream ss;
	std::string t = "\t";
	std::string pre = "";
	for (int i=0;i<tabs;i++)
		pre += "\t";

	ss << pre << "<ManipulationObject name='" << name << "'>\n";
	
	if (storeLinkToFile)
	{
		std::string relFile = filename;
		if (!basePath.empty())
		{
			BaseIO::makeRelativePath(basePath, relFile);
		}
		ss << pre << t << "<File>" << relFile << "</File>\n";
		Eigen::Matrix4f gp = getGlobalPose();
		if (!gp.isIdentity())
		{
			ss << pre << t << "<GlobalPose>\n";
			ss << pre << t  << t  << "<Transform>\n";
			ss << MathTools::getTransformXMLString(gp,tabs+3);
			ss << pre << t  << t  << "</Transform>\n";
			ss << pre << t << "</GlobalPose>\n";
		}
	} else
	{

		ss << getSceneObjectXMLString(basePath,tabs+1);

		for (size_t i=0;i<graspSets.size();i++)
		{
			ss << graspSets[i]->getXMLString(tabs+1) << "\n";
		}
	}
	ss << pre << "</ManipulationObject>\n";

	return ss.str();
}

ManipulationObject* ManipulationObject::_clone( const std::string &name, CollisionCheckerPtr colChecker ) const
{
	VisualizationNodePtr clonedVisualizationNode;
	if (visualizationModel)
		clonedVisualizationNode = visualizationModel->clone();
	CollisionModelPtr clonedCollisionModel;
	if (collisionModel)
		clonedCollisionModel = collisionModel->clone(colChecker);

	ManipulationObject* result = new ManipulationObject(name, clonedVisualizationNode, clonedCollisionModel, physics, colChecker);

	if (!result)
	{
		VR_ERROR << "Cloning failed.." << endl;
		return result;
	}

	result->setGlobalPose(getGlobalPose());

	for (size_t i=0;i<graspSets.size();i++)
	{
		result->addGraspSet(graspSets[i]->clone());
	}

	return result;
}
/*
void ManipulationObject::setFilename( const std::string &filename )
{
	this->filename = filename;
}

std::string ManipulationObject::getFilename()
{
	return filename;
}
*/


} //  namespace


