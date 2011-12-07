
#include "ManipulationObject.h"
#include "VirtualRobotException.h"
#include "Visualization/VisualizationNode.h"
#include "GraspSet.h"
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
	THROW_VR_EXCEPTION_IF(hasGraspSet(graspSet->getRobotType(), graspSet->getEndEffector()), "Only one GraspSet per EEF allowed.");
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

std::string ManipulationObject::getXMLString(const std::string &basePath)
{
	std::stringstream ss;
	std::string t = "\t";

	ss << "<ManipulationObject name='" << name << "'>\n";
	if (visualizationModel)
	{
		ss << t << "<Visualization";
		if (visualizationModel->usedBoundingBoxVisu())
		{
			ss << " BoundingBox='true'";
		}
		ss << ">\n";
		std::string fnV = visualizationModel->getFilename();
		if (!fnV.empty())
		{
			if (!basePath.empty())
				BaseIO::makeRelativePath(basePath,fnV);
			ss << t << t << "<File type='" << visualizationModel->getType() << "'>" << fnV << "</File>\n";
		}
		ss << t << "</Visualization>\n";
	}

	if (collisionModel && collisionModel->getVisualization())
	{

		ss << t << "<CollisionModel";
		if (collisionModel->getVisualization()->usedBoundingBoxVisu())
		{
			ss << " BoundingBox='true'";
		}
		ss << ">\n";
		std::string fnC = collisionModel->getVisualization()->getFilename();
		if (!fnC.empty())
		{
			if (!basePath.empty())
				BaseIO::makeRelativePath(basePath,fnC);
			ss << t << t << "<File type='" << collisionModel->getVisualization()->getType() << "'>" << fnC << "</File>\n";
		}
		ss << t << "</CollisionModel>\n";
	}
	ss << "\n";
	for (size_t i=0;i<graspSets.size();i++)
	{
		ss << graspSets[i]->getXMLString() << "\n";
	}
	ss << "</ManipulationObject>\n";

	return ss.str();
}

ManipulationObjectPtr ManipulationObject::clone( const std::string &name, CollisionCheckerPtr colChecker )
{
	VisualizationNodePtr clonedVisualizationNode;
	if (visualizationModel)
		clonedVisualizationNode = visualizationModel->clone();
	CollisionModelPtr clonedCollisionModel;
	if (collisionModel)
		clonedCollisionModel = collisionModel->clone(colChecker);

	ManipulationObjectPtr result(new ManipulationObject(name, clonedVisualizationNode, clonedCollisionModel, physics, colChecker));

	if (!result)
	{
		VR_ERROR << "Cloning failed.." << endl;
		return result;
	}
	for (size_t i=0;i<graspSets.size();i++)
	{
		result->addGraspSet(graspSets[i]->clone());
	}

	return result;
}



} //  namespace


