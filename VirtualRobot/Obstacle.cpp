
#include "Obstacle.h"
#include "CollisionDetection/CollisionModel.h"
#include "CollisionDetection/CollisionChecker.h"
#include "Nodes/RobotNode.h"
#include "Visualization/VisualizationFactory.h"
#include <vector>

namespace VirtualRobot
{

// obstacle models start with 20000
int Obstacle::idCounter = 20000;


Obstacle::Obstacle(const std::string &name, VisualizationNodePtr visualization, CollisionModelPtr collisionModel, const SceneObject::Physics &p, CollisionCheckerPtr colChecker)
:SceneObject(name,visualization,collisionModel,p,colChecker)
{
	if (name=="")
	{
		// my id
		id = idCounter++;

		std::stringstream ss;
		ss << "VirtualRobot Obstacle <" << id <<">";
		this->name = ss.str();
	} else
	{
		if (collisionModel)
			id = collisionModel->getId();
		else
		{
			// my id
			id = idCounter++;
		}
	}
}

Obstacle::~Obstacle()
{
}

int Obstacle::getID()
{
	return id;
}

VirtualRobot::ObstaclePtr Obstacle::createBox( float width, float height, float depth, VisualizationFactory::Color color, std::string visualizationType , CollisionCheckerPtr colChecker )
{
	ObstaclePtr result;
	VisualizationFactoryPtr visualizationFactory;
	if (visualizationType.empty())
		visualizationFactory=VisualizationFactory::first(NULL);
	else
		visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
	if (!visualizationFactory)
	{
		VR_ERROR << "Could not create factory for visu type " << visualizationType << endl;
		return result;
	}
	VisualizationNodePtr visu = visualizationFactory->createBox(width,height,depth,color.r,color.g,color.b);
	if (!visu)
	{
		VR_ERROR << "Could not create box visualization with visu type " << visualizationType << endl;
		return result;
	}

	//TriMeshModelPtr trimesh = visu->getTriMeshModel();

	int id = idCounter;
	idCounter++;

	std::stringstream ss;
	ss << "Box_" << id;

	std::string name = ss.str();

	CollisionModelPtr colModel(new CollisionModel(visu,name,colChecker,id));
	result.reset(new Obstacle(name,visu,colModel, SceneObject::Physics(), colChecker));

	return result;
}


VirtualRobot::ObstaclePtr Obstacle::createSphere( float radius, VisualizationFactory::Color color, std::string visualizationType , CollisionCheckerPtr colChecker )
{
	ObstaclePtr result;
	VisualizationFactoryPtr visualizationFactory;
	if (visualizationType.empty())
		visualizationFactory=VisualizationFactory::first(NULL);
	else
		visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
	if (!visualizationFactory)
	{
		VR_ERROR << "Could not create factory for visu type " << visualizationType << endl;
		return result;
	}
	VisualizationNodePtr visu = visualizationFactory->createSphere(radius,color.r,color.g,color.b);
	if (!visu)
	{
		VR_ERROR << "Could not create sphere visualization with visu type " << visualizationType << endl;
		return result;
	}

	//TriMeshModelPtr trimesh = visu->getTriMeshModel();

	int id = idCounter;
	idCounter++;

	std::stringstream ss;
	ss << "Sphere_" << id;

	std::string name = ss.str();

	CollisionModelPtr colModel(new CollisionModel(visu,name,colChecker,id));
	result.reset(new Obstacle(name,visu,colModel, SceneObject::Physics(), colChecker));

	return result;
}

void Obstacle::print( bool printDecoration /*= true*/ )
{
	if (printDecoration)
		cout << "**** Obstacle ****" << endl;

	SceneObject::print(false);
	cout << " * id: " << id << endl;

	if (printDecoration)
		cout << endl;
}

ObstaclePtr Obstacle::clone( const std::string &name, CollisionCheckerPtr colChecker )
{
	VisualizationNodePtr clonedVisualizationNode;
	if (visualizationModel)
		clonedVisualizationNode = visualizationModel->clone();
	CollisionModelPtr clonedCollisionModel;
	if (collisionModel)
		clonedCollisionModel = collisionModel->clone(colChecker);

	ObstaclePtr result(new Obstacle(name, clonedVisualizationNode, clonedCollisionModel, physics, colChecker));

	if (!result)
	{
		VR_ERROR << "Cloning failed.." << endl;
		return result;
	}

	return result;
}

std::string Obstacle::getXMLString(const std::string &basePath, int tabs)
{
	std::stringstream ss;
	std::string t = "\t";
	std::string pre = "";
	for (int i=0;i<tabs;i++)
		pre += "\t";

	ss << pre << "<Obstacle name='" << name << "'>\n";

	ss << getSceneObjectXMLString(basePath,tabs);
	
	ss << pre << "</Obstacle>\n";

	return ss.str();
}

} //  namespace


